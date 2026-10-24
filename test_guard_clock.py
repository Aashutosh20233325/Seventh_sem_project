"""B3 runtime-guard clock tests (offline): PX4 speed from reconstructed HRT, PX4 link liveness from local-position
receipt, slow-versus-stalled discrimination, and preservation of the 0.35x gate."""
import threading
from types import SimpleNamespace

import numpy as np
import pytest

import run_target_reaching_episode as runner
from test_sim_clock import FIXTURE, bare_node, load_rows, position_msg
from test_px4_uav_env_safety import load_environment


# ---------- recorded-slew replay through the real callbacks and the real adapter ----------

def old_adapter(env):
    """The pre-B3 guard input: slewed local-position stamp accumulator + position receipt."""
    node = env.node

    def read():
        with node.condition:
            px4 = None
            if node.sim_clock_us is not None and node.position_received_at is not None:
                px4 = (node.sim_clock_us / 1_000_000, node.position_received_at)
            camera = None
            if node.latest_perception is not None and node.perception_received_at is not None:
                camera = (float(node.latest_perception["image_stamp"]), node.perception_received_at)
        return {"px4": px4, "camera": camera, "px4_link": node.position_received_at}
    return read


def replay_guards(monkeypatch, rows):
    module = load_environment(monkeypatch)
    now = [rows[0][1]]
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: now[0]))
    node = bare_node(module)
    node.latest_perception = None
    node.perception_received_at = None
    env = SimpleNamespace(node=node)
    clock = lambda: now[0]                                          # noqa: E731
    new = runner.RuntimeGuard(runner.px4_telemetry(env), memory=lambda: 2000.0, clock=clock)
    old = runner.RuntimeGuard(old_adapter(env), memory=lambda: 2000.0, clock=clock)
    verdicts, next_sample = [], rows[0][1] + 1.0
    for row in rows:
        while row[1] >= next_sample:                                # 1 Hz guard sampling, as in the runner
            now[0] = next_sample
            new.sample(); old.sample()
            verdicts.append((next_sample, new.check(), old.check()))
            next_sample += 1.0
        now[0] = row[1]
        if row[0] == "p":
            module.PX4ControllerNode.position_callback(node, position_msg(row[2]))
        elif row[0] == "t":
            module.PX4ControllerNode.timesync_status_callback(
                node, SimpleNamespace(timestamp=row[2], estimated_offset=row[3]))
        else:
            node.latest_perception = {"image_stamp": row[2]}
            node.perception_received_at = row[1]
    return verdicts


def test_recorded_slew_old_px4_term_is_inflated_new_matches_the_camera(monkeypatch):
    verdicts = replay_guards(monkeypatch, load_rows(FIXTURE))
    rated = [(n, o) for _, n, o in verdicts if n["camera_status"] == "ok" and n["px4_status"] == "ok"
             and o["px4_status"] == "ok"]
    assert len(rated) > 30
    new_err = np.array([n["px4_rate"] - n["camera_rate"] for n, _ in rated])
    old_err = np.array([o["px4_rate"] - o["camera_rate"] for _, o in rated])
    # The camera term's own noise: perception receipt latency varies by up to ~0.3 s, i.e. ~0.005 of rate per
    # 25 s window endpoint at ~0.38x, ~0.01 for two. Measured: new mean -0.0007, max 0.0101; old mean +0.046.
    assert np.abs(new_err).max() < 0.015 and abs(new_err.mean()) < 0.003   # HRT agrees with the camera clock
    assert old_err.mean() > 0.03 and old_err.max() > 0.05                  # slewed stamps overstate PX4 speed
    assert all(n["px4_link_status"] == "ok" for n, _ in rated)
    assert all(n["reason"] in (None, "speed_below_floor") for n, _ in rated)   # never a spurious clock failure


# ---------- synthetic telemetry ----------

class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


class Sim:
    """A simulated run: camera stamps every 0.1 s wall, PX4 HRT every 1.0 s simulated (+ optional bursts),
    PX4 local position (link) every 0.06 s wall. Each stream can be stopped independently."""

    def __init__(self, clock):
        self.clock = clock
        self.sim = 50.0
        self.latest = {"px4": None, "camera": None, "px4_link": None}
        self.next_hrt = 51.0
        self.hrt_offset = 0.0

    def __call__(self):
        return dict(self.latest)

    def run(self, guard, seconds, speed=0.4, camera=True, hrt=True, link=True, sim_runs=True,
            hrt_period=1.0, dt=0.02):
        steps = int(round(seconds / dt))
        for k in range(steps):
            self.clock.t += dt
            if sim_runs:
                self.sim += speed * dt
            if camera and sim_runs and k % 5 == 0:
                self.latest["camera"] = (self.sim, self.clock.t)
            if hrt and self.sim >= self.next_hrt:
                self.latest["px4"] = (self.next_hrt + self.hrt_offset, self.clock.t)
                self.next_hrt += hrt_period
            if link and k % 3 == 0:
                self.latest["px4_link"] = self.clock.t
            if k % 50 == 0:                                         # guard samples at 1 Hz
                guard.sample()
        guard.sample()


def make(memory=2000.0):
    clock = Clock()
    sim = Sim(clock)
    return runner.RuntimeGuard(sim, memory=lambda: memory, clock=clock), sim, clock


def test_healthy_slow_simulation_passes_with_sparse_hrt():
    guard, sim, _ = make()
    sim.run(guard, 40, speed=0.36)                                  # HRT every ~2.8 s wall: normal at 0.36x
    v = guard.check()
    assert v["ok"] and v["reason"] is None
    assert v["px4_rate"] == pytest.approx(0.36, abs=0.02) and v["camera_rate"] == pytest.approx(0.36, abs=0.01)
    assert v["px4_link_status"] == "ok"


def test_simulation_slower_than_the_floor_fails_on_speed_not_staleness():
    guard, sim, _ = make()
    sim.run(guard, 40, speed=0.30)                                  # HRT every ~3.3 s wall: still < 5 s
    v = guard.check()
    assert not v["ok"] and v["reason"] == "speed_below_floor"
    assert v["px4_status"] == "ok" and v["speed"] == pytest.approx(0.30, abs=0.02)


def test_very_slow_simulation_never_passes():
    guard, sim, _ = make()
    sim.run(guard, 60, speed=0.15)                                  # HRT every ~6.7 s wall
    v = guard.check()
    assert not v["ok"] and v["reason"] in ("px4_clock_stale", "px4_clock_gap", "speed_below_floor")


def test_whole_simulation_stall_fails_on_the_camera_clock_first():
    guard, sim, _ = make()
    sim.run(guard, 30)
    sim.run(guard, 2.0, sim_runs=False)                             # nothing advances; PX4 link still alive
    v = guard.check()
    assert not v["ok"] and v["reason"] == "camera_clock_stale"


def test_px4_clock_stall_while_the_simulation_runs_is_detected():
    guard, sim, _ = make()
    sim.run(guard, 30, speed=0.8)
    sim.run(guard, 1.0, speed=0.8, hrt=False)
    assert guard.check()["ok"]                                      # < 2 s of camera time without HRT: normal
    sim.run(guard, 2.5, speed=0.8, hrt=False)                       # camera +2.8 s simulated, PX4 time frozen
    v = guard.check()
    assert not v["ok"] and v["reason"] == "px4_clock_stalled"


def test_px4_clock_silence_is_stale_after_the_window_gap_limit():
    guard, sim, _ = make()
    sim.run(guard, 30, speed=0.36)
    sim.run(guard, 5.5, speed=0.36, hrt=False)                      # 5.5 s wall but < 2 s camera time
    v = guard.check()
    assert not v["ok"] and v["reason"] in ("px4_clock_stale", "px4_clock_stalled")


def test_duplicate_hrt_samples_do_not_refresh_freshness():
    guard, sim, clock = make()
    sim.run(guard, 30, speed=0.36)
    frozen = sim.latest["px4"][0]
    for _ in range(12):                                             # same HRT re-delivered every 0.5 s
        sim.run(guard, 0.5, speed=0.36, hrt=False)
        sim.latest["px4"] = (frozen, clock.t)
        guard.sample()
    v = guard.check()
    assert not v["ok"] and v["reason"] in ("px4_clock_stale", "px4_clock_stalled")


def test_filter_reset_burst_does_not_change_the_rate():
    guard, sim, clock = make()
    sim.run(guard, 30, speed=0.38)
    before = guard.check()["px4_rate"]
    for _ in range(8):                                              # post-reset burst: exact HRT values, close together
        clock.t += 0.1
        sim.sim += 0.038
        sim.latest["px4"] = (sim.sim, clock.t)
        guard.sample()
    sim.run(guard, 2.0, speed=0.38)
    v = guard.check()
    assert v["ok"] and v["px4_rate"] == pytest.approx(before, abs=0.01)


def test_backward_hrt_restarts_history_and_fails_closed():
    guard, sim, _ = make()
    sim.run(guard, 30)
    sim.hrt_offset = -20.0                                          # HRT jumps backwards
    sim.run(guard, 3.0)                                             # 1.2 s simulated: crosses an HRT period
    assert guard.check()["reason"] == "px4_clock_insufficient_history"


@pytest.mark.parametrize("missing", ["link", "hrt", "camera"])
def test_missing_inputs_fail_closed_and_preflight_waits_then_refuses(missing):
    guard, sim, _ = make()
    kw = {"link": dict(link=False), "hrt": dict(hrt=False), "camera": dict(camera=False)}[missing]
    sim.run(guard, 30, **kw)
    v = guard.check()
    expected = {"link": "px4_telemetry_missing", "hrt": "px4_clock_missing", "camera": "camera_clock_missing"}[missing]
    assert not v["ok"] and v["reason"] == expected and v["speed"] is None
    sleeps = []
    verdict = guard.preflight(wait_s=3, sleep=lambda s: (sleeps.append(s), sim.run(guard, 1.0, **kw)))
    assert not verdict["ok"] and len(sleeps) >= 2                   # waited (pending), then refused at the deadline


def test_px4_link_loss_fails_within_one_second_even_with_fresh_hrt():
    guard, sim, _ = make()
    sim.run(guard, 30)
    sim.run(guard, 1.2, link=False)
    v = guard.check()
    assert not v["ok"] and v["reason"] == "px4_telemetry_stale" and v["speed"] is None


def test_camera_stall_with_running_px4_fails_on_the_camera():
    guard, sim, _ = make()
    sim.run(guard, 30)
    sim.run(guard, 2.0, camera=False)
    assert guard.check()["reason"] == "camera_clock_stale"


@pytest.mark.parametrize("hrt_speed, camera_speed", [(0.45, 0.30), (0.30, 0.45)])
def test_conflicting_clocks_take_the_slower(hrt_speed, camera_speed):
    clock = Clock()
    t = {"px4": None, "camera": None, "px4_link": None}
    guard = runner.RuntimeGuard(lambda: dict(t), memory=lambda: 2000.0, clock=clock)
    hrt, cam, next_hrt = 50.0, 50.0, 51.0
    for _ in range(int(40 / 0.1)):
        clock.t += 0.1
        hrt += hrt_speed * 0.1
        cam += camera_speed * 0.1
        if hrt >= next_hrt:
            t["px4"] = (next_hrt, clock.t)
            next_hrt += 1.0
        t["camera"] = (cam, clock.t)
        t["px4_link"] = clock.t
        guard.sample()
    v = guard.check()
    assert not v["ok"] and v["reason"] in ("speed_below_floor", "px4_clock_stalled")
    if v["reason"] == "speed_below_floor":
        assert v["speed"] == pytest.approx(0.30, abs=0.02)


@pytest.mark.parametrize("speed, ok", [(0.348, False), (0.355, True)])
def test_the_035_floor_is_unchanged(speed, ok):
    guard, sim, _ = make()
    sim.run(guard, 40, speed=speed)
    v = guard.check()
    assert runner.SPEED_FLOOR == 0.35 and v["thresholds"]["speed_floor"] == 0.35
    assert v["ok"] is ok
    if not ok:
        assert v["reason"] == "speed_below_floor"


def test_guard_constants_are_pinned():
    assert (runner.SPEED_FLOOR, runner.SPEED_WINDOW_S, runner.MEM_START_MIB, runner.MEM_FLOOR_MIB) == \
        (0.35, 25.0, 1.2 * 1024, 600.0)
    assert (runner.PX4_MAX_AGE_S, runner.CAMERA_MAX_AGE_S, runner.MAX_SAMPLE_GAP_S) == (1.0, 1.5, 5.0)
    assert runner.PX4_HRT_MAX_AGE_S == runner.MAX_SAMPLE_GAP_S and runner.PX4_HRT_STALL_CAMERA_S == 2.0


def test_adapter_reads_hrt_and_link_from_the_env_node():
    node = SimpleNamespace(condition=threading.Condition(), px4_sim_time_us=12_345_000, px4_sim_time_received_at=7.0,
                           sim_clock_us=99_000_000, position_received_at=8.0,
                           latest_perception={"image_stamp": 3.5}, perception_received_at=6.0)
    read = runner.px4_telemetry(SimpleNamespace(node=node))()
    assert read == {"px4": (12.345, 7.0), "camera": (3.5, 6.0), "px4_link": 8.0}
    node.px4_sim_time_us = None
    assert runner.px4_telemetry(SimpleNamespace(node=node))()["px4"] is None   # never falls back to sim_clock_us
