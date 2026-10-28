import json

import pytest

import run_target_reaching_episode as runner


def test_standin_dry_run_reaches_target_and_logs_every_step(tmp_path):
    log = tmp_path / "episode.jsonl"
    assert runner.main(["--standin", "--log", str(log)]) == 0
    events = [json.loads(line) for line in log.read_text().splitlines()]
    kinds = [e["event"] for e in events]
    assert kinds[:2] == ["world", "reset_start"] and kinds[-2:] == ["close_start", "close_returned"]
    steps = [e for e in events if e["event"] == "step"]
    assert steps and steps[-1]["info"]["reason"] == "success"
    assert all("wall_utc" in e and "mono" in e for e in events)
    assert all("distance_to_target" not in e["fields"] for e in steps)


def test_sitl_run_requires_explicit_supervision_flag(tmp_path):
    with pytest.raises(SystemExit):
        runner.main(["--log", str(tmp_path / "x.jsonl")])


def test_existing_log_is_never_overwritten(tmp_path):
    log = tmp_path / "episode.jsonl"
    log.write_text("keep\n")
    with pytest.raises(FileExistsError):
        runner.main(["--standin", "--log", str(log)])
    assert log.read_text() == "keep\n"


# ---------- runtime guard ----------

import numpy as np

from sar_standin_env import DT, SarStandInEnv


class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


class Feed:
    """Fake telemetry: per-clock (sim_seconds, received_at) advanced by the test."""

    def __init__(self, clock):
        self.clock = clock
        self.latest = {"px4": None, "camera": None, "px4_link": None}
        self.sim = {"px4": 0.0, "camera": 100.0}

    def __call__(self):
        return dict(self.latest)

    def run(self, guard, seconds, px4_rate=0.4, camera_rate=0.4, px4=True, camera=True, period=1.0):
        for _ in range(int(round(seconds / period))):
            self.clock.t += period
            for name, rate, alive in (("px4", px4_rate, px4), ("camera", camera_rate, camera)):
                self.sim[name] += rate * period
                if alive:
                    self.latest[name] = (self.sim[name], self.clock.t)
            if px4:                                             # PX4 local-position receipt (liveness)
                self.latest["px4_link"] = self.clock.t
            guard.sample()


def make_guard(memory=2000.0):
    clock = Clock()
    feed = Feed(clock)
    guard = runner.RuntimeGuard(feed, memory=lambda: memory, clock=clock)
    return guard, feed, clock


def test_guard_healthy_window_uses_the_lower_clock():
    guard, feed, _ = make_guard()
    feed.run(guard, 30, px4_rate=0.5, camera_rate=0.4)
    verdict = guard.check()
    assert verdict["ok"] and verdict["reason"] is None
    assert verdict["px4_rate"] == pytest.approx(0.5) and verdict["camera_rate"] == pytest.approx(0.4)
    assert verdict["speed"] == pytest.approx(0.4)


@pytest.mark.parametrize("px4_rate, camera_rate", [(0.34, 0.5), (0.5, 0.34)])
def test_guard_aborts_when_either_clock_is_below_the_floor(px4_rate, camera_rate):
    guard, feed, _ = make_guard()
    feed.run(guard, 30, px4_rate=px4_rate, camera_rate=camera_rate)
    verdict = guard.check()
    assert not verdict["ok"] and verdict["reason"] == "speed_below_floor"
    assert verdict["speed"] == pytest.approx(0.34)
    assert verdict["thresholds"]["speed_floor"] == 0.35 and verdict["thresholds"]["speed_window_s"] == 25.0


def test_guard_rolling_window_catches_a_recent_slowdown():
    guard, feed, _ = make_guard()
    feed.run(guard, 60, camera_rate=0.45)
    assert guard.check()["ok"]
    feed.run(guard, 25, camera_rate=0.30)
    verdict = guard.check()
    assert verdict["reason"] == "speed_below_floor" and verdict["camera_rate"] == pytest.approx(0.30)


@pytest.mark.parametrize("memory, reason", [(599.0, "memory_below_floor"), (None, "memory_unavailable")])
def test_guard_aborts_on_low_or_unreadable_memory(memory, reason):
    guard, feed, _ = make_guard(memory=memory)
    feed.run(guard, 30)
    verdict = guard.check()
    assert not verdict["ok"] and verdict["reason"] == reason
    assert verdict["mem_available_mib"] == memory and verdict["thresholds"]["mem_floor_mib"] == 600.0


def test_guard_preflight_refuses_below_start_memory_without_waiting():
    guard, feed, _ = make_guard(memory=1200.0)             # >= 600 floor but < 1.2 GiB start threshold
    feed.run(guard, 30)
    sleeps = []
    verdict = guard.preflight(sleep=sleeps.append)
    assert not verdict["ok"] and verdict["reason"] == "memory_below_floor"
    assert verdict["thresholds"]["mem_floor_mib"] == pytest.approx(1228.8)
    assert sleeps == []


def test_guard_preflight_waits_for_history_but_never_passes_on_missing_data():
    guard, feed, clock = make_guard()

    def sleep_without_camera(_):
        feed.run(guard, 1, camera=False)

    verdict = guard.preflight(wait_s=40, sleep=sleep_without_camera)
    assert not verdict["ok"] and verdict["reason"] == "camera_clock_missing"
    guard2, feed2, _ = make_guard()
    verdict = guard2.preflight(wait_s=40, sleep=lambda _: feed2.run(guard2, 1))
    assert verdict["ok"]


def test_guard_insufficient_history_is_not_healthy():
    guard, feed, _ = make_guard()
    feed.run(guard, 10)
    assert guard.check()["reason"] == "px4_clock_insufficient_history"


@pytest.mark.parametrize("stream", ["px4", "camera"])
def test_guard_stale_clock_aborts_even_with_a_good_history(stream):
    guard, feed, _ = make_guard()
    feed.run(guard, 30)
    feed.run(guard, 3, **{stream: False})                   # the stream stops (beyond the camera's 2.5 s); the other runs
    verdict = guard.check()
    # PX4 data stopping entirely is first a PX4 link loss (local-position receipt > 1.0 s).
    expected = "px4_telemetry_stale" if stream == "px4" else "camera_clock_stale"
    assert not verdict["ok"] and verdict["reason"] == expected
    assert verdict["speed"] is None


def test_guard_repeated_stale_value_is_not_resampled_as_new():
    guard, feed, clock = make_guard()
    feed.run(guard, 30)
    count = len(guard.samples["camera"])
    for _ in range(5):
        clock.t += 1.0
        feed.latest["px4"] = (feed.sim["px4"] + 0.4, clock.t)
        feed.latest["px4_link"] = clock.t
        guard.sample()                                       # camera value unchanged
    assert len(guard.samples["camera"]) == count
    assert guard.check()["reason"] == "camera_clock_stale"


def test_guard_gap_inside_window_invalidates_it():
    guard, feed, _ = make_guard()
    feed.run(guard, 30)
    feed.run(guard, 8, camera=False)                         # 8 s without camera frames
    feed.run(guard, 1)                                       # frames resume (fresh latest sample)
    assert guard.check()["reason"] == "camera_clock_gap"


def test_guard_clock_reset_restarts_history():
    guard, feed, _ = make_guard()
    feed.run(guard, 30)
    feed.sim["px4"] = 0.0                                    # PX4 clock went backwards
    feed.run(guard, 1)
    assert guard.check()["reason"] == "px4_clock_insufficient_history"


def test_mem_available_reads_meminfo_and_reports_unreadable(tmp_path):
    meminfo = tmp_path / "meminfo"
    meminfo.write_text("MemTotal: 4000000 kB\nMemAvailable: 2097152 kB\n")
    assert runner.mem_available_mib(meminfo) == 2048.0
    assert runner.mem_available_mib(tmp_path / "missing") is None
    meminfo.write_text("MemTotal: 1 kB\n")
    assert runner.mem_available_mib(meminfo) is None


# ---------- runner integration (stand-in env, no ROS) ----------

class CountingEnv:
    def __init__(self, guard_feed=None, wall_per_step=None):
        self.env = SarStandInEnv()
        self.resets = self.steps = self.closes = 0
        self.feed = guard_feed
        self.wall_per_step = wall_per_step

    def reset(self):
        self.resets += 1
        return self.env.reset()

    def step(self, action):
        self.steps += 1
        if self.feed is not None:
            rate = DT / self.wall_per_step(self.steps)
            self.feed.run(self.feed_guard, self.wall_per_step(self.steps), px4_rate=rate, camera_rate=rate,
                          period=self.wall_per_step(self.steps))
        return self.env.step(action)

    def close(self):
        self.closes += 1


class ScriptedGuard:
    def __init__(self, preflight, checks):
        self._preflight = preflight
        self.checks = list(checks)
        self.started = self.stopped = 0

    def preflight(self):
        return self._preflight

    def check(self):
        return self.checks.pop(0) if self.checks else OK

    def start(self, log):
        self.started += 1

    def stop(self):
        self.stopped += 1


OK = {"ok": True, "reason": None, "speed": 0.4, "mem_available_mib": 2000.0}


def events(log):
    return [json.loads(line) for line in log.read_text().splitlines()]


def test_low_initial_memory_refuses_before_reset_and_still_closes(tmp_path):
    env = CountingEnv()
    guard = runner.RuntimeGuard(lambda: {"px4": None, "camera": None}, memory=lambda: 1000.0)
    log_path = tmp_path / "episode.jsonl"
    with log_path.open("x") as log:
        assert runner.execute(env, log, 600, guard) == "guard_refuse"
    assert (env.resets, env.steps, env.closes) == (0, 0, 1)
    kinds = [e["event"] for e in events(log_path)]
    assert "reset_start" not in kinds and kinds[-2:] == ["close_start", "close_returned"]
    refuse = next(e for e in events(log_path) if e["event"] == "guard_refuse")
    assert refuse["reason"] == "memory_below_floor"
    assert refuse["guard"]["mem_available_mib"] == 1000.0
    assert refuse["guard"]["thresholds"]["mem_floor_mib"] == pytest.approx(1228.8)


@pytest.mark.parametrize("reason", ["speed_below_floor", "memory_below_floor", "camera_clock_stale"])
def test_guard_abort_stops_actions_logs_and_closes_once_without_retry(tmp_path, reason):
    bad = {"ok": False, "reason": reason, "speed": 0.30, "mem_available_mib": 550.0,
           "thresholds": {"speed_floor": 0.35, "mem_floor_mib": 600.0}}
    env = CountingEnv()
    guard = ScriptedGuard(OK, [OK, OK, OK, bad])
    log_path = tmp_path / "episode.jsonl"
    with log_path.open("x") as log:
        assert runner.execute(env, log, 600, guard) == "guard_abort"
    assert (env.resets, env.steps, env.closes) == (1, 3, 1)
    assert guard.started == guard.stopped == 1
    recs = events(log_path)
    abort = next(e for e in recs if e["event"] == "guard_abort")
    assert abort["step"] == 3 and abort["reason"] == reason and abort["guard"] == bad
    assert "wall_utc" in abort and "mono" in abort
    assert [e["event"] for e in recs][-3:] == ["episode_end", "close_start", "close_returned"]
    assert recs[-3]["reason"] == "guard_abort"


def test_normal_guarded_run_reaches_target_with_real_guard(tmp_path):
    guard, feed, _ = make_guard()
    feed.run(guard, 30)                                      # history from before reset
    env = CountingEnv(guard_feed=feed, wall_per_step=lambda step: 0.25)   # 0.4x
    env.feed_guard = guard
    guard.start = lambda log: None                           # no background thread with a fake clock
    log_path = tmp_path / "episode.jsonl"
    with log_path.open("x") as log:
        assert runner.execute(env, log, 600, guard) == "success"
    assert env.closes == 1 and "guard_abort" not in [e["event"] for e in events(log_path)]


def test_real_guard_aborts_mid_episode_when_simulation_slows(tmp_path):
    guard, feed, _ = make_guard()
    feed.run(guard, 30)
    # 0.4x for 40 steps, then 0.2x: the 25 s window falls below 0.35 well before the target is reached.
    env = CountingEnv(guard_feed=feed, wall_per_step=lambda step: 0.25 if step <= 40 else 0.5)
    env.feed_guard = guard
    guard.start = lambda log: None
    log_path = tmp_path / "episode.jsonl"
    with log_path.open("x") as log:
        assert runner.execute(env, log, 600, guard) == "guard_abort"
    abort = next(e for e in events(log_path) if e["event"] == "guard_abort")
    assert abort["reason"] == "speed_below_floor" and abort["guard"]["speed"] < 0.35
    assert 40 < env.steps < 600 and env.closes == 1


# ---------- the abort's landing path: PX4UAVEnv.close() from an active, armed episode ----------

def test_close_during_active_armed_episode_lands_once_and_waits_for_disarm(monkeypatch):
    import threading
    import time

    from test_px4_uav_env_safety import load_environment, make_close_environment, report_landed_and_disarmed

    module = load_environment(monkeypatch)
    env = make_close_environment(module)
    env.LAND_WAIT_TIMEOUT = 0.5
    monkeypatch.setattr(module.rclpy, "ok", lambda: False)
    env.node.arming_state = 2                                # armed, mid-episode
    env.node.phase = "VELOCITY"
    env.node.action[:] = [0.6, -0.3]
    seen = {}

    def px4_lands_later():
        time.sleep(0.05)
        seen["phase"], seen["action"] = env.node.phase, env.node.action.copy()
        seen["land_calls"], seen["alive"] = env.node.land_calls, env.spin_thread.join_calls == 0
        report_landed_and_disarmed(module, env.node)

    reporter = threading.Thread(target=px4_lands_later)
    reporter.start()
    env.close()
    reporter.join()

    assert seen["phase"] == "LAND" and seen["land_calls"] == 1 and seen["alive"]
    np.testing.assert_array_equal(seen["action"], [0.0, 0.0])
    assert not env._episode_active
    assert env.node.land_calls == 1 and env.node.landing_complete is True
    assert env.node.destroy_calls == 1                       # ROS released only after landed + disarmed
