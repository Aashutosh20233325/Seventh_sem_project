"""Frontier explorer (design A): map layers, view prediction, path search, frontier ranking, mode hand-over, the
safety filter on explorer actions, and that it never reads the target position. Offline."""
import math

import numpy as np
import pytest

import explorer as ex
import sar_observation as so


def obs_with(ranges_m, det=False, est=False):
    obs = np.zeros(so.DIM, np.float32)
    obs[so.SLICES["lidar"]] = np.clip(np.asarray(ranges_m, np.float32) / so.LIDAR_RANGE_MAX, 0, 1)
    obs[so.SLICES["det_now"]] = 1.0 if det else 0.0
    obs[so.SLICES["est_valid"]] = 1.0 if est else 0.0
    return obs


OPEN = obs_with(np.full(so.LIDAR_SECTORS, 30.0))


def test_lidar_marks_free_space_and_hits():
    m = ex.ExplorerMap()
    ranges = np.full(so.LIDAR_SECTORS, 30.0)
    front = int(np.argmin(np.abs(ex.SECTOR_ANGLES)))
    ranges[front] = 5.0                                    # wall 5 m straight ahead (heading north)
    m.update_lidar(0.0, 0.0, 0.0, obs_with(ranges))
    assert m.state[m.cell(5.0, 0.0)] == ex.OCC
    assert m.state[m.cell(3.0, 0.0)] == ex.FREE and m.state[m.cell(0.0, 10.0)] == ex.FREE
    assert m.state[m.cell(-10.0, 0.0)] == ex.UNKNOWN      # behind: outside the 270 deg scan
    m.update_lidar(0.0, 0.0, 0.0, OPEN)                    # an occupied cell is never cleared
    assert m.state[m.cell(5.0, 0.0)] == ex.OCC


def test_camera_view_stops_at_walls_and_marks_searched():
    m = ex.ExplorerMap()
    i, j = m.cell(4.0, 0.0)
    m.state[i, j - 3:j + 4] = ex.OCC                       # wall across the view 4 m ahead
    n = m.view(0.0, 0.0, 0.0, mark=True)
    assert n > 0 and m.searched[m.cell(2.0, 0.0)] and not m.searched[m.cell(6.0, 0.0)]
    assert m.view(0.0, 0.0, 0.0) == 0                      # already searched: no gain
    assert m.view(0.0, 0.0, math.pi) > 0                   # unknown behind is optimistic (see-through)


def test_path_search_goes_around_a_wall():
    m = ex.ExplorerMap()
    m.state[:, :] = ex.FREE
    i0, _ = m.cell(3.0, 0.0)
    _, j_lo = m.cell(0.0, -6.0)
    _, j_hi = m.cell(0.0, 6.0)
    m.state[i0, j_lo:j_hi] = ex.OCC                        # 12 m wall between start and goal
    start, goal = m.cell(0.0, 0.0), m.cell(6.0, 0.0)
    blocked = m.blocked()
    dist, pred = ex.dijkstra(start, blocked, m.state)
    path = ex.path_to(pred, start, goal)
    assert path is not None and dist[goal] > 6.0 + 2 * 6.0 - 2  # had to go around an end of the wall
    assert not any(blocked[c] for c in path)


def test_unreachable_goal_has_no_path():
    m = ex.ExplorerMap()
    m.state[:, :] = ex.FREE
    gi, gj = m.cell(5.0, 5.0)
    m.state[gi - 4:gi + 5, gj - 4:gj + 5] = ex.OCC
    m.state[gi - 3:gi + 4, gj - 3:gj + 4] = ex.FREE         # enclosed courtyard
    dist, pred = ex.dijkstra(m.cell(0.0, 0.0), m.blocked(), m.state)
    assert not np.isfinite(dist[gi, gj]) and ex.path_to(pred, m.cell(0.0, 0.0), (gi, gj)) is None


class Inner:
    def __init__(self):
        self.calls = 0

    def act(self, obs):
        self.calls += 1
        return np.array([0.5, 0.1], np.float32)


def controller(pose=(0.0, 0.0, 0.0)):
    state = {"pose": pose}
    inner = Inner()
    return ex.ExplorerController(inner, lambda: state["pose"]), inner, state


def test_explores_until_detection_then_hands_over_and_back():
    c, inner, _ = controller()
    a = c.act(OPEN)
    assert c.mode == "EXPLORE" and inner.calls == 0 and c.last["mode"] == "EXPLORE"
    assert a.shape == (2,) and 0 <= a[0] <= 1 and -1 <= a[1] <= 1
    a = c.act(obs_with(np.full(so.LIDAR_SECTORS, 30.0), det=True))
    assert c.mode == "APPROACH" and inner.calls == 1 and np.allclose(a, [0.5, 0.1])
    c.act(obs_with(np.full(so.LIDAR_SECTORS, 30.0), est=True))   # estimate still valid: keep approaching
    assert c.mode == "APPROACH" and inner.calls == 2
    for _ in range(ex.LOST_STEPS):
        c.act(OPEN)
    assert c.mode == "EXPLORE"


def test_explorer_actions_pass_the_lidar_safety_filter():
    c, _, _ = controller()
    blocked_front = obs_with(np.full(so.LIDAR_SECTORS, 1.0))     # everything 1 m away: stop clearance
    a = c.act(blocked_front)
    assert a[0] == 0.0 and c.last["filter"] in ("stop", "emergency")


def test_frontier_ranking_prefers_nearby_gain():
    c, _, _ = controller()
    c.map.state[:, :] = ex.FREE
    c.map.searched[:, :] = True
    for (x, y) in ((4.0, 0.0), (15.0, 0.0)):                      # two equal unsearched patches
        i, j = c.map.cell(x + 3.0, y)
        c.map.searched[i - 3:i + 4, j - 3:j + 4] = False
    goal, n = c._plan(0.0, 0.0)
    gx, gy = c.map.centre(goal[0], goal[1])
    assert n >= 2 and gx < 10.0                                   # the near patch wins at equal gain


def test_far_shallow_hits_do_not_put_phantom_walls_in_a_tunnel():
    """Looking down a 5 m wide tunnel, a sector's minimum comes from a side wall at the edge of its wedge, far away; it
    must not be marked on the centre ray inside the free tunnel (HIT_MARK_MAX), while near hits still are."""
    m = ex.ExplorerMap()
    ranges = np.full(so.LIDAR_SECTORS, 30.0)
    for k, a in enumerate(ex.SECTOR_ANGLES):             # side walls at +-2.5 m along a north-going tunnel
        for edge in (a - 0.048, a + 0.048):
            if abs(math.sin(edge)) > 1e-6:
                ranges[k] = min(ranges[k], abs(2.5 / math.sin(edge)))
    m.update_lidar(0.0, 0.0, 0.0, obs_with(np.minimum(ranges, 30.0)))
    occ_inside = [(x, y) for x in np.arange(1.0, 20.0, 0.4) for y in np.arange(-1.6, 1.6, 0.4)
                  if m.state[m.cell(x, y)] == ex.OCC]
    assert occ_inside == []                                # no phantom wall in the tunnel's free middle
    assert m.state[m.cell(0.3, 2.6)] == ex.OCC or m.state[m.cell(0.3, -2.6)] == ex.OCC or any(
        m.state[m.cell(x, s * 2.6)] == ex.OCC for x in np.arange(0.0, 6.0, 0.4) for s in (-1, 1))   # near walls marked


def test_search_area_bounds_goals_and_paths():
    m = ex.ExplorerMap()
    edge = m.cell(ex.SEARCH_HALF + 1.0, 0.0)
    assert m.searched[edge] and m.blocked()[edge]              # outside: no gain, never on a path
    assert not m.searched[m.cell(ex.SEARCH_HALF - 1.0, 0.0)]
    m.state[:, :] = ex.FREE
    assert m.view(ex.SEARCH_HALF - 2.0, 0.0, 0.0) < m.view(0.0, 0.0, 0.0)   # looking out of the area gains less


def test_a_reached_viewpoint_is_not_chosen_again():
    c, _, state = controller()
    c.map.state[:, :] = ex.FREE
    c.map.searched[:, :] = True
    i, j = c.map.cell(3.0, 0.0)
    c.map.searched[i - 2:i + 3, j - 2:j + 3] = False           # one unsearched patch just ahead
    goal, _ = c._plan(0.0, 0.0, 0.0)
    gx, gy = c.map.centre(goal[0], goal[1])
    state["pose"] = (gx, gy, goal[2])                           # arrived, facing the viewing heading
    c.path = []
    c._follow(gx, gy, goal[2])
    assert c.goal is None and (goal[0], goal[1]) in c.failed
    c._plan(gx, gy, goal[2])
    assert c.goal is None or (c.goal[0], c.goal[1]) != (goal[0], goal[1])


def test_approach_hands_over_at_the_closest_reachable_point():
    """A target behind a wall: the explorer plans around, and once the planned path is as close as the obstacle
    clearance allows, the inner controller (with its own safety filter) flies the rest instead of stopping."""
    c, inner, state = controller()
    c.map.state[:, :] = ex.FREE
    i0, j0 = c.map.cell(2.2, -0.8)
    i1, j1 = c.map.cell(3.8, 0.8)
    c.map.state[i0:i1 + 1, j0:j1 + 1] = ex.OCC                   # ring of walls around the target (in a corner of
    c.map.state[i0 + 1:i1, j0 + 1:j1] = ex.FREE                  # rubble): no reachable cell has a clear line
    c.mode, c.target = "APPROACH", (3.0, 0.0)
    c._plan_to(0.0, 0.0, 3.0, 0.0)
    gx, gy = c.map.centre(c.goal[0], c.goal[1])
    assert not c._line_clear(gx, gy, 3.0, 0.0)
    state["pose"] = (gx, gy, 0.0)
    c.path = []                                                  # at the end of the planned path
    obs = obs_with(np.full(so.LIDAR_SECTORS, 30.0), est=True)
    b = -math.atan2(0.0 - gy, 3.0 - gx)                          # bearing +left at heading 0
    obs[so.SLICES["est_bearing_sin"]], obs[so.SLICES["est_bearing_cos"]] = math.sin(b), math.cos(b)
    obs[so.SLICES["est_range"]] = math.hypot(3.0 - gx, 0.0 - gy) / so.TARGET_RANGE_SCALE
    calls = inner.calls
    c.act(obs)
    assert c.target == pytest.approx((3.0, 0.0), abs=1e-3)
    assert inner.calls == calls + 1 and c.last["via"] == "inner"


def test_never_reads_the_target_position():
    import inspect
    src = inspect.getsource(ex)
    for forbidden in ("target_ned", "privileged", "distance_to_target", "load_target", "sar_task"):
        assert forbidden not in src, forbidden


def test_runner_explorer_option_in_the_standin(tmp_path):
    import json
    import run_target_reaching_episode as runner
    log = tmp_path / "e.jsonl"
    rc = runner.main(["--standin", "--explorer", "--policy", "models/e1r_300k_s2/final_policy_numpy.npz",
                      "--max-steps", "300", "--log", str(log)])
    recs = [json.loads(l) for l in log.read_text().splitlines()]
    assert any(r["event"] == "explorer" and r["inner"] == "policy" for r in recs)
    steps = [r for r in recs if r["event"] == "step"]
    modes = {s["policy"]["mode"] for s in steps}
    assert modes <= {"EXPLORE", "APPROACH"} and steps and rc in (0, 1)
    assert recs[-1]["event"] == "close_returned"


def test_existing_controllers_log_as_before(tmp_path):
    import json
    import run_target_reaching_episode as runner
    log = tmp_path / "e.jsonl"
    runner.main(["--standin", "--log", str(log)])                 # baseline: no per-step policy record
    assert all("policy" not in json.loads(l) for l in log.read_text().splitlines() if '"step"' in l)


def test_no_goal_recovers_then_returns_to_the_start_and_reports_search_complete(monkeypatch):
    """With nothing to plan, the explorer forgets its wall marks and failed viewpoints after NO_GOAL_STEPS (keeping the
    searched record), at most RECOVERIES times; then it flies back to the start and sets search_complete there,
    instead of turning in place until a guard stops it."""
    pose = {"p": (6.0, 0.0, 0.0)}
    c = ex.ExplorerController(Inner(), lambda: pose["p"])
    monkeypatch.setattr(c, "_plan", lambda *a: (setattr(c, "goal", None), (None, 0))[1])
    c.act(OPEN)
    searched = int(c.map.searched.sum())
    i, j = c.map.cell(10.0, 5.0)
    for k in range(1, ex.RECOVERIES + 1):
        c.map.state[i, j] = ex.OCC
        c.failed.add((1, 1))
        for _ in range(ex.NO_GOAL_STEPS):
            c.act(OPEN)
        assert c.recoveries == k and c.map.state[i, j] != ex.OCC and not c.failed and not c.returning
        assert int(c.map.searched.sum()) >= searched                  # the searched record is kept
    for _ in range(ex.NO_GOAL_STEPS):
        c.act(OPEN)
    assert c.returning and not c.search_complete
    a = c.act(OPEN)
    assert c.last["reason"] == "return_to_start" and abs(a[1]) > 0.5   # the start is behind it: turns first
    pose["p"] = (6.0, 0.0, math.pi)
    a = c.act(OPEN)
    assert c.last["reason"] == "return_to_start" and a[0] > 0.5       # facing the start: flies back
    pose["p"] = (0.3, 0.1, math.pi)
    a = c.act(OPEN)
    assert c.search_complete and c.last["reason"] == "search_complete" and np.allclose(a, 0.0)


def test_runner_ends_the_episode_on_search_complete(tmp_path):
    import json
    import run_target_reaching_episode as r
    from sar_standin_env import SarStandInEnv

    class Done:
        search_complete = False
        last = {"mode": "EXPLORE"}
        n = 0

        def act(self, obs):
            self.n += 1
            self.search_complete = self.n >= 5
            return np.zeros(2, np.float32)
    log = tmp_path / "e.jsonl"
    with open(log, "w") as f:
        result = r.run(SarStandInEnv(max_steps=100), f, 100, controller=Done())
    assert result == "search_complete"
    rows = [json.loads(line) for line in open(log)]
    assert sum(row["event"] == "step" for row in rows) == 5 and rows[-1]["reason"] == "search_complete"
