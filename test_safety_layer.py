import math

import numpy as np
import pytest

import baseline_controller as bc
import sar_observation as so
import safety_layer as sl
from sar_standin_env import SarStandInEnv


def obs_with(ranges_m):
    obs = np.zeros(so.DIM, np.float32)
    obs[so.SLICES["lidar"]] = np.clip(np.asarray(ranges_m, np.float32) / so.LIDAR_RANGE_MAX, 0, 1)
    return obs


def ranges(default=30.0, **sectors):
    r = np.full(so.LIDAR_SECTORS, default, np.float32)
    for idx, val in sectors.items():
        r[int(idx[1:])] = val
    return r


F = sl.LidarSafetyFilter()
FRONT_IDX = np.flatnonzero(bc.FRONT)


def test_thresholds_are_the_baselines():
    base = bc.BaselineController()
    assert (sl.STOP_CLEARANCE, sl.SLOW_CLEARANCE) == (base.stop_clearance, base.slow_clearance) == (1.2, 3.0)


def test_clear_path_passes_the_action_unchanged():
    safe, info = F.filter(obs_with(ranges()), [0.7, -0.3], lidar_age_s=0.0)
    np.testing.assert_allclose(safe, [0.7, -0.3])
    assert info["reason"] is None and not info["intervened"] and info["original"] == pytest.approx([0.7, -0.3])


def test_slow_zone_caps_speed_but_keeps_turn():
    r = ranges(); r[FRONT_IDX[len(FRONT_IDX) // 2]] = 2.1           # straight ahead
    safe, info = F.filter(obs_with(r), [1.0, 0.4], lidar_age_s=0.0)
    assert info["reason"] == "slow" and info["intervened"]
    assert safe[0] == pytest.approx(0.8 * (2.1 - 1.2) / 1.8, abs=1e-5) and safe[1] == pytest.approx(0.4)
    slow_ok, info = F.filter(obs_with(r), [0.1, 0.4], lidar_age_s=0.0)  # already slower: untouched
    assert info["reason"] is None and slow_ok[0] == pytest.approx(0.1)


@pytest.mark.parametrize("clearer, expected_turn", [("left", -0.8), ("right", 0.8)])
def test_blocked_front_stops_and_turns_toward_the_clearer_side(clearer, expected_turn):
    r = ranges(default=5.0)
    r[FRONT_IDX] = 1.0
    side = sl.SECTOR_ANGLES > 0 if clearer == "left" else sl.SECTOR_ANGLES < 0
    r[side & ~bc.FRONT] = 20.0
    safe, info = F.filter(obs_with(r), [0.9, 0.0], lidar_age_s=0.0)
    assert info["reason"] == "stop" and safe[0] == 0.0 and safe[1] == pytest.approx(expected_turn)


def test_emergency_close_obstacle_anywhere_stops_forward_motion():
    r = ranges(); r[2] = 0.4                                          # rear-right, not in the front cone
    safe, info = F.filter(obs_with(r), [1.0, 0.2], lidar_age_s=0.0)
    assert info["reason"] == "emergency" and safe[0] == 0.0 and abs(safe[1]) == pytest.approx(0.8)
    assert info["min_clear_m"] == pytest.approx(0.4, abs=1e-5)


@pytest.mark.parametrize("bad", ["nan", "inf", "negative", "above_one", "short", "missing"])
def test_invalid_lidar_hovers_and_is_never_treated_as_clear(bad):
    obs = obs_with(ranges())
    if bad == "nan":
        obs[so.SLICES["lidar"].start + 5] = np.nan
    elif bad == "inf":
        obs[so.SLICES["lidar"].start + 5] = np.inf
    elif bad == "negative":
        obs[so.SLICES["lidar"].start + 5] = -0.1
    elif bad == "above_one":
        obs[so.SLICES["lidar"].start + 5] = 1.5
    elif bad == "short":
        obs = obs[:-3]
    else:
        obs = np.zeros(0, np.float32)
    safe, info = F.filter(obs, [1.0, 0.5], lidar_age_s=0.0)
    assert info["reason"] == "lidar_invalid" and safe.tolist() == [0.0, 0.0]


@pytest.mark.parametrize("age", [None, 0.51, 5.0, math.inf, math.nan])
def test_stale_or_unknown_lidar_age_hovers(age):
    safe, info = F.filter(obs_with(ranges()), [1.0, 0.5], lidar_age_s=age)
    assert info["reason"] == "lidar_stale" and safe.tolist() == [0.0, 0.0]


def test_filter_never_increases_speed():
    rng = np.random.default_rng(0)
    for _ in range(500):
        r = rng.uniform(0.1, 30.0, so.LIDAR_SECTORS)
        a = rng.uniform([0, -1], [1, 1])
        safe, _ = F.filter(obs_with(r), a, lidar_age_s=0.0)
        assert 0.0 <= safe[0] <= a[0] + 1e-6 and -1.0 <= safe[1] <= 1.0


def test_invalid_threshold_order_is_rejected():
    with pytest.raises(ValueError):
        sl.LidarSafetyFilter(emergency_clearance=1.5)


def test_filtered_reckless_policy_does_not_hit_a_wall_in_the_standin():
    """Full speed straight at the north wall: unfiltered collides, filtered never does."""
    def run(filtered):
        env = SarStandInEnv(perception_noise=False)
        obs, _ = env.reset(seed=0)
        env.x, env.y, env.heading = 2.0, -3.0, 0.0                    # facing the north wall at x = 4.5
        obs = env._observe(env._lidar())
        for _ in range(200):
            a = np.array([1.0, 0.0], np.float32)
            if filtered:
                a, _ = F.filter(obs, a, lidar_age_s=0.0)
            obs, _, term, trunc, info = env.step(a)
            if term or trunc:
                return info["reason"]
        return None
    assert run(False) == "collision"
    assert run(True) != "collision"
