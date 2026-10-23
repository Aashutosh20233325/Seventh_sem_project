"""Stand-in camera occlusion: line-of-sight geometry, detection gating, opt-in default, and no leakage."""
import dataclasses
import math

import numpy as np
import pytest

import sar_observation as so
import sar_standin_env as se
from sar_standin_env import SarStandInEnv


# ---------- segment / box geometry ----------

@pytest.mark.parametrize("seg, hit", [
    ((0, 0, 4, 0), True),          # straight through
    ((0, 0, 1.5, 0), False),       # stops short of the box
    ((0, 0, 2, 0), True),          # ends exactly on the face (conservative: touching counts)
    ((0, 2, 4, 2), True),          # grazes the top edge
    ((0, 2.01, 4, 2.01), False),   # just misses
    ((2.5, -5, 2.5, 5), True),     # vertical through
    ((5, -5, 5, 5), False),        # vertical, outside
    ((2.5, 0.5, 2.6, 0.6), True),  # entirely inside
    ((0, -3, 4, 3), True),         # diagonal through
    ((0, 3, 1.9, 0), False),       # diagonal past the corner
])
def test_segment_box_intersection(seg, hit):
    assert se.segment_hits_box(*seg, (2.0, 3.0, -2.0, 2.0)) is hit


# ---------- occlusion in the environment ----------

WALL = (-1.6, -1.4, -2.0, 2.0)        # thin wall 1.5 m south of the origin, 4 m wide (NED x, y)


def env_facing_target(occlusion=True, target=(-3.0, 0.0), walls=(WALL,), x=0.0, y=0.0):
    env = SarStandInEnv(perception_noise=False, detection_prob=1.0, camera_occlusion=occlusion)
    env.reset(seed=0)
    env.walls = list(walls)
    env.target = dataclasses.replace(env.nominal_target, x_ned=target[0], y_ned=target[1])
    env.x, env.y = x, y
    env.heading = math.atan2(target[1] - y, target[0] - x)
    env.vx = env.vy = env.yaw_rate_ned = 0.0
    return env


def detects(env):
    return env._perceive()["detected"]


def test_clear_line_of_sight_is_detected():
    env = env_facing_target(walls=())
    assert env._camera_sees(*env._true_relative()) and not env._occluded() and detects(env)


def test_wall_between_camera_and_target_blocks_detection():
    env = env_facing_target()
    assert env._camera_sees(*env._true_relative())           # in the field of view...
    assert env._occluded() and not detects(env)              # ...but behind the wall
    assert env.occluded_frames == 1


def test_target_visible_again_once_the_wall_no_longer_blocks():
    env = env_facing_target()
    assert not detects(env)
    env.x, env.y = -1.0, 3.5                                  # move around the wall's end
    env.heading = math.atan2(0.0 - 3.5, -3.0 - (-1.0))
    assert env._camera_sees(*env._true_relative()) and not env._occluded() and detects(env)


def test_partially_hidden_target_is_still_visible():
    """Sweeping the target past the wall's end (y = 2.0) passes through a band where the centre sightline is
    blocked but an edge sightline (target radius 0.5) is clear: the target must be detectable there, and fully
    hidden before it."""
    states = []
    for y in np.arange(0.0, 6.0, 0.02):
        env = env_facing_target(target=(-3.0, float(y)))
        cx, cy = env._camera_xy()
        centre_blocked = se.segment_hits_box(cx, cy, -3.0, float(y), WALL)
        states.append((y, centre_blocked, env._occluded(), env))
    partial = [st for st in states if st[1] and not st[2]]
    assert partial, "no partially visible position found"
    assert detects(partial[0][3])
    assert states[0][2]                                         # y = 0: fully behind the wall
    assert not states[-1][1] and not states[-1][2]              # far past the end: fully clear


def test_fully_hidden_wide_wall_and_distant_wall_behind_target():
    assert env_facing_target(walls=((-1.6, -1.4, -10, 10),))._occluded()
    behind = env_facing_target(walls=((-4.6, -4.4, -10, 10),))      # wall beyond the target does not hide it
    assert not behind._occluded() and detects(behind)


def test_occlusion_is_opt_in_and_default_ignores_walls():
    env = env_facing_target(occlusion=False)
    assert detects(env) and env.occluded_frames == 0
    assert SarStandInEnv().camera_occlusion is False


def test_occlusion_off_reproduces_the_previous_environment_exactly():
    a, b = SarStandInEnv(randomize_start=True, randomize_target=True), SarStandInEnv(randomize_start=True, randomize_target=True)
    b.camera_occlusion = False
    rng = np.random.default_rng(3)
    oa, _ = a.reset(seed=5); ob, _ = b.reset(seed=5)
    for _ in range(300):
        act = rng.uniform([0, -1], [1, 1]).astype(np.float32)
        oa, ra, ta, tra, _ = a.step(act); ob, rb, tb, trb, _ = b.step(act)
        np.testing.assert_array_equal(oa, ob); assert ra == rb
        if ta or tra:
            break


def test_occluded_episode_is_deterministic_by_seed():
    def roll():
        env = SarStandInEnv(randomize_start=True, randomize_target=True, camera_occlusion=True)
        env.walls = env.walls + [(-2.4, -2.1, 0.5, 4.0)]
        rng = np.random.default_rng(7)
        obs, _ = env.reset(seed=11)
        out = [obs]
        for _ in range(200):
            obs, r, t, tr, _ = env.step(rng.uniform([0, -1], [1, 1]).astype(np.float32))
            out.append(obs)
            if t or tr:
                break
        return np.array(out), env.occluded_frames
    (o1, f1), (o2, f2) = roll(), roll()
    np.testing.assert_array_equal(o1, o2) and f1 == f2


def test_occlusion_adds_no_wall_or_target_information_to_the_observation():
    """The only effect of occlusion on the observation is through detection fields: with the camera blind,
    occlusion on/off and different targets give identical observations."""
    def blind(occ, target):
        env = SarStandInEnv(randomize_start=True, detection_prob=0.0, camera_occlusion=occ)
        env.walls = env.walls + [WALL]
        env.target = dataclasses.replace(env.nominal_target, x_ned=target[0], y_ned=target[1])
        obs, _ = env.reset(seed=4)
        rng = np.random.default_rng(2)
        out = [obs]
        for _ in range(100):
            obs, *_ = env.step(rng.uniform([0, -1], [1, 1]).astype(np.float32))
            out.append(obs)
        return np.array(out)
    ref = blind(False, (-3.0, 3.0))
    np.testing.assert_array_equal(ref, blind(True, (-3.0, 3.0)))
    np.testing.assert_array_equal(ref, blind(True, (2.0, -3.0)))
    assert so.DIM == 65


def test_occluded_target_never_appears_in_detection_fields():
    env = env_facing_target()
    obs = None
    for _ in range(10):                                       # hover facing the hidden target
        obs, *_ = env.step(np.array([0.0, 0.0], np.float32))
    assert obs[so.SLICES["det_now"]][0] == 0.0 and obs[so.SLICES["est_valid"]][0] == 0.0
