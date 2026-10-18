"""Synthetic OAK-D Lite frames for offline perception tests (ground truth is known here by construction).

This renders a flat floor plus the red target sphere by ray casting through camera intrinsics, using the
UAV camera mount from target_perception. World frame is Gazebo ENU; the UAV pose is its model-root
position plus ENU yaw/pitch/roll.
"""

from __future__ import annotations

import math

import numpy as np

import target_perception as tp

# OAK-D Lite intrinsics as published by Gazebo camera_info (hfov 1.204 / 1.274 rad).
RGB_SIZE = (1920, 1080)
DEPTH_SIZE = (640, 480)


def intrinsics(width, height, hfov):
    f = (width / 2) / math.tan(hfov / 2)
    return np.array([[f, 0, width / 2], [0, f, height / 2], [0, 0, 1.0]])


K_RGB = intrinsics(*RGB_SIZE, 1.204)
K_DEPTH = intrinsics(*DEPTH_SIZE, 1.274)


def body_rotation(yaw, pitch=0.0, roll=0.0):
    """Body FLU -> world ENU (Z-Y-X); positive pitch is nose-down as in SDF poses."""
    cy, sy, cp, sp, cr, sr = (math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch),
                              math.cos(roll), math.sin(roll))
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def camera_pose(uav_pos, yaw, pitch=0.0, roll=0.0):
    R_body = body_rotation(yaw, pitch, roll)
    return np.asarray(uav_pos, float) + R_body @ tp.CAMERA_IN_BODY, R_body @ tp.CAMERA_TO_BODY_R


def truth_in_body(uav_pos, yaw, target, pitch=0.0, roll=0.0):
    """Ground-truth target centre relative to the UAV body: bearing (rad, +left) and horizontal range."""
    v = body_rotation(yaw, pitch, roll).T @ (np.asarray(target, float) - np.asarray(uav_pos, float))
    return math.atan2(v[1], v[0]), math.hypot(v[0], v[1])


def _cast(origin, dirs, target, radius):
    """Distances along unit rays to the floor and the sphere (inf for no hit) and the sphere normals."""
    t_floor = np.where(dirs[:, 2] < -1e-9, -origin[2] / np.minimum(dirs[:, 2], -1e-9), np.inf)
    t_sphere = np.full(len(dirs), np.inf)
    if target is not None:
        oc = origin - target
        b = dirs @ oc
        disc = b * b - (oc @ oc - radius * radius)
        t = -b - np.sqrt(np.maximum(disc, 0.0))
        t_sphere = np.where((disc >= 0) & (t > 1e-6), t, np.inf)
    return t_floor, t_sphere


def render(uav_pos, yaw, target=(3.0, -3.0, 0.055), radius=0.5, pitch=0.0, roll=0.0,
           rgb_size=RGB_SIZE, depth_size=DEPTH_SIZE, K_rgb=None, K_depth=None):
    """Return (rgb uint8 HxWx3, planar depth float32 hxw). ``target=None`` renders an empty floor."""
    K_rgb = K_RGB if K_rgb is None else K_rgb
    K_depth = K_DEPTH if K_depth is None else K_depth
    cam, R = camera_pose(uav_pos, yaw, pitch, roll)
    tgt = None if target is None else np.asarray(target, float)
    light = np.array([0.3, 0.2, 0.93]) / np.linalg.norm([0.3, 0.2, 0.93])

    def rays(size, K):
        u, v = np.meshgrid(np.arange(size[0]) + 0.5, np.arange(size[1]) + 0.5)
        r_cam = tp.pixel_rays(u.ravel(), v.ravel(), K)          # x = 1 (planar depth = t_cam_x)
        r_world = r_cam @ R.T
        norm = np.linalg.norm(r_world, axis=1)
        return r_world / norm[:, None], norm

    dirs, _ = rays(rgb_size, K_rgb)
    t_floor, t_sphere = _cast(cam, dirs, tgt, radius)
    rgb = np.empty((len(dirs), 3), np.float32)
    rgb[:] = (85, 85, 85)                                       # background
    floor = np.isfinite(t_floor) & (t_floor < t_sphere)
    rgb[floor] = (190, 190, 190)
    sphere = np.isfinite(t_sphere) & (t_sphere <= t_floor)
    if sphere.any():
        n = cam + dirs[sphere] * t_sphere[sphere, None] - tgt
        n /= np.linalg.norm(n, axis=1, keepdims=True)
        lam = np.clip(n @ light, 0.0, 1.0)
        rgb[sphere] = np.stack([60 + 180 * lam, 8 * lam, 8 * lam], 1)
    rgb = rgb.reshape(rgb_size[1], rgb_size[0], 3).astype(np.uint8)

    dirs_d, norm_d = rays(depth_size, K_depth)
    tf, ts = _cast(cam, dirs_d, tgt, radius)
    t = np.minimum(tf, ts)
    depth = (t / norm_d).astype(np.float32)                     # planar: distance along the optical axis
    depth[~np.isfinite(t) | (depth > tp.DEPTH_MAX)] = np.inf
    return rgb, depth.reshape(depth_size[1], depth_size[0])
