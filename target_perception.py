"""Sensor-only detection of the red SAR target from OAK-D Lite RGB + depth.

This module is ROS-free and simulator-free. Its inputs are an RGB image, a depth image, the two cameras'
intrinsic matrices (from camera_info), and the fixed camera mount from the versioned UAV model
(sim/models/x500_lidar_2d_oakd). It never reads Gazebo poses, target coordinates, or other ground truth.

Frames:
- Camera frame: camera_link optical convention used by Gazebo, +x along the optical axis, +y left, +z up.
  Image pixels have +u right and +v down.
- Body frame: UAV FLU (+x forward, +y left, +z up), origin at the model root. Bearing is positive to the
  left, so body-frame bearing b corresponds to NED heading offset -b.

The detector is a prototype for the known red marker sphere. It is not a victim detector.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy import ndimage, optimize

# Camera mount from sim/models/x500_lidar_2d_oakd/model.sdf plus the OakD-Lite sensor offset.
MOUNT_PITCH = math.radians(30.0)                      # nose-down
MOUNT_POSITION = np.array([0.17, 0.03, 0.242])        # OakD-Lite model frame in body
SENSOR_OFFSET = np.array([0.01233, -0.03, 0.01878])   # optical centre in the OakD-Lite frame
DEPTH_MIN, DEPTH_MAX = 0.2, 19.1                      # StereoOV7251 clip planes (m)

TARGET_RADIUS = 0.5          # known-marker prior used by the sphere fit
MIN_BLOB_PX_FULLRES = 150    # minimum red blob area, in pixels of a REFERENCE_SIZE image (scaled to the actual size)
REFERENCE_SIZE = (1920, 1080)  # the RGB resolution the threshold was set for (the original IMX214 model)
MIN_DEPTH_SAMPLES = 20
FIT_MAX_RMS = 0.05           # m; worse fits fall back to the surface estimate

STATUS_DETECTED = 0
STATUS_NO_TARGET = 1
STATUS_DEPTH_INVALID = 2
STATUS_INPUT_INVALID = 3


def _rot_pitch(pitch):
    c, s = math.cos(pitch), math.sin(pitch)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


CAMERA_TO_BODY_R = _rot_pitch(MOUNT_PITCH)
CAMERA_IN_BODY = MOUNT_POSITION + CAMERA_TO_BODY_R @ SENSOR_OFFSET


@dataclass(frozen=True)
class Detection:
    detected: bool
    status: int
    range_valid: bool = False
    bearing: float = math.nan            # rad, body frame, +left
    elevation: float = math.nan          # rad, body frame, +up
    horizontal_range: float = math.nan   # m, body origin to target centre
    slant_range: float = math.nan        # m, camera to target centre
    surface_range: float = math.nan      # m, camera to nearest visible surface (5th percentile)
    confidence: float = 0.0
    blob_px: int = 0
    touches_edge: bool = False
    bearing_source: str = ""             # "fit" or "centroid"


def red_mask(rgb):
    r = rgb[..., 0].astype(np.int16)
    g = rgb[..., 1].astype(np.int16)
    b = rgb[..., 2].astype(np.int16)
    return (r > 40) & (r > 2 * g) & (r > 2 * b)


def hivis_mask(rgb):
    """High-visibility yellow-green (a worker's or survivor's hi-vis jacket). Measured in rendered Gazebo frames: the
    jacket has red/green 0.92-0.99 (tunnel lamps to sunlight) and blue/green 0.3-0.46; yellow (supply cases, caution
    tape, warning signs) has red/green >= 1.14, foliage <= 0.75, sand and concrete blue/green >= 0.8."""
    r = rgb[..., 0].astype(np.int16)
    g = rgb[..., 1].astype(np.int16)
    b = rgb[..., 2].astype(np.int16)
    return (g >= 60) & (20 * r >= 17 * g) & (20 * r <= 21 * g) & (2 * b <= g)


def target_mask(rgb):
    """Pixels of either target marker: the red sphere or a hi-vis jacket."""
    return red_mask(rgb) | hivis_mask(rgb)


def pixel_rays(u, v, K):
    """Camera-frame ray directions with x = 1 for pixel coordinates (u, v)."""
    u = np.asarray(u, float)
    v = np.asarray(v, float)
    return np.stack(
        [np.ones_like(u), -(u - K[0, 2]) / K[0, 0], -(v - K[1, 2]) / K[1, 1]], -1
    )


def camera_to_body(points):
    return points @ CAMERA_TO_BODY_R.T + CAMERA_IN_BODY


def _valid_matrix(K, width, height):
    K = np.asarray(K, float)
    return (
        K.shape == (3, 3)
        and np.all(np.isfinite(K))
        and K[0, 0] > 0
        and K[1, 1] > 0
        and 0 <= K[0, 2] <= width
        and 0 <= K[1, 2] <= height
    )


def detect(rgb, depth, K_rgb, K_depth, stride=None):
    """Detect the red sphere in one RGB/depth pair.

    ``depth`` may be None when no synchronised depth frame exists; the bearing is then reported from the
    blob centroid with ``range_valid`` False. ``stride`` subsamples the RGB mask for speed (default: 2 for images
    at least 1600 px wide, else 1, so the mask grid is about 960 px wide either way); pixel coordinates are reported
    at full resolution. The minimum blob area scales with the image area, so the same target is detected at the same
    distance at 1920x1080 and 960x540.
    """
    rgb = np.asarray(rgb)
    if rgb.ndim != 3 or rgb.shape[2] != 3 or not _valid_matrix(K_rgb, rgb.shape[1], rgb.shape[0]):
        return Detection(False, STATUS_INPUT_INVALID)
    K_rgb = np.asarray(K_rgb, float)
    if stride is None:
        stride = 2 if rgb.shape[1] >= 1600 else 1
    min_blob_px = MIN_BLOB_PX_FULLRES * rgb.shape[0] * rgb.shape[1] / (REFERENCE_SIZE[0] * REFERENCE_SIZE[1])
    mask = target_mask(rgb[::stride, ::stride])
    labels, n = ndimage.label(mask)
    if n == 0:
        return Detection(False, STATUS_NO_TARGET)
    sizes = ndimage.sum(mask, labels, range(1, n + 1))
    k = int(np.argmax(sizes)) + 1
    blob_px = int(sizes[k - 1]) * stride * stride
    if blob_px < min_blob_px:
        return Detection(False, STATUS_NO_TARGET, blob_px=blob_px)
    vs, us = np.nonzero(labels == k)
    us = us * stride + (stride - 1) / 2.0
    vs = vs * stride + (stride - 1) / 2.0
    h, w = rgb.shape[:2]
    touches_edge = bool(
        us.min() <= stride or vs.min() <= stride or us.max() >= w - 1 - stride or vs.max() >= h - 1 - stride
    )

    ray = pixel_rays(us.mean(), vs.mean(), K_rgb)
    centroid_dir = CAMERA_TO_BODY_R @ ray
    centroid_bearing = math.atan2(centroid_dir[1], centroid_dir[0])
    centroid_elevation = math.atan2(centroid_dir[2], math.hypot(centroid_dir[0], centroid_dir[1]))
    partial = Detection(
        True, STATUS_DEPTH_INVALID, bearing=centroid_bearing, elevation=centroid_elevation,
        confidence=0.2 if touches_edge else 0.5, blob_px=blob_px, touches_edge=touches_edge,
        bearing_source="centroid",
    )
    if depth is None:
        return partial
    depth = np.asarray(depth, float)
    if depth.ndim != 2 or not _valid_matrix(K_depth, depth.shape[1], depth.shape[0]):
        return partial
    K_depth = np.asarray(K_depth, float)

    # Same optical centre (SDF sensor poses are identical): map RGB rays into the depth image.
    step = max(1, int(math.sqrt(len(us) / 3000)))
    rays = pixel_rays(us[::step], vs[::step], K_rgb)
    ud = np.rint(K_depth[0, 2] - K_depth[0, 0] * rays[:, 1]).astype(int)
    vd = np.rint(K_depth[1, 2] - K_depth[1, 1] * rays[:, 2]).astype(int)
    inside = (ud >= 0) & (ud < depth.shape[1]) & (vd >= 0) & (vd < depth.shape[0])
    d = np.full(len(ud), np.nan)
    d[inside] = depth[vd[inside], ud[inside]]
    valid = np.isfinite(d) & (d >= DEPTH_MIN) & (d <= DEPTH_MAX)
    if valid.sum() < MIN_DEPTH_SAMPLES:
        return partial
    pts = rays[valid] * d[valid, None]          # planar depth: x component equals the depth value
    ranges = np.linalg.norm(pts, axis=1)
    surface = float(np.percentile(ranges, 5))

    mean_dir = pts.mean(0) / np.linalg.norm(pts.mean(0))
    centre = pts.mean(0) + TARGET_RADIUS * mean_dir
    fit = optimize.least_squares(
        lambda c: np.linalg.norm(pts - c, axis=1) - TARGET_RADIUS, centre, max_nfev=50
    )
    rms = float(np.sqrt(np.mean(fit.fun ** 2)))
    if fit.success and np.all(np.isfinite(fit.x)) and rms <= FIT_MAX_RMS:
        centre, source = fit.x, "fit"
        confidence = 0.8 if touches_edge else 1.0
    else:
        # Fall back to the nearest surface along the centroid ray plus the known radius.
        centre = ray / np.linalg.norm(ray) * (surface + TARGET_RADIUS)
        source = "centroid"
        confidence = 0.3 if touches_edge else 0.6
    centre_body = camera_to_body(centre)
    return Detection(
        True, STATUS_DETECTED, range_valid=True,
        bearing=math.atan2(centre_body[1], centre_body[0]),
        elevation=math.atan2(centre_body[2], math.hypot(centre_body[0], centre_body[1])),
        horizontal_range=math.hypot(centre_body[0], centre_body[1]),
        slant_range=float(np.linalg.norm(centre)), surface_range=surface,
        confidence=confidence, blob_px=blob_px, touches_edge=touches_edge, bearing_source=source,
    )


# ---- wire format shared with the ROS node and the environment (std_msgs/Float64MultiArray) ----
MESSAGE_VERSION = 1.0
MESSAGE_FIELDS = (
    "version", "image_stamp", "depth_stamp", "detected", "range_valid", "bearing", "elevation",
    "horizontal_range", "slant_range", "confidence", "blob_px", "touches_edge", "processing_ms", "status",
)


def encode(detection, image_stamp, depth_stamp, processing_ms):
    """Flat float list in MESSAGE_FIELDS order. Stamps are sensor (simulation) seconds; NaN means unknown."""
    d = detection
    return [
        MESSAGE_VERSION, float(image_stamp), float(depth_stamp), float(d.detected), float(d.range_valid),
        d.bearing, d.elevation, d.horizontal_range, d.slant_range, d.confidence, float(d.blob_px),
        float(d.touches_edge), float(processing_ms), float(d.status),
    ]


def decode(values):
    """Return a dict for a valid message, or None if it is malformed or from another version."""
    values = [float(v) for v in values]
    if len(values) != len(MESSAGE_FIELDS) or values[0] != MESSAGE_VERSION:
        return None
    msg = dict(zip(MESSAGE_FIELDS, values))
    if not math.isfinite(msg["image_stamp"]) or msg["detected"] not in (0.0, 1.0):
        return None
    msg["detected"] = msg["detected"] == 1.0
    msg["range_valid"] = msg["detected"] and msg["range_valid"] == 1.0 and math.isfinite(
        msg["horizontal_range"]) and msg["horizontal_range"] > 0
    if msg["detected"] and not math.isfinite(msg["bearing"]):
        return None
    return msg
