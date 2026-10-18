# Recorded Gazebo OAK-D Lite RGB frames (stationary stand)

These frames come from `/home/hp/uav_ros_experiments/oakd_sensor_rig_20261018/` (2026-10-18). They are real Gazebo renders of the unmodified IMX214 camera on a static stand at about 1.84 m, pitched 30° down, with the walls-world red target. No depth was saved, so these fixtures exercise the RGB-only (centroid) detection path.

- The PNGs were downscaled from 1920x1080 to **960x540**; scale the intrinsics by 0.5 (fx = fy = 698.61, cx = 480, cy = 270).
- The ground-truth azimuth is measured from the camera optical centre in the stand frame (+left, deg) and was computed from the commanded fixture pose.

| File | Distance (m) | Bearing (deg) | GT azimuth (deg) | Full-res detector azimuth (deg) | Touches edge |
|---|---|---|---|---|---|
| stand_d4.31_b0.png | 4.31 | 0 | 0.401 | 0.428 | no |
| stand_d8_b0.png | 8 | 0 | 0.215 | 0.235 | no |
| stand_d2_b20_edge.png | 2 | 20 | 21.008 | 21.643 | yes |
