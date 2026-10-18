# Versioned simulation assets

These files replace the uncommitted target edit in the PX4 gz-models submodule. They are used by
`tools/isolated_sitl.py start --profile sar_oakd`; the default `legacy` profile still uses PX4's
`walls` world and stock `x500_lidar_2d`.

| File | Content |
|---|---|
| `worlds/sar_walls.sdf` | Copy of PX4-gazebo-models `worlds/walls.sdf` (bb0b9cf), renamed `sar_walls`, plus the static red `target` sphere: radius 0.5 m, centre ENU (3, -3, 0.055). This pose is the single source of simulator ground truth (reward and evaluation only, never the policy). |
| `models/x500_lidar_2d_oakd/` | PX4 `x500`, plus the unmodified `lidar_2d_v2` at the stock x500_lidar_2d mount (.12, 0, .26), plus the unmodified `OakD-Lite` at (.17, .03, .242) pitched 30° nose-down. |

## How PX4 finds them

- The launcher writes the run-local `px4_rootfs/gz_env.sh` (PX4's generated file plus overrides).
- The overrides point `PX4_GZ_MODELS` and `PX4_GZ_WORLDS` at this directory and prepend it to `GZ_SIM_RESOURCE_PATH`; the `x500`, `lidar_2d_v2` and `OakD-Lite` includes still resolve from PX4's model library.
- PX4 starts with `PX4_SYS_AUTOSTART=4013`, because the x500_lidar_2d airframe only aliases the x500 airframe.

## Sensors

| Sensor | Type | Resolution / samples | Specified rate | Other |
|---|---|---|---|---|
| RGB `IMX214` | camera | 1920x1080 | 30 Hz | hfov 1.204 rad, no noise or distortion |
| Depth `StereoOV7251` | depth_camera | 640x480 R_FLOAT32 | 30 Hz | hfov 1.274 rad, 0.2–19.1 m, planar depth (distance along the optical axis) |
| LiDAR `lidar_2d_v2` | gpu_lidar | 1080 samples over ±135°, horizontal | 30 Hz | 0.1–30 m |

- The bridge remaps the camera topics to `/uav/camera/{rgb,depth}/{image,camera_info}`.
- Under this WSL host's software rendering (Mesa llvmpipe), the stand experiment measured RGB at about 8 Hz sim and depth at about 28 Hz sim. See `/home/hp/uav_ros_experiments/oakd_sensor_rig_20261018/RESULTS.md`.
- Camera rendering competes with the PX4 simulation for CPU.
