from pathlib import Path
import json
import subprocess

import pytest

from tools import isolated_sitl


def test_px4_uses_new_working_directory_and_existing_startup_data():
    run_dir = Path("/tmp/sar-isolated-run")
    command = isolated_sitl.px4_command(run_dir)

    assert command == [
        str(isolated_sitl.PX4_BINARY),
        "-i",
        str(isolated_sitl.PX4_INSTANCE),
        "-d",
        "-w",
        str(run_dir / "px4_rootfs"),
        str(isolated_sitl.PX4_DATA),
    ]
    assert command[2] != str(isolated_sitl.PX4_BUILD / "rootfs")


def test_px4_and_ros_share_isolated_domain_but_keep_agent_transport_separate():
    px4 = isolated_sitl.px4_environment({"PATH": "/usr/bin"})
    ros = isolated_sitl.ros_environment({"PATH": "/usr/bin"})

    assert px4["ROS_DOMAIN_ID"] == ros["ROS_DOMAIN_ID"] == str(isolated_sitl.DOMAIN_ID)
    assert px4["PX4_UXRCE_DDS_PORT"] == str(isolated_sitl.XRCE_PORT)
    assert px4["PX4_UXRCE_DDS_NS"] == ""
    assert px4["GZ_PARTITION"] == ros["GZ_PARTITION"]
    assert px4["PX4_GZ_WORLD"] == "walls"
    assert px4["PX4_SIM_MODEL"] == "gz_x500_lidar_2d"
    assert ros["SAR_UAV_MODEL_NAME"] == isolated_sitl.MODEL_INSTANCE
    assert ros["SAR_LIDAR_TOPIC"] == isolated_sitl.BRIDGE_TOPIC.split("@")[0]


def _owned_record(rootfs: str) -> dict:
    return {
        "pid": 10,
        "pgid": 10,
        "start_time": 200,
        "command": ["/px4", "-w", rootfs],
        "owned_command": [
            "python", "/tools/owned_process_group.py", "--child-record",
            "/tmp/px4.child.json", "--", "/px4", "-w", rootfs,
        ],
    }


def test_manifest_verification_refuses_mismatched_process_identity(monkeypatch):
    record = _owned_record("/isolated/rootfs")
    monkeypatch.setattr(isolated_sitl, "_proc_start_time", lambda _pid: 201)
    monkeypatch.setattr(
        isolated_sitl,
        "_cmdline",
        lambda _pid: record["owned_command"],
    )
    assert not isolated_sitl._verified_record(record)


def test_manifest_verification_requires_exact_owned_group_and_command(monkeypatch):
    record = _owned_record("/isolated/rootfs")
    monkeypatch.setattr(isolated_sitl, "_proc_start_time", lambda _pid: 200)
    actual = _owned_record("/other/rootfs")["owned_command"]
    monkeypatch.setattr(isolated_sitl, "_cmdline", lambda _pid: actual)
    assert not isolated_sitl._verified_record(record)


def test_manifest_verification_accepts_exact_session_leader(monkeypatch):
    record = _owned_record("/isolated/rootfs")
    monkeypatch.setattr(isolated_sitl, "_proc_start_time", lambda _pid: 200)
    monkeypatch.setattr(isolated_sitl, "_cmdline", lambda _pid: record["owned_command"])
    assert isolated_sitl._verified_record(record)


def test_shutdown_requires_fresh_safe_state_telemetry(tmp_path, monkeypatch):
    (tmp_path / isolated_sitl.MANIFEST).write_text(
        '{"ros_domain_id": 42, "px4_instance": 1, "components": {}}'
    )
    monkeypatch.setattr(
        isolated_sitl,
        "sourced_environment",
        lambda domain_id: {"ROS_DOMAIN_ID": str(domain_id)},
    )
    monkeypatch.setattr(
        isolated_sitl,
        "ros_environment",
        lambda env, domain_id: {**env, "ROS_DOMAIN_ID": str(domain_id)},
    )
    observed = {}

    def failed_probe(*args, **kwargs):
        observed.update(kwargs)
        observed["argv"] = args[0]
        observed["env_domain"] = kwargs["env"]["ROS_DOMAIN_ID"]
        return subprocess.CompletedProcess(args[0], 1, "armed", "")

    monkeypatch.setattr(
        isolated_sitl.subprocess,
        "run",
        failed_probe,
    )
    monkeypatch.setattr(
        isolated_sitl,
        "_signal_owned_groups",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("signalled")),
    )

    with pytest.raises(isolated_sitl.IsolationError, match="Refusing shutdown"):
        isolated_sitl.stop(tmp_path)
    assert observed["argv"][observed["argv"].index("--duration") + 1] == "15"
    assert observed["argv"][observed["argv"].index("--expected-system-id") + 1] == "2"
    assert observed["timeout"] == 20
    assert observed["env_domain"] == "42"


def test_failed_start_cleanup_only_accepts_proven_pre_server_refusal(tmp_path, monkeypatch):
    px4_log = tmp_path / "px4.log"
    px4_log.write_text("INFO [px4] PX4 server already running for instance 0\n")
    process = _owned_record("/isolated/rootfs")
    process.update(
        {
            "pid": 10,
            "pgid": 10,
            "child_pid": 20,
            "child_start_time": 300,
            "log": str(px4_log),
        }
    )
    (tmp_path / isolated_sitl.MANIFEST).write_text(
        json.dumps(
            {
                "components": {"px4_gazebo": process},
            }
        )
    )
    signalled = []
    monkeypatch.setattr(isolated_sitl, "_verified_record", lambda _record: True)
    monkeypatch.setattr(isolated_sitl, "_proc_start_time", lambda pid: 200 if pid == 10 else None)
    monkeypatch.setattr(isolated_sitl, "_live_group_members", lambda _pgid: {10})
    monkeypatch.setattr(
        isolated_sitl,
        "_signal_owned_groups",
        lambda manifest: signalled.append(manifest) or [],
    )

    isolated_sitl.cleanup_failed_start(tmp_path)

    assert len(signalled) == 1
    assert "failed_start_cleaned_at_utc" in isolated_sitl._load_manifest(tmp_path)


def test_failed_start_cleanup_refuses_live_px4_child(tmp_path, monkeypatch):
    px4_log = tmp_path / "px4.log"
    px4_log.write_text("INFO [px4] PX4 server already running for instance 0\n")
    process = _owned_record("/isolated/rootfs")
    process.update(
        {
            "pid": 10,
            "pgid": 10,
            "child_pid": 20,
            "child_start_time": 300,
            "log": str(px4_log),
        }
    )
    (tmp_path / isolated_sitl.MANIFEST).write_text(
        json.dumps({"components": {"px4_gazebo": process}})
    )
    monkeypatch.setattr(isolated_sitl, "_verified_record", lambda _record: True)
    monkeypatch.setattr(isolated_sitl, "_proc_start_time", lambda _pid: 300)
    monkeypatch.setattr(
        isolated_sitl,
        "_signal_owned_groups",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("signalled")),
    )

    with pytest.raises(isolated_sitl.IsolationError, match="PX4 child is still running"):
        isolated_sitl.cleanup_failed_start(tmp_path)


def test_legacy_profile_keeps_original_bridge_and_environment():
    legacy = isolated_sitl.PROFILES["legacy"]

    assert legacy.bridge_arguments() == [isolated_sitl.BRIDGE_TOPIC]
    assert legacy.model_instance == isolated_sitl.MODEL_INSTANCE
    assert "PX4_SYS_AUTOSTART" not in isolated_sitl.px4_environment({})
    assert isolated_sitl.gz_env_script(legacy) == isolated_sitl.PX4_GZ_ENV.read_bytes()


def test_sar_oakd_profile_uses_versioned_assets_and_lidar_airframe():
    profile = isolated_sitl.PROFILES["sar_oakd"]
    px4 = isolated_sitl.px4_environment({}, profile)
    ros = isolated_sitl.ros_environment({}, profile=profile)

    assert px4["PX4_GZ_WORLD"] == "sar_walls"
    assert px4["PX4_SIM_MODEL"] == "gz_x500_lidar_2d_oakd"
    assert px4["PX4_SYS_AUTOSTART"] == "4013"
    assert ros["SAR_UAV_MODEL_NAME"] == "x500_lidar_2d_oakd_1"
    assert ros["SAR_LIDAR_TOPIC"] == (
        "/world/sar_walls/model/x500_lidar_2d_oakd_1/link/link/sensor/lidar_2d_v2/scan"
    )
    script = isolated_sitl.gz_env_script(profile).decode()
    assert script.startswith(isolated_sitl.PX4_GZ_ENV.read_text())
    assert f"export PX4_GZ_MODELS={isolated_sitl.SIM_MODELS}\n" in script
    assert f"export PX4_GZ_WORLDS={isolated_sitl.SIM_WORLDS}\n" in script
    assert (isolated_sitl.SIM_WORLDS / "sar_walls.sdf").is_file()
    assert (isolated_sitl.SIM_MODELS / "x500_lidar_2d_oakd/model.sdf").is_file()


def test_sar_oakd_bridge_remaps_global_camera_topics_to_project_names():
    args = isolated_sitl.PROFILES["sar_oakd"].bridge_arguments()
    rgb = "/world/sar_walls/model/x500_lidar_2d_oakd_1/link/camera_link/sensor/IMX214"

    assert args[0].startswith(isolated_sitl.PROFILES["sar_oakd"].lidar_topic + "@")
    assert f"{rgb}/image@sensor_msgs/msg/Image[gz.msgs.Image" in args
    assert "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image" in args
    remaps = args[args.index("--ros-args") + 1:]
    assert remaps[0::2] == ["-r"] * 4
    assert set(remaps[1::2]) == {
        f"{rgb}/image:={isolated_sitl.CAMERA_RGB_TOPIC}",
        f"{rgb}/camera_info:={isolated_sitl.CAMERA_RGB_INFO_TOPIC}",
        f"/depth_camera:={isolated_sitl.CAMERA_DEPTH_TOPIC}",
        f"/camera_info:={isolated_sitl.CAMERA_DEPTH_INFO_TOPIC}",
    }


def test_project_profile_refuses_start_without_versioned_assets(tmp_path, monkeypatch):
    monkeypatch.setattr(isolated_sitl, "SIM_WORLDS", tmp_path / "missing")

    with pytest.raises(isolated_sitl.IsolationError, match="sar_walls.sdf"):
        isolated_sitl._check_paths(tmp_path / "run", isolated_sitl.PROFILES["sar_oakd"])
