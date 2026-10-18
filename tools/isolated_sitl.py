#!/usr/bin/env python3
"""Own and isolate the SAR PX4 SITL stack from the shared desktop instance.

This script never searches for or signals processes by executable name. Each
component is started in its own session and recorded by PID, process start time,
and process group. Stop is bounded and requires a read-only landed/disarmed
confirmation.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shlex
import signal
import socket
import subprocess
import sys
import time
from datetime import datetime, timezone
from typing import Any


PROJECT = Path(__file__).resolve().parents[1]
PX4_SOURCE = Path("/home/hp/PX4-Autopilot")
PX4_BUILD = PX4_SOURCE / "build/px4_sitl_default"
PX4_BINARY = PX4_BUILD / "bin/px4"
PX4_DATA = PX4_BUILD / "etc"
PX4_GZ_ENV = PX4_BUILD / "rootfs/gz_env.sh"
PX4_INSTANCE = 1
XRCE_AGENT = Path("/home/hp/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent")
ROS_SETUP = Path("/opt/ros/jazzy/setup.bash")
ROS_OVERLAY = Path("/home/hp/px4_ros2_ws/install/setup.bash")
ROS2 = Path("/opt/ros/jazzy/bin/ros2")
PYTHON = PROJECT / ".venv/bin/python"
RUN_DIR = PROJECT / "runs/sar_demo_20261018/isolated_sitl_instance1"
MANIFEST = "processes.json"
DOMAIN_ID = 43
XRCE_PORT = 18890
GZ_PARTITION = "uav_sar_validation_20261018_instance1"
WORLD = "walls"
MODEL = "gz_x500_lidar_2d"
MODEL_INSTANCE = f"x500_lidar_2d_{PX4_INSTANCE}"
BRIDGE_TOPIC = (
    f"/world/{WORLD}/model/{MODEL_INSTANCE}/link/link/sensor/lidar_2d_v2/scan"
    "@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
)
STOP_GRACE_SECONDS = 10.0
TERM_GRACE_SECONDS = 5.0

# Project-versioned simulation assets (profile "sar_oakd").
SIM_MODELS = PROJECT / "sim/models"
SIM_WORLDS = PROJECT / "sim/worlds"
# ROS names for the camera; the gz depth topics are global ("/depth_camera",
# "/camera_info"), so the bridge remaps them onto unambiguous project names.
CAMERA_RGB_TOPIC = "/uav/camera/rgb/image"
CAMERA_RGB_INFO_TOPIC = "/uav/camera/rgb/camera_info"
CAMERA_DEPTH_TOPIC = "/uav/camera/depth/image"
CAMERA_DEPTH_INFO_TOPIC = "/uav/camera/depth/camera_info"


class Profile:
    """Simulation assets and ROS bridge configuration for one launch profile."""

    def __init__(
        self,
        name: str,
        world: str,
        sim_model: str,
        autostart: int | None,
        project_assets: bool,
        camera: bool,
    ) -> None:
        self.name = name
        self.world = world
        self.sim_model = sim_model
        self.autostart = autostart
        self.project_assets = project_assets
        self.camera = camera
        self.model_instance = f"{sim_model.removeprefix('gz_')}_{PX4_INSTANCE}"
        model_prefix = f"/world/{world}/model/{self.model_instance}"
        self.lidar_topic = f"{model_prefix}/link/link/sensor/lidar_2d_v2/scan"
        self.camera_prefix = f"{model_prefix}/link/camera_link/sensor"

    def bridge_arguments(self) -> list[str]:
        args = [f"{self.lidar_topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"]
        if not self.camera:
            return args
        rgb = f"{self.camera_prefix}/IMX214"
        args += [
            f"{rgb}/image@sensor_msgs/msg/Image[gz.msgs.Image",
            f"{rgb}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "--ros-args",
            "-r", f"{rgb}/image:={CAMERA_RGB_TOPIC}",
            "-r", f"{rgb}/camera_info:={CAMERA_RGB_INFO_TOPIC}",
            "-r", f"/depth_camera:={CAMERA_DEPTH_TOPIC}",
            "-r", f"/camera_info:={CAMERA_DEPTH_INFO_TOPIC}",
        ]
        return args


PROFILES = {
    # Original stack: PX4 submodule world and stock x500_lidar_2d model.
    "legacy": Profile("legacy", WORLD, MODEL, None, False, False),
    # Versioned project world/model: target sphere plus LiDAR and OAK-D Lite.
    # Airframe 4013 (x500_lidar_2d) only aliases the x500 airframe, so it is
    # reused explicitly for the combined model.
    "sar_oakd": Profile("sar_oakd", "sar_walls", "gz_x500_lidar_2d_oakd", 4013, True, True),
}
LEGACY = PROFILES["legacy"]


class IsolationError(RuntimeError):
    pass


def sourced_environment(domain_id: int = DOMAIN_ID) -> dict[str, str]:
    """Load the installed ROS env without printing it or changing this process."""
    source = (
        f"source {shlex.quote(str(ROS_SETUP))} && "
        f"source {shlex.quote(str(ROS_OVERLAY))} && "
        "env -0"
    )
    completed = subprocess.run(
        ["bash", "-c", source], check=True, capture_output=True, timeout=15
    )
    env = dict(os.environ)
    for item in completed.stdout.split(b"\0"):
        if item and b"=" in item:
            key, value = item.split(b"=", 1)
            env[key.decode()] = value.decode(errors="surrogateescape")
    env["ROS_DOMAIN_ID"] = str(domain_id)
    return env


def px4_command(run_dir: Path) -> list[str]:
    return [
        str(PX4_BINARY),
        "-i",
        str(PX4_INSTANCE),
        "-d",
        "-w",
        str(run_dir / "px4_rootfs"),
        str(PX4_DATA),
    ]


def px4_environment(base: dict[str, str], profile: Profile = LEGACY) -> dict[str, str]:
    env = dict(base)
    env.update(
        {
            "ROS_DOMAIN_ID": str(DOMAIN_ID),
            "PX4_UXRCE_DDS_PORT": str(XRCE_PORT),
            "PX4_GZ_WORLD": profile.world,
            "PX4_SIM_MODEL": profile.sim_model,
            # PX4 normally namespaces nonzero instances. The unique ROS
            # domain is the isolation boundary here; clear the namespace so
            # this project's existing /fmu/out subscriptions stay correct.
            "PX4_UXRCE_DDS_NS": "",
            "GZ_PARTITION": GZ_PARTITION,
        }
    )
    if profile.autostart is not None:
        # The combined model has no PX4 airframe file of its own.
        env["PX4_SYS_AUTOSTART"] = str(profile.autostart)
    return env


def ros_environment(
    base: dict[str, str], domain_id: int = DOMAIN_ID, profile: Profile = LEGACY
) -> dict[str, str]:
    env = dict(base)
    env.update(
        {
            "ROS_DOMAIN_ID": str(domain_id),
            "GZ_PARTITION": GZ_PARTITION,
            "SAR_UAV_MODEL_NAME": profile.model_instance,
            "SAR_LIDAR_TOPIC": profile.lidar_topic,
        }
    )
    return env


def gz_env_script(profile: Profile = LEGACY) -> bytes:
    """PX4's generated gz_env.sh, plus project asset paths for project profiles."""
    script = PX4_GZ_ENV.read_bytes()
    if not profile.project_assets:
        return script
    override = (
        "\n# --- uav_ros project assets (tools/isolated_sitl.py profile "
        f"{profile.name}) ---\n"
        f"export PX4_GZ_MODELS={shlex.quote(str(SIM_MODELS))}\n"
        f"export PX4_GZ_WORLDS={shlex.quote(str(SIM_WORLDS))}\n"
        f"export GZ_SIM_RESOURCE_PATH={shlex.quote(str(SIM_MODELS))}:"
        f"{shlex.quote(str(SIM_WORLDS))}:$GZ_SIM_RESOURCE_PATH\n"
    )
    return script + override.encode()


def _proc_start_time(pid: int) -> int | None:
    try:
        stat = Path(f"/proc/{pid}/stat").read_text()
        # The comm field can contain spaces/parentheses; fields after its final
        # ')' start at proc stat field 3. Start time is field 22.
        fields_after_comm = stat[stat.rfind(")") + 2 :].split()
        return int(fields_after_comm[19])
    except (OSError, ValueError, IndexError):
        return None


def _cmdline(pid: int) -> list[str] | None:
    try:
        return [
            item.decode(errors="replace")
            for item in Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
            if item
        ]
    except OSError:
        return None


def _group_exists(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def _live_group_members(pgid: int) -> set[int]:
    members: set[int] = set()
    for stat_path in Path("/proc").glob("[0-9]*/stat"):
        try:
            raw = stat_path.read_text()
            fields = raw[raw.rfind(")") + 2 :].split()
            state = fields[0]
            process_group = int(fields[2])
            if process_group == pgid and state not in {"Z", "X"}:
                members.add(int(stat_path.parent.name))
        except (OSError, ValueError, IndexError):
            continue
    return members


def _wait_group_exit(pgid: int, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not _group_exists(pgid):
            return True
        time.sleep(0.1)
    return not _group_exists(pgid)


def _load_manifest(run_dir: Path) -> dict[str, Any]:
    path = run_dir / MANIFEST
    try:
        return json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise IsolationError(f"Cannot read process ownership manifest {path}: {exc}") from exc


def _write_manifest(run_dir: Path, manifest: dict[str, Any]) -> None:
    destination = run_dir / MANIFEST
    temporary = destination.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(manifest, indent=2) + "\n")
    temporary.replace(destination)


def _assert_endpoint_free(port: int) -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        try:
            probe.bind(("0.0.0.0", port))
        except OSError as exc:
            raise IsolationError(f"XRCE UDP port {port} is unavailable: {exc}") from exc


def _assert_domain_and_partition_unused(env: dict[str, str]) -> None:
    # Domain discovery must be empty before launch. `ros2 node list` is a
    # read-only graph query; it creates no persistent ROS node or publisher.
    try:
        nodes = subprocess.run(
            [str(ROS2), "node", "list", "--no-daemon", "--spin-time", "1"],
            env=env,
            text=True,
            capture_output=True,
            timeout=5,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise IsolationError(f"Could not check ROS domain {DOMAIN_ID}: {exc}") from exc
    found = [line.strip() for line in nodes.stdout.splitlines() if line.strip()]
    if nodes.returncode != 0 or found:
        raise IsolationError(
            f"ROS domain {DOMAIN_ID} is not demonstrably empty: "
            f"returncode={nodes.returncode}, nodes={found}"
        )

    gz_env = dict(env, GZ_PARTITION=GZ_PARTITION)
    try:
        topics = subprocess.run(
            ["gz", "topic", "-l"],
            env=gz_env,
            text=True,
            capture_output=True,
            timeout=5,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise IsolationError(f"Could not check Gazebo partition: {exc}") from exc
    found_topics = [line.strip() for line in topics.stdout.splitlines() if line.strip()]
    if topics.returncode != 0 or found_topics:
        raise IsolationError(
            f"Gazebo partition {GZ_PARTITION!r} is not demonstrably empty: "
            f"returncode={topics.returncode}, topics={found_topics[:8]}"
        )


def _check_paths(run_dir: Path, profile: Profile = LEGACY) -> None:
    if not PX4_BINARY.is_file() or not os.access(PX4_BINARY, os.X_OK):
        raise IsolationError(f"PX4 SITL binary is unavailable: {PX4_BINARY}")
    required = [PX4_DATA, PX4_GZ_ENV, XRCE_AGENT, ROS_SETUP, ROS_OVERLAY, ROS2, PYTHON]
    if profile.project_assets:
        required += [
            SIM_WORLDS / f"{profile.world}.sdf",
            SIM_MODELS / profile.sim_model.removeprefix("gz_") / "model.sdf",
        ]
    for path in required:
        if not path.exists():
            raise IsolationError(f"Required isolated-stack dependency is missing: {path}")
    resolved = run_dir.resolve()
    shared_root = (PX4_BUILD / "rootfs").resolve()
    if resolved == shared_root or resolved in shared_root.parents or shared_root in resolved.parents:
        raise IsolationError("Run directory must be separate from the PX4 build/rootfs tree")
    if run_dir.exists() and any(run_dir.iterdir()):
        raise IsolationError(f"Run directory already contains data; refusing overwrite: {run_dir}")


def _prepare_run_dir(run_dir: Path, profile: Profile = LEGACY) -> None:
    run_dir.mkdir(parents=True, exist_ok=False)
    rootfs = run_dir / "px4_rootfs"
    (rootfs / "fs").mkdir(parents=True)
    # PX4's local gz startup script sources ./gz_env.sh from its working dir.
    # Copy the configuration, never link writable PX4 storage to the shared tree.
    (rootfs / "gz_env.sh").write_bytes(gz_env_script(profile))
    (run_dir / "logs").mkdir()


def _spawn(
    run_dir: Path,
    name: str,
    command: list[str],
    env: dict[str, str],
    cwd: Path,
) -> dict[str, Any]:
    log_path = run_dir / "logs" / f"{name}.log"
    child_record_path = run_dir / f"{name}.child.json"
    owned_command = [
        sys.executable,
        str(PROJECT / "tools/owned_process_group.py"),
        "--child-record",
        str(child_record_path),
        "--",
        *command,
    ]
    with log_path.open("ab", buffering=0) as log:
        process = subprocess.Popen(
            owned_command,
            cwd=cwd,
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            close_fds=True,
        )
    deadline = time.monotonic() + 3.0
    child_info = None
    while time.monotonic() < deadline:
        try:
            child_info = json.loads(child_record_path.read_text())
            break
        except (OSError, json.JSONDecodeError):
            if process.poll() is not None:
                break
            time.sleep(0.02)
    if child_info is None:
        raise IsolationError(f"{name} process guard failed to start; see {log_path}")
    # The guard is session leader. It remains alive if its direct client exits,
    # retaining an attributable process group for any simulator descendants.
    return {
        "pid": process.pid,
        "pgid": process.pid,
        "start_time": _proc_start_time(process.pid),
        "command": command,
        "owned_command": owned_command,
        "child_pid": child_info["pid"],
        "child_start_time": child_info["start_time"],
        "log": str(log_path),
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
    }


def _verified_record(record: dict[str, Any]) -> bool:
    pid = int(record["pid"])
    pgid = int(record["pgid"])
    if pid != pgid:
        return False
    if _proc_start_time(pid) != record.get("start_time"):
        return False
    actual = _cmdline(pid)
    if not actual:
        return False
    expected = [str(arg) for arg in record.get("owned_command", [])]
    # The manifest is the ownership source, but verify the executable and
    # arguments still match before signalling a group.
    return any(
        actual[index : index + len(expected)] == expected
        for index in range(len(actual) - len(expected) + 1)
    )


def start(run_dir: Path, profile: Profile = LEGACY) -> None:
    run_dir = run_dir.resolve()
    _check_paths(run_dir, profile)
    _assert_endpoint_free(XRCE_PORT)
    env_base = sourced_environment()
    env_ros = ros_environment(env_base, profile=profile)
    _assert_domain_and_partition_unused(env_ros)
    _prepare_run_dir(run_dir, profile)

    manifest: dict[str, Any] = {
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "project": str(PROJECT),
        "px4_source": str(PX4_SOURCE),
        "px4_instance": PX4_INSTANCE,
        "px4_working_directory": str(run_dir / "px4_rootfs"),
        "ros_domain_id": DOMAIN_ID,
        "xrce_udp_port": XRCE_PORT,
        "gazebo_partition": GZ_PARTITION,
        "profile": profile.name,
        "world": profile.world,
        "model": profile.sim_model,
        "components": {},
    }
    _write_manifest(run_dir, manifest)

    try:
        agent = _spawn(
            run_dir,
            "xrce_agent",
            [str(XRCE_AGENT), "udp4", "-p", str(XRCE_PORT)],
            env_ros,
            run_dir,
        )
        manifest["components"]["xrce_agent"] = agent
        _write_manifest(run_dir, manifest)
        time.sleep(0.5)
        if _proc_start_time(agent["child_pid"]) != agent["child_start_time"]:
            raise IsolationError("Isolated XRCE agent exited during startup; see its log")

        px4 = _spawn(
            run_dir,
            "px4_gazebo",
            px4_command(run_dir),
            px4_environment(env_base, profile),
            run_dir / "px4_rootfs",
        )
        manifest["components"]["px4_gazebo"] = px4
        _write_manifest(run_dir, manifest)

        # PX4 starts the Gazebo server and spawns the vehicle from rcS. Bound
        # startup before launching ROS consumers; never signal shared PIDs.
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            if _proc_start_time(px4["child_pid"]) != px4["child_start_time"]:
                raise IsolationError("Isolated PX4 exited during startup; see px4_gazebo.log")
            time.sleep(0.25)
            if (time.monotonic() + 0.0) >= deadline - 24.0:
                # at least six seconds after PX4 launch; check for its partition
                # clock before allowing consumers to start
                gz_env = dict(env_ros, GZ_PARTITION=GZ_PARTITION)
                probe = subprocess.run(
                    ["gz", "topic", "-l"], env=gz_env, capture_output=True, timeout=3
                )
                if f"/world/{profile.world}/clock".encode() in probe.stdout:
                    break
        else:
            raise IsolationError("Isolated Gazebo world did not publish its clock within 30s")

        bridge = _spawn(
            run_dir,
            "lidar_bridge",
            [str(ROS2), "run", "ros_gz_bridge", "parameter_bridge", *profile.bridge_arguments()],
            env_ros,
            run_dir,
        )
        manifest["components"]["lidar_bridge"] = bridge
        _write_manifest(run_dir, manifest)
        time.sleep(1.0)
        if _proc_start_time(bridge["child_pid"]) != bridge["child_start_time"]:
            raise IsolationError("Isolated LiDAR bridge exited during startup; see its log")

        publisher = _spawn(
            run_dir,
            "observation_publisher",
            [str(PYTHON), str(PROJECT / "observation_publisher.py")],
            env_ros,
            run_dir,
        )
        manifest["components"]["observation_publisher"] = publisher
        _write_manifest(run_dir, manifest)
        print(f"Isolated stack started. Ownership manifest: {run_dir / MANIFEST}")
        print(f"PX4/Gazebo log: {px4['log']}")
    except Exception as exc:
        raise IsolationError(
            f"Startup stopped after an error: {exc}. Any components listed in "
            f"{run_dir / MANIFEST} were deliberately left running for inspection; "
            "no process group was signalled."
        ) from exc


def status(run_dir: Path) -> None:
    manifest = _load_manifest(run_dir)
    print(json.dumps(manifest, indent=2))
    for name, record in manifest.get("components", {}).items():
        guard_alive = _proc_start_time(int(record["pid"])) == record.get("start_time")
        child_alive = (
            _proc_start_time(int(record["child_pid"])) == record.get("child_start_time")
        )
        print(
            f"{name}: guard={'alive' if guard_alive else 'exited'}, "
            f"client={'alive' if child_alive else 'exited'}; log={record['log']}"
        )


def _signal_owned_groups(
    manifest: dict[str, Any], initial_signal: int = signal.SIGINT
) -> list[str]:
    failures: list[str] = []
    components = manifest.get("components", {})
    for name, record in reversed(list(components.items())):
        if not _verified_record(record):
            failures.append(f"{name}: leader identity could not be verified; left untouched")
            continue
        pgid = int(record["pgid"])
        try:
            os.killpg(pgid, initial_signal)
        except ProcessLookupError:
            continue
        if _wait_group_exit(pgid, STOP_GRACE_SECONDS):
            continue
        try:
            os.killpg(pgid, signal.SIGTERM)
        except ProcessLookupError:
            continue
        if not _wait_group_exit(pgid, TERM_GRACE_SECONDS):
            failures.append(
                f"{name}: still running after bounded SIGINT/SIGTERM; no SIGKILL sent"
            )
    return failures


def stop(run_dir: Path) -> None:
    manifest = _load_manifest(run_dir)
    domain_id = manifest.get("ros_domain_id")
    if not isinstance(domain_id, int) or not 0 <= domain_id <= 232:
        raise IsolationError(
            "Refusing shutdown because the manifest has no valid ROS domain ID."
        )
    instance_id = manifest.get("px4_instance")
    if not isinstance(instance_id, int) or not 0 <= instance_id <= 253:
        raise IsolationError(
            "Refusing shutdown because the manifest has no valid PX4 instance ID."
        )
    try:
        env = ros_environment(sourced_environment(domain_id), domain_id=domain_id)
        probe = subprocess.run(
            [
                str(PYTHON),
                str(PROJECT / "tools/verify_shutdown_state.py"),
                "--duration",
                "15",
                "--expected-system-id",
                str(instance_id + 1),
            ],
            env=env,
            cwd=run_dir,
            capture_output=True,
            text=True,
            timeout=20,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise IsolationError(f"Could not verify isolated landed/disarmed telemetry: {exc}") from exc
    if probe.returncode != 0:
        raise IsolationError(
            "Refusing shutdown because fresh landed/at-rest/disarmed telemetry "
            f"was not confirmed:\n{probe.stdout}{probe.stderr}"
        )
    print(probe.stdout.strip())
    failures = _signal_owned_groups(manifest)
    if failures:
        raise IsolationError("Bounded shutdown incomplete: " + "; ".join(failures))
    manifest["stopped_at_utc"] = datetime.now(timezone.utc).isoformat()
    _write_manifest(run_dir, manifest)
    print("Only the manifest-owned isolated process groups were stopped.")


def cleanup_failed_start(run_dir: Path) -> None:
    """Clean an owned partial start only when PX4 never created a server/Gazebo.

    This exception to the telemetry-gated stop is deliberately narrow: the
    launch log must contain PX4's per-instance singleton refusal, the PX4 child
    must have exited, and its recorded group may contain only our verified
    session guard. That proves this attempt never owned a vehicle process.
    """
    manifest = _load_manifest(run_dir)
    components = manifest.get("components", {})
    px4 = components.get("px4_gazebo")
    if not isinstance(px4, dict) or not _verified_record(px4):
        raise IsolationError("Cannot verify the failed PX4 process-group owner.")
    if _proc_start_time(int(px4["child_pid"])) == px4.get("child_start_time"):
        raise IsolationError("PX4 child is still running; use telemetry-gated stop.")
    if _live_group_members(int(px4["pgid"])) != {int(px4["pid"])}:
        raise IsolationError(
            "PX4 process group contains another live process; refusing failed-start cleanup."
        )
    log_path = Path(px4["log"])
    try:
        log = log_path.read_text(errors="replace")
    except OSError as exc:
        raise IsolationError(f"Cannot verify failed PX4 startup log {log_path}: {exc}") from exc
    if (
        "PX4 server already running for instance" not in log
        or "Startup script returned successfully" in log
    ):
        raise IsolationError(
            "PX4 log does not prove a pre-start singleton refusal; refusing cleanup."
        )
    failures = _signal_owned_groups(manifest)
    if failures:
        raise IsolationError("Failed-start cleanup incomplete: " + "; ".join(failures))
    manifest["failed_start_cleaned_at_utc"] = datetime.now(timezone.utc).isoformat()
    _write_manifest(run_dir, manifest)
    print("Verified PX4 never started; only manifest-owned partial-start groups were stopped.")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "command", choices=("start", "status", "stop", "cleanup-failed-start")
    )
    parser.add_argument("--run-dir", type=Path, default=RUN_DIR)
    parser.add_argument(
        "--profile",
        choices=sorted(PROFILES),
        default="legacy",
        help="start only: simulation assets (legacy: PX4 walls/x500_lidar_2d; "
        "sar_oakd: versioned sim/ world with target and LiDAR+OAK-D model)",
    )
    args = parser.parse_args(argv)
    try:
        if args.command == "start":
            start(args.run_dir, PROFILES[args.profile])
        elif args.command == "status":
            status(args.run_dir)
        elif args.command == "stop":
            stop(args.run_dir)
        else:
            cleanup_failed_start(args.run_dir)
    except (IsolationError, OSError, subprocess.SubprocessError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
