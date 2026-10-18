import importlib.util
import sys
import threading
import types
from pathlib import Path
from types import SimpleNamespace
import time

import numpy as np
import pytest


def load_environment(monkeypatch):
    """Load the environment with lightweight ROS/PX4/Gymnasium shims."""
    gym = types.ModuleType("gymnasium")

    class Env:
        def reset(self, *, seed=None):
            pass

    class Box:
        def __init__(self, low, high, shape=None, dtype=None):
            self.low = np.asarray(low, dtype=dtype)
            self.high = np.asarray(high, dtype=dtype)

    gym.Env = Env
    gym.spaces = SimpleNamespace(Box=Box)

    rclpy = types.ModuleType("rclpy")
    rclpy.ok = lambda: False
    rclpy.init = lambda: None
    rclpy.spin = lambda node: None

    rclpy_node = types.ModuleType("rclpy.node")

    class Node:
        pass

    rclpy_node.Node = Node
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.QoSProfile = lambda **kwargs: None
    rclpy_qos.ReliabilityPolicy = SimpleNamespace(BEST_EFFORT=1)
    rclpy_qos.DurabilityPolicy = SimpleNamespace(VOLATILE=1)

    std_msgs = types.ModuleType("std_msgs.msg")
    std_msgs.Float32MultiArray = type("Float32MultiArray", (), {})

    px4_msgs = types.ModuleType("px4_msgs.msg")
    for name in (
        "OffboardControlMode",
        "TrajectorySetpoint",
        "VehicleCommand",
        "VehicleLocalPosition",
        "VehicleStatus",
        "VehicleAttitude",
        "VehicleLandDetected",
        "VehicleCommandAck",
        "TimesyncStatus",
    ):
        setattr(px4_msgs, name, type(name, (), {}))
    px4_msgs.VehicleStatus.ARMING_STATE_DISARMED = 1
    px4_msgs.VehicleStatus.ARMING_STATE_ARMED = 2
    px4_msgs.VehicleStatus.NAVIGATION_STATE_OFFBOARD = 14
    px4_msgs.VehicleStatus.NAVIGATION_STATE_AUTO_LAND = 18
    px4_msgs.VehicleCommand.VEHICLE_CMD_DO_SET_MODE = 176
    px4_msgs.VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
    px4_msgs.VehicleCommand.VEHICLE_CMD_NAV_LAND = 21
    px4_msgs.VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED = 0
    px4_msgs.VehicleCommandAck.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED = 1
    px4_msgs.VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED = 2
    px4_msgs.VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED = 4
    px4_msgs.VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS = 5

    for name, module in (
        ("gymnasium", gym),
        ("rclpy", rclpy),
        ("rclpy.node", rclpy_node),
        ("rclpy.qos", rclpy_qos),
        ("std_msgs.msg", std_msgs),
        ("px4_msgs.msg", px4_msgs),
    ):
        monkeypatch.setitem(sys.modules, name, module)

    source = Path(__file__).with_name("px4_uav_env.py")
    spec = importlib.util.spec_from_file_location(
        "px4_uav_env_safety_subject", source
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeNode:
    def __init__(self):
        self.action = np.zeros(2, dtype=np.float32)
        self.phase = "VELOCITY"
        self.observation_counter = 1
        self.latest_observation = np.zeros(54, dtype=np.float32)
        self.observation_received_at = time.monotonic()
        self.x = 0.0
        self.y = 0.0
        self.z = -1.0
        self.heading = 0.0
        self.heading_valid = True
        self.position_valid = True
        self.roll = 0.0
        self.pitch = 0.0
        self.failsafe = False
        self.land_calls = 0
        self.condition = threading.Condition()
        self.landed = None
        self.land_detected_received_at = None
        self.status_received_at = None
        self.arming_state = 1
        self.nav_state = 4
        self.system_id = 1
        self.pre_flight_checks_pass = True
        self.latest_arming_reason = 0
        self.latest_disarming_reason = 0
        self.status_received_at = time.monotonic()
        self.position_received_at = time.monotonic()
        self.px4_sim_time_us = 1_000_000
        self.px4_sim_time_received_at = time.monotonic()
        self.takeoff_setpoint_count = 20
        self.takeoff_stream_started_at = time.monotonic() - 1.2
        self.takeoff_stream_last_at = time.monotonic()
        self.px4_setpoint_subscribers = True
        self.command_ack_counter = 0
        self.command_ack_history = []
        self.mode_ack_result = 0
        self.arm_ack_result = 0
        self.mode_ack_missing = False
        self.arm_ack_missing = False
        self.mode_transition = True
        self.arm_transition = True
        self.arm_calls = 0
        self.landing_complete = False
        self.destroy_calls = 0

    def land(self):
        self.land_calls += 1
        self.landing_complete = False

    def has_px4_setpoint_subscribers(self):
        return self.px4_setpoint_subscribers

    def _ack(self, command, result):
        self.command_ack_counter += 1
        self.command_ack_history.append(
            {
                "counter": self.command_ack_counter,
                "command": command,
                "result": result,
                "result_param1": 0,
                "result_param2": 0,
                "received_at": time.monotonic(),
            }
        )

    def request_offboard(self):
        counter = self.command_ack_counter
        command = 176
        if not self.mode_ack_missing:
            self._ack(command, self.mode_ack_result)
        if self.mode_transition:
            self.nav_state = 14
            self.status_received_at = time.monotonic()
        return counter

    def arm(self):
        counter = self.command_ack_counter
        self.arm_calls += 1
        command = 400
        if not self.arm_ack_missing:
            self._ack(command, self.arm_ack_result)
        if self.arm_transition:
            self.arming_state = 2
            self.status_received_at = time.monotonic()
            self.position_received_at = time.monotonic()
            self.z = -1.0
        return counter

    def destroy_node(self):
        self.destroy_calls += 1


class FakeSpinThread:
    def __init__(self):
        self.alive = True
        self.join_calls = 0

    def is_alive(self):
        return self.alive

    def join(self, timeout=None):
        self.join_calls += 1
        self.alive = False


def make_environment(module):
    env = module.PX4UAVEnv.__new__(module.PX4UAVEnv)
    env.LAND_WAIT_TIMEOUT = 0.02
    env.node = FakeNode()
    env._episode_active = True
    env._landing_pending = False
    env._landing_request_in_progress = False
    env._closed = False
    env._close_waiting = False
    env._close_lock = threading.Lock()
    env._landing_cleanup_thread = None
    env.current_step = 0
    env.max_steps = 10
    env.previous_action = np.zeros(2, dtype=np.float32)
    env.action_space = SimpleNamespace(
        low=np.array([0.0, -1.0], dtype=np.float32),
        high=np.array([1.0, 1.0], dtype=np.float32),
    )
    env._wait_for_observation = lambda **kwargs: np.full(
        54, 5.0, dtype=np.float32
    )
    env._distance_to_target = lambda: 10.0
    env._collision_detected = lambda raw: False
    env._convert_observation = lambda raw: np.zeros(54, dtype=np.float32)
    return env


def make_close_environment(module):
    env = make_environment(module)
    env._closed = False
    env._close_waiting = False
    env._close_lock = threading.Lock()
    env._landing_cleanup_thread = None
    env.spin_thread = FakeSpinThread()
    return env


def make_takeoff_environment(module):
    env = make_environment(module)
    env.TAKEOFF_STREAM_TIMEOUT = 0.02
    env.TAKEOFF_ACK_TIMEOUT = 0.02
    env.TAKEOFF_MODE_TIMEOUT = 0.02
    env.TAKEOFF_TIMEOUT = 0.02
    env.TAKEOFF_WALL_TIMEOUT = 0.2
    env.TAKEOFF_CLOCK_STALL_TIMEOUT = 0.2
    # The timer-driven setpoint stream is tested separately; transition tests
    # begin after that stream-readiness gate has succeeded.
    env._wait_for_takeoff_setpoint_stream = lambda: None
    return env


def report_landed_and_disarmed(module, node):
    with node.condition:
        now = time.monotonic()
        node.landed = True
        node.land_detected_received_at = now
        node.arming_state = 1
        node.status_received_at = now
        module.PX4ControllerNode._update_landing_complete(node)
        node.condition.notify_all()


def test_step_timeout_enters_safe_state_and_reraises_original(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_environment(module)
    env.node.arming_state = 2
    original_error = TimeoutError("observation timeout")

    def timeout(**kwargs):
        raise original_error

    env._wait_for_observation = timeout

    with pytest.raises(TimeoutError) as caught:
        env.step(np.array([0.7, 0.2], dtype=np.float32))

    assert caught.value is original_error
    assert not env._episode_active
    assert env.node.phase == "LAND"
    np.testing.assert_array_equal(env.node.action, [0.0, 0.0])
    assert env.node.land_calls == 1


def test_post_action_conversion_failure_enters_safe_state(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_environment(module)
    env.node.arming_state = 2
    original_error = ValueError("conversion failed")

    def fail_conversion(raw):
        raise original_error

    env._convert_observation = fail_conversion

    with pytest.raises(ValueError) as caught:
        env.step(np.array([0.4, -0.3], dtype=np.float32))

    assert caught.value is original_error
    assert not env._episode_active
    assert env.node.phase == "LAND"
    np.testing.assert_array_equal(env.node.action, [0.0, 0.0])
    assert env.node.land_calls == 1


def test_safe_state_does_not_send_duplicate_land_requests(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_environment(module)
    env.node.arming_state = 2

    env._enter_safe_state()
    env._enter_safe_state()

    assert env.node.land_calls == 1
    assert env._landing_pending


def test_reset_waits_for_landed_and_disarmed_before_takeoff(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_environment(module)
    env._landing_pending = True
    started = []
    def takeoff():
        assert env.node.landed is True
        assert env.node.arming_state == 1
        started.append(True)

    env._takeoff = takeoff
    env._wait_for_observation = lambda **kwargs: np.zeros(54, dtype=np.float32)

    def report_landed():
        time.sleep(0.005)
        with env.node.condition:
            env.node.landed = True
            env.node.land_detected_received_at = time.monotonic()
            env.node.arming_state = 1
            env.node.status_received_at = time.monotonic()
            env.node.condition.notify_all()

    reporter = threading.Thread(target=report_landed)
    reporter.start()
    env.reset()
    reporter.join()

    assert env._landing_pending is False
    assert started == [True]
    assert env._episode_active is True


def make_wait_environment(module):
    env = make_environment(module)
    env.node.condition = threading.Condition()
    env.node.landed = None
    env.node.land_detected_received_at = None
    env.node.status_received_at = None
    env.node.arming_state = 1
    return env


def test_wait_accepts_fresh_landed_and_disarmed_state(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_wait_environment(module)
    now = time.monotonic()
    env.node.landed = True
    env.node.land_detected_received_at = now
    env.node.arming_state = 1
    env.node.status_received_at = now
    env._landing_pending = True

    env._wait_until_landed(timeout=0.01)

    assert env._landing_pending is False


def test_wait_rejects_landed_but_armed(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_wait_environment(module)
    now = time.monotonic()
    env.node.landed = True
    env.node.land_detected_received_at = now
    env.node.arming_state = 2
    env.node.status_received_at = now

    with pytest.raises(TimeoutError, match="landed and disarmed"):
        env._wait_until_landed(timeout=0.01)


def test_wait_rejects_stale_telemetry(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_wait_environment(module)
    stale = time.monotonic() - 5.0
    env.node.landed = True
    env.node.land_detected_received_at = stale
    env.node.arming_state = 1
    env.node.status_received_at = stale

    with pytest.raises(TimeoutError, match="landed and disarmed"):
        env._wait_until_landed(timeout=0.01, max_age=0.1)


def test_reset_missing_telemetry_fails_without_takeoff(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_wait_environment(module)
    env.TAKEOFF_STATUS_UPDATE_TIMEOUT = 0.02
    env._takeoff = lambda: pytest.fail("takeoff must not start")

    with pytest.raises(TimeoutError, match="landed and disarmed"):
        env.reset()

    assert not env._episode_active
    assert env.node.phase == "LAND"
    assert env.node.land_calls == 1


def test_wait_times_out_while_landing_is_incomplete(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_wait_environment(module)
    now = time.monotonic()
    env.node.landed = False
    env.node.land_detected_received_at = now
    env.node.arming_state = 1
    env.node.status_received_at = now

    with pytest.raises(TimeoutError, match="landed and disarmed"):
        env._wait_until_landed(timeout=0.01)


def test_close_waits_for_landed_and_disarmed_before_cleanup(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_close_environment(module)
    env.LAND_WAIT_TIMEOUT = 0.2
    monkeypatch.setattr(module.rclpy, "ok", lambda: False)

    def telemetry_callback():
        time.sleep(0.01)
        assert env.spin_thread.is_alive()
        report_landed_and_disarmed(module, env.node)

    reporter = threading.Thread(target=telemetry_callback)
    reporter.start()
    env.close()
    reporter.join()

    assert env.node.landing_complete is True
    assert env.node.destroy_calls == 1
    assert env.spin_thread.join_calls == 1


def test_close_landing_wait_is_bounded_above_observed_sitl_delay(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_close_environment(module)
    del env.LAND_WAIT_TIMEOUT
    monkeypatch.setattr(module.rclpy, "ok", lambda: False)
    observed_timeouts = []
    env._wait_until_landed = lambda timeout: observed_timeouts.append(timeout)

    env.close()

    assert observed_timeouts == [30.0]


@pytest.mark.parametrize("telemetry", ["missing", "stale"])
def test_close_timeout_does_not_claim_landing(monkeypatch, telemetry):
    module = load_environment(monkeypatch)
    env = make_close_environment(module)
    env.LAND_WAIT_TIMEOUT = 0.01
    monkeypatch.setattr(module.rclpy, "ok", lambda: False)
    env._schedule_cleanup_after_landing = lambda: None

    if telemetry == "stale":
        old = time.monotonic() - 5.0
        env.node.landed = True
        env.node.land_detected_received_at = old
        env.node.arming_state = 1
        env.node.status_received_at = old

    with pytest.warns(RuntimeWarning, match="Landing was not confirmed"):
        env.close()

    assert env.node.landing_complete is False
    assert env.node.destroy_calls == 0
    assert env.spin_thread.join_calls == 0
    assert env._closed is False
    assert env._close_waiting is True


def test_close_timeout_when_fresh_telemetry_reports_not_landed(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_close_environment(module)
    env.LAND_WAIT_TIMEOUT = 0.01
    monkeypatch.setattr(module.rclpy, "ok", lambda: False)
    env._schedule_cleanup_after_landing = lambda: None
    now = time.monotonic()
    env.node.landed = False
    env.node.land_detected_received_at = now
    env.node.arming_state = 1
    env.node.status_received_at = now

    with pytest.warns(RuntimeWarning, match="Landing was not confirmed"):
        env.close()

    assert env.node.landing_complete is False
    assert env.node.destroy_calls == 0


def test_repeated_close_is_idempotent(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_close_environment(module)
    env.LAND_WAIT_TIMEOUT = 0.2
    monkeypatch.setattr(module.rclpy, "ok", lambda: False)
    report_landed_and_disarmed(module, env.node)

    env.close()
    env.close()

    assert env._closed is True
    assert env.node.land_calls == 0
    assert env.node.destroy_calls == 1
    assert env.spin_thread.join_calls == 1


def test_close_timeout_keeps_ros_active_until_landing_is_confirmed(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_close_environment(module)
    env.LAND_WAIT_TIMEOUT = 0.01
    monkeypatch.setattr(module.rclpy, "ok", lambda: False)
    wait_calls = []

    def wait_then_confirm(timeout):
        wait_calls.append(timeout)
        if len(wait_calls) == 1:
            raise TimeoutError("landing still in progress")
        report_landed_and_disarmed(module, env.node)

    env._wait_until_landed = wait_then_confirm

    with pytest.warns(RuntimeWarning, match="ROS control remains active"):
        env.close()

    env._landing_cleanup_thread.join(timeout=1.0)

    assert not env._landing_cleanup_thread.is_alive()
    assert env.node.landing_complete is True
    assert env.node.destroy_calls == 1
    assert env.spin_thread.join_calls == 1
    assert env._closed is True


def test_close_timeout_then_delayed_disarm_confirms_cleanup(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_close_environment(module)
    env.LAND_WAIT_TIMEOUT = 0.01
    monkeypatch.setattr(module.rclpy, "ok", lambda: False)
    now = time.monotonic()
    env.node.arming_state = 2
    env.node.nav_state = 18
    env.node.landed = True
    env.node.status_received_at = now
    env.node.land_detected_received_at = now
    reporter_finished = threading.Event()

    def report_disarmed_after_landing_delay():
        time.sleep(0.05)
        assert env.node.landing_complete is False
        report_landed_and_disarmed(module, env.node)
        reporter_finished.set()

    reporter = threading.Thread(target=report_disarmed_after_landing_delay)
    reporter.start()

    with pytest.warns(RuntimeWarning, match="Landing was not confirmed"):
        env.close()

    assert env.node.landing_complete is False
    assert env.node.destroy_calls == 0
    env._landing_cleanup_thread.join(timeout=1.0)
    reporter.join(timeout=1.0)

    assert reporter_finished.is_set()
    assert not env._landing_cleanup_thread.is_alive()
    assert env.node.landing_complete is True
    assert env.node.destroy_calls == 1
    assert env._closed is True


def test_close_continues_cleanup_after_node_destroy_error(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_close_environment(module)
    monkeypatch.setattr(module.rclpy, "ok", lambda: False)
    env.node.destroy_node = lambda: (_ for _ in ()).throw(
        RuntimeError("destroy failed")
    )
    report_landed_and_disarmed(module, env.node)

    with pytest.warns(RuntimeWarning, match="Could not destroy"):
        env.close()

    assert env.spin_thread.join_calls == 1


@pytest.mark.parametrize(
    ("collision", "step_count", "expected_terminated", "expected_truncated"),
    [
        (True, 0, True, False),
        (False, 9, False, True),
    ],
)
def test_episode_endings_enter_safe_state(
    monkeypatch, collision, step_count, expected_terminated, expected_truncated
):
    module = load_environment(monkeypatch)
    env = make_environment(module)
    env.node.arming_state = 2
    env.current_step = step_count
    env._collision_detected = lambda raw: collision

    _, _, terminated, truncated, _ = env.step(
        np.array([0.2, 0.0], dtype=np.float32)
    )

    assert terminated is expected_terminated
    assert truncated is expected_truncated
    assert not env._episode_active
    assert env.node.phase == "LAND"
    np.testing.assert_array_equal(env.node.action, [0.0, 0.0])
    assert env.node.land_calls == 1


def test_takeoff_waits_for_acknowledged_mode_and_arm_transitions(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_takeoff_environment(module)
    env.node.x = 1.25
    env.node.y = -0.75

    env._takeoff()

    assert env.node.phase == "VELOCITY"
    assert env.node.arm_calls == 1
    assert env.node.takeoff_target_x == 1.25
    assert env.node.takeoff_target_y == -0.75


def test_takeoff_refuses_invalid_local_position_before_offboard(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_takeoff_environment(module)
    env.node.position_valid = False

    with pytest.raises(RuntimeError, match="local-position telemetry.*invalid"):
        env._takeoff()

    assert env.node.phase == "VELOCITY"
    assert env.node.command_ack_counter == 0
    assert env.node.arm_calls == 0


def test_takeoff_position_setpoint_holds_current_xy_and_keeps_altitude_gate(
    monkeypatch,
):
    module = load_environment(monkeypatch)

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    node = module.PX4ControllerNode.__new__(module.PX4ControllerNode)
    node.setpoint_pub = Publisher()
    node.timestamp = lambda: 123

    module.PX4ControllerNode.publish_position_setpoint(
        node,
        z_target=-1.0,
        x_target=-0.4,
        y_target=0.25,
    )

    assert node.setpoint_pub.messages[0].position == [-0.4, 0.25, -1.0]


def test_observation_wait_rejects_cached_stale_array(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_environment(module)
    env._wait_for_observation = module.PX4UAVEnv._wait_for_observation.__get__(env)
    env.node.observation_received_at = time.monotonic() - 2.0

    with pytest.raises(TimeoutError, match="/uav/observation"):
        env._wait_for_observation(timeout=0.01)


def test_observation_callback_rejects_non_finite_values(monkeypatch):
    module = load_environment(monkeypatch)
    node = module.PX4ControllerNode.__new__(module.PX4ControllerNode)
    node.condition = threading.Condition()
    node.observation_counter = 0
    node.latest_observation = None
    node.observation_received_at = None

    message = SimpleNamespace(data=[0.0] * 53 + [float("nan")])
    module.PX4ControllerNode.observation_callback(node, message)

    assert node.observation_counter == 0
    assert node.latest_observation is None


def test_target_world_position_is_converted_to_px4_ned_for_reward(monkeypatch):
    module = load_environment(monkeypatch)
    env = module.PX4UAVEnv.__new__(module.PX4UAVEnv)
    env.node = SimpleNamespace(x=-3.0, y=3.0)

    assert env._distance_to_target() == pytest.approx(0.0)


def test_positive_speed_without_progress_gets_no_speed_bonus(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_environment(module)
    raw = np.full(54, 5.0, dtype=np.float32)
    env._wait_for_observation = lambda **kwargs: raw
    env._distance_to_target = lambda: 10.0

    _, reward, terminated, truncated, _ = env._step_after_action(
        np.array([0.8, 0.0], dtype=np.float32),
        old_distance=10.0,
        previous_counter=env.node.observation_counter,
    )

    assert reward == pytest.approx(-0.1)
    assert not terminated
    assert not truncated


def test_safe_state_serializes_zero_action_before_land_setpoints(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_environment(module)
    env.node.arming_state = 2
    env.node.action[:] = [0.8, 0.5]
    env.node.phase = "VELOCITY"
    published_velocity = []
    env.node.publish_offboard_mode = lambda position_mode: None
    env.node.publish_velocity_setpoint = lambda: published_velocity.append(
        env.node.action.copy()
    )

    env._enter_safe_state()
    module.PX4ControllerNode.control_loop(env.node)

    assert env.node.phase == "LAND"
    np.testing.assert_array_equal(published_velocity[0], [0.0, 0.0])


def test_safe_state_waits_for_inflight_velocity_publish_then_stops_old_action(
    monkeypatch,
):
    module = load_environment(monkeypatch)
    env = make_environment(module)
    env.node.arming_state = 2
    env.node.action[:] = [0.8, 0.5]
    env.node.phase = "VELOCITY"
    publishing = threading.Event()
    release_publish = threading.Event()
    safe_state_done = threading.Event()
    published_actions = []
    env.node.publish_offboard_mode = lambda position_mode: None

    def block_first_velocity_publish():
        published_actions.append(env.node.action.copy())
        if len(published_actions) == 1:
            publishing.set()
            assert release_publish.wait(timeout=1.0)

    env.node.publish_velocity_setpoint = block_first_velocity_publish
    timer_thread = threading.Thread(
        target=module.PX4ControllerNode.control_loop,
        args=(env.node,),
    )
    timer_thread.start()
    assert publishing.wait(timeout=1.0)

    safe_thread = threading.Thread(
        target=lambda: (env._enter_safe_state(), safe_state_done.set())
    )
    safe_thread.start()
    time.sleep(0.01)
    assert not safe_state_done.is_set()

    release_publish.set()
    timer_thread.join(timeout=1.0)
    safe_thread.join(timeout=1.0)
    assert safe_state_done.is_set()

    module.PX4ControllerNode.control_loop(env.node)
    assert env.node.phase == "LAND"
    np.testing.assert_allclose(published_actions[0], [0.8, 0.5])
    np.testing.assert_array_equal(published_actions[1], [0.0, 0.0])


def test_command_ack_subscription_matches_px4_dds_topic(monkeypatch):
    module = load_environment(monkeypatch)

    assert (
        module.VEHICLE_COMMAND_ACK_TOPIC
        == "/fmu/out/vehicle_command_ack_v1"
    )


def test_status_callback_stores_px4_system_id(monkeypatch):
    module = load_environment(monkeypatch)
    node = module.PX4ControllerNode.__new__(module.PX4ControllerNode)
    node.condition = threading.Condition()
    node._update_landing_complete = lambda: None
    node.system_id = None
    message = SimpleNamespace(
        arming_state=1,
        nav_state=4,
        system_id=2,
        failsafe=False,
        pre_flight_checks_pass=True,
        latest_arming_reason=0,
        latest_disarming_reason=0,
    )

    module.PX4ControllerNode.status_callback(node, message)

    assert node.system_id == 2


@pytest.mark.parametrize(
    ("method_name", "expected_command", "expected_param1", "expected_param2"),
    [
        ("request_offboard", 176, 1.0, 6.0),
        ("arm", 400, 1.0, 0.0),
    ],
)
def test_flight_command_helpers_return_ack_counter_and_publish_expected_command(
    monkeypatch, method_name, expected_command, expected_param1, expected_param2
):
    module = load_environment(monkeypatch)

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    node = module.PX4ControllerNode.__new__(module.PX4ControllerNode)
    node.condition = threading.Condition()
    node.command_ack_counter = 7
    node.system_id = 2
    node.status_received_at = time.monotonic()
    node.command_pub = Publisher()
    node.timestamp = lambda: 123

    counter = getattr(node, method_name)()

    assert counter == 7
    assert len(node.command_pub.messages) == 1
    message = node.command_pub.messages[0]
    assert message.command == expected_command
    assert message.param1 == expected_param1
    assert message.param2 == expected_param2
    assert message.target_system == 2


def test_send_command_refuses_to_guess_px4_system_id(monkeypatch):
    module = load_environment(monkeypatch)

    class Publisher:
        def publish(self, message):
            raise AssertionError("command must not be published without a system ID")

    node = module.PX4ControllerNode.__new__(module.PX4ControllerNode)
    node.condition = threading.Condition()
    node.command_ack_counter = 0
    node.system_id = None
    node.status_received_at = time.monotonic()
    node.command_pub = Publisher()

    with pytest.raises(RuntimeError, match="VehicleStatus provides the system ID"):
        node.send_command(176)


def test_send_command_refuses_stale_system_id_telemetry(monkeypatch):
    module = load_environment(monkeypatch)

    class Publisher:
        def publish(self, message):
            raise AssertionError("command must not be published with stale status")

    node = module.PX4ControllerNode.__new__(module.PX4ControllerNode)
    node.condition = threading.Condition()
    node.command_ack_counter = 0
    node.system_id = 2
    node.status_received_at = time.monotonic() - 1.001
    node.command_pub = Publisher()

    with pytest.raises(RuntimeError, match="missing or stale VehicleStatus"):
        node.send_command(176)


def test_land_command_allows_stale_status_with_observed_system_id(monkeypatch):
    module = load_environment(monkeypatch)

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    node = module.PX4ControllerNode.__new__(module.PX4ControllerNode)
    node.condition = threading.Condition()
    node.landing_complete = False
    node.command_ack_counter = 0
    node.command_ack_history = []
    node.system_id = 2
    node.status_received_at = time.monotonic() - 5.0
    node.command_pub = Publisher()
    node.timestamp = lambda: 123

    node.land()

    assert len(node.command_pub.messages) == 1
    assert node.command_pub.messages[0].command == (
        module.VehicleCommand.VEHICLE_CMD_NAV_LAND
    )
    assert node.command_pub.messages[0].target_system == 2


@pytest.mark.parametrize("command_stage", ["mode", "arm"])
def test_takeoff_rejected_command_fails_with_command_diagnostic(
    monkeypatch, command_stage
):
    module = load_environment(monkeypatch)
    env = make_takeoff_environment(module)
    if command_stage == "mode":
        env.node.mode_ack_result = 2
        expected = "command 176: result=2"
    else:
        env.node.arm_ack_result = 4
        expected = "command 400: result=4"

    with pytest.raises(RuntimeError, match=expected):
        env._takeoff()


def test_takeoff_missing_ack_fails_without_arming(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_takeoff_environment(module)
    env.node.mode_ack_missing = True

    with pytest.raises(TimeoutError, match="VehicleCommandAck.*176"):
        env._takeoff()

    assert env.node.arm_calls == 0


def test_takeoff_ignores_previously_received_matching_ack(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_takeoff_environment(module)
    env.node.command_ack_counter = 1
    env.node.command_ack_history.append(
        {
            "counter": 1,
            "command": 176,
            "result": 0,
            "result_param1": 0,
            "result_param2": 0,
            "received_at": time.monotonic(),
        }
    )
    env.node.mode_ack_missing = True

    with pytest.raises(TimeoutError, match="VehicleCommandAck.*176"):
        env._takeoff()

    assert env.node.arm_calls == 0


def test_reset_missing_arm_ack_enters_safe_state(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_takeoff_environment(module)
    env.node.arm_ack_missing = True
    env._wait_until_landed = lambda **kwargs: None

    with pytest.raises(TimeoutError, match="VehicleCommandAck.*400"):
        env.reset()

    assert env.node.arm_calls == 1
    assert not env._episode_active
    assert env.node.phase == "LAND"
    assert env.node.land_calls == 1


def test_takeoff_preflight_failure_prevents_arm(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_takeoff_environment(module)
    env.node.pre_flight_checks_pass = False

    with pytest.raises(RuntimeError, match="pre-flight checks are failing"):
        env._takeoff()

    assert env.node.arm_calls == 0


def test_reset_preflight_failure_does_not_land_while_disarmed(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_takeoff_environment(module)
    env._wait_until_landed = lambda **kwargs: None
    env.node.pre_flight_checks_pass = False

    with pytest.raises(RuntimeError, match="pre-flight checks are failing"):
        env.reset()

    assert not env._episode_active
    assert env.node.phase == "LAND"
    np.testing.assert_array_equal(env.node.action, [0.0, 0.0])
    assert env.node.land_calls == 0


def test_reset_altitude_timeout_then_close_keeps_ros_active(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_takeoff_environment(module)
    env.TAKEOFF_TIMEOUT = 0.01
    env.LAND_WAIT_TIMEOUT = 0.01
    wait_until_landed = env._wait_until_landed
    env._wait_until_landed = lambda **kwargs: None

    arm = env.node.arm

    def arm_without_reaching_altitude():
        counter = arm()
        env.node.z = -0.623
        return counter

    env.node.arm = arm_without_reaching_altitude

    def advance_past_takeoff_deadline():
        time.sleep(0.02)
        with env.node.condition:
            env.node.px4_sim_time_us += 11_000
            env.node.px4_sim_time_received_at = time.monotonic()
            env.node.condition.notify_all()

    clock_updater = threading.Thread(target=advance_past_takeoff_deadline)
    clock_updater.start()

    with pytest.raises(TimeoutError, match="altitude.*NED z=-0.623"):
        env.reset()
    clock_updater.join()

    assert env.node.arming_state == 2
    assert env.node.phase == "LAND"
    assert env.node.land_calls == 1
    assert not env._episode_active

    env._wait_until_landed = wait_until_landed
    monkeypatch.setattr(module.rclpy, "ok", lambda: False)
    env._schedule_cleanup_after_landing = lambda: None
    with pytest.warns(RuntimeWarning, match="Landing was not confirmed"):
        env.close()

    assert env.node.landing_complete is False
    assert env.node.land_calls == 1
    assert env.node.destroy_calls == 0
    assert env._closed is False
    assert env._close_waiting is True


def test_safe_state_does_not_claim_ground_from_disarmed_status_alone(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_environment(module)

    env._enter_safe_state()

    assert not env._episode_active
    assert env.node.phase == "LAND"
    np.testing.assert_array_equal(env.node.action, [0.0, 0.0])
    assert env.node.land_calls == 0


def test_safe_state_enters_ground_only_with_fresh_landed_disarmed_telemetry(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_environment(module)
    env.node.landed = True
    env.node.land_detected_received_at = time.monotonic()

    env._enter_safe_state()

    assert env.node.phase == "GROUND"
    assert env.node.land_calls == 0


def test_failed_land_publication_can_be_retried(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_environment(module)
    env.node.arming_state = 2
    calls = []

    def fail_once():
        calls.append("called")
        if len(calls) == 1:
            raise RuntimeError("publisher unavailable")

    env.node.land = fail_once
    with pytest.raises(RuntimeError, match="publisher unavailable"):
        env._enter_safe_state()
    assert env._landing_pending is False
    assert env._landing_request_in_progress is False

    env._enter_safe_state()
    assert calls == ["called", "called"]
    assert env._landing_pending is True


def test_safe_state_sends_best_effort_land_during_stale_status(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_environment(module)
    env.node.arming_state = 2
    env.node.status_received_at = time.monotonic() - 1.1

    env._enter_safe_state()

    assert env.node.phase == "LAND"
    assert env.node.land_calls == 1
    assert env._landing_pending is True
    assert env._landing_request_in_progress is False


def test_landing_complete_clears_when_vehicle_is_no_longer_landed(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_environment(module)
    env.node.landing_complete = True
    env.node.landed = True
    env.node.land_detected_received_at = time.monotonic()
    env.node.status_received_at = time.monotonic()
    env.node.arming_state = 2

    module.PX4ControllerNode._update_landing_complete(env.node)

    assert env.node.landing_complete is False


def test_takeoff_stale_status_fails_before_arm(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_takeoff_environment(module)
    env.node.mode_transition = False
    env.node.status_received_at = time.monotonic() - 5.0

    with pytest.raises(TimeoutError, match="VehicleStatus did not provide a fresh update"):
        env._takeoff()

    assert env.node.arm_calls == 0


def test_airborne_wait_allows_observed_status_interval_for_fresh_update(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_takeoff_environment(module)
    env.node.arming_state = 2
    env.node.nav_state = 14
    env.node.z = 0.0
    env.node.status_received_at = time.monotonic() - 1.1

    def report_fresh_airborne_state():
        time.sleep(0.02)
        with env.node.condition:
            now = time.monotonic()
            env.node.status_received_at = now
            env.node.position_received_at = now
            env.node.px4_sim_time_us += 1_000
            env.node.px4_sim_time_received_at = now
            env.node.z = -0.8
            env.node.condition.notify_all()

    reporter = threading.Thread(target=report_fresh_airborne_state)
    reporter.start()
    env._wait_for_airborne_state()
    reporter.join()

    assert env.node.status_received_at > time.monotonic() - 1.0


def test_airborne_wait_survives_observed_1_14_second_status_interval(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_takeoff_environment(module)
    env.TAKEOFF_TIMEOUT = 15.0
    env.TAKEOFF_WALL_TIMEOUT = 45.0
    env.TAKEOFF_CLOCK_STALL_TIMEOUT = 5.0
    env.TAKEOFF_STATUS_UPDATE_TIMEOUT = 1.5
    env.node.arming_state = 2
    env.node.nav_state = 14
    env.node.z = 0.0
    env.node.status_received_at = time.monotonic() - 0.02
    env.node.position_received_at = time.monotonic()
    env.node.px4_sim_time_us = 1_000_000
    env.node.px4_sim_time_received_at = time.monotonic()

    def publish_status_at_observed_cadence():
        deadline = time.monotonic() + 1.12
        while time.monotonic() < deadline:
            time.sleep(0.02)
            with env.node.condition:
                now = time.monotonic()
                env.node.position_received_at = now
                env.node.px4_sim_time_us += 8_800  # about 0.44x wall rate
                env.node.px4_sim_time_received_at = now
                env.node.condition.notify_all()
        with env.node.condition:
            now = time.monotonic()
            env.node.status_received_at = now
            env.node.position_received_at = now
            env.node.px4_sim_time_us += 8_800
            env.node.px4_sim_time_received_at = now
            env.node.z = -0.8
            env.node.condition.notify_all()

    reporter = threading.Thread(target=publish_status_at_observed_cadence)
    reporter.start()
    env._wait_for_airborne_state()
    reporter.join()

    assert env.node.z <= -0.8
    assert env.node.status_received_at > time.monotonic() - 1.0


def test_airborne_wait_survives_observed_1_71_second_status_interval(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_takeoff_environment(module)
    env.TAKEOFF_TIMEOUT = 15.0
    env.TAKEOFF_WALL_TIMEOUT = 45.0
    env.TAKEOFF_CLOCK_STALL_TIMEOUT = 5.0
    env.TAKEOFF_STATUS_UPDATE_TIMEOUT = 2.0
    env.node.arming_state = 2
    env.node.nav_state = 14
    env.node.z = 0.0
    env.node.status_received_at = time.monotonic() - 0.02
    env.node.position_received_at = time.monotonic()
    env.node.px4_sim_time_us = 1_000_000
    env.node.px4_sim_time_received_at = time.monotonic()

    def publish_status_at_observed_cadence():
        deadline = time.monotonic() + 1.72
        while time.monotonic() < deadline:
            time.sleep(0.02)
            with env.node.condition:
                now = time.monotonic()
                env.node.position_received_at = now
                env.node.px4_sim_time_us += 6_000  # about 0.30x wall rate
                env.node.px4_sim_time_received_at = now
                env.node.condition.notify_all()
        with env.node.condition:
            now = time.monotonic()
            env.node.status_received_at = now
            env.node.position_received_at = now
            env.node.px4_sim_time_us += 6_000
            env.node.px4_sim_time_received_at = now
            env.node.z = -0.8
            env.node.condition.notify_all()

    reporter = threading.Thread(target=publish_status_at_observed_cadence)
    reporter.start()
    env._wait_for_airborne_state()
    reporter.join()

    assert env.node.z <= -0.8
    assert env.node.status_received_at > time.monotonic() - 1.0


def test_velocity_forward_action_is_suppressed_without_valid_heading(monkeypatch):
    module = load_environment(monkeypatch)
    published = []

    class Publisher:
        def publish(self, msg):
            published.append(msg)

    node = SimpleNamespace(
        action=np.array([0.8, 0.4]),
        heading=0.7,
        heading_valid=False,
        timestamp=lambda: 123,
        setpoint_pub=Publisher(),
    )

    module.PX4ControllerNode.publish_velocity_setpoint(node)

    assert len(published) == 1
    assert published[0].velocity == [0.0, 0.0, 0.0]
    assert published[0].yawspeed == pytest.approx(0.2)


def test_airborne_wait_stops_if_status_update_exceeds_bounded_interval(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_takeoff_environment(module)
    env.TAKEOFF_STATUS_UPDATE_TIMEOUT = 0.03
    env.node.status_received_at = None
    env.node.arming_state = 2
    env.node.nav_state = 14
    env.node.z = 0.0

    with pytest.raises(
        TimeoutError,
        match="VehicleStatus did not provide a fresh update within 0.030 s",
    ):
        env._wait_for_airborne_state()


def test_takeoff_timeout_reports_missing_altitude(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_takeoff_environment(module)
    env.node.arm_transition = False
    env.node.arming_state = 2
    env.node.z = 0.0
    def advance_past_takeoff_deadline():
        time.sleep(0.03)
        with env.node.condition:
            env.node.px4_sim_time_us += 21_000
            env.node.px4_sim_time_received_at = time.monotonic()
            env.node.condition.notify_all()

    clock_updater = threading.Thread(target=advance_past_takeoff_deadline)
    clock_updater.start()

    with pytest.raises(TimeoutError, match="altitude.*need <= -0.8 m"):
        env._takeoff()
    clock_updater.join()


def test_takeoff_deadline_tracks_px4_time_when_wall_time_runs_ahead(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_takeoff_environment(module)
    env.TAKEOFF_TIMEOUT = 0.02
    env.TAKEOFF_CLOCK_STALL_TIMEOUT = 0.2
    env.TAKEOFF_WALL_TIMEOUT = 0.2
    env.node.arming_state = 2
    env.node.nav_state = 14
    env.node.z = 0.0
    start_px4_time = env.node.px4_sim_time_us

    def advance_slow_simulation():
        time.sleep(0.04)  # Longer than the simulated takeoff limit in wall time.
        with env.node.condition:
            env.node.px4_sim_time_us = start_px4_time + 15_000
            env.node.px4_sim_time_received_at = time.monotonic()
            env.node.z = -0.8
            env.node.status_received_at = time.monotonic()
            env.node.position_received_at = time.monotonic()
            env.node.condition.notify_all()

    updater = threading.Thread(target=advance_slow_simulation)
    updater.start()
    started_at = time.monotonic()
    env._wait_for_airborne_state()
    wall_elapsed = time.monotonic() - started_at
    updater.join()

    assert wall_elapsed > env.TAKEOFF_TIMEOUT
    assert env.node.px4_sim_time_us - start_px4_time < int(
        env.TAKEOFF_TIMEOUT * 1_000_000
    )


def test_takeoff_fails_safely_if_simulation_clock_is_paused(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_takeoff_environment(module)
    env.TAKEOFF_CLOCK_STALL_TIMEOUT = 0.02
    env.TAKEOFF_WALL_TIMEOUT = 0.2
    env.node.arming_state = 2
    env.node.nav_state = 14
    env.node.z = 0.0

    with pytest.raises(TimeoutError, match="simulation clock.*did not advance"):
        env._wait_for_airborne_state()


def test_takeoff_has_independent_wall_clock_ceiling(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_takeoff_environment(module)
    env.TAKEOFF_TIMEOUT = 1.0
    env.TAKEOFF_WALL_TIMEOUT = 0.05
    env.TAKEOFF_CLOCK_STALL_TIMEOUT = 0.2
    env.node.arming_state = 2
    env.node.nav_state = 14
    env.node.z = 0.0

    def advance_simulation_slowly():
        for _ in range(10):
            time.sleep(0.01)
            with env.node.condition:
                env.node.px4_sim_time_us += 1_000
                now = time.monotonic()
                env.node.px4_sim_time_received_at = now
                env.node.status_received_at = now
                env.node.position_received_at = now
                env.node.condition.notify_all()

    updater = threading.Thread(target=advance_simulation_slowly)
    updater.start()

    with pytest.raises(TimeoutError, match="wall-clock limit.*PX4 elapsed"):
        env._wait_for_airborne_state()
    updater.join()


def test_timesync_offset_step_cancels_in_reconstructed_clock(monkeypatch):
    module = load_environment(monkeypatch)
    node = module.PX4ControllerNode.__new__(module.PX4ControllerNode)
    node.condition = threading.Condition()
    node.px4_sim_time_us = None
    node.px4_sim_time_received_at = None

    # Simulate a 7-second offset change. The serialized timestamp changes by
    # the opposite amount, so timestamp + estimated_offset still advances by
    # the underlying 0.2 seconds of PX4 time.
    node.timesync_status_callback(
        SimpleNamespace(timestamp=13_000_000, estimated_offset=-5_000_000)
    )
    first_reconstructed = node.px4_sim_time_us
    node.timesync_status_callback(
        SimpleNamespace(timestamp=6_200_000, estimated_offset=2_000_000)
    )

    assert first_reconstructed == 8_000_000
    assert node.px4_sim_time_us == 8_200_000


def test_takeoff_rejects_backward_reconstructed_clock(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_takeoff_environment(module)
    env.TAKEOFF_CLOCK_STALL_TIMEOUT = 0.2
    env.node.arming_state = 2
    env.node.nav_state = 14
    env.node.z = 0.0

    def move_clock_backwards():
        time.sleep(0.02)
        with env.node.condition:
            env.node.px4_sim_time_us -= 100_000
            env.node.px4_sim_time_received_at = time.monotonic()
            env.node.condition.notify_all()

    updater = threading.Thread(target=move_clock_backwards)
    updater.start()
    with pytest.raises(RuntimeError, match="simulation clock moved backwards"):
        env._wait_for_airborne_state()
    updater.join()


def test_timesync_callback_reconstructs_px4_clock(monkeypatch):
    module = load_environment(monkeypatch)
    node = module.PX4ControllerNode.__new__(module.PX4ControllerNode)
    node.condition = threading.Condition()
    node.px4_sim_time_us = None
    node.px4_sim_time_received_at = None
    message = SimpleNamespace(timestamp=8_000_000, estimated_offset=2_000_000)

    node.timesync_status_callback(message)

    assert node.px4_sim_time_us == 10_000_000
    assert node.px4_sim_time_received_at is not None


def test_takeoff_setpoint_stream_gate_requires_duration_and_samples(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_takeoff_environment(module)
    env._wait_for_takeoff_setpoint_stream = (
        module.PX4UAVEnv._wait_for_takeoff_setpoint_stream.__get__(env)
    )
    env.node.takeoff_setpoint_count = 3
    env.node.takeoff_stream_started_at = time.monotonic() - 1.1
    env.node.takeoff_stream_last_at = time.monotonic()

    env._wait_for_takeoff_setpoint_stream()

    env.node.takeoff_setpoint_count = 2
    with pytest.raises(TimeoutError, match="setpoint stream"):
        env._wait_for_takeoff_setpoint_stream()


def test_takeoff_setpoint_stream_requires_px4_topic_subscriptions(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_takeoff_environment(module)
    env._wait_for_takeoff_setpoint_stream = (
        module.PX4UAVEnv._wait_for_takeoff_setpoint_stream.__get__(env)
    )
    env.TAKEOFF_STREAM_TIMEOUT = 0.02
    env.node.px4_setpoint_subscribers = False

    with pytest.raises(TimeoutError, match="PX4 topic subscriptions"):
        env._wait_for_takeoff_setpoint_stream()
