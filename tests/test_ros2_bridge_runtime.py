from __future__ import annotations

import importlib.util
import json
import sys
import types
from pathlib import Path

BRIDGE_NODE_PATH = Path(
    "hardware/ros2_ws/src/agi_walker_ros2/agi_walker_ros2/bridge_node.py"
)
RUNTIME_PROBE_PATH = Path(
    "hardware/ros2_ws/src/agi_walker_ros2/agi_walker_ros2/runtime_probe.py"
)
REPLAY_FIXTURE = Path("tests/fixtures/ros2_bridge_replay.json")


def _load_bridge_module(monkeypatch):
    class FakeLogger:
        def info(self, _message):
            pass

        def warn(self, _message):
            pass

        def error(self, _message):
            pass

    class FakePublisher:
        def __init__(self):
            self.published = []

        def publish(self, message):
            self.published.append(message)

    class FakeParameter:
        def __init__(self, value):
            self.value = value

    class FakeClock:
        class _Now:
            @staticmethod
            def to_msg():
                return {"stamp": "fake"}

        @staticmethod
        def now():
            return FakeClock._Now()

    class FakeNode:
        def __init__(self, _name):
            self._parameters = {}
            self._logger = FakeLogger()
            self._timers = []

        def declare_parameter(self, name, value):
            self._parameters[name] = value

        def get_parameter(self, name):
            return FakeParameter(self._parameters[name])

        def create_publisher(self, _msg_type, _topic, _qos):
            return FakePublisher()

        def create_subscription(self, _msg_type, _topic, callback, _qos):
            return types.SimpleNamespace(callback=callback)

        def create_service(self, _srv_type, _name, callback):
            return types.SimpleNamespace(callback=callback)

        def create_timer(self, period, callback):
            timer = types.SimpleNamespace(period=period, callback=callback)
            self._timers.append(timer)
            return timer

        def get_clock(self):
            return FakeClock()

        def get_logger(self):
            return self._logger

    class FakeQoSProfile:
        def __init__(self, reliability=None, history=None, depth=10):
            self.reliability = reliability
            self.history = history
            self.depth = depth

    class FakeHeader:
        def __init__(self):
            self.stamp = None
            self.frame_id = ""

    class FakeJointState:
        def __init__(self):
            self.header = FakeHeader()
            self.name = []
            self.position = []
            self.velocity = []
            self.effort = []

    class FakeBatteryState:
        def __init__(self):
            self.header = FakeHeader()

    class FakeImu:
        def __init__(self):
            self.header = FakeHeader()

    class FakeString:
        def __init__(self):
            self.data = ""

    class FakeVector3:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class FakeTwist:
        def __init__(self):
            self.linear = FakeVector3()
            self.angular = FakeVector3()

    class FakeRobotState:
        def __init__(self):
            self.header = FakeHeader()
            self.battery_level = 0.0
            self.cpu_usage = 0.0
            self.temperature = 0.0
            self.status = ""

    class FakeTransformBroadcaster:
        def __init__(self, _node):
            pass

    class FakeGodotSimulationClient:
        def __init__(self, host, port):
            self.host = host
            self.port = port
            self._callback = None
            self._connected = True
            self.updated_parameters = []
            self.instruction_sets = []
            self.circuit_configs = []
            self.started = []
            self.loaded = []
            self.stopped = 0

        def set_data_callback(self, callback):
            self._callback = callback

        def connect(self, timeout=3.0):
            return True

        def is_connected(self):
            return self._connected

        def update_parameters(self, params):
            self.updated_parameters.append(params)
            return True

        def send_instruction_set(self, payload):
            self.instruction_sets.append(payload)
            return True

        def configure_simulated_circuit(self, payload):
            self.circuit_configs.append(payload)
            return True

        def start_simulation(self, robot_config):
            self.started.append(robot_config)
            return True

        def stop_simulation(self):
            self.stopped += 1

        def load_robot_config(self, parts, connections):
            self.loaded.append({"parts": parts, "connections": connections})
            return True

    monkeypatch.setitem(
        sys.modules,
        "rclpy",
        types.SimpleNamespace(
            init=lambda args=None: None,
            spin=lambda node: None,
            ok=lambda: True,
            shutdown=lambda: None,
        ),
    )
    monkeypatch.setitem(sys.modules, "rclpy.node", types.SimpleNamespace(Node=FakeNode))
    monkeypatch.setitem(
        sys.modules,
        "rclpy.qos",
        types.SimpleNamespace(
            QoSProfile=FakeQoSProfile,
            ReliabilityPolicy=types.SimpleNamespace(RELIABLE="reliable"),
            HistoryPolicy=types.SimpleNamespace(KEEP_LAST="keep_last"),
        ),
    )
    monkeypatch.setitem(
        sys.modules,
        "sensor_msgs.msg",
        types.SimpleNamespace(
            JointState=FakeJointState,
            Imu=FakeImu,
            BatteryState=FakeBatteryState,
        ),
    )
    monkeypatch.setitem(
        sys.modules,
        "geometry_msgs.msg",
        types.SimpleNamespace(Twist=FakeTwist),
    )
    monkeypatch.setitem(
        sys.modules,
        "std_msgs.msg",
        types.SimpleNamespace(String=FakeString),
    )
    monkeypatch.setitem(
        sys.modules,
        "std_srvs.srv",
        types.SimpleNamespace(Trigger=type("Trigger", (), {})),
    )
    monkeypatch.setitem(
        sys.modules,
        "tf2_ros",
        types.SimpleNamespace(TransformBroadcaster=FakeTransformBroadcaster),
    )
    monkeypatch.setitem(
        sys.modules,
        "agi_walker_msgs.msg",
        types.SimpleNamespace(RobotState=FakeRobotState),
    )
    monkeypatch.setitem(
        sys.modules,
        "agi_walker_msgs.srv",
        types.SimpleNamespace(LoadRobot=type("LoadRobot", (), {})),
    )
    monkeypatch.setitem(
        sys.modules,
        "agi_walker.core.api.comm.godot_client",
        types.SimpleNamespace(GodotSimulationClient=FakeGodotSimulationClient),
    )

    spec = importlib.util.spec_from_file_location(
        "tests.fake_ros2_bridge_module", BRIDGE_NODE_PATH
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _load_runtime_probe_module():
    spec = importlib.util.spec_from_file_location(
        "tests.fake_ros2_runtime_probe_module", RUNTIME_PROBE_PATH
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_ros2_runtime_probe_reports_missing_modules() -> None:
    module = _load_runtime_probe_module()

    def fake_import(name: str):
        if name in {"rclpy", "sensor_msgs.msg"}:
            return object()
        raise ModuleNotFoundError(name)

    report = module.probe_ros2_python_runtime(fake_import)

    assert report["available"] is False
    assert report["modules"]["rclpy"]["available"] is True
    assert report["modules"]["sensor_msgs.msg"]["available"] is True
    assert report["modules"]["tf2_ros"]["available"] is False
    assert report["missing_modules"] == [
        "tf2_ros",
        "geometry_msgs.msg",
        "std_srvs.srv",
    ]


def test_ros2_bridge_replay_payload_validation(monkeypatch) -> None:
    module = _load_bridge_module(monkeypatch)
    assert "declare_parameters" not in module.AGIWalkerROS2Bridge.__dict__

    payload = module.load_ros2_bridge_replay_payload(REPLAY_FIXTURE)
    assert module.validate_ros2_bridge_replay_payload(payload) == []

    errors = module.validate_ros2_bridge_replay_payload(
        {"schema_version": "0.0", "latest_data": {"joint_names": []}}
    )
    assert "schema_version must be '1.0'" in errors
    assert "latest_data.joint_names must be a non-empty list" in errors


def test_ros2_bridge_runtime_replay_flow(monkeypatch) -> None:
    module = _load_bridge_module(monkeypatch)
    payload = module.load_ros2_bridge_replay_payload(REPLAY_FIXTURE)

    bridge = module.AGIWalkerROS2Bridge()
    bridge.on_godot_data(payload["latest_data"])
    bridge.simulation_running = True

    twist = module.Twist()
    twist.linear.x = payload["cmd_vel"]["linear_x"]
    twist.linear.y = payload["cmd_vel"]["linear_y"]
    twist.angular.z = payload["cmd_vel"]["angular_z"]

    bridge.cmd_vel_callback(twist)
    bridge.publish_joint_states()
    bridge.publish_robot_state()

    joint_state = bridge.joint_state_pub.published[0]
    robot_state = bridge.robot_state_pub.published[0]

    assert joint_state.name == payload["latest_data"]["joint_names"]
    assert joint_state.position == payload["latest_data"]["joint_positions"]
    assert joint_state.velocity == payload["latest_data"]["joint_velocities"]
    assert joint_state.effort == payload["latest_data"]["joint_efforts"]
    assert robot_state.battery_level == payload["latest_data"]["battery"]
    assert robot_state.status == "RUNNING"
    assert bridge.godot_client.updated_parameters[-1] == {
        "cmd_linear_x": payload["cmd_vel"]["linear_x"],
        "cmd_linear_y": payload["cmd_vel"]["linear_y"],
        "cmd_angular_z": payload["cmd_vel"]["angular_z"],
    }

    response = types.SimpleNamespace(success=None, message=None)
    start_response = bridge.start_simulation_callback(None, response)
    assert start_response.success is True
    assert bridge.simulation_running is True

    stop_response = bridge.stop_simulation_callback(None, response)
    assert stop_response.success is True
    assert bridge.simulation_running is False


def test_ros2_bridge_instruction_runtime_contract(monkeypatch) -> None:
    module = _load_bridge_module(monkeypatch)
    bridge = module.AGIWalkerROS2Bridge()

    result = bridge.apply_instruction_set_payload(
        {
            "schema_version": "1.0",
            "sequence_name": "walk-demo",
            "simulated_circuit": {
                "schema_version": "1.0",
                "transport": "imc22_can_fd",
                "channel": "simulated-can0",
                "bustype": "virtual",
                "bitrate": 1_000_000,
                "control_freq_hz": 100,
                "status_rate_hz": 200,
                "command_base_id": 0x200,
                "status_base_id": 0x100,
                "config_base_id": 0x300,
                "default_compliance": 0.4,
                "joint_order": [
                    "hip_left",
                    "knee_left",
                    "hip_right",
                    "knee_right",
                ],
            },
            "steps": [
                {
                    "kind": "set_velocity",
                    "linear_x": 0.25,
                    "linear_y": 0.0,
                    "angular_z": 0.1,
                },
                {
                    "kind": "set_joint_targets",
                    "joint_targets": {"hip_left": 0.3, "knee_right": -0.2},
                    "compliance": 0.4,
                },
                {"kind": "set_pid", "pid_kp": 1.0, "pid_ki": 0.0, "pid_kd": 0.1},
            ],
        }
    )

    assert result["status"] == "applied"
    assert result["instruction_step_count"] == 3
    assert result["sent_instruction"] is True
    assert result["sent_circuit_config"] is True
    assert result["sent_compatibility_params"] is True
    assert result["compatibility_params"]["cmd_linear_x"] == 0.25
    assert result["compatibility_params"]["joint_targets"] == {
        "hip_left": 0.3,
        "knee_right": -0.2,
    }
    assert result["compatibility_params"]["pid_kp"] == 1.0
    assert result["simulated_circuit_command_batch"] == [
        {
            "node_id": 1,
            "joint_name": "hip_left",
            "target_angle": 0.3,
            "compliance": 0.4,
            "command_id": 0x201,
        },
        {
            "node_id": 4,
            "joint_name": "knee_right",
            "target_angle": -0.2,
            "compliance": 0.4,
            "command_id": 0x204,
        },
    ]
    assert bridge.godot_client.circuit_configs[-1]["transport"] == "imc22_can_fd"
    assert bridge.godot_client.instruction_sets[-1]["sequence_name"] == "walk-demo"
    assert result["simulated_circuit_feedback"]["node_ids"] == [1, 4]
    assert result["simulated_circuit_feedback"]["states"][1]["angle"] == 0.3


def test_ros2_bridge_instruction_topics_and_services(monkeypatch) -> None:
    module = _load_bridge_module(monkeypatch)
    bridge = module.AGIWalkerROS2Bridge()

    instruction_message = module.String()
    instruction_message.data = json.dumps(
        {
            "schema_version": "1.0",
            "sequence_name": "topic-demo",
            "steps": [
                {
                    "kind": "set_velocity",
                    "linear_x": 0.12,
                    "linear_y": 0.0,
                    "angular_z": 0.03,
                }
            ],
        }
    )
    bridge.instruction_set_callback(instruction_message)

    circuit_message = module.String()
    circuit_message.data = json.dumps(
        {
            "schema_version": "1.0",
            "transport": "imc22_can_fd",
            "channel": "simulated-can0",
            "bustype": "virtual",
            "bitrate": 1_000_000,
            "control_freq_hz": 100,
            "status_rate_hz": 200,
            "command_base_id": 0x200,
            "status_base_id": 0x100,
            "config_base_id": 0x300,
            "default_compliance": 0.5,
            "joint_order": [
                "hip_left",
                "knee_left",
                "hip_right",
                "knee_right",
            ],
        }
    )
    bridge.simulated_circuit_callback(circuit_message)

    replay_response = types.SimpleNamespace(success=None, message=None)
    bridge.replay_instruction_set_callback(None, replay_response)

    default_response = types.SimpleNamespace(success=None, message=None)
    bridge.apply_default_circuit_callback(None, default_response)

    assert bridge.godot_client.instruction_sets[-1]["sequence_name"] == "topic-demo"
    assert bridge.godot_client.circuit_configs[-1]["transport"] == "imc22_can_fd"
    assert replay_response.success is True
    assert default_response.success is True
    runtime_messages = bridge.instruction_runtime_pub.published
    assert len(runtime_messages) >= 3
    latest_payload = json.loads(runtime_messages[-1].data)
    assert latest_payload["event"] == "simulated_circuit_default_applied"
    assert (
        latest_payload["simulated_circuit_config"]["transport"] == "imc22_can_fd"
    )
    assert "simulated_circuit_feedback" in latest_payload
