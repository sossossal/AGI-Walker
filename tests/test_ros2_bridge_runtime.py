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
BAG_REPLAY_FIXTURE = Path("tests/fixtures/ros2_bridge_bag_replay.json")


def _fake_default_simulated_circuit_config() -> dict[str, object]:
    return {
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
        "joint_order": ["hip_left", "knee_left", "hip_right", "knee_right"],
    }


def _fake_validate_instruction_set_payload(payload: dict[str, object]) -> list[str]:
    if not isinstance(payload, dict):
        return ["instruction_set payload must be a dict"]
    if payload.get("schema_version") != "1.0":
        return ["schema_version must be '1.0'"]
    if not payload.get("sequence_name"):
        return ["sequence_name must be a non-empty string"]
    steps = payload.get("steps")
    if not isinstance(steps, list) or not steps:
        return ["steps must be a non-empty list"]
    return []


def _fake_validate_simulated_circuit_config(payload: dict[str, object]) -> list[str]:
    if not isinstance(payload, dict):
        return ["simulated circuit config must be a dict"]
    for key in ["schema_version", "transport", "channel", "bustype"]:
        if not payload.get(key):
            return [f"simulated_circuit.{key} must be set"]
    return []


def _fake_instruction_runtime_contract(payload: dict[str, object]) -> dict[str, object]:
    simulated_circuit = _fake_default_simulated_circuit_config()
    simulated_circuit.update(payload.get("simulated_circuit") or {})
    compatibility_params: dict[str, object] = {}
    command_batch: list[dict[str, object]] = []
    for step in payload["steps"]:
        if step["kind"] == "set_velocity":
            compatibility_params.update(
                {
                    "cmd_linear_x": step["linear_x"],
                    "cmd_linear_y": step["linear_y"],
                    "cmd_angular_z": step["angular_z"],
                }
            )
        if step["kind"] == "set_pid":
            compatibility_params.update(
                {
                    "pid_kp": step["pid_kp"],
                    "pid_ki": step["pid_ki"],
                    "pid_kd": step["pid_kd"],
                }
            )
        if step["kind"] == "set_joint_targets":
            compatibility_params["joint_targets"] = step["joint_targets"]
            for index, joint_name in enumerate(simulated_circuit["joint_order"], start=1):
                if joint_name in step["joint_targets"]:
                    command_batch.append(
                        {
                            "node_id": index,
                            "joint_name": joint_name,
                            "target_angle": step["joint_targets"][joint_name],
                            "compliance": step.get("compliance", 0.5),
                            "command_id": simulated_circuit["command_base_id"] + index,
                        }
                    )
    normalized = dict(payload)
    normalized["simulated_circuit"] = simulated_circuit
    return {
        "instruction_set": normalized,
        "compatibility_params": compatibility_params,
        "simulated_circuit": simulated_circuit,
        "simulated_circuit_command_batch": command_batch,
    }


def _fake_simulate_feedback(command_batch: list[dict[str, object]]) -> dict[str, object]:
    states = {
        int(command["node_id"]): {
            "node_id": int(command["node_id"]),
            "angle": float(command["target_angle"]),
            "current": 0.1,
            "error": 0.0,
        }
        for command in command_batch
    }
    return {
        "schema_version": "1.0",
        "replay_payload": {"schema_version": "1.0", "frames": []},
        "states": states,
        "node_ids": sorted(states),
        "fault_telemetry_report": {
            "entries": [
                {
                    "node_id": node_id,
                    "raw_error": state["error"],
                    "fault_class": "ok",
                }
                for node_id, state in states.items()
            ],
            "fault_summary": {"fault_counts": {}, "per_node": {}},
        },
    }


class _FakeRecoveryController:
    def __init__(self, feedback: dict[str, object]):
        self.feedback = feedback

    def build_recovery_plan(self) -> dict[str, object]:
        fault_summary = self.feedback["fault_telemetry_report"]["fault_summary"]
        actions = [
            {"node_id": int(node_id), "fault_class": node["fault_class"], "action": "recover_hold_position"}
            for node_id, node in fault_summary.get("per_node", {}).items()
        ]
        return {"status": "ready", "actions": actions, "fault_summary": fault_summary}

    def recover_by_fault_class(self) -> dict[str, object]:
        return {
            "status": "applied",
            "recovery_plan": self.build_recovery_plan(),
            "safety_status": {
                "state": "ready",
                "watchdog_tripped": False,
                "fault_summary": self.feedback["fault_telemetry_report"]["fault_summary"],
            },
        }

    def clear_faults(self) -> dict[str, object]:
        return {
            "state": "ready",
            "watchdog_tripped": False,
            "fault_summary": {"fault_counts": {}, "per_node": {}},
        }


def _fake_controller_from_feedback(feedback: dict[str, object]) -> _FakeRecoveryController:
    return _FakeRecoveryController(feedback)


def _fake_recovery_plan_summary(plan: dict[str, object]) -> dict[str, object]:
    return {"status": plan.get("status", "unknown"), "action_count": len(plan.get("actions", []))}


def _fake_recovery_result_summary(result: dict[str, object]) -> dict[str, object]:
    return {"status": result.get("status", "unknown")}


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

    class FakeInstructionStep:
        def __init__(self):
            self.kind = ""
            self.linear_x = 0.0
            self.linear_y = 0.0
            self.angular_z = 0.0
            self.target_angle = 0.0
            self.compliance = 0.0
            self.payload_json = ""

    class FakeInstructionSet:
        def __init__(self):
            self.header = FakeHeader()
            self.schema_version = "1.0"
            self.sequence_name = ""
            self.steps = []
            self.metadata_json = ""

    class FakeSimulatedCircuit:
        def __init__(self):
            self.header = FakeHeader()
            self.schema_version = "1.0"
            self.transport = ""
            self.channel = ""
            self.bustype = ""
            self.bitrate = 0
            self.control_freq_hz = 0.0
            self.status_rate_hz = 0.0
            self.config_json = ""

    class FakeBehaviorCommand:
        def __init__(self):
            self.header = FakeHeader()
            self.behavior_name = ""
            self.gait = ""
            self.cadence_hz = 0.0
            self.stride_scale = 0.0
            self.target_posture = ""
            self.balance_gain = 0.0
            self.command_json = ""

    class FakeNavigationGoal:
        def __init__(self):
            self.header = FakeHeader()
            self.frame_id = ""
            self.target_x = 0.0
            self.target_y = 0.0
            self.target_yaw = 0.0
            self.max_linear_speed = 0.0
            self.max_angular_speed = 0.0
            self.goal_json = ""

    class FakePerceptionSnapshot:
        def __init__(self):
            self.header = FakeHeader()
            self.frame_id = ""
            self.obstacle_count = 0
            self.nearest_obstacle_m = 0.0
            self.battery_level = 0.0
            self.summary_json = ""

    class FakeRecoveryStatus:
        def __init__(self):
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
        types.SimpleNamespace(
            RobotState=FakeRobotState,
            InstructionStep=FakeInstructionStep,
            InstructionSet=FakeInstructionSet,
            SimulatedCircuit=FakeSimulatedCircuit,
            BehaviorCommand=FakeBehaviorCommand,
            NavigationGoal=FakeNavigationGoal,
            PerceptionSnapshot=FakePerceptionSnapshot,
            HardwareRecoveryStatus=FakeRecoveryStatus,
        ),
    )
    monkeypatch.setitem(
        sys.modules,
        "agi_walker_msgs.srv",
        types.SimpleNamespace(
            LoadRobot=type("LoadRobot", (), {}),
            ApplyInstructionSet=type("ApplyInstructionSet", (), {}),
            ConfigureSimulatedCircuit=type("ConfigureSimulatedCircuit", (), {}),
            HardwareRecovery=type("HardwareRecovery", (), {}),
        ),
    )
    monkeypatch.setitem(
        sys.modules,
        "agi_walker.core.api.comm.godot_client",
        types.SimpleNamespace(GodotSimulationClient=FakeGodotSimulationClient),
    )
    monkeypatch.setitem(
        sys.modules,
        "agi_walker.core.api.comm.instruction_control_contracts",
        types.SimpleNamespace(
            build_instruction_runtime_contract=_fake_instruction_runtime_contract,
            default_simulated_circuit_config=_fake_default_simulated_circuit_config,
            validate_instruction_set_payload=_fake_validate_instruction_set_payload,
            validate_simulated_circuit_config=_fake_validate_simulated_circuit_config,
        ),
    )
    monkeypatch.setitem(
        sys.modules,
        "agi_walker.core.api.godot_robot_env.hardware_controller",
        types.SimpleNamespace(
            build_imc22_recovery_plan_summary=_fake_recovery_plan_summary,
            build_imc22_recovery_result_summary=_fake_recovery_result_summary,
            build_imc22_controller_from_feedback=_fake_controller_from_feedback,
            simulate_imc22_command_batch_feedback=_fake_simulate_feedback,
        ),
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


def test_ros2_bridge_bag_replay_payload_validation(monkeypatch) -> None:
    module = _load_bridge_module(monkeypatch)
    payload = module.load_ros2_bridge_bag_replay_payload(BAG_REPLAY_FIXTURE)

    assert module.validate_ros2_bridge_bag_replay_payload(payload) == []
    errors = module.validate_ros2_bridge_bag_replay_payload(
        {
            "schema_version": "1.0",
            "messages": [{"topic": "/unknown", "timestamp_sec": -1, "payload": {}}],
        }
    )
    assert "messages[0].topic must be one of" in errors[0]
    assert "messages[0].timestamp_sec must be non-negative" in errors


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


def test_ros2_bridge_bag_replay_flow(monkeypatch) -> None:
    module = _load_bridge_module(monkeypatch)
    bridge = module.AGIWalkerROS2Bridge()
    payload = module.load_ros2_bridge_bag_replay_payload(BAG_REPLAY_FIXTURE)

    result = bridge.apply_bag_replay_payload(payload)

    assert result["status"] == "applied"
    assert result["message_count"] == 3
    assert [item["topic"] for item in result["applied_messages"]] == [
        "/cmd_vel",
        "/simulated_circuit/json",
        "/instruction_set/json",
    ]
    assert bridge.godot_client.updated_parameters[-1]["cmd_linear_x"] == 0.15
    assert bridge.godot_client.circuit_configs[-1]["channel"] == "simulated-can0"
    assert bridge.godot_client.instruction_sets[-1]["sequence_name"] == "bag-replay-demo"
    runtime_messages = [
        json.loads(item.data) for item in bridge.instruction_runtime_pub.published
    ]
    assert runtime_messages[-1]["event"] == "bag_replay_applied"


def test_ros2_bridge_multi_node_smoke_report(monkeypatch) -> None:
    module = _load_bridge_module(monkeypatch)
    bridge = module.AGIWalkerROS2Bridge()
    payload = module.load_ros2_bridge_bag_replay_payload(BAG_REPLAY_FIXTURE)
    bridge.apply_bag_replay_payload(payload)

    report = bridge.build_multi_node_smoke_report(expected_node_ids=[1, 4])

    assert report["status"] == "passed"
    assert report["node_ids"] == [1, 4]
    assert report["sequence_name"] == "bag-replay-demo"
    assert all(check["status"] == "passed" for check in report["checks"])
    runtime_messages = [
        json.loads(item.data) for item in bridge.instruction_runtime_pub.published
    ]
    assert runtime_messages[-1]["event"] == "multi_node_smoke_reported"


def test_ros2_bridge_multi_node_smoke_report_flags_missing_nodes(monkeypatch) -> None:
    module = _load_bridge_module(monkeypatch)
    bridge = module.AGIWalkerROS2Bridge()
    payload = module.load_ros2_bridge_bag_replay_payload(BAG_REPLAY_FIXTURE)
    bridge.apply_bag_replay_payload(payload)

    report = bridge.build_multi_node_smoke_report(expected_node_ids=[1, 2, 4])

    assert report["status"] == "failed"
    expected_nodes_check = next(
        check for check in report["checks"] if check["id"] == "expected_nodes"
    )
    assert expected_nodes_check["missing_nodes"] == [2]


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
    assert "fault_telemetry_report" in result["simulated_circuit_feedback"]
    assert "hardware_fault_summary" in result


def test_ros2_bridge_typed_instruction_and_circuit_callbacks(monkeypatch) -> None:
    module = _load_bridge_module(monkeypatch)
    bridge = module.AGIWalkerROS2Bridge()

    step = module.InstructionStep()
    step.kind = "set_velocity"
    step.linear_x = 0.3
    step.linear_y = 0.0
    step.angular_z = 0.1
    instruction = module.InstructionSet()
    instruction.sequence_name = "typed-demo"
    instruction.steps = [step]

    circuit = module.SimulatedCircuit()
    circuit.transport = "imc22_can_fd"
    circuit.channel = "simulated-can0"
    circuit.bustype = "virtual"
    circuit.bitrate = 1_000_000
    circuit.control_freq_hz = 100.0
    circuit.status_rate_hz = 200.0
    circuit.config_json = json.dumps(
        {
            "command_base_id": 0x200,
            "status_base_id": 0x100,
            "config_base_id": 0x300,
            "default_compliance": 0.5,
            "joint_order": ["hip_left", "knee_left"],
        }
    )

    bridge.typed_instruction_set_callback(instruction)
    bridge.typed_simulated_circuit_callback(circuit)

    assert bridge.godot_client.instruction_sets[-1]["sequence_name"] == "typed-demo"
    assert bridge.godot_client.circuit_configs[-1]["channel"] == "simulated-can0"
    runtime_messages = [
        json.loads(item.data) for item in bridge.instruction_runtime_pub.published
    ]
    assert any(item["event"] == "typed_instruction_set_applied" for item in runtime_messages)
    assert any(
        item["event"] == "typed_simulated_circuit_configured"
        for item in runtime_messages
    )


def test_ros2_bridge_typed_services(monkeypatch) -> None:
    module = _load_bridge_module(monkeypatch)
    bridge = module.AGIWalkerROS2Bridge()

    step = module.InstructionStep()
    step.kind = "set_velocity"
    step.linear_x = 0.1
    step.linear_y = 0.0
    step.angular_z = 0.0
    instruction = module.InstructionSet()
    instruction.sequence_name = "typed-service"
    instruction.steps = [step]
    instruction_request = types.SimpleNamespace(instruction_set=instruction)
    instruction_response = types.SimpleNamespace(
        success=False, message="", result_json=""
    )

    circuit = module.SimulatedCircuit()
    circuit.transport = "imc22_can_fd"
    circuit.channel = "simulated-can0"
    circuit.bustype = "virtual"
    circuit.bitrate = 1_000_000
    circuit.control_freq_hz = 100.0
    circuit.status_rate_hz = 200.0
    circuit.config_json = json.dumps(
        {
            "command_base_id": 0x200,
            "status_base_id": 0x100,
            "config_base_id": 0x300,
            "default_compliance": 0.5,
            "joint_order": ["hip_left", "knee_left"],
        }
    )
    circuit_request = types.SimpleNamespace(simulated_circuit=circuit)
    circuit_response = types.SimpleNamespace(success=False, message="", result_json="")

    bridge.apply_instruction_set_callback(instruction_request, instruction_response)
    bridge.configure_simulated_circuit_callback(circuit_request, circuit_response)

    assert instruction_response.success is True
    assert json.loads(instruction_response.result_json)["instruction_step_count"] == 1
    assert circuit_response.success is True
    assert (
        json.loads(circuit_response.result_json)["simulated_circuit"]["bitrate"]
        == 1_000_000
    )


def test_ros2_bridge_behavior_navigation_perception_topics(monkeypatch) -> None:
    module = _load_bridge_module(monkeypatch)
    bridge = module.AGIWalkerROS2Bridge()

    behavior = module.BehaviorCommand()
    behavior.behavior_name = "patrol"
    behavior.gait = "crawl"
    behavior.cadence_hz = 1.2
    behavior.stride_scale = 0.8

    navigation = module.NavigationGoal()
    navigation.target_x = 2.0
    navigation.target_y = -0.4
    navigation.target_yaw = 1.0
    navigation.max_linear_speed = 0.3
    navigation.max_angular_speed = 0.5

    perception = module.PerceptionSnapshot()
    perception.frame_id = "front_camera"
    perception.obstacle_count = 2
    perception.nearest_obstacle_m = 1.4
    perception.battery_level = 86.0

    bridge.behavior_command_callback(behavior)
    bridge.navigation_goal_callback(navigation)
    bridge.perception_snapshot_callback(perception)

    assert bridge.godot_client.instruction_sets[-1]["sequence_name"] == "patrol"
    assert bridge.godot_client.instruction_sets[-1]["steps"][0]["kind"] == "set_gait"
    assert bridge.godot_client.updated_parameters[-1] == {
        "cmd_linear_x": 0.3,
        "cmd_linear_y": -0.3,
        "cmd_angular_z": 0.5,
        "nav_goal_x": 2.0,
        "nav_goal_y": -0.4,
        "nav_goal_yaw": 1.0,
    }
    assert bridge.latest_data["battery"] == 86.0
    assert bridge.latest_data["perception"]["obstacle_count"] == 2
    runtime_messages = [
        json.loads(item.data) for item in bridge.instruction_runtime_pub.published
    ]
    assert any(item["event"] == "behavior_command_applied" for item in runtime_messages)
    assert any(item["event"] == "navigation_goal_applied" for item in runtime_messages)
    assert any(item["event"] == "perception_snapshot_applied" for item in runtime_messages)


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
    assert "hardware_fault_summary" in latest_payload


def test_ros2_bridge_hardware_recovery_services(monkeypatch) -> None:
    module = _load_bridge_module(monkeypatch)
    bridge = module.AGIWalkerROS2Bridge()

    bridge.apply_instruction_set_payload(
        {
            "schema_version": "1.0",
            "sequence_name": "fault-demo",
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
                    "kind": "set_joint_targets",
                    "joint_targets": {"hip_left": 1.2},
                    "compliance": 0.4,
                }
            ],
        }
    )
    assert bridge.latest_simulated_feedback is not None
    bridge.latest_simulated_feedback["states"][1]["error"] = 11.0
    bridge.latest_simulated_feedback["fault_telemetry_report"]["entries"][0][
        "raw_error_value"
    ] = 11.0
    bridge.latest_simulated_feedback["fault_telemetry_report"]["entries"][0][
        "fault_class"
    ] = "overload"
    bridge.latest_simulated_feedback["fault_telemetry_report"]["fault_summary"] = {
        "schema_version": "1.0",
        "fault_table_schema_version": "1.0",
        "fault_vendor": "imc22_reflex",
        "fault_counts": {"overload": 1},
        "per_node": {1: {"fault_class": "overload", "error": 11.0, "angle": 1.2, "current": 1.26}},
        "watchdog_tripped": False,
    }

    plan_response = types.SimpleNamespace(success=None, message=None)
    bridge.recovery_plan_callback(None, plan_response)
    recover_response = types.SimpleNamespace(success=None, message=None)
    bridge.recover_by_fault_class_callback(None, recover_response)
    clear_response = types.SimpleNamespace(success=None, message=None)
    bridge.clear_faults_callback(None, clear_response)

    assert plan_response.success is True
    assert recover_response.success is True
    assert clear_response.success is True

    plan_payload = json.loads(plan_response.message)
    recover_payload = json.loads(recover_response.message)
    clear_payload = json.loads(clear_response.message)
    assert plan_payload["status"] == "success"
    assert plan_payload["hardware_recovery_plan_summary"]["action_count"] == 1
    assert recover_payload["status"] == "success"
    assert recover_payload["hardware_recovery_result_summary"]["status"] == "applied"
    assert clear_payload["status"] == "success"
    assert clear_payload["hardware_clear_result_summary"]["status"] == "success"

    runtime_messages = [json.loads(item.data) for item in bridge.instruction_runtime_pub.published]
    assert any(item["event"] == "hardware_recovery_plan_built" for item in runtime_messages)
    assert any(item["event"] == "hardware_faults_recovered" for item in runtime_messages)
    assert any(item["event"] == "hardware_faults_cleared" for item in runtime_messages)
    assert any("hardware_recovery_plan_summary" in item["latest_result"] for item in runtime_messages if item["event"] == "hardware_recovery_plan_built")
    assert any("hardware_recovery_result_summary" in item["latest_result"] for item in runtime_messages if item["event"] == "hardware_faults_recovered")
    assert any("hardware_clear_result_summary" in item["latest_result"] for item in runtime_messages if item["event"] == "hardware_faults_cleared")
