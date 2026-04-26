#!/usr/bin/env python3
"""
AGI-Walker ROS 2 桥接节点

连接AGI-Walker仿真平台与ROS 2生态系统
提供标准的ROS 2接口访问Godot仿真
"""

import json
import rclpy
import threading
from pathlib import Path
import sys
from typing import Any, Dict, Optional

from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState, Imu, BatteryState
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

try:
    from agi_walker_msgs.msg import RobotState
    from agi_walker_msgs.srv import LoadRobot
except ImportError:
    print("Warning: agi_walker_msgs not found, some features will be limited")
    RobotState = None
    LoadRobot = None

ROS2_BRIDGE_REPLAY_SCHEMA_VERSION = "1.0"
DEFAULT_JOINT_NAMES = ["hip_left", "knee_left", "hip_right", "knee_right"]


def _add_repo_root_to_path() -> None:
    """Add the repository root when the bridge is launched from a source checkout."""
    module_path = Path("agi_walker/core/api/comm/godot_client.py")

    for parent in Path(__file__).resolve().parents:
        if (parent / module_path).exists():
            sys.path.insert(0, str(parent))
            return


def cmd_vel_to_godot_params(msg: Twist) -> Dict[str, float]:
    return {
        "cmd_linear_x": msg.linear.x,
        "cmd_linear_y": msg.linear.y,
        "cmd_angular_z": msg.angular.z,
    }


def joint_state_fields_from_latest_data(latest_data: Dict) -> Dict[str, list]:
    joint_names = latest_data.get("joint_names", DEFAULT_JOINT_NAMES)
    joint_count = len(joint_names)
    return {
        "name": list(joint_names),
        "position": list(latest_data.get("joint_positions", [0.0] * joint_count)),
        "velocity": list(latest_data.get("joint_velocities", [0.0] * joint_count)),
        "effort": list(latest_data.get("joint_efforts", [0.0] * joint_count)),
    }


def robot_state_fields_from_latest_data(
    latest_data: Dict, simulation_running: bool
) -> Dict[str, float | str]:
    return {
        "battery_level": float(latest_data.get("battery", 100.0)),
        "cpu_usage": 0.0,
        "temperature": 25.0,
        "status": "RUNNING" if simulation_running else "IDLE",
    }


def validate_ros2_bridge_replay_payload(payload: Dict) -> list[str]:
    errors: list[str] = []
    if not isinstance(payload, dict):
        return ["replay payload must be a dict"]
    if payload.get("schema_version") != ROS2_BRIDGE_REPLAY_SCHEMA_VERSION:
        errors.append(f"schema_version must be {ROS2_BRIDGE_REPLAY_SCHEMA_VERSION!r}")
    latest_data = payload.get("latest_data")
    if not isinstance(latest_data, dict):
        errors.append("latest_data must be a dict")
    else:
        joint_names = latest_data.get("joint_names", DEFAULT_JOINT_NAMES)
        if not isinstance(joint_names, list) or not joint_names:
            errors.append("latest_data.joint_names must be a non-empty list")
        for key in ["joint_positions", "joint_velocities", "joint_efforts"]:
            values = latest_data.get(key)
            if not isinstance(values, list):
                errors.append(f"latest_data.{key} must be a list")
                continue
            if len(values) != len(joint_names):
                errors.append(f"latest_data.{key} length must match joint_names")
            if not all(isinstance(value, (int, float)) for value in values):
                errors.append(f"latest_data.{key} must contain only numeric values")
        battery = latest_data.get("battery", 100.0)
        if not isinstance(battery, (int, float)):
            errors.append("latest_data.battery must be numeric")

    cmd_vel = payload.get("cmd_vel")
    if cmd_vel is not None:
        if not isinstance(cmd_vel, dict):
            errors.append("cmd_vel must be a dict when provided")
        else:
            for key in ["linear_x", "linear_y", "angular_z"]:
                if not isinstance(cmd_vel.get(key), (int, float)):
                    errors.append(f"cmd_vel.{key} must be numeric")
    instruction_set = payload.get("instruction_set")
    if instruction_set is not None:
        errors.extend(validate_instruction_set_payload(instruction_set))
    simulated_circuit = payload.get("simulated_circuit")
    if simulated_circuit is not None:
        errors.extend(validate_simulated_circuit_config(simulated_circuit))
    return errors


def load_ros2_bridge_replay_payload(source: str | Path | Dict) -> Dict:
    if isinstance(source, dict):
        payload = source
    else:
        payload = json.loads(Path(source).read_text(encoding="utf-8"))
    errors = validate_ros2_bridge_replay_payload(payload)
    if errors:
        raise ValueError("; ".join(errors))
    return payload


_add_repo_root_to_path()

try:
    from agi_walker.core.api.comm.godot_client import GodotSimulationClient
    from agi_walker.core.api.comm.instruction_control_contracts import (
        build_instruction_runtime_contract,
        default_simulated_circuit_config,
        validate_instruction_set_payload,
        validate_simulated_circuit_config,
    )
    from agi_walker.core.api.godot_robot_env.hardware_controller import (
        simulate_imc22_command_batch_feedback,
    )
except ImportError:
    print("Error: Cannot import AGI-Walker communication contracts.")
    GodotSimulationClient = None

    def default_simulated_circuit_config() -> Dict[str, Any]:
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
            "joint_order": list(DEFAULT_JOINT_NAMES),
        }

    def validate_instruction_set_payload(payload: Dict[str, Any]) -> list[str]:
        return [] if isinstance(payload, dict) else ["instruction_set must be a dict"]

    def validate_simulated_circuit_config(payload: Dict[str, Any]) -> list[str]:
        return [] if isinstance(payload, dict) else ["simulated_circuit must be a dict"]

    def build_instruction_runtime_contract(payload: Dict[str, Any]) -> Dict[str, Any]:
        return {
            "instruction_set": payload,
            "compatibility_params": {},
            "simulated_circuit": default_simulated_circuit_config(),
            "simulated_circuit_command_batch": [],
        }

    def simulate_imc22_command_batch_feedback(
        command_batch: list[Dict[str, Any]],
    ) -> Dict[str, Any]:
        return {
            "schema_version": "1.0",
            "replay_payload": {"schema_version": "1.0", "frames": []},
            "states": {},
            "node_ids": [],
        }


class AGIWalkerROS2Bridge(Node):
    """AGI-Walker ROS 2 桥接节点"""

    def __init__(self):
        super().__init__("agi_walker_bridge")

        # 声明参数
        self._declare_bridge_parameters()

        # Godot客户端
        self.godot_client: Optional[GodotSimulationClient] = None
        self.latest_data: Dict = {}
        self.simulation_running = False
        self.latest_instruction_runtime: Optional[Dict[str, Any]] = None
        self.latest_instruction_result: Optional[Dict[str, Any]] = None
        self.latest_simulated_feedback: Optional[Dict[str, Any]] = None
        self.simulated_circuit_config: Dict[str, Any] = default_simulated_circuit_config()

        # 连接到Godot
        self.connect_to_godot()

        # QoS配置
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # 设置发布器
        self.setup_publishers()

        # 设置订阅器
        self.setup_subscribers()

        # 设置服务
        self.setup_services()

        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)

        # 设置定时器
        self.setup_timers()

        self.get_logger().info("✅ AGI-Walker ROS 2 Bridge initialized")

    def _declare_bridge_parameters(self):
        """声明所有ROS参数"""
        # 连接参数
        self.declare_parameter("godot_host", "127.0.0.1")
        self.declare_parameter("godot_port", 9999)
        self.declare_parameter("reconnect_timeout", 5.0)

        # 发布频率
        self.declare_parameter("joint_state_rate", 50.0)
        self.declare_parameter("robot_state_rate", 20.0)
        self.declare_parameter("tf_rate", 50.0)

        # 控制参数
        self.declare_parameter("motor_power_multiplier", 1.0)
        self.declare_parameter("joint_stiffness", 1.0)
        self.declare_parameter("joint_damping", 0.5)
        self.declare_parameter("balance_gain", 1.0)

        # PID参数
        self.declare_parameter("pid_kp", 1.0)
        self.declare_parameter("pid_ki", 0.0)
        self.declare_parameter("pid_kd", 0.1)

    def connect_to_godot(self):
        """连接到Godot仿真服务器"""
        if GodotSimulationClient is None:
            self.get_logger().error("❌ GodotSimulationClient not available")
            return

        host = self.get_parameter("godot_host").value
        port = self.get_parameter("godot_port").value

        self.godot_client = GodotSimulationClient(host, port)
        self.godot_client.set_data_callback(self.on_godot_data)

        # 在后台线程尝试连接
        def try_connect():
            if self.godot_client.connect(timeout=3.0):
                self.get_logger().info(f"✅ Connected to Godot at {host}:{port}")
            else:
                self.get_logger().warn(
                    f"⚠️  Failed to connect to Godot at {host}:{port}"
                )
                self.get_logger().info(
                    "Bridge is running. Will retry connection when services are called."
                )

        connect_thread = threading.Thread(target=try_connect, daemon=True)
        connect_thread.start()

    def setup_publishers(self):
        """设置所有发布器"""
        self.joint_state_pub = self.create_publisher(
            JointState, "/joint_states", self.qos_profile
        )

        if RobotState is not None:
            self.robot_state_pub = self.create_publisher(
                RobotState, "/robot_state", self.qos_profile
            )
        else:
            self.robot_state_pub = None
            self.get_logger().warn(
                "RobotState publisher not available (custom msgs not built)"
            )

        self.battery_pub = self.create_publisher(BatteryState, "/battery", 10)

        self.imu_pub = self.create_publisher(Imu, "/imu", self.qos_profile)
        self.instruction_runtime_pub = self.create_publisher(
            String, "/instruction_runtime/json", 10
        )

        self.get_logger().info("Publishers initialized")

    def setup_subscribers(self):
        """设置所有订阅器"""
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )
        self.instruction_set_sub = self.create_subscription(
            String, "/instruction_set/json", self.instruction_set_callback, 10
        )
        self.simulated_circuit_sub = self.create_subscription(
            String,
            "/simulated_circuit/json",
            self.simulated_circuit_callback,
            10,
        )

        self.get_logger().info("Subscribers initialized")

    def setup_services(self):
        """设置所有服务"""
        self.start_sim_srv = self.create_service(
            Trigger, "/start_simulation", self.start_simulation_callback
        )

        self.stop_sim_srv = self.create_service(
            Trigger, "/stop_simulation", self.stop_simulation_callback
        )

        if LoadRobot is not None:
            self.load_robot_srv = self.create_service(
                LoadRobot, "/load_robot", self.load_robot_callback
            )
        else:
            self.load_robot_srv = None
            self.get_logger().warn(
                "LoadRobot service not available (custom msgs not built)"
            )

        self.replay_instruction_srv = self.create_service(
            Trigger,
            "/instruction_set/replay_last",
            self.replay_instruction_set_callback,
        )
        self.apply_default_circuit_srv = self.create_service(
            Trigger,
            "/simulated_circuit/apply_default",
            self.apply_default_circuit_callback,
        )

        self.get_logger().info("Services initialized")

    def setup_timers(self):
        """设置定时发布器"""
        joint_rate = self.get_parameter("joint_state_rate").value
        if joint_rate > 0:
            self.joint_timer = self.create_timer(
                1.0 / joint_rate, self.publish_joint_states
            )

        state_rate = self.get_parameter("robot_state_rate").value
        if state_rate > 0 and self.robot_state_pub is not None:
            self.state_timer = self.create_timer(
                1.0 / state_rate, self.publish_robot_state
            )

        self.get_logger().info("Timers initialized")

    def on_godot_data(self, data: Dict):
        """接收Godot数据回调"""
        self.latest_data = data

    def publish_joint_states(self):
        """发布关节状态"""
        if not self.latest_data:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        fields = joint_state_fields_from_latest_data(self.latest_data)
        msg.name = fields["name"]
        msg.position = fields["position"]
        msg.velocity = fields["velocity"]
        msg.effort = fields["effort"]

        self.joint_state_pub.publish(msg)

    def publish_robot_state(self):
        """发布机器人整体状态"""
        if not self.latest_data or self.robot_state_pub is None:
            return

        msg = RobotState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        fields = robot_state_fields_from_latest_data(
            self.latest_data, self.simulation_running
        )
        msg.battery_level = fields["battery_level"]
        msg.cpu_usage = fields["cpu_usage"]
        msg.temperature = fields["temperature"]
        msg.status = fields["status"]

        self.robot_state_pub.publish(msg)

    def cmd_vel_callback(self, msg: Twist):
        """速度命令回调"""
        if self.godot_client and self.godot_client.is_connected():
            params = cmd_vel_to_godot_params(msg)
            self.godot_client.update_parameters(params)
            self.get_logger().info(
                f"Sent cmd_vel: linear_x={msg.linear.x:.2f}, angular_z={msg.angular.z:.2f}"
            )
        else:
            self.get_logger().warn("Cannot send cmd_vel: not connected to Godot")

    def instruction_set_callback(self, msg: String) -> None:
        """结构化指令集 topic 回调。"""
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Invalid instruction_set JSON: {exc}")
            return

        errors = validate_instruction_set_payload(payload)
        if errors:
            self.get_logger().error(
                f"Invalid instruction_set payload: {'; '.join(errors)}"
            )
            return

        result = self.apply_instruction_set_payload(payload)
        self.publish_instruction_runtime("instruction_set_applied", result)

    def simulated_circuit_callback(self, msg: String) -> None:
        """模拟电路配置 topic 回调。"""
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Invalid simulated_circuit JSON: {exc}")
            return

        errors = validate_simulated_circuit_config(payload)
        if errors:
            self.get_logger().error(
                f"Invalid simulated_circuit payload: {'; '.join(errors)}"
            )
            return

        self.simulated_circuit_config = payload
        sent = False
        if self.godot_client and self.godot_client.is_connected():
            if hasattr(self.godot_client, "configure_simulated_circuit"):
                sent = self.godot_client.configure_simulated_circuit(payload)
        result = {
            "status": "applied",
            "simulated_circuit": payload,
            "sent_circuit_config": sent,
        }
        self.publish_instruction_runtime("simulated_circuit_configured", result)

    def apply_instruction_set_payload(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Apply a structured ROS2/Godot instruction-set payload in simulation mode."""
        runtime_contract = build_instruction_runtime_contract(payload)
        self.latest_instruction_runtime = runtime_contract
        self.simulated_circuit_config = runtime_contract["simulated_circuit"]

        sent_instruction = False
        sent_circuit_config = False
        sent_compatibility_params = False
        if self.godot_client and self.godot_client.is_connected():
            if hasattr(self.godot_client, "configure_simulated_circuit"):
                sent_circuit_config = self.godot_client.configure_simulated_circuit(
                    runtime_contract["simulated_circuit"]
                )
            if hasattr(self.godot_client, "send_instruction_set"):
                sent_instruction = self.godot_client.send_instruction_set(
                    runtime_contract["instruction_set"]
                )
            compatibility_params = runtime_contract["compatibility_params"]
            if compatibility_params:
                sent_compatibility_params = self.godot_client.update_parameters(
                    compatibility_params
                )

        result = {
            "status": "applied",
            "instruction_step_count": len(runtime_contract["instruction_set"]["steps"]),
            "simulated_circuit": runtime_contract["simulated_circuit"],
            "simulated_circuit_command_batch": runtime_contract[
                "simulated_circuit_command_batch"
            ],
            "compatibility_params": runtime_contract["compatibility_params"],
            "sent_instruction": sent_instruction,
            "sent_circuit_config": sent_circuit_config,
            "sent_compatibility_params": sent_compatibility_params,
        }
        if runtime_contract["simulated_circuit_command_batch"]:
            replay_feedback = simulate_imc22_command_batch_feedback(
                runtime_contract["simulated_circuit_command_batch"]
            )
            result["simulated_circuit_feedback"] = replay_feedback
            result["hardware_fault_summary"] = replay_feedback.get(
                "fault_telemetry_report",
                {},
            ).get("fault_summary", {})
            self.latest_simulated_feedback = replay_feedback
        self.latest_instruction_result = result
        return result

    def publish_instruction_runtime(
        self, event: str, result: Dict[str, Any]
    ) -> None:
        """发布结构化指令运行态快照。"""
        message = String()
        message.data = json.dumps(
            {
                "event": event,
                "instruction_runtime": self.latest_instruction_runtime,
                "latest_result": result,
                "simulated_circuit_config": self.simulated_circuit_config,
                "simulated_circuit_feedback": self.latest_simulated_feedback,
                "hardware_fault_summary": (
                    self.latest_simulated_feedback or {}
                ).get("fault_telemetry_report", {}).get("fault_summary", {}),
            }
        )
        self.instruction_runtime_pub.publish(message)

    def replay_instruction_set_callback(self, request, response):
        """重放最近一次结构化指令集。"""
        if not self.latest_instruction_runtime:
            response.success = False
            response.message = "❌ No instruction_set payload has been applied yet"
            return response

        result = self.apply_instruction_set_payload(
            self.latest_instruction_runtime["instruction_set"]
        )
        self.publish_instruction_runtime("instruction_set_replayed", result)
        response.success = True
        response.message = "✅ Replayed latest instruction_set payload"
        return response

    def apply_default_circuit_callback(self, request, response):
        """应用默认模拟电路配置。"""
        self.simulated_circuit_config = default_simulated_circuit_config()
        sent = False
        if self.godot_client and self.godot_client.is_connected():
            if hasattr(self.godot_client, "configure_simulated_circuit"):
                sent = self.godot_client.configure_simulated_circuit(
                    self.simulated_circuit_config
                )
        self.publish_instruction_runtime(
            "simulated_circuit_default_applied",
            {
                "status": "applied",
                "simulated_circuit": self.simulated_circuit_config,
                "sent_circuit_config": sent,
            },
        )
        response.success = True
        response.message = "✅ Applied default simulated circuit config"
        return response

    def start_simulation_callback(self, request, response):
        """启动仿真服务"""
        if self.godot_client and self.godot_client.is_connected():
            robot_config = {}  # 使用默认配置
            success = self.godot_client.start_simulation(robot_config)
            response.success = success
            response.message = (
                "✅ Simulation started" if success else "❌ Failed to start simulation"
            )
            self.simulation_running = success
        else:
            response.success = False
            response.message = "❌ Not connected to Godot"

        self.get_logger().info(response.message)
        return response

    def stop_simulation_callback(self, request, response):
        """停止仿真服务"""
        if self.godot_client:
            self.godot_client.stop_simulation()
            response.success = True
            response.message = "✅ Simulation stopped"
            self.simulation_running = False
        else:
            response.success = False
            response.message = "❌ No Godot client available"

        self.get_logger().info(response.message)
        return response

    def load_robot_callback(self, request, response):
        """加载机器人服务"""
        # 转换ROS消息为Godot格式
        parts = [
            {
                "id": p.part_id,
                "type": p.part_type,
                "model": p.model,
                "position": [p.position.x, p.position.y, p.position.z],
                "parameters": list(p.parameters),
            }
            for p in request.parts
        ]

        connections = [
            {"from": c.from_part, "to": c.to_part, "type": c.connection_type}
            for c in request.connections
        ]

        if self.godot_client and self.godot_client.is_connected():
            success = self.godot_client.load_robot_config(parts, connections)
            response.success = success
            response.message = (
                f'✅ Robot "{request.robot_name}" loaded'
                if success
                else "❌ Failed to load robot"
            )
        else:
            response.success = False
            response.message = "❌ Not connected to Godot"

        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)

    try:
        node = AGIWalkerROS2Bridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
