from __future__ import annotations

import importlib.util
import json
import os
import socket
import struct
import threading
import time
from pathlib import Path
from typing import Any

import pytest

pytestmark = [pytest.mark.integration, pytest.mark.live]

ENABLE_ENV_VAR = "AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE"
ARTIFACT_DIR_ENV = "AGI_WALKER_ROS2_BRIDGE_SMOKE_ARTIFACT_DIR"
DEFAULT_ARTIFACT_DIR = Path("test_env") / "ros2_bridge_smoke"
BRIDGE_NODE_PATH = Path(
    "hardware/ros2_ws/src/agi_walker_ros2/agi_walker_ros2/bridge_node.py"
)
RUNTIME_PROBE_PATH = Path(
    "hardware/ros2_ws/src/agi_walker_ros2/agi_walker_ros2/runtime_probe.py"
)


def _reserve_free_tcp_port() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(("127.0.0.1", 0))
    _, port = sock.getsockname()
    sock.close()
    return port


def _artifact_dir() -> Path:
    configured = os.getenv(ARTIFACT_DIR_ENV, "").strip()
    target = Path(configured) if configured else DEFAULT_ARTIFACT_DIR
    target.mkdir(parents=True, exist_ok=True)
    return target


def _write_report(artifact_dir: Path, report: dict[str, Any]) -> None:
    report_path = artifact_dir / "ros2_bridge_smoke_report.json"
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def _base_report(artifact_dir: Path, port: int) -> dict[str, Any]:
    return {
        "test_name": "test_ros2_bridge_humble_smoke",
        "bridge_module": str(BRIDGE_NODE_PATH),
        "runtime_probe_module": str(RUNTIME_PROBE_PATH),
        "artifact_dir": str(artifact_dir.resolve()),
        "godot_port": port,
        "status": "starting",
        "failure_stage": None,
        "message": None,
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "finished_at": None,
        "command_names": [],
        "received_joint_state_count": 0,
        "joint_state_names": [],
        "joint_state_positions": [],
        "cmd_vel_sent": None,
        "start_response": None,
        "stop_response": None,
        "ros2_runtime": None,
    }


def _record_skip(artifact_dir: Path, report: dict[str, Any], reason: str) -> None:
    report["status"] = "skipped"
    report["failure_stage"] = "gating"
    report["message"] = reason
    report["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    _write_report(artifact_dir, report)
    pytest.skip(reason)


def _recv_exactly(sock: socket.socket, size: int) -> bytes | None:
    data = b""
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            return None
        data += chunk
    return data


def _send_message(sock: socket.socket, payload: dict[str, Any]) -> None:
    body = json.dumps(payload).encode("utf-8")
    sock.sendall(struct.pack("<I", len(body)))
    sock.sendall(body)


class RecordingGodotServer:
    def __init__(self, port: int):
        self.port = port
        self._ready = threading.Event()
        self._client_connected = threading.Event()
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None
        self.commands: list[dict[str, Any]] = []
        self.telemetry = {
            "joint_names": ["hip_left", "knee_left", "hip_right", "knee_right"],
            "joint_positions": [0.1, -0.2, 0.3, -0.4],
            "joint_velocities": [0.0, 0.1, 0.0, -0.1],
            "joint_efforts": [1.0, 1.1, 0.9, 1.2],
            "battery": 97.5,
        }

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        if not self._ready.wait(timeout=3.0):
            raise AssertionError("mock Godot server did not become ready")

    def stop(self) -> None:
        self._running = False
        try:
            with socket.create_connection(("127.0.0.1", self.port), timeout=0.2):
                pass
        except OSError:
            pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def command_names(self) -> list[str]:
        with self._lock:
            return [str(item.get("command")) for item in self.commands]

    def wait_for_client(self, timeout: float = 5.0) -> bool:
        return self._client_connected.wait(timeout=timeout)

    def get_command(self, name: str) -> dict[str, Any]:
        with self._lock:
            return next(item for item in self.commands if item.get("command") == name)

    def _serve(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(("127.0.0.1", self.port))
            server.listen(1)
            self._ready.set()
            while self._running:
                try:
                    server.settimeout(0.5)
                    client, _ = server.accept()
                except socket.timeout:
                    continue
                with client:
                    self._client_connected.set()
                    while self._running:
                        header = _recv_exactly(client, 4)
                        if not header:
                            break
                        length = struct.unpack("<I", header)[0]
                        body = _recv_exactly(client, length)
                        if not body:
                            break
                        payload = json.loads(body.decode("utf-8"))
                        with self._lock:
                            self.commands.append(payload)
                        _send_message(client, self.telemetry)


def _load_bridge_module():
    spec = importlib.util.spec_from_file_location(
        "tests.real_ros2_bridge_module", BRIDGE_NODE_PATH
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _load_runtime_probe_module():
    spec = importlib.util.spec_from_file_location(
        "tests.ros2_runtime_probe_module", RUNTIME_PROBE_PATH
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _spin_nodes(rclpy, *nodes, cycles: int = 1, timeout_sec: float = 0.05) -> None:
    for _ in range(cycles):
        for node in nodes:
            rclpy.spin_once(node, timeout_sec=timeout_sec)


def _wait_until(
    predicate,
    *,
    message: str,
    rclpy,
    nodes: list[Any],
    timeout: float = 5.0,
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        _spin_nodes(rclpy, *nodes)
    raise AssertionError(message)


def _wait_for_future(future, *, rclpy, nodes: list[Any], timeout: float = 5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if future.done():
            return future.result()
        _spin_nodes(rclpy, *nodes)
    raise AssertionError("ROS2 future did not complete in time")


def test_ros2_bridge_humble_smoke() -> None:
    artifact_dir = _artifact_dir()
    port = _reserve_free_tcp_port()
    report = _base_report(artifact_dir, port)

    if os.getenv(ENABLE_ENV_VAR) != "1":
        _record_skip(
            artifact_dir,
            report,
            f"set {ENABLE_ENV_VAR}=1 to enable the real ROS2 bridge smoke test",
        )

    runtime_probe = _load_runtime_probe_module().probe_ros2_python_runtime()
    report["ros2_runtime"] = runtime_probe
    if not runtime_probe["available"]:
        missing = ", ".join(runtime_probe["missing_modules"])
        _record_skip(
            artifact_dir,
            report,
            f"missing ROS2 Python runtime modules: {missing}",
        )

    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("tf2_ros")
    pytest.importorskip("sensor_msgs.msg")
    geometry_msgs = pytest.importorskip("geometry_msgs.msg")
    std_srvs = pytest.importorskip("std_srvs.srv")
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    bridge_module = _load_bridge_module()
    server = RecordingGodotServer(port)
    server.start()

    helper_node = None
    bridge = None
    initialized_here = False
    try:
        if not rclpy.ok():
            rclpy.init(args=["--ros-args", "-p", f"godot_port:={port}"])
            initialized_here = True

        class ProbeNode(Node):
            def __init__(self):
                super().__init__("agi_walker_bridge_smoke_probe")
                self.joint_states: list[JointState] = []
                self.joint_state_sub = self.create_subscription(
                    JointState, "/joint_states", self._on_joint_state, 10
                )
                self.cmd_vel_pub = self.create_publisher(
                    geometry_msgs.Twist, "/cmd_vel", 10
                )
                self.start_client = self.create_client(
                    std_srvs.Trigger, "/start_simulation"
                )
                self.stop_client = self.create_client(
                    std_srvs.Trigger, "/stop_simulation"
                )

            def _on_joint_state(self, message: JointState) -> None:
                self.joint_states.append(message)

        bridge = bridge_module.AGIWalkerROS2Bridge()
        helper_node = ProbeNode()
        nodes = [bridge, helper_node]

        assert server.wait_for_client(
            timeout=5.0
        ), "bridge did not connect to mock Godot"

        _wait_until(
            lambda: bridge.godot_client is not None
            and bridge.godot_client.is_connected(),
            message="bridge Godot client did not become connected",
            rclpy=rclpy,
            nodes=nodes,
        )
        _wait_until(
            lambda: helper_node.start_client.wait_for_service(timeout_sec=0.1),
            message="start_simulation service was not available",
            rclpy=rclpy,
            nodes=nodes,
        )
        _wait_until(
            lambda: helper_node.stop_client.wait_for_service(timeout_sec=0.1),
            message="stop_simulation service was not available",
            rclpy=rclpy,
            nodes=nodes,
        )

        start_future = helper_node.start_client.call_async(std_srvs.Trigger.Request())
        start_response = _wait_for_future(start_future, rclpy=rclpy, nodes=nodes)
        report["start_response"] = {
            "success": bool(start_response.success),
            "message": str(start_response.message),
        }
        assert start_response.success, start_response.message

        _wait_until(
            lambda: bool(bridge.latest_data.get("joint_positions")),
            message="bridge did not receive telemetry from mock Godot",
            rclpy=rclpy,
            nodes=nodes,
        )
        _wait_until(
            lambda: len(helper_node.joint_states) > 0,
            message="no /joint_states message was published",
            rclpy=rclpy,
            nodes=nodes,
        )

        joint_state = helper_node.joint_states[-1]
        report["received_joint_state_count"] = len(helper_node.joint_states)
        report["joint_state_names"] = list(joint_state.name)
        report["joint_state_positions"] = list(joint_state.position)
        assert list(joint_state.name) == server.telemetry["joint_names"]
        assert list(joint_state.position) == server.telemetry["joint_positions"]

        cmd_vel = geometry_msgs.Twist()
        cmd_vel.linear.x = 0.25
        cmd_vel.linear.y = 0.05
        cmd_vel.angular.z = -0.1
        helper_node.cmd_vel_pub.publish(cmd_vel)
        report["cmd_vel_sent"] = {
            "linear_x": cmd_vel.linear.x,
            "linear_y": cmd_vel.linear.y,
            "angular_z": cmd_vel.angular.z,
        }

        _wait_until(
            lambda: "update_params" in server.command_names(),
            message="bridge did not forward /cmd_vel to Godot update_params",
            rclpy=rclpy,
            nodes=nodes,
        )

        update_command = server.get_command("update_params")
        assert update_command["data"]["cmd_linear_x"] == pytest.approx(0.25)
        assert update_command["data"]["cmd_linear_y"] == pytest.approx(0.05)
        assert update_command["data"]["cmd_angular_z"] == pytest.approx(-0.1)

        stop_future = helper_node.stop_client.call_async(std_srvs.Trigger.Request())
        stop_response = _wait_for_future(stop_future, rclpy=rclpy, nodes=nodes)
        report["stop_response"] = {
            "success": bool(stop_response.success),
            "message": str(stop_response.message),
        }
        assert stop_response.success, stop_response.message

        report["command_names"] = server.command_names()
        report["status"] = "passed"
    except Exception as exc:
        report["status"] = "failed"
        report["failure_stage"] = report["failure_stage"] or "runtime"
        report["message"] = str(exc)
        report["command_names"] = server.command_names()
        raise
    finally:
        report["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        _write_report(artifact_dir, report)
        if helper_node is not None:
            helper_node.destroy_node()
        if bridge is not None:
            bridge.destroy_node()
        if initialized_here and rclpy.ok():
            rclpy.shutdown()
        server.stop()
