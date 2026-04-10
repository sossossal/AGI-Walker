"""
Real Godot headless smoke test.

This integration test validates the full session-bridge lifecycle:
- locate a usable Godot executable
- launch one headless Godot scene
- wait for TCP readiness
- fetch schema
- send a robot payload
- step the loop and receive telemetry

The test also writes a structured diagnostic report so CI/nightly failures can
be triaged without immediately reproducing them locally.
"""

from __future__ import annotations

import asyncio
import json
import os
import socket
import time
from pathlib import Path
from typing import Any, Dict

import pytest

from web_panel.godot_session_bridge import GODOT_PROJECT_DIR, GodotBridge

ARTIFACT_DIR_ENV = "AGI_WALKER_GODOT_HEADLESS_ARTIFACT_DIR"
DEFAULT_ARTIFACT_DIR = Path("test_env") / "godot_headless_smoke"
DEFAULT_SCENE = "demo_generated_biped.tscn"


def _reserve_free_tcp_port() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(("127.0.0.1", 0))
    _, port = sock.getsockname()
    sock.close()
    return port


def _env_float(name: str, default: float) -> float:
    raw = os.getenv(name, "").strip()
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError:
        return default


def _env_int(name: str, default: int) -> int:
    raw = os.getenv(name, "").strip()
    if not raw:
        return default
    try:
        return int(raw)
    except ValueError:
        return default


def _artifact_dir() -> Path:
    configured = os.getenv(ARTIFACT_DIR_ENV, "").strip()
    target = Path(configured) if configured else DEFAULT_ARTIFACT_DIR
    target.mkdir(parents=True, exist_ok=True)
    return target


def _scene_name() -> str:
    return (
        os.getenv("AGI_WALKER_GODOT_HEADLESS_SCENE", DEFAULT_SCENE).strip()
        or DEFAULT_SCENE
    )


def _write_report(artifact_dir: Path, report: Dict[str, Any]) -> None:
    report_path = artifact_dir / "headless_smoke_report.json"
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def _base_report(
    session_id: str, scene_name: str, port: int, artifact_dir: Path
) -> Dict[str, Any]:
    return {
        "test_name": "test_godot_headless_smoke_lifecycle",
        "session_id": session_id,
        "scene": scene_name,
        "project_dir": str(Path(GODOT_PROJECT_DIR).resolve()),
        "artifact_dir": str(artifact_dir.resolve()),
        "tcp_port": port,
        "status": "starting",
        "failure_stage": None,
        "message": None,
        "godot_executable": None,
        "connect_timeout_seconds": _env_float(
            "AGI_WALKER_GODOT_HEADLESS_CONNECT_TIMEOUT_SECONDS", 15.0
        ),
        "schema_timeout_seconds": _env_float(
            "AGI_WALKER_GODOT_HEADLESS_SCHEMA_TIMEOUT_SECONDS", 10.0
        ),
        "step_count": _env_int("AGI_WALKER_GODOT_HEADLESS_STEP_COUNT", 3),
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "finished_at": None,
        "duration_seconds": None,
        "launch_result": None,
        "schema_summary": None,
        "load_result": None,
        "step_samples": [],
        "stop_result": None,
        "failure_diagnostics": None,
        "process_diagnostics": {},
    }


def _record_skip(
    artifact_dir: Path, report: Dict[str, Any], stage: str, reason: str
) -> None:
    report["status"] = "skipped"
    report["failure_stage"] = stage
    report["message"] = reason
    report["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    _write_report(artifact_dir, report)
    pytest.skip(reason)


def _fail(
    report: Dict[str, Any], stage: str, message: str, bridge: GodotBridge | None = None
) -> None:
    report["status"] = "failed"
    report["failure_stage"] = stage
    report["message"] = message
    if bridge is not None:
        report["failure_diagnostics"] = bridge.get_process_diagnostics()
    pytest.fail(message)


async def _run_godot_headless_smoke_lifecycle() -> None:
    artifact_dir = _artifact_dir()
    session_id = f"smoke_test_{int(time.time())}"
    port = _reserve_free_tcp_port()
    scene_name = _scene_name()
    report = _base_report(session_id, scene_name, port, artifact_dir)
    started_at = time.perf_counter()
    bridge = GodotBridge(session_id, port=port)

    if os.getenv("AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE") != "1":
        _record_skip(
            artifact_dir,
            report,
            "gating",
            "未显式启用真实 Godot headless smoke；设置 AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1 后再运行。",
        )

    executable = bridge._find_godot_exe()
    report["godot_executable"] = executable or None
    if not executable:
        _record_skip(
            artifact_dir,
            report,
            "environment",
            "未发现可用的 Godot 可执行文件；请配置 GODOT_EXECUTABLE / GODOT / GODOT_EXE / GODOT_PATH。",
        )

    project_dir = Path(GODOT_PROJECT_DIR)
    if not project_dir.exists():
        _record_skip(
            artifact_dir,
            report,
            "environment",
            f"Godot 项目目录不存在: {project_dir}",
        )

    scene_path = project_dir / scene_name
    if not scene_path.exists():
        _record_skip(
            artifact_dir,
            report,
            "environment",
            f"Godot 场景不存在: {scene_path}",
        )

    connect_timeout = float(report["connect_timeout_seconds"])
    schema_timeout = float(report["schema_timeout_seconds"])
    step_count = int(report["step_count"])

    try:
        launch_result = bridge.launch(scene=scene_name, headless=True)
        report["launch_result"] = launch_result
        if launch_result.get("status") not in {"launched", "already_running"}:
            _fail(report, "launch", f"进程拉起失败: {launch_result}", bridge)

        connected = await bridge.wait_until_connected(timeout_seconds=connect_timeout)
        if not connected:
            _fail(
                report,
                "tcp_connect",
                f"Godot TCP 连接未在 {connect_timeout}s 内建立: {bridge.get_process_diagnostics()}",
                bridge,
            )
        if not bridge.is_running():
            _fail(
                report,
                "tcp_connect",
                f"Godot 进程在 TCP 建连前已退出: {bridge.get_process_diagnostics()}",
                bridge,
            )

        schema = await bridge.wait_until_schema(timeout_seconds=schema_timeout)
        if not schema:
            _fail(
                report,
                "schema",
                f"获取 Schema 失败或超时: {bridge.get_process_diagnostics()}",
                bridge,
            )
        if "sensors" not in schema or "actuators" not in schema:
            _fail(
                report,
                "schema",
                f"Schema 结构不完整: keys={sorted(schema.keys())}",
                bridge,
            )
        report["schema_summary"] = {
            "keys": sorted(schema.keys()),
            "sensor_keys": sorted(schema.get("sensors", {}).keys()),
            "actuator_keys": sorted(schema.get("actuators", {}).keys()),
        }

        dummy_config = {"parts": [], "connections": []}
        load_result = await bridge.send_load_robot(dummy_config)
        report["load_result"] = load_result
        if load_result is None:
            _fail(report, "load_robot", "未接收到 load_robot 回复。", bridge)
        if not isinstance(load_result, dict):
            _fail(
                report,
                "load_robot",
                f"load_robot 返回类型错误: {type(load_result)!r}",
                bridge,
            )
        if load_result.get("status") not in {None, "success"}:
            _fail(
                report, "load_robot", f"Godot 拒绝装载机器人配置: {load_result}", bridge
            )

        for index in range(step_count):
            sensors = await bridge.get_sensors()
            if sensors is None:
                _fail(report, "step_loop", f"第 {index} 步未接收到传感器数据。", bridge)
            report["step_samples"].append(
                {
                    "index": index,
                    "sensor_keys": sorted((sensors or {}).keys()),
                    "sample_size": len(sensors or {}),
                }
            )
            await asyncio.sleep(0.1)

        delayed_connect_task = bridge._delayed_connect_task
        stop_result = bridge.stop()
        report["stop_result"] = stop_result
        if delayed_connect_task is not None:
            await asyncio.gather(delayed_connect_task, return_exceptions=True)
        await asyncio.sleep(0.5)
        if bridge.is_running():
            _fail(report, "teardown", "资源释放失败，Godot 进程仍在运行。", bridge)

        report["status"] = "passed"
        report["message"] = "Headless smoke completed successfully."
    finally:
        delayed_connect_task = bridge._delayed_connect_task
        if bridge.is_running():
            try:
                report["stop_result"] = bridge.stop()
            except Exception as exc:  # pragma: no cover - cleanup fallback
                report["stop_result"] = {"status": "error", "message": str(exc)}
        elif bridge._delayed_connect_task is not None:
            bridge.stop()

        if delayed_connect_task is not None:
            await asyncio.gather(delayed_connect_task, return_exceptions=True)

        report["process_diagnostics"] = bridge.get_process_diagnostics()
        report["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        report["duration_seconds"] = round(time.perf_counter() - started_at, 3)
        _write_report(artifact_dir, report)


@pytest.mark.integration
def test_godot_headless_smoke_lifecycle() -> None:
    asyncio.run(_run_godot_headless_smoke_lifecycle())


if __name__ == "__main__":
    asyncio.run(_run_godot_headless_smoke_lifecycle())
