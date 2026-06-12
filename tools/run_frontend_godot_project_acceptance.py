from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from datetime import UTC, datetime
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Callable

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_ROOT = ROOT / "test_env" / "frontend_godot_project_acceptance"
REPORT_SCHEMA = "frontend_godot_project_acceptance_report.v1"


def frontend_input_fixture(output_root: Path) -> dict[str, Any]:
    return {
        "schema_version": "frontend_project_input.v1",
        "source": "frontend_form_simulation",
        "command_text": "创建一个双足机器人 高度0.62 质量7.5 叫walkerbot",
        "task": {
            "name": "frontend-godot-acceptance",
            "description": "Automated frontend input project smoke",
            "priority": "high",
            "updated_status": "running",
        },
        "godot": {
            "session_id": "frontend-acceptance-session",
            "host": "127.0.0.1",
            "port": 9999,
            "physics": {"gravity": 9.81, "timestep": 0.01},
            "params": {"target_speed": 1.08},
            "robot_config": {"template": "biped", "parts": [], "connections": []},
        },
        "workflow": {
            "list_required": True,
            "output_root": str(output_root / "workflow_inputs"),
        },
    }


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False)
        handle.write("\n")


def timed_call(name: str, call: Callable[[], Any]) -> dict[str, Any]:
    started = time.perf_counter()
    try:
        result = call()
        return {
            "name": name,
            "status": "passed",
            "elapsed_ms": round((time.perf_counter() - started) * 1000.0, 3),
            "result": result,
        }
    except Exception as exc:  # noqa: BLE001 - report generator must retain failures.
        return {
            "name": name,
            "status": "failed",
            "elapsed_ms": round((time.perf_counter() - started) * 1000.0, 3),
            "error": str(exc),
        }


def require_response(response: Any, expected_status: int = 200) -> dict[str, Any]:
    if response.status_code != expected_status:
        raise RuntimeError(
            f"expected HTTP {expected_status}, got {response.status_code}: {response.text}"
        )
    try:
        return response.json()
    except json.JSONDecodeError:
        return {"text": response.text}


def install_safe_web_overrides() -> Callable[[], None]:
    import web_panel.server
    from web_panel.auth_api import get_current_user
    from web_panel.command_parser import CommandParser

    fake_user = SimpleNamespace(
        id=None,
        username="frontend-acceptance-user",
        is_admin=False,
        created_at=datetime(2026, 6, 11, 12, 0, 0),
    )

    async def override_current_user() -> SimpleNamespace:
        return fake_user

    controller = web_panel.server.godot_controller
    original = {
        "dependency_overrides": dict(web_panel.server.app.dependency_overrides),
        "llm_parse": CommandParser._llm_parse,
        "connect": controller.connect,
        "disconnect": controller.disconnect,
        "is_connected": controller.is_connected,
        "load_robot": controller.load_robot,
        "start_simulation": controller.start_simulation,
        "stop_simulation": controller.stop_simulation,
        "update_params": controller.update_params,
        "client_running": getattr(controller.client, "running", None),
    }

    web_panel.server.app.dependency_overrides[get_current_user] = override_current_user
    CommandParser._llm_parse = lambda self, prompt: None
    controller.connect = lambda host, port, session_id=None: True
    controller.disconnect = lambda session_id=None: None
    controller.is_connected = lambda: True
    controller.load_robot = lambda parts, connections, session_id=None: True
    controller.start_simulation = lambda physics, session_id=None: True
    controller.stop_simulation = lambda session_id=None: True
    controller.update_params = lambda params, session_id=None: True
    controller.client.running = True

    def restore() -> None:
        web_panel.server.app.dependency_overrides.clear()
        web_panel.server.app.dependency_overrides.update(original["dependency_overrides"])
        CommandParser._llm_parse = original["llm_parse"]
        controller.connect = original["connect"]
        controller.disconnect = original["disconnect"]
        controller.is_connected = original["is_connected"]
        controller.load_robot = original["load_robot"]
        controller.start_simulation = original["start_simulation"]
        controller.stop_simulation = original["stop_simulation"]
        controller.update_params = original["update_params"]
        if original["client_running"] is not None:
            controller.client.running = original["client_running"]

    return restore


def run_frontend_route_checks(input_payload: dict[str, Any]) -> list[dict[str, Any]]:
    if sys.version_info >= (3, 14):
        static_page = ROOT / "web_panel" / "static" / "godot-control.html"
        return [
            {
                "name": "frontend_static_page_contract",
                "status": "passed" if static_page.is_file() else "failed",
                "elapsed_ms": 0.0,
                "result": {
                    "exists": static_page.is_file(),
                    "required_input_ids": [
                        "gravityInput",
                        "timestepInput",
                        "speedInput",
                        "robotTemplateSelect",
                    ],
                },
            },
            {
                "name": "web_api_route_runtime",
                "status": "failed",
                "elapsed_ms": 0.0,
                "error": (
                    "Python 3.14 alpha is incompatible with the installed "
                    "FastAPI/Pydantic stack in this workspace. Run with the "
                    "bundled workspace Python or Python 3.12."
                ),
            },
        ]

    from fastapi.testclient import TestClient

    import web_panel.server

    restore = install_safe_web_overrides()
    checks: list[dict[str, Any]] = []
    try:
        with TestClient(web_panel.server.app) as client:
            command_text = input_payload["command_text"]
            task = input_payload["task"]
            godot = input_payload["godot"]
            session_id = godot["session_id"]

            checks.append(
                timed_call(
                    "frontend_static_page_contract",
                    lambda: {
                        "exists": (ROOT / "web_panel" / "static" / "godot-control.html").is_file(),
                        "required_input_ids": [
                            "gravityInput",
                            "timestepInput",
                            "speedInput",
                            "robotTemplateSelect",
                        ],
                    },
                )
            )
            checks.append(
                timed_call(
                    "agent_parse_frontend_command",
                    lambda: require_response(
                        client.post("/api/agent/parse-command", json={"command": command_text})
                    ),
                )
            )
            task_state: dict[str, Any] = {}

            def task_crud() -> dict[str, Any]:
                created = require_response(
                    client.post(
                        "/api/tasks",
                        json={
                            "name": task["name"],
                            "description": task["description"],
                            "priority": task["priority"],
                        },
                    )
                )
                task_id = created["task_id"]
                task_state["task_id"] = task_id
                fetched = require_response(client.get(f"/api/tasks/{task_id}"))
                updated = require_response(
                    client.put(
                        f"/api/tasks/{task_id}",
                        json={"status": task["updated_status"], "description": "updated"},
                    )
                )
                deleted = require_response(client.delete(f"/api/tasks/{task_id}"))
                return {
                    "created_status": created["status"],
                    "fetched_name": fetched["task"]["name"],
                    "updated_status": updated["task"]["status"],
                    "deleted_status": deleted["status"],
                }

            checks.append(timed_call("tasks_frontend_crud", task_crud))
            checks.append(
                timed_call(
                    "workflow_discovery",
                    lambda: {
                        "workflows": [
                            {"name": row["name"], "valid": row["validation"]["valid"]}
                            for row in require_response(client.get("/api/workflows/"))
                        ]
                    },
                )
            )

            def godot_control_flow() -> dict[str, Any]:
                connect = require_response(
                    client.post(
                        f"/api/godot/connect?session_id={session_id}",
                        json={"host": godot["host"], "port": godot["port"]},
                    )
                )
                load_robot = require_response(
                    client.post(
                        f"/api/godot/load-robot?session_id={session_id}",
                        json=godot["robot_config"],
                    )
                )
                start = require_response(
                    client.post(
                        f"/api/godot/start?session_id={session_id}",
                        json={"physics": godot["physics"]},
                    )
                )
                update = require_response(
                    client.post(
                        f"/api/godot/update-params?session_id={session_id}",
                        json={"params": godot["params"]},
                    )
                )
                stop = require_response(client.post(f"/api/godot/stop?session_id={session_id}"))
                disconnect = require_response(
                    client.post(f"/api/godot/disconnect?session_id={session_id}")
                )
                return {
                    "connect": connect["status"],
                    "load_robot": load_robot["status"],
                    "start": start["status"],
                    "update": update["status"],
                    "stop": stop["status"],
                    "disconnect": disconnect["status"],
                }

            checks.append(timed_call("godot_frontend_control_routes", godot_control_flow))
    finally:
        restore()
    return checks


def run_biped_acceptance(skip: bool) -> dict[str, Any]:
    if skip:
        return {"status": "skipped", "reason": "--skip-biped-acceptance was supplied"}
    command = [sys.executable, str(ROOT / "biped_robot" / "tools" / "run_local_acceptance.py")]
    completed = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, check=False)
    coverage_report = ROOT / "biped_robot" / "test_env" / "full_coverage_report.json"
    coverage_payload: dict[str, Any] | None = None
    if coverage_report.is_file():
        coverage_payload = json.loads(coverage_report.read_text(encoding="utf-8"))
    return {
        "status": "passed" if completed.returncode == 0 else "failed",
        "command": command,
        "exit_code": completed.returncode,
        "stdout": completed.stdout.strip(),
        "stderr": completed.stderr.strip(),
        "coverage_report": str(coverage_report),
        "coverage_summary": coverage_payload,
    }


def build_report(output_root: Path, skip_biped_acceptance: bool) -> dict[str, Any]:
    input_payload = frontend_input_fixture(output_root)
    route_checks = run_frontend_route_checks(input_payload)
    biped_acceptance = run_biped_acceptance(skip_biped_acceptance)
    passed = all(check["status"] == "passed" for check in route_checks) and biped_acceptance["status"] in {
        "passed",
        "skipped",
    }
    if biped_acceptance["status"] == "skipped":
        passed = False
    return {
        "schema_version": REPORT_SCHEMA,
        "created_utc": datetime.now(UTC).isoformat(),
        "status": "passed" if passed else "failed",
        "frontend_input": input_payload,
        "route_checks": route_checks,
        "biped_godot_acceptance": biped_acceptance,
        "coverage": {
            "frontend_input_mapping": all(check["status"] == "passed" for check in route_checks),
            "godot_demo_invoked": biped_acceptance["status"] == "passed",
            "detailed_report_generated": True,
        },
        "residual_risks": [
            "Browser runtime automation is not run by this script; FastAPI TestClient route evidence is used for frontend input mapping.",
            "Web Godot control routes are mocked to avoid issuing real process or hardware commands.",
            "Real hardware, serial/CAN, ROS2 and external simulator validation remain blocked without external runtime evidence.",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run frontend-input project acceptance and Godot demonstration evidence."
    )
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--skip-biped-acceptance", action="store_true")
    args = parser.parse_args()

    report = build_report(args.output_root, args.skip_biped_acceptance)
    output = args.output_root / "frontend_godot_project_acceptance_report.json"
    write_json(output, report)
    print(json.dumps({"status": report["status"], "output": str(output)}, ensure_ascii=False))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
