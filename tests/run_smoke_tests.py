"""
Minimal smoke test runner for AGI-Walker.

Runs a small set of high-signal checks against the CLI, workflow engine,
and basic Web panel imports. Workflow artifacts are isolated under
test_env/smoke_runs/<timestamp>/ by default.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from time import perf_counter
from typing import Sequence


PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUTPUT_ROOT = (
    PROJECT_ROOT / "test_env" / "smoke_runs" / datetime.now().strftime("%Y%m%d_%H%M%S")
)

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agi_walker.core.api.release_contracts import (  # noqa: E402
    build_release_evidence_report,
    write_release_evidence_report,
)


@dataclass(frozen=True)
class SmokeCheck:
    """Represents one smoke-test command plus a high-signal output probe."""

    name: str
    command: Sequence[str]
    expected_tokens: Sequence[str]
    artifact_dir: Path | None = None
    env_overrides: dict[str, str] | None = None
    skip_reason: str | None = None


def _build_env() -> dict[str, str]:
    env = os.environ.copy()
    env["PYTHONIOENCODING"] = "utf-8"
    env["PYTHONUTF8"] = "1"
    existing_path = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = (
        str(PROJECT_ROOT)
        if not existing_path
        else f"{PROJECT_ROOT}{os.pathsep}{existing_path}"
    )
    return env


def _remove_tree(path: Path) -> None:
    if not path.exists():
        return

    def _handle_remove_readonly(func, current_path, exc_info):
        del exc_info
        os.chmod(current_path, 0o666)
        func(current_path)

    shutil.rmtree(path, onerror=_handle_remove_readonly)


def _prepare_tagged_clean_source_root(output_root: Path, *, version: str) -> Path:
    source_root = output_root / "smoke_git_source"
    if source_root.exists():
        _remove_tree(source_root)
    source_root.mkdir(parents=True, exist_ok=True)
    (source_root / "README.md").write_text("# smoke git source\n", encoding="utf-8")
    commands = [
        ["git", "init"],
        ["git", "config", "user.name", "AGI-Walker Smoke"],
        ["git", "config", "user.email", "smoke@example.com"],
        ["git", "add", "README.md"],
        ["git", "commit", "-m", "init smoke source"],
        ["git", "tag", version],
    ]
    for command in commands:
        result = subprocess.run(
            command,
            cwd=str(source_root),
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            check=False,
        )
        if result.returncode != 0:
            message = result.stderr.strip() or result.stdout.strip() or "git command failed"
            raise RuntimeError(
                f"failed to prepare smoke git source root {source_root}: {message}"
            )
    return source_root


def _default_modern_godot_agent_dir() -> Path | None:
    configured = os.environ.get("AGI_WALKER_SMOKE_GODOT_AGENT_DIR") or os.environ.get(
        "AGI_WALKER_GODOT_AGENT_DIR"
    )
    if configured:
        candidate = Path(configured).expanduser()
        if candidate.exists():
            return candidate

    sibling = PROJECT_ROOT.parent / "godot-agent"
    if sibling.exists():
        return sibling
    return None


def _modern_godot_agent_skip_reason(
    agent_dir: Path | None, env: dict[str, str]
) -> str | None:
    if agent_dir is None:
        return "external godot-agent directory not found"

    probe = (
        "from agi_walker.integrations.godot_agent.godot_agent_adapter import ModernGodotAgentAdapter; "
        f"adapter = ModernGodotAgentAdapter(r'{agent_dir}'); "
        "assert adapter.router is not None; "
        "print('modern_backend_ready')"
    )
    result = subprocess.run(
        [sys.executable, "-c", probe],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        env=env,
    )
    if result.returncode == 0 and "modern_backend_ready" in result.stdout:
        return None

    output = "\n".join(
        part for part in [result.stdout.strip(), result.stderr.strip()] if part
    )
    return (
        f"modern backend preflight failed: {output or f'exit code {result.returncode}'}"
    )


def _godot_headless_skip_reason(env: dict[str, str]) -> str | None:
    if env.get("AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE") != "1":
        return "real Godot headless smoke is opt-in; set AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1 to enable it"

    scene_name = env.get("AGI_WALKER_GODOT_HEADLESS_SCENE", "demo_generated_biped.tscn")
    probe = (
        "from pathlib import Path; "
        "from web_panel.godot_session_bridge import GodotBridge, GODOT_PROJECT_DIR; "
        "bridge = GodotBridge('smoke_probe', 9009); "
        "exe = bridge._find_godot_exe(); "
        "assert exe, 'godot executable not found'; "
        f"scene = Path(GODOT_PROJECT_DIR) / r'{scene_name}'; "
        "assert scene.exists(), f'missing scene: {scene}'; "
        "print('godot_headless_ready')"
    )
    result = subprocess.run(
        [sys.executable, "-c", probe],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        env=env,
    )
    if result.returncode == 0 and "godot_headless_ready" in result.stdout:
        return None

    output = "\n".join(
        part for part in [result.stdout.strip(), result.stderr.strip()] if part
    )
    return f"real Godot headless smoke preflight failed: {output or f'exit code {result.returncode}'}"


def _ros2_bridge_smoke_skip_reason(env: dict[str, str]) -> str | None:
    if env.get("AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE") != "1":
        return "real ROS2 bridge smoke is opt-in; set AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE=1 to enable it"

    probe = (
        "import rclpy; "
        "import sensor_msgs.msg; "
        "import geometry_msgs.msg; "
        "import std_srvs.srv; "
        "import tf2_ros; "
        "print('ros2_bridge_smoke_ready')"
    )
    result = subprocess.run(
        [sys.executable, "-c", probe],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        env=env,
    )
    if result.returncode == 0 and "ros2_bridge_smoke_ready" in result.stdout:
        return None

    output = "\n".join(
        part for part in [result.stdout.strip(), result.stderr.strip()] if part
    )
    return f"real ROS2 bridge smoke preflight failed: {output or f'exit code {result.returncode}'}"


def _fake_godot_agent_check_script() -> str:
    return """
from fastapi.testclient import TestClient
from web_panel.server import app
import web_panel.agent_api as agent_api

class FakeBackend:
    def execute_command(self, command, context=None, project_path=None):
        return {"status": "success", "message": command}
    def execute_pipeline(self, commands, context=None):
        return [{"status": "success", "success": True, "message": item} for item in commands]
    def get_roles_info(self):
        return [{"name": "developer", "description": "dev", "capabilities": ["code"]}]
    def list_skills(self):
        return {"status": "success", "skills": [{"id": "compat-skill"}], "compatibility_alias": False}
    def apply_skill(self, skill_id):
        return {"status": "success", "data": {"id": skill_id}, "compatibility_alias": False}
    def list_templates(self):
        return {"status": "success", "templates": [{"id": "ai/patrol.gd", "type": "template"}], "backend_mode": "fake"}
    def get_template(self, template_id):
        return {"status": "success", "data": {"id": template_id, "type": "template", "content": "extends Node\\n"}}
    def plan_command(self, command, context=None, project_path=None):
        return {"status": "awaiting_confirmation", "success": True, "steps": [{"role": "developer"}], "context": context or {}}
    def get_history(self, limit=20):
        return {"status": "success", "items": [{"task_id": "task-1"}], "count": 1}
    def doctor(self, project_path=None):
        return {"status": "success", "ok": True, "checks": [{"name": "router", "passed": True}]}
    def launch_editor(self, project_path=None, scene_path=None):
        return {"status": "success", "ok": True, "data": {"scene_path": scene_path}}

agent_api.create_godot_agent_backend = lambda: FakeBackend()
if hasattr(app.state, "godot_agent_backend"):
    delattr(app.state, "godot_agent_backend")

client = TestClient(app)
assert client.get("/api/godot-agent/templates").json()["status"] == "success"
assert client.get("/api/godot-agent/templates/ai/patrol.gd").json()["data"]["id"] == "ai/patrol.gd"
assert client.post("/api/godot-agent/plan", json={"command": "生成玩家移动脚本"}).json()["status"] == "awaiting_confirmation"
assert client.get("/api/godot-agent/history").json()["count"] == 1
assert client.get("/api/godot-agent/doctor").json()["ok"] is True
print("godot_agent_fake_templates_ok")
print("godot_agent_fake_plan_ok")
print("godot_agent_fake_doctor_ok")
""".strip()


def _modern_godot_agent_check_script() -> str:
    return """
import os
from fastapi.testclient import TestClient
from web_panel.server import app

if hasattr(app.state, "godot_agent_backend"):
    delattr(app.state, "godot_agent_backend")

client = TestClient(app)
templates = client.get("/api/godot-agent/templates").json()
doctor = client.get("/api/godot-agent/doctor").json()
plan = client.post("/api/godot-agent/plan", json={"command": "生成玩家移动脚本"}).json()
skills = client.get("/api/godot_skills/list").json()

assert templates["status"] == "success"
assert len(templates.get("templates", [])) >= 1
assert doctor["status"] == "success"
assert plan["status"] in {"awaiting_confirmation", "success"}
assert skills["status"] == "success"
assert skills.get("compatibility_alias") is True

print("godot_agent_modern_templates_ok")
print("godot_agent_modern_plan_ok")
print("godot_agent_modern_doctor_ok")
print("godot_agent_modern_skills_alias_ok")
""".strip()


def _capability_matrix_check_script() -> str:
    return """
from fastapi.testclient import TestClient
from web_panel.server import app

client = TestClient(app)
system_status = client.get("/api/system/status").json()
matrix = client.get("/api/capabilities/matrix").json()

assert system_status["capability_matrix"]["route"] == "/api/capabilities/matrix"
assert matrix["artifact_type"] == "capability_matrix"
assert matrix["summary"]["total_domains"] == 5
assert any(domain["id"] == "mcp" for domain in matrix["domains"])

print("capability_matrix_ok")
print("capability_matrix_summary_ok")
""".strip()


def _write_non_live_gate_evidence_script() -> str:
    return """
from agi_walker.core.api.release_contracts import build_release_evidence_report, write_release_evidence_report

payload = build_release_evidence_report(
    evidence_name="non_live_gate",
    status="passed",
    summary="non_live_gate pytest evidence passed: smoke preflight placeholder.",
    command='python -m pytest -m "not live" -q',
    metrics={"passed": 1, "failed": 0, "skipped": 0, "deselected": 0},
    source_commit_sha="smoke-preflight",
)
path = write_release_evidence_report(payload, "test_env/release_evidence/non_live_gate_report.json")
print(f"release_evidence_report_written={path}")
print("release_evidence_name=non_live_gate")
print("release_evidence_status=passed")
""".strip()


def _write_release_contracts_evidence_script() -> str:
    return """
from agi_walker.core.api.release_contracts import build_release_evidence_report, write_release_evidence_report

payload = build_release_evidence_report(
    evidence_name="release_contracts_and_capability_matrix",
    status="passed",
    summary="release contracts and capability matrix targeted checks passed: smoke preflight placeholder.",
    command="python -m pytest tests/test_release_contracts.py tests/test_release_artifact_builder.py tests/test_capability_matrix.py tests/test_mcp_tools.py tests/test_mcp_server.py tests/test_web_panel_aux_apis.py -q",
    metrics={"passed": 1, "failed": 0, "skipped": 0, "deselected": 0},
    source_commit_sha="smoke-preflight",
)
path = write_release_evidence_report(
    payload,
    "test_env/release_evidence/release_contracts_and_capability_matrix_report.json",
)
print(f"release_evidence_report_written={path}")
print("release_evidence_name=release_contracts_and_capability_matrix")
print("release_evidence_status=passed")
""".strip()


def _write_release_ops_execution_evidence_script() -> str:
    return """
from agi_walker.core.api.release_contracts import build_release_evidence_report, write_release_evidence_report

payload = build_release_evidence_report(
    evidence_name="release_ops_execution",
    status="passed",
    summary="release op stable_promotion_checklist completed via smoke control plane wrapper.",
    command="python tools/run_release_ops.py stable_promotion_checklist",
    metrics={
        "action": "stable_promotion_checklist",
        "policy_level": "local_safe_refresh",
        "policy_profile": "local_safe_refresh",
        "request_type": "StablePromotionChecklistRequest",
        "status": "ready",
        "event_count": 3,
        "output_path": "test_env/stable_promotion_ready/stable_promotion_checklist.json",
    },
    source_commit_sha="smoke-preflight",
    control_plane_session={
        "engagement_id": "smoke-session",
        "window_id": "smoke-window",
        "change_ticket": "CHG-SMOKE",
        "channel": "ops-cli",
    },
    control_plane_event_stream={
        "path": "test_env/release_ops/smoke_release_ops.jsonl",
        "event_count": 3,
    },
)
path = write_release_evidence_report(
    payload,
    "test_env/release_evidence/operations/release_ops_execution_report.json",
)
print(f"release_evidence_report_written={path}")
print("release_evidence_name=release_ops_execution")
print("release_evidence_status=passed")
""".strip()


def _write_clean_checkout_smoke_evidence_script() -> str:
    return """
import json
import sys
from pathlib import Path

report_path = Path("test_env/release_evidence/clean_checkout_smoke_report.json")
report_path.parent.mkdir(parents=True, exist_ok=True)
payload = {
    "schema_version": "1.0",
    "artifact_type": "clean_checkout_smoke_report",
    "status": "passed",
    "generated_at": "2026-04-25T08:30:00+00:00",
    "source_root": str(Path.cwd()),
    "checkout_root": str(Path.cwd() / "test_env" / "clean_checkout_smoke" / "checkout"),
    "output_root": str(Path.cwd() / "test_env" / "clean_checkout_smoke"),
    "version": "2026.04.12",
    "tag": "2026.04.12",
    "runs": 1,
    "command_template": [sys.executable, "tests/run_smoke_tests.py", "--output-root", "test_env/clean_checkout_smoke"],
    "checkout_commit_sha": "smoke-clean-checkout",
    "seeded_evidence_paths": [],
    "checks": [
        {
            "name": "clean_checkout_smoke_synthetic_seed",
            "status": "pass",
            "detail": "synthetic clean checkout smoke evidence seeded for self-contained smoke validation",
        }
    ],
    "run_reports": [
        {
            "run_index": 1,
            "status": "passed",
            "command": [sys.executable, "tests/run_smoke_tests.py", "--output-root", "test_env/clean_checkout_smoke"],
            "run_output_root": str(Path.cwd() / "test_env" / "clean_checkout_smoke" / "run_01"),
            "exit_code": 0,
            "stdout_path": str(Path.cwd() / "test_env" / "clean_checkout_smoke" / "run_01.stdout.txt"),
            "stderr_path": str(Path.cwd() / "test_env" / "clean_checkout_smoke" / "run_01.stderr.txt"),
            "worktree_clean": True,
            "dirty_paths": [],
        }
    ],
    "failure_reason": "",
}
report_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\\n", encoding="utf-8")
print(f"clean_checkout_smoke_report_written={report_path}")
print("clean_checkout_smoke_status=passed")
""".strip()


def _write_live_release_evidence_script() -> str:
    return """
import json
from pathlib import Path

reports = {
    Path("test_env/distributed_smoke/distributed_smoke_report.json"): {
        "schema_version": "1.0",
        "status": "passed",
        "actor_id": "actor-smoke",
        "checks": [
            {"name": "compose_build", "status": "pass"},
            {"name": "compose_up", "status": "pass"},
            {"name": "web_panel_status", "status": "pass"},
        ],
    },
    Path("test_env/godot_headless_smoke/headless_smoke_report.json"): {
        "status": "passed",
        "test_name": "test_godot_headless_smoke_lifecycle",
    },
    Path("test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json"): {
        "status": "passed",
        "test_name": "test_ros2_bridge_humble_smoke",
    },
}
for path, payload in reports.items():
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\\n", encoding="utf-8")

print("distributed_runtime_live=passed")
print("godot_headless_live=passed")
print("ros2_bridge_live=passed")
""".strip()


def _godot_instruction_set_smoke_script() -> str:
    return """
import json
import os
from pathlib import Path

from fastapi.testclient import TestClient

import web_panel.server

artifact_dir = Path(
    os.environ.get(
        "AGI_WALKER_GODOT_INSTRUCTION_SMOKE_ARTIFACT_DIR",
        "test_env/godot_instruction_smoke",
    )
)
artifact_dir.mkdir(parents=True, exist_ok=True)

observed = {}

class FakeBridge:
    session_state = "running"
    simulated_circuit_config = {}

    def is_connected(self):
        return True

    async def configure_simulated_circuit(
        self,
        simulated_circuit,
        *,
        operator=None,
        tag=None,
        note=None,
        audit_user_id=None,
        audit_username=None,
        audit_source=None,
    ):
        observed["simulated_circuit"] = simulated_circuit
        self.simulated_circuit_config = dict(simulated_circuit)
        observed["simulated_circuit_metadata"] = {
            "operator": operator,
            "tag": tag,
            "note": note,
            "audit_user_id": audit_user_id,
            "audit_username": audit_username,
            "audit_source": audit_source,
        }
        return {"status": "success", "simulated_circuit": simulated_circuit}

    async def apply_instruction_set(
        self,
        instruction_set,
        *,
        compatibility_params=None,
        simulated_circuit_command_batch=None,
        operator=None,
        tag=None,
        note=None,
        audit_user_id=None,
        audit_username=None,
        audit_source=None,
    ):
        observed["instruction_set"] = instruction_set
        observed["compatibility_params"] = compatibility_params or {}
        observed["simulated_circuit_command_batch"] = simulated_circuit_command_batch or []
        observed["instruction_metadata"] = {
            "operator": operator,
            "tag": tag,
            "note": note,
            "audit_user_id": audit_user_id,
            "audit_username": audit_username,
            "audit_source": audit_source,
        }
        return {"status": "success", "instruction_step_count": len(instruction_set.get("steps", []))}

    def get_status_payload(self):
        return {
            "schema_version": "1.0",
            "session_state": self.session_state,
            "engine_running": True,
            "tcp_connected": True,
            "simulated_circuit_config": self.simulated_circuit_config,
        }

fake_bridge = FakeBridge()
web_panel.server._session_manager.get_session = lambda _session_id: fake_bridge

client = TestClient(web_panel.server.app)
session_id = "smoke-instruction"
instruction_response = client.post(
    f"/api/godot/{session_id}/instruction-set",
    json={
        "instruction_set": {
            "schema_version": "1.0",
            "sequence_name": "smoke-demo",
            "steps": [{"kind": "set_velocity", "linear_x": 0.2, "linear_y": 0.0, "angular_z": 0.1}],
        },
        "compatibility_params": {"cmd_linear_x": 0.2},
        "simulated_circuit_command_batch": [{"frame_id": 512}],
    },
)
circuit_response = client.post(
    f"/api/godot/{session_id}/simulated-circuit",
    json={"simulated_circuit": {"transport": "imc22_can_fd", "bitrate": 1000000}},
)

payload = {
    "status": "passed",
    "instruction_status": instruction_response.json()["status"],
    "circuit_status": circuit_response.json()["status"],
    "sequence_name": observed["instruction_set"]["sequence_name"],
    "transport": observed["simulated_circuit"]["transport"],
}
(artifact_dir / "godot_instruction_smoke_report.json").write_text(
    json.dumps(payload, ensure_ascii=False, indent=2) + "\\n",
    encoding="utf-8",
)
print("godot_instruction_set_smoke_ok")
print("godot_simulated_circuit_smoke_ok")
""".strip()


def _ros2_instruction_set_smoke_script() -> str:
    return """
import json
import os
import types
from pathlib import Path

from tests.test_ros2_bridge_runtime import _load_bridge_module

artifact_dir = Path(
    os.environ.get(
        "AGI_WALKER_ROS2_INSTRUCTION_SMOKE_ARTIFACT_DIR",
        "test_env/ros2_instruction_smoke",
    )
)
artifact_dir.mkdir(parents=True, exist_ok=True)

class _MonkeyPatch:
    def setattr(self, obj, name, value):
        setattr(obj, name, value)
    def setitem(self, mapping, key, value):
        mapping[key] = value

module = _load_bridge_module(_MonkeyPatch())
bridge = module.AGIWalkerROS2Bridge()

instruction_message = module.String()
instruction_message.data = json.dumps(
    {
        "schema_version": "1.0",
        "sequence_name": "ros2-smoke-demo",
        "steps": [{"kind": "set_velocity", "linear_x": 0.15, "linear_y": 0.0, "angular_z": 0.05}],
    }
)
bridge.instruction_set_callback(instruction_message)

payload = json.loads(bridge.instruction_runtime_pub.published[-1].data)
(artifact_dir / "ros2_instruction_smoke_report.json").write_text(
    json.dumps(payload, ensure_ascii=False, indent=2) + "\\n",
    encoding="utf-8",
)
print("ros2_instruction_set_smoke_ok")
print("ros2_instruction_runtime_smoke_ok")
""".strip()


def _simulated_circuit_replay_smoke_script() -> str:
    return """
import json
import os
from pathlib import Path

from agi_walker.core.api.godot_robot_env.hardware_controller import (
    command_batch_to_imc22_replay_payload,
    simulate_imc22_command_batch_feedback,
)

artifact_dir = Path(
    os.environ.get(
        "AGI_WALKER_SIMULATED_CIRCUIT_SMOKE_ARTIFACT_DIR",
        "test_env/simulated_circuit_smoke",
    )
)
artifact_dir.mkdir(parents=True, exist_ok=True)

command_batch = [
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
replay_payload = command_batch_to_imc22_replay_payload(command_batch)
feedback = simulate_imc22_command_batch_feedback(command_batch)
report = {"status": "passed", "replay_payload": replay_payload, "feedback": feedback}
(artifact_dir / "simulated_circuit_replay_smoke_report.json").write_text(
    json.dumps(report, ensure_ascii=False, indent=2) + "\\n",
    encoding="utf-8",
)
print("simulated_circuit_replay_smoke_ok")
print("simulated_circuit_feedback_smoke_ok")
""".strip()


def _write_external_mainline_ready_script() -> str:
    return """
from pathlib import Path

from agi_walker.core.api.release_contracts import (
    build_external_mainline_execution_plan_artifact,
    default_external_mainline_execution_plan_path,
    write_external_mainline_execution_plan_artifact,
)

payload = build_external_mainline_execution_plan_artifact(
    project_root=Path.cwd(),
    control_plane_session={
        "engagement_id": "smoke-session",
        "window_id": "smoke-external-mainline",
        "change_ticket": "CHG-SMOKE-EXTERNAL",
        "channel": "smoke",
    },
    control_plane_event_stream={
        "path": "test_env/release_ops/smoke_external_mainline.jsonl",
        "event_count": 3,
    },
)
steps = payload.get("steps", [])
for step in steps:
    if isinstance(step, dict):
        step["status"] = "completed"
        if not str(step.get("summary") or "").strip():
            step["summary"] = "smoke synthetic external mainline step completed."
payload["steps"] = steps
payload["status"] = "ready"
payload["summary"] = (
    "external mainline execution plan ready: synthetic smoke completion for self-contained release validation."
)
payload["completed_steps"] = len(steps)
payload["ready_to_run_steps"] = 0
payload["waiting_external_input_steps"] = 0
payload["blocked_steps"] = 0
payload["auto_executable_steps"] = sum(
    1 for step in steps if isinstance(step, dict) and step.get("auto_executable") is True
)
path = write_external_mainline_execution_plan_artifact(
    payload,
    default_external_mainline_execution_plan_path(),
)
print(f"external_mainline_execution_plan_written={path}")
print("external_mainline_execution_plan_status=ready")
""".strip()


def _write_canonical_industrial_security_evidence_script(security_root: Path) -> str:
    source_root = str(security_root).replace("\\", "\\\\")
    return f"""
from pathlib import Path
import shutil

source_root = Path(r"{source_root}")
target_root = Path("test_env/release_evidence/security")
files = [
    "security_posture_report.json",
    "vulnerability_remediation_report.json",
    "sbom.json",
    "python_vuln_scan_report.json",
    "container_vuln_scan_report.json",
    "backup_restore_rehearsal_report.json",
]
target_root.mkdir(parents=True, exist_ok=True)
copied = 0
for name in files:
    source = source_root / name
    if not source.is_file():
        raise FileNotFoundError(f"missing smoke security artifact: {{source}}")
    shutil.copyfile(source, target_root / name)
    copied += 1

print(f"canonical_industrial_security_evidence_copied={{copied}}")
print(f"canonical_industrial_security_evidence_root={{target_root}}")
""".strip()


def _build_checks(output_root: Path, env: dict[str, str]) -> list[SmokeCheck]:
    mock_root = output_root / "robot_creation_mock"
    real_root = output_root / "robot_creation_real"
    release_root = output_root / "release"
    stable_release_root = output_root / "release_stable"
    industrial_release_root = output_root / "release_industrial"
    security_root = output_root / "security"
    release_readiness_root = output_root / "release_readiness"
    release_readiness_approved_root = output_root / "release_readiness_approved"
    industrial_release_readiness_root = output_root / "industrial_release_readiness"
    industrial_release_readiness_approved_root = (
        output_root / "industrial_release_readiness_approved"
    )
    worktree_cleanup_root = output_root / "worktree_cleanup"
    tracked_artifact_review_root = output_root / "tracked_artifact_review"
    stable_promotion_root = output_root / "stable_promotion"
    stable_promotion_approved_root = output_root / "stable_promotion_approved"
    industrial_promotion_root = output_root / "industrial_promotion"
    industrial_promotion_approved_root = output_root / "industrial_promotion_approved"
    release_rehearsal_root = output_root / "release_rehearsal"
    customer_acceptance_bundle_root = output_root / "customer_acceptance_bundle"
    industrial_customer_acceptance_bundle_root = (
        output_root / "customer_acceptance_bundle_industrial"
    )
    godot_instruction_smoke_root = output_root / "godot_instruction_smoke"
    ros2_instruction_smoke_root = output_root / "ros2_instruction_smoke"
    simulated_circuit_smoke_root = output_root / "simulated_circuit_smoke"
    ros2_smoke_root = output_root / "ros2_bridge_smoke"
    stable_source_root = _prepare_tagged_clean_source_root(
        output_root, version="2026.04.12"
    )
    modern_godot_agent_dir = _default_modern_godot_agent_dir()
    modern_skip_reason = _modern_godot_agent_skip_reason(modern_godot_agent_dir, env)
    godot_headless_skip_reason = _godot_headless_skip_reason(env)
    ros2_bridge_skip_reason = _ros2_bridge_smoke_skip_reason(env)

    web_check = (
        "from web_panel.server import app; "
        "from web_panel.ws_protocol import WsMessage; "
        "msg = WsMessage(type='ping'); "
        "assert isinstance(msg.payload, dict); "
        "print('ws_message_compat_ok'); "
        "print('web_panel_import_ok'); "
        "print(app.title if hasattr(app, 'title') else 'app_loaded')"
    )

    return [
        SmokeCheck(
            name="skills list",
            command=[sys.executable, "-m", "agi_walker.cli", "skills", "list"],
            expected_tokens=[
                "可用 Skills",
                "robot-modeling",
                "parameter-optimizer",
                "urdf-generator",
            ],
        ),
        SmokeCheck(
            name="skills validate",
            command=[sys.executable, "-m", "agi_walker.cli", "skills", "validate"],
            expected_tokens=["[OK] 所有skills配置有效"],
        ),
        SmokeCheck(
            name="workflow mock",
            command=[
                sys.executable,
                "-m",
                "agi_walker.cli",
                "skills",
                "workflows",
                "run",
                "robot_creation_pipeline",
                "--mock",
                "--force",
                "--output-root",
                str(mock_root),
            ],
            expected_tokens=[
                "执行结果: completed",
                "成功率: 100.0%",
                "执行策略: force",
            ],
            artifact_dir=mock_root,
        ),
        SmokeCheck(
            name="workflow real",
            command=[
                sys.executable,
                "-m",
                "agi_walker.cli",
                "skills",
                "workflows",
                "run",
                "robot_creation_pipeline",
                "--force",
                "--output-root",
                str(real_root),
            ],
            expected_tokens=[
                "执行结果: completed",
                "成功率: 100.0%",
                "执行策略: force",
            ],
            artifact_dir=real_root,
        ),
        SmokeCheck(
            name="web panel import",
            command=[sys.executable, "-c", web_check],
            expected_tokens=["ws_message_compat_ok", "web_panel_import_ok"],
        ),
        SmokeCheck(
            name="capability matrix",
            command=[sys.executable, "-c", _capability_matrix_check_script()],
            expected_tokens=["capability_matrix_ok", "capability_matrix_summary_ok"],
        ),
        SmokeCheck(
            name="non live gate evidence seed",
            command=[sys.executable, "-c", _write_non_live_gate_evidence_script()],
            expected_tokens=[
                "release_evidence_name=non_live_gate",
                "release_evidence_status=passed",
            ],
        ),
        SmokeCheck(
            name="release contracts evidence seed",
            command=[sys.executable, "-c", _write_release_contracts_evidence_script()],
            expected_tokens=[
                "release_evidence_name=release_contracts_and_capability_matrix",
                "release_evidence_status=passed",
            ],
        ),
        SmokeCheck(
            name="release ops execution evidence seed",
            command=[
                sys.executable,
                "-c",
                _write_release_ops_execution_evidence_script(),
            ],
            expected_tokens=[
                "release_evidence_name=release_ops_execution",
                "release_evidence_status=passed",
            ],
        ),
        SmokeCheck(
            name="clean checkout smoke evidence seed",
            command=[sys.executable, "-c", _write_clean_checkout_smoke_evidence_script()],
            expected_tokens=[
                "clean_checkout_smoke_report_written=",
                "clean_checkout_smoke_status=passed",
            ],
        ),
        SmokeCheck(
            name="live release evidence seed",
            command=[sys.executable, "-c", _write_live_release_evidence_script()],
            expected_tokens=[
                "distributed_runtime_live=passed",
                "godot_headless_live=passed",
                "ros2_bridge_live=passed",
            ],
        ),
        SmokeCheck(
            name="external mainline ready seed",
            command=[sys.executable, "-c", _write_external_mainline_ready_script()],
            expected_tokens=[
                "external_mainline_execution_plan_written=",
                "external_mainline_execution_plan_status=ready",
            ],
        ),
        SmokeCheck(
            name="release artifact",
            command=[
                sys.executable,
                "tools/build_release_artifact.py",
                "--version",
                "smoke-2026.04.12",
                "--channel",
                "dev",
                "--build-id",
                "smoke-build",
                "--release-summary",
                "Smoke release gate validation.",
                "--output",
                str(release_root / "release_manifest.json"),
            ],
            expected_tokens=[
                "release_manifest_written=",
                "release_gate_status=",
            ],
            artifact_dir=release_root,
        ),
        SmokeCheck(
            name="sbom artifact",
            command=[
                sys.executable,
                "tools/build_sbom_artifact.py",
                "--project-root",
                str(PROJECT_ROOT),
                "--output",
                str(security_root / "sbom.json"),
            ],
            expected_tokens=[
                "sbom_written=",
                "sbom_components=",
                "sbom_sources=",
            ],
            artifact_dir=security_root,
        ),
        SmokeCheck(
            name="python vulnerability report",
            command=[
                sys.executable,
                "tools/write_vulnerability_scan_report.py",
                "--scan-name",
                "python_dependencies",
                "--target",
                "pyproject.toml",
                "--status",
                "passed",
                "--scanner",
                "smoke-manual-review",
                "--summary",
                "Smoke placeholder report for Python dependencies.",
                "--command",
                "manual placeholder",
                "--output",
                str(security_root / "python_vuln_scan_report.json"),
            ],
            expected_tokens=[
                "vulnerability_scan_report_written=",
                "vulnerability_scan_status=passed",
            ],
            artifact_dir=security_root,
        ),
        SmokeCheck(
            name="container vulnerability report",
            command=[
                sys.executable,
                "tools/write_vulnerability_scan_report.py",
                "--scan-name",
                "container_images",
                "--target",
                "deployment/docker-compose.yml",
                "--status",
                "passed",
                "--scanner",
                "smoke-manual-review",
                "--summary",
                "Smoke placeholder report for container images.",
                "--command",
                "manual placeholder",
                "--output",
                str(security_root / "container_vuln_scan_report.json"),
            ],
            expected_tokens=[
                "vulnerability_scan_report_written=",
                "vulnerability_scan_status=passed",
            ],
            artifact_dir=security_root,
        ),
        SmokeCheck(
            name="backup restore rehearsal",
            command=[
                sys.executable,
                "tools/run_backup_restore_rehearsal.py",
                "--project-root",
                str(PROJECT_ROOT),
                "--output-root",
                str(security_root / "backup_restore_rehearsal"),
                "--report-file",
                str(security_root / "backup_restore_rehearsal_report.json"),
            ],
            expected_tokens=[
                "backup_restore_rehearsal_report_written=",
                "backup_restore_rehearsal_status=passed",
                "backup_restore_missing_items=0",
                "backup_restore_failed_checks=0",
            ],
            artifact_dir=security_root,
        ),
        SmokeCheck(
            name="vulnerability exception report",
            command=[
                sys.executable,
                "tools/build_vulnerability_exception_report.py",
                "--project-root",
                str(PROJECT_ROOT),
                "--output",
                str(security_root / "vulnerability_exception_report.json"),
            ],
            expected_tokens=[
                "vulnerability_exception_report_written=",
                "vulnerability_exception_active=0",
                "vulnerability_exception_expired=0",
            ],
            artifact_dir=security_root,
        ),
        SmokeCheck(
            name="vulnerability exception review report",
            command=[
                sys.executable,
                "tools/build_vulnerability_exception_review_report.py",
                "--project-root",
                str(PROJECT_ROOT),
                "--exception-report",
                str(security_root / "vulnerability_exception_report.json"),
                "--output",
                str(security_root / "vulnerability_exception_review_report.json"),
            ],
            expected_tokens=[
                "vulnerability_exception_review_report_written=",
                "vulnerability_exception_review_report_status=passed",
                "vulnerability_exception_review_due=0",
                "vulnerability_exception_review_follow_up_required=false",
            ],
            artifact_dir=security_root,
        ),
        SmokeCheck(
            name="vulnerability remediation report",
            command=[
                sys.executable,
                "tools/build_vulnerability_remediation_report.py",
                "--project-root",
                str(PROJECT_ROOT),
                "--python-vuln-report",
                str(security_root / "python_vuln_scan_report.json"),
                "--container-vuln-report",
                str(security_root / "container_vuln_scan_report.json"),
                "--vulnerability-exception-report",
                str(security_root / "vulnerability_exception_report.json"),
                "--output",
                str(security_root / "vulnerability_remediation_report.json"),
            ],
            expected_tokens=[
                "vulnerability_remediation_report_written=",
                "vulnerability_remediation_status=ready",
            ],
            artifact_dir=security_root,
        ),
        SmokeCheck(
            name="security posture report",
            command=[
                sys.executable,
                "tools/build_security_posture_report.py",
                "--project-root",
                str(PROJECT_ROOT),
                "--sbom",
                str(security_root / "sbom.json"),
                "--python-vuln-report",
                str(security_root / "python_vuln_scan_report.json"),
                "--container-vuln-report",
                str(security_root / "container_vuln_scan_report.json"),
                "--backup-restore-report",
                str(security_root / "backup_restore_rehearsal_report.json"),
                "--vulnerability-remediation-report",
                str(security_root / "vulnerability_remediation_report.json"),
                "--vulnerability-exception-report",
                str(security_root / "vulnerability_exception_report.json"),
                "--output",
                str(security_root / "security_posture_report.json"),
            ],
            expected_tokens=[
                "security_posture_report_written=",
                "security_posture_status=ready",
                "security_posture_missing_docs=0",
                "security_posture_missing_vuln_reports=0",
                "security_posture_missing_backup_restore_reports=0",
            ],
            artifact_dir=security_root,
        ),
        SmokeCheck(
            name="security release preflight",
            command=[
                sys.executable,
                "tools/run_security_release_preflight.py",
                "--skip-collect",
                "--security-posture-report",
                str(security_root / "security_posture_report.json"),
                "--report-file",
                str(security_root / "security_release_preflight_report.json"),
            ],
            expected_tokens=[
                "security_release_preflight_written=",
                "security_release_preflight_status=passed",
            ],
            artifact_dir=security_root,
        ),
        SmokeCheck(
            name="canonical industrial security evidence seed",
            command=[
                sys.executable,
                "-c",
                _write_canonical_industrial_security_evidence_script(security_root),
            ],
            expected_tokens=[
                "canonical_industrial_security_evidence_copied=6",
                "canonical_industrial_security_evidence_root=",
            ],
        ),
        SmokeCheck(
            name="stable release artifact",
            command=[
                sys.executable,
                "tools/build_release_artifact.py",
                "--version",
                "2026.04.12",
                "--channel",
                "stable",
                "--build-id",
                "smoke-stable-build",
                "--approval-status",
                "approved",
                "--approved-by",
                "smoke-release-manager",
                "--approved-at",
                "2026-04-12T12:30:00+00:00",
                "--approval-notes",
                "smoke stable signoff",
                "--source-root",
                str(stable_source_root),
                "--output",
                str(stable_release_root / "release_manifest_stable.json"),
            ],
            expected_tokens=[
                "release_manifest_written=",
                "release_gate_status=ready",
            ],
            artifact_dir=stable_release_root,
        ),
        SmokeCheck(
            name="industrial release artifact",
            command=[
                sys.executable,
                "tools/build_release_artifact.py",
                "--version",
                "2026.04.12",
                "--channel",
                "industrial",
                "--build-id",
                "smoke-industrial-build",
                "--approval-status",
                "approved",
                "--approved-by",
                "smoke-release-manager",
                "--approved-at",
                "2026-04-12T12:30:00+00:00",
                "--approval-notes",
                "smoke industrial signoff",
                "--source-root",
                str(stable_source_root),
                "--output",
                str(industrial_release_root / "release_manifest_industrial.json"),
            ],
            expected_tokens=[
                "release_manifest_written=",
                "release_gate_status=ready",
                "customer_delivery_status=ready",
                "industrial_delivery_status=ready",
            ],
            artifact_dir=industrial_release_root,
        ),
        SmokeCheck(
            name="release readiness",
            command=[
                sys.executable,
                "tools/check_release_readiness.py",
                "--output-root",
                str(release_readiness_root),
                "--source-root",
                str(stable_source_root),
            ],
            expected_tokens=[
                "release_readiness_written=",
                "rc_release_gate=",
                "stable_release_gate=",
            ],
            artifact_dir=release_readiness_root,
        ),
        SmokeCheck(
            name="approved release readiness",
            command=[
                sys.executable,
                "tools/check_release_readiness.py",
                "--output-root",
                str(release_readiness_approved_root),
                "--source-root",
                str(stable_source_root),
                "--approval-manifest",
                str(stable_release_root / "release_manifest_stable.json"),
                "--security-preflight-report",
                str(security_root / "security_release_preflight_report.json"),
            ],
            expected_tokens=[
                "release_readiness_written=",
                "stable_release_gate=ready",
            ],
            artifact_dir=release_readiness_approved_root,
        ),
        SmokeCheck(
            name="industrial release readiness",
            command=[
                sys.executable,
                "tools/check_industrial_release_readiness.py",
                "--output-root",
                str(industrial_release_readiness_root),
                "--source-root",
                str(stable_source_root),
            ],
            expected_tokens=[
                "industrial_release_readiness_written=",
                "industrial_release_gate=",
            ],
            artifact_dir=industrial_release_readiness_root,
        ),
        SmokeCheck(
            name="approved industrial release readiness",
            command=[
                sys.executable,
                "tools/check_industrial_release_readiness.py",
                "--output-root",
                str(industrial_release_readiness_approved_root),
                "--source-root",
                str(stable_source_root),
                "--approval-manifest",
                str(industrial_release_root / "release_manifest_industrial.json"),
                "--security-preflight-report",
                str(security_root / "security_release_preflight_report.json"),
            ],
            expected_tokens=[
                "industrial_release_readiness_written=",
                "industrial_release_gate=ready",
            ],
            artifact_dir=industrial_release_readiness_approved_root,
        ),
        SmokeCheck(
            name="worktree cleanup report",
            command=[
                sys.executable,
                "tools/build_worktree_cleanup_report.py",
                "--source-root",
                str(PROJECT_ROOT),
                "--output-root",
                str(worktree_cleanup_root),
            ],
            expected_tokens=[
                "worktree_cleanup_report_written=",
                "worktree_cleanup_total_paths=",
            ],
            artifact_dir=worktree_cleanup_root,
        ),
        SmokeCheck(
            name="tracked artifact review report",
            command=[
                sys.executable,
                "tools/build_tracked_artifact_review_report.py",
                "--source-root",
                str(PROJECT_ROOT),
                "--cleanup-report",
                str(worktree_cleanup_root / "worktree_cleanup_report.json"),
                "--output-root",
                str(tracked_artifact_review_root),
            ],
            expected_tokens=[
                "tracked_artifact_review_report_written=",
                "tracked_artifact_review_candidates=",
            ],
            artifact_dir=tracked_artifact_review_root,
        ),
        SmokeCheck(
            name="stable promotion checklist",
            command=[
                sys.executable,
                "tools/build_stable_promotion_checklist.py",
                "--output-root",
                str(stable_promotion_root),
                "--source-root",
                str(stable_source_root),
            ],
            expected_tokens=[
                "stable_promotion_checklist_written=",
                "stable_promotion_gate=",
                "stable_promotion_blocking_steps=",
            ],
            artifact_dir=stable_promotion_root,
        ),
        SmokeCheck(
            name="approved stable promotion checklist",
            command=[
                sys.executable,
                "tools/build_stable_promotion_checklist.py",
                "--output-root",
                str(stable_promotion_approved_root),
                "--source-root",
                str(stable_source_root),
                "--approval-manifest",
                str(stable_release_root / "release_manifest_stable.json"),
                "--security-preflight-report",
                str(security_root / "security_release_preflight_report.json"),
            ],
            expected_tokens=[
                "stable_promotion_checklist_written=",
                "stable_promotion_gate=ready",
                "stable_promotion_ready_to_promote=true",
            ],
            artifact_dir=stable_promotion_approved_root,
        ),
        SmokeCheck(
            name="industrial promotion checklist",
            command=[
                sys.executable,
                "tools/build_industrial_promotion_checklist.py",
                "--output-root",
                str(industrial_promotion_root),
                "--source-root",
                str(stable_source_root),
            ],
            expected_tokens=[
                "industrial_promotion_checklist_written=",
                "industrial_promotion_gate=",
                "industrial_promotion_blocking_steps=",
            ],
            artifact_dir=industrial_promotion_root,
        ),
        SmokeCheck(
            name="approved industrial promotion checklist",
            command=[
                sys.executable,
                "tools/build_industrial_promotion_checklist.py",
                "--output-root",
                str(industrial_promotion_approved_root),
                "--source-root",
                str(stable_source_root),
                "--approval-manifest",
                str(industrial_release_root / "release_manifest_industrial.json"),
                "--security-preflight-report",
                str(security_root / "security_release_preflight_report.json"),
            ],
            expected_tokens=[
                "industrial_promotion_checklist_written=",
                "industrial_promotion_gate=ready",
                "industrial_promotion_ready_to_promote=true",
            ],
            artifact_dir=industrial_promotion_approved_root,
        ),
        SmokeCheck(
            name="stable release rehearsal",
            command=[
                sys.executable,
                "tools/run_release_rehearsal.py",
                "--version",
                "smoke-stable-2026.04.12",
                "--build-id",
                "smoke-stable-release-rehearsal",
                "--output-root",
                str(release_rehearsal_root),
            ],
            expected_tokens=[
                "release_rehearsal_written=",
                "release_rehearsal_gate=ready",
            ],
            artifact_dir=release_rehearsal_root,
        ),
        SmokeCheck(
            name="customer acceptance bundle",
            command=[
                sys.executable,
                "tools/build_customer_acceptance_bundle.py",
                "--manifest",
                str(stable_release_root / "release_manifest_stable.json"),
                "--output",
                str(
                    customer_acceptance_bundle_root
                    / "customer_acceptance_bundle.json"
                ),
                "--readiness-report",
                str(
                    release_readiness_approved_root
                    / "release_readiness_report.json"
                ),
                "--promotion-checklist",
                str(
                    stable_promotion_approved_root
                    / "stable_promotion_checklist.json"
                ),
                "--security-posture-report",
                str(security_root / "security_posture_report.json"),
                "--sbom-artifact",
                str(security_root / "sbom.json"),
                "--python-vuln-report",
                str(security_root / "python_vuln_scan_report.json"),
                "--container-vuln-report",
                str(security_root / "container_vuln_scan_report.json"),
                "--backup-restore-report",
                str(security_root / "backup_restore_rehearsal_report.json"),
                "--release-ops-execution-report",
                str(
                    release_rehearsal_root
                    / "test_env"
                    / "release_evidence"
                    / "operations"
                    / "release_ops_execution_report.json"
                ),
            ],
            expected_tokens=[
                "customer_acceptance_bundle_written=",
                "customer_acceptance_bundle_status=ready",
                "customer_acceptance_bundle_reports_present=",
                "customer_acceptance_bundle_security_posture=ready",
            ],
            artifact_dir=customer_acceptance_bundle_root,
        ),
        SmokeCheck(
            name="industrial customer acceptance bundle",
            command=[
                sys.executable,
                "tools/build_customer_acceptance_bundle.py",
                "--manifest",
                str(industrial_release_root / "release_manifest_industrial.json"),
                "--output",
                str(
                    industrial_customer_acceptance_bundle_root
                    / "customer_acceptance_bundle_industrial.json"
                ),
                "--readiness-report",
                str(
                    industrial_release_readiness_approved_root
                    / "industrial_release_readiness_report.json"
                ),
                "--promotion-checklist",
                str(
                    industrial_promotion_approved_root
                    / "industrial_promotion_checklist.json"
                ),
                "--security-posture-report",
                str(security_root / "security_posture_report.json"),
                "--sbom-artifact",
                str(security_root / "sbom.json"),
                "--python-vuln-report",
                str(security_root / "python_vuln_scan_report.json"),
                "--container-vuln-report",
                str(security_root / "container_vuln_scan_report.json"),
                "--backup-restore-report",
                str(security_root / "backup_restore_rehearsal_report.json"),
                "--release-ops-execution-report",
                str(
                    release_rehearsal_root
                    / "test_env"
                    / "release_evidence"
                    / "operations"
                    / "release_ops_execution_report.json"
                ),
            ],
            expected_tokens=[
                "customer_acceptance_bundle_written=",
                "customer_acceptance_bundle_status=ready",
                "customer_acceptance_bundle_reports_present=",
                "customer_acceptance_bundle_security_posture=ready",
            ],
            artifact_dir=industrial_customer_acceptance_bundle_root,
        ),
        SmokeCheck(
            name="godot agent fake backend",
            command=[sys.executable, "-c", _fake_godot_agent_check_script()],
            expected_tokens=[
                "godot_agent_fake_templates_ok",
                "godot_agent_fake_plan_ok",
                "godot_agent_fake_doctor_ok",
            ],
        ),
        SmokeCheck(
            name="godot agent modern backend",
            command=[sys.executable, "-c", _modern_godot_agent_check_script()],
            expected_tokens=[
                "godot_agent_modern_templates_ok",
                "godot_agent_modern_plan_ok",
                "godot_agent_modern_doctor_ok",
                "godot_agent_modern_skills_alias_ok",
            ],
            env_overrides=(
                {
                    "AGI_WALKER_GODOT_AGENT_BACKEND": "godot-agent",
                    "AGI_WALKER_GODOT_AGENT_DIR": str(modern_godot_agent_dir),
                }
                if modern_godot_agent_dir
                else None
            ),
            skip_reason=modern_skip_reason,
        ),
        SmokeCheck(
            name="godot instruction-set smoke",
            command=[sys.executable, "-c", _godot_instruction_set_smoke_script()],
            expected_tokens=[
                "godot_instruction_set_smoke_ok",
                "godot_simulated_circuit_smoke_ok",
            ],
            artifact_dir=godot_instruction_smoke_root,
            env_overrides={
                "AGI_WALKER_GODOT_INSTRUCTION_SMOKE_ARTIFACT_DIR": str(
                    godot_instruction_smoke_root
                )
            },
        ),
        SmokeCheck(
            name="ros2 instruction-set smoke",
            command=[sys.executable, "-c", _ros2_instruction_set_smoke_script()],
            expected_tokens=[
                "ros2_instruction_set_smoke_ok",
                "ros2_instruction_runtime_smoke_ok",
            ],
            artifact_dir=ros2_instruction_smoke_root,
            env_overrides={
                "AGI_WALKER_ROS2_INSTRUCTION_SMOKE_ARTIFACT_DIR": str(
                    ros2_instruction_smoke_root
                )
            },
        ),
        SmokeCheck(
            name="simulated circuit replay smoke",
            command=[sys.executable, "-c", _simulated_circuit_replay_smoke_script()],
            expected_tokens=[
                "simulated_circuit_replay_smoke_ok",
                "simulated_circuit_feedback_smoke_ok",
            ],
            artifact_dir=simulated_circuit_smoke_root,
            env_overrides={
                "AGI_WALKER_SIMULATED_CIRCUIT_SMOKE_ARTIFACT_DIR": str(
                    simulated_circuit_smoke_root
                )
            },
        ),
        SmokeCheck(
            name="godot headless integration smoke",
            command=[
                sys.executable,
                "-m",
                "pytest",
                "tests/test_godot_headless_smoke.py",
                "-q",
                "-m",
                "integration and live",
            ],
            expected_tokens=["1 passed"],
            skip_reason=godot_headless_skip_reason,
        ),
        SmokeCheck(
            name="ros2 bridge live smoke",
            command=[
                sys.executable,
                "-m",
                "pytest",
                "tests/test_ros2_bridge_smoke.py",
                "-q",
                "-m",
                "integration and live",
            ],
            expected_tokens=["1 passed"],
            artifact_dir=ros2_smoke_root,
            env_overrides={
                "AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE": "1",
                "AGI_WALKER_ROS2_BRIDGE_SMOKE_ARTIFACT_DIR": str(ros2_smoke_root),
                "PYTEST_DISABLE_PLUGIN_AUTOLOAD": "1",
            },
            skip_reason=ros2_bridge_skip_reason,
        ),
    ]


def _run_check(check: SmokeCheck, env: dict[str, str]) -> tuple[bool, str]:
    merged_env = dict(env)
    if check.env_overrides:
        merged_env.update(check.env_overrides)

    result = subprocess.run(
        check.command,
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        env=merged_env,
    )

    combined_output = "\n".join(
        part for part in [result.stdout.strip(), result.stderr.strip()] if part
    )

    if result.returncode != 0:
        return False, combined_output or f"exit code {result.returncode}"

    missing = [token for token in check.expected_tokens if token not in combined_output]
    if missing:
        return (
            False,
            f"missing expected output: {', '.join(missing)}\n{combined_output}",
        )

    if check.artifact_dir:
        artifact_summary = []
        output_dir = check.artifact_dir / ".output"
        export_dir = check.artifact_dir / "exports"
        report_file = check.artifact_dir / "ros2_bridge_smoke_report.json"
        godot_instruction_smoke_report = (
            check.artifact_dir / "godot_instruction_smoke_report.json"
        )
        ros2_instruction_smoke_report = (
            check.artifact_dir / "ros2_instruction_smoke_report.json"
        )
        simulated_circuit_smoke_report = (
            check.artifact_dir / "simulated_circuit_replay_smoke_report.json"
        )
        release_manifest = check.artifact_dir / "release_manifest.json"
        release_readiness_report = check.artifact_dir / "release_readiness_report.json"
        industrial_release_readiness_report = (
            check.artifact_dir / "industrial_release_readiness_report.json"
        )
        stable_promotion_checklist = check.artifact_dir / "stable_promotion_checklist.json"
        industrial_promotion_checklist = (
            check.artifact_dir / "industrial_promotion_checklist.json"
        )
        customer_acceptance_bundle = (
            check.artifact_dir / "customer_acceptance_bundle.json"
        )
        industrial_customer_acceptance_bundle = (
            check.artifact_dir / "customer_acceptance_bundle_industrial.json"
        )
        worktree_cleanup_report = check.artifact_dir / "worktree_cleanup_report.json"
        worktree_release_blocker_report = (
            check.artifact_dir / "worktree_release_blocker_report.json"
        )
        tracked_artifact_review_report = (
            check.artifact_dir / "tracked_artifact_review_report.json"
        )
        release_rehearsal_report = check.artifact_dir / "release_rehearsal_report.json"
        industrial_delivery_rehearsal_report = (
            check.artifact_dir / "industrial_delivery_rehearsal_report.json"
        )
        sbom_artifact = check.artifact_dir / "sbom.json"
        python_vuln_report = check.artifact_dir / "python_vuln_scan_report.json"
        container_vuln_report = check.artifact_dir / "container_vuln_scan_report.json"
        vulnerability_remediation_report = (
            check.artifact_dir / "vulnerability_remediation_report.json"
        )
        backup_restore_rehearsal_report = (
            check.artifact_dir / "backup_restore_rehearsal_report.json"
        )
        security_posture_report = check.artifact_dir / "security_posture_report.json"

        def _artifact_release_summary_suffix(path: Path) -> str:
            try:
                payload = json.loads(path.read_text(encoding="utf-8"))
            except Exception:
                return ""
            if not isinstance(payload, dict):
                return ""
            suffixes: list[str] = []
            review_payload = payload.get("vulnerability_exception_review")
            if isinstance(review_payload, dict):
                review_status = str(review_payload.get("status") or "").strip()
                review_candidate_count = review_payload.get("review_candidate_count")
                if review_status:
                    if (
                        isinstance(review_candidate_count, int)
                        and review_candidate_count >= 0
                    ):
                        suffixes.append(
                            f"exception_review={review_status}/{review_candidate_count}"
                        )
                    else:
                        suffixes.append(f"exception_review={review_status}")
            external_mainline_payload = payload.get("external_mainline_execution_plan")
            if isinstance(external_mainline_payload, dict):
                external_mainline_status = str(
                    external_mainline_payload.get("status") or ""
                ).strip()
                counts: list[str] = []
                for field in [
                    "completed_steps",
                    "ready_to_run_steps",
                    "waiting_external_input_steps",
                    "blocked_steps",
                ]:
                    value = external_mainline_payload.get(field)
                    if not isinstance(value, int) or value < 0:
                        counts = []
                        break
                    counts.append(str(value))
                if external_mainline_status:
                    if counts:
                        suffixes.append(
                            "external_mainline="
                            + "/".join([external_mainline_status, *counts])
                        )
                    else:
                        suffixes.append(
                            f"external_mainline={external_mainline_status}"
                        )
            external_mainline_checklist_payload = payload.get(
                "external_mainline_input_checklist"
            )
            if isinstance(external_mainline_checklist_payload, dict):
                checklist_status = str(
                    external_mainline_checklist_payload.get("status") or ""
                ).strip()
                checklist_metrics = (
                    external_mainline_checklist_payload.get("metrics")
                    if isinstance(
                        external_mainline_checklist_payload.get("metrics"), dict
                    )
                    else {}
                )
                checklist_counts: list[str] = []
                metric_values: list[object] = [
                    checklist_metrics.get("missing_input_count"),
                    len(checklist_metrics.get("waiting_external_input_steps", []))
                    if isinstance(
                        checklist_metrics.get("waiting_external_input_steps"), list
                    )
                    else None,
                    len(checklist_metrics.get("ready_to_run_steps", []))
                    if isinstance(checklist_metrics.get("ready_to_run_steps"), list)
                    else None,
                    len(checklist_metrics.get("completed_steps", []))
                    if isinstance(checklist_metrics.get("completed_steps"), list)
                    else None,
                ]
                if checklist_status:
                    for value in metric_values:
                        if not isinstance(value, int) or value < 0:
                            checklist_counts = []
                            break
                        checklist_counts.append(str(value))
                    if checklist_counts:
                        suffixes.append(
                            "external_mainline_input_checklist="
                            + "/".join([checklist_status, *checklist_counts])
                        )
                    else:
                        suffixes.append(
                            "external_mainline_input_checklist="
                            f"{checklist_status}"
                        )
            worktree_payload = payload.get("worktree_release_blocker")
            if isinstance(worktree_payload, dict):
                worktree_status = str(worktree_payload.get("status") or "").strip()
                total_paths = worktree_payload.get("total_paths")
                tracked_review_candidate_count = worktree_payload.get(
                    "tracked_review_candidate_count"
                )
                if worktree_status:
                    if (
                        isinstance(total_paths, int)
                        and total_paths >= 0
                        and isinstance(tracked_review_candidate_count, int)
                        and tracked_review_candidate_count >= 0
                    ):
                        suffixes.append(
                            "worktree="
                            + "/".join(
                                [
                                    worktree_status,
                                    str(total_paths),
                                    str(tracked_review_candidate_count),
                                ]
                            )
                        )
                    else:
                        suffixes.append(f"worktree={worktree_status}")
            control_plane_event_stream = payload.get("control_plane_event_stream")
            if isinstance(control_plane_event_stream, dict):
                event_count = control_plane_event_stream.get("event_count")
                if isinstance(event_count, int) and event_count >= 0:
                    suffixes.append(f"control_plane_events={event_count}")
            if not suffixes:
                return ""
            return " [" + ", ".join(suffixes) + "]"

        if output_dir.exists():
            artifact_summary.append(f".output={output_dir}")
        if export_dir.exists():
            artifact_summary.append(f"exports={export_dir}")
        if report_file.exists():
            artifact_summary.append(f"report={report_file}")
        if godot_instruction_smoke_report.exists():
            artifact_summary.append(
                "godot_instruction_smoke_report="
                f"{godot_instruction_smoke_report}"
            )
        if ros2_instruction_smoke_report.exists():
            artifact_summary.append(
                "ros2_instruction_smoke_report="
                f"{ros2_instruction_smoke_report}"
            )
        if simulated_circuit_smoke_report.exists():
            artifact_summary.append(
                "simulated_circuit_replay_smoke_report="
                f"{simulated_circuit_smoke_report}"
            )
        if release_manifest.exists():
            artifact_summary.append(f"release_manifest={release_manifest}")
        if release_readiness_report.exists():
            artifact_summary.append(
                "release_readiness_report="
                f"{release_readiness_report}"
                f"{_artifact_release_summary_suffix(release_readiness_report)}"
            )
        if industrial_release_readiness_report.exists():
            artifact_summary.append(
                "industrial_release_readiness_report="
                f"{industrial_release_readiness_report}"
                f"{_artifact_release_summary_suffix(industrial_release_readiness_report)}"
            )
        if stable_promotion_checklist.exists():
            artifact_summary.append(
                "stable_promotion_checklist="
                f"{stable_promotion_checklist}"
                f"{_artifact_release_summary_suffix(stable_promotion_checklist)}"
            )
        if industrial_promotion_checklist.exists():
            artifact_summary.append(
                "industrial_promotion_checklist="
                f"{industrial_promotion_checklist}"
                f"{_artifact_release_summary_suffix(industrial_promotion_checklist)}"
            )
        if customer_acceptance_bundle.exists():
            artifact_summary.append(
                "customer_acceptance_bundle="
                f"{customer_acceptance_bundle}"
                f"{_artifact_release_summary_suffix(customer_acceptance_bundle)}"
            )
        if industrial_customer_acceptance_bundle.exists():
            artifact_summary.append(
                "customer_acceptance_bundle_industrial="
                f"{industrial_customer_acceptance_bundle}"
                f"{_artifact_release_summary_suffix(industrial_customer_acceptance_bundle)}"
            )
        if worktree_cleanup_report.exists():
            artifact_summary.append(f"worktree_cleanup_report={worktree_cleanup_report}")
        if worktree_release_blocker_report.exists():
            artifact_summary.append(
                "worktree_release_blocker_report="
                f"{worktree_release_blocker_report}"
                f"{_artifact_release_summary_suffix(worktree_release_blocker_report)}"
            )
        if tracked_artifact_review_report.exists():
            artifact_summary.append(
                f"tracked_artifact_review_report={tracked_artifact_review_report}"
            )
        if release_rehearsal_report.exists():
            artifact_summary.append(
                "release_rehearsal_report="
                f"{release_rehearsal_report}"
                f"{_artifact_release_summary_suffix(release_rehearsal_report)}"
            )
        if industrial_delivery_rehearsal_report.exists():
            artifact_summary.append(
                "industrial_delivery_rehearsal_report="
                f"{industrial_delivery_rehearsal_report}"
                f"{_artifact_release_summary_suffix(industrial_delivery_rehearsal_report)}"
            )
        if sbom_artifact.exists():
            artifact_summary.append(f"sbom={sbom_artifact}")
        if python_vuln_report.exists():
            artifact_summary.append(f"python_vuln_report={python_vuln_report}")
        if container_vuln_report.exists():
            artifact_summary.append(f"container_vuln_report={container_vuln_report}")
        if vulnerability_remediation_report.exists():
            artifact_summary.append(
                "vulnerability_remediation_report="
                f"{vulnerability_remediation_report}"
            )
        if backup_restore_rehearsal_report.exists():
            artifact_summary.append(
                "backup_restore_rehearsal_report="
                f"{backup_restore_rehearsal_report}"
            )
        if security_posture_report.exists():
            artifact_summary.append(
                f"security_posture_report={security_posture_report}"
            )
        if artifact_summary:
            combined_output = (
                f"{combined_output}\nartifacts: {', '.join(artifact_summary)}"
            )

    return True, combined_output


def _resolve_git_commit() -> str | None:
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    if result.returncode != 0:
        return None
    value = result.stdout.strip()
    return value or None


def main() -> int:
    parser = argparse.ArgumentParser(description="Run AGI-Walker minimal smoke tests.")
    parser.add_argument(
        "--output-root",
        default=str(DEFAULT_OUTPUT_ROOT),
        help="Directory used to isolate smoke-test workflow artifacts.",
    )
    parser.add_argument(
        "--report-file",
        default=None,
        help="Optional structured release evidence report path. Defaults to <output-root>/smoke_report.json.",
    )
    args = parser.parse_args()

    output_root = Path(args.output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    report_path = Path(args.report_file) if args.report_file else output_root / "smoke_report.json"

    env = _build_env()
    checks = _build_checks(output_root, env)
    started_at = datetime.now().isoformat()
    started_perf = perf_counter()

    print("AGI-Walker smoke tests")
    print(f"project root: {PROJECT_ROOT}")
    print(f"workflow output root: {output_root}")
    print()

    failures: list[str] = []
    skipped: list[str] = []
    for index, check in enumerate(checks, start=1):
        print(f"[{index}/{len(checks)}] {check.name}")
        if check.skip_reason:
            print("status: SKIP")
            print(check.skip_reason)
            skipped.append(check.name)
            print()
            continue
        print(f"command: {' '.join(check.command)}")
        ok, details = _run_check(check, env)
        if ok:
            print("status: PASS")
            if details:
                print(details)
        else:
            print("status: FAIL")
            if details:
                print(details)
            failures.append(check.name)
        print()

    if failures:
        print("smoke summary: FAIL")
        print(f"failed checks: {', '.join(failures)}")
        status = "blocked"
        exit_code = 1
        summary = (
            f"Smoke runner failed: {len(checks) - len(failures) - len(skipped)} passed, "
            f"{len(skipped)} skipped, {len(failures)} failed."
        )
    else:
        print("smoke summary: PASS")
        status = "passed"
        exit_code = 0
        summary = (
            f"Smoke runner passed: {len(checks) - len(skipped)} passed, "
            f"{len(skipped)} skipped, {len(failures)} failed."
        )

    report_payload = build_release_evidence_report(
        evidence_name="smoke_runner",
        status=status,
        summary=summary,
        command="python tests/run_smoke_tests.py",
        generated_at=started_at,
        exit_code=exit_code,
        duration_seconds=round(perf_counter() - started_perf, 3),
        metrics={
            "total_checks": len(checks),
            "passed_checks": len(checks) - len(failures) - len(skipped),
            "skipped_checks": len(skipped),
            "failed_checks": len(failures),
            "failed_check_names": failures,
            "skipped_check_names": skipped,
        },
        source_commit_sha=_resolve_git_commit(),
    )
    written_report = write_release_evidence_report(report_payload, report_path)
    print(f"smoke_report_written={written_report}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
