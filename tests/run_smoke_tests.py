"""
Minimal smoke test runner for AGI-Walker.

Runs a small set of high-signal checks against the CLI, workflow engine,
and basic Web panel imports. Workflow artifacts are isolated under
test_env/smoke_runs/<timestamp>/ by default.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Sequence


PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUTPUT_ROOT = (
    PROJECT_ROOT / "test_env" / "smoke_runs" / datetime.now().strftime("%Y%m%d_%H%M%S")
)

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")


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


def _build_checks(output_root: Path, env: dict[str, str]) -> list[SmokeCheck]:
    mock_root = output_root / "robot_creation_mock"
    real_root = output_root / "robot_creation_real"
    release_root = output_root / "release"
    stable_release_root = output_root / "release_stable"
    release_readiness_root = output_root / "release_readiness"
    release_readiness_approved_root = output_root / "release_readiness_approved"
    worktree_cleanup_root = output_root / "worktree_cleanup"
    tracked_artifact_review_root = output_root / "tracked_artifact_review"
    stable_promotion_root = output_root / "stable_promotion"
    stable_promotion_approved_root = output_root / "stable_promotion_approved"
    release_rehearsal_root = output_root / "release_rehearsal"
    ros2_smoke_root = output_root / "ros2_bridge_smoke"
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
            name="release readiness",
            command=[
                sys.executable,
                "tools/check_release_readiness.py",
                "--output-root",
                str(release_readiness_root),
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
                "--approval-manifest",
                str(stable_release_root / "release_manifest_stable.json"),
            ],
            expected_tokens=[
                "release_readiness_written=",
                "stable_release_gate=ready",
            ],
            artifact_dir=release_readiness_approved_root,
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
                "--approval-manifest",
                str(stable_release_root / "release_manifest_stable.json"),
            ],
            expected_tokens=[
                "stable_promotion_checklist_written=",
                "stable_promotion_gate=ready",
                "stable_promotion_ready_to_promote=true",
            ],
            artifact_dir=stable_promotion_approved_root,
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
        release_manifest = check.artifact_dir / "release_manifest.json"
        release_readiness_report = check.artifact_dir / "release_readiness_report.json"
        stable_promotion_checklist = check.artifact_dir / "stable_promotion_checklist.json"
        worktree_cleanup_report = check.artifact_dir / "worktree_cleanup_report.json"
        tracked_artifact_review_report = (
            check.artifact_dir / "tracked_artifact_review_report.json"
        )
        release_rehearsal_report = check.artifact_dir / "release_rehearsal_report.json"
        if output_dir.exists():
            artifact_summary.append(f".output={output_dir}")
        if export_dir.exists():
            artifact_summary.append(f"exports={export_dir}")
        if report_file.exists():
            artifact_summary.append(f"report={report_file}")
        if release_manifest.exists():
            artifact_summary.append(f"release_manifest={release_manifest}")
        if release_readiness_report.exists():
            artifact_summary.append(f"release_readiness_report={release_readiness_report}")
        if stable_promotion_checklist.exists():
            artifact_summary.append(
                f"stable_promotion_checklist={stable_promotion_checklist}"
            )
        if worktree_cleanup_report.exists():
            artifact_summary.append(f"worktree_cleanup_report={worktree_cleanup_report}")
        if tracked_artifact_review_report.exists():
            artifact_summary.append(
                f"tracked_artifact_review_report={tracked_artifact_review_report}"
            )
        if release_rehearsal_report.exists():
            artifact_summary.append(f"release_rehearsal_report={release_rehearsal_report}")
        if artifact_summary:
            combined_output = (
                f"{combined_output}\nartifacts: {', '.join(artifact_summary)}"
            )

    return True, combined_output


def main() -> int:
    parser = argparse.ArgumentParser(description="Run AGI-Walker minimal smoke tests.")
    parser.add_argument(
        "--output-root",
        default=str(DEFAULT_OUTPUT_ROOT),
        help="Directory used to isolate smoke-test workflow artifacts.",
    )
    args = parser.parse_args()

    output_root = Path(args.output_root)
    output_root.mkdir(parents=True, exist_ok=True)

    env = _build_env()
    checks = _build_checks(output_root, env)

    print("AGI-Walker smoke tests")
    print(f"project root: {PROJECT_ROOT}")
    print(f"workflow output root: {output_root}")
    print()

    failures: list[str] = []
    for index, check in enumerate(checks, start=1):
        print(f"[{index}/{len(checks)}] {check.name}")
        if check.skip_reason:
            print("status: SKIP")
            print(check.skip_reason)
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
        return 1

    print("smoke summary: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
