from pathlib import Path
try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - Python 3.10 compatibility path.
    import tomli as tomllib

import yaml

ROOT_VERSION = Path("VERSION")
DOCKERFILE = Path("Dockerfile")
CONTRIBUTING = Path("CONTRIBUTING.md")
URDF_BATCH_CONVERT = Path("agi_walker/skills/urdf-generator/scripts/batch_convert.py")
PARAMETER_BATCH_OPTIMIZE = Path(
    "agi_walker/skills/parameter-optimizer/scripts/batch_optimize.py"
)
QUICK_START_SH = Path("scripts/quick_start.sh")
QUICK_START_BAT = Path("scripts/quick_start.bat")
INSTALL_SH = Path("scripts/install.sh")
INSTALL_BAT = Path("scripts/install.bat")
ROOT_RELEASE_NOTES = Path("RELEASE_NOTES.md")
GITIGNORE = Path(".gitignore")
API_REFERENCE = Path("docs/API_REFERENCE.md")
MCP_GUIDE = Path("docs/mcp.md")
WEB_PANEL_GUIDE = Path("docs/guides/WEB_PANEL_GUIDE.md")
GODOT_TESTING_GUIDE = Path("docs/guides/GODOT_TESTING_GUIDE.md")
RELEASE_GUIDE = Path("docs/guides/RELEASE_GUIDE.md")
DEPLOYMENT_MATRIX_GUIDE = Path("docs/guides/DEPLOYMENT_MATRIX.md")
CUSTOMER_INSTALLATION_GUIDE = Path("docs/guides/CUSTOMER_INSTALLATION_GUIDE.md")
SUPPORT_MATRIX_GUIDE = Path("docs/guides/SUPPORT_MATRIX.md")
CAPACITY_AND_SCALE_GUIDE = Path("docs/guides/CAPACITY_AND_SCALE.md")
CUSTOMER_ACCEPTANCE_CHECKLIST_GUIDE = Path(
    "docs/guides/CUSTOMER_ACCEPTANCE_CHECKLIST.md"
)
KNOWN_LIMITATIONS_GUIDE = Path("docs/guides/KNOWN_LIMITATIONS.md")
SECURITY_BASELINE_GUIDE = Path("docs/guides/SECURITY_BASELINE.md")
AUDIT_TRAIL_POLICY_GUIDE = Path("docs/guides/AUDIT_TRAIL_POLICY.md")
BACKUP_RESTORE_RUNBOOK_GUIDE = Path("docs/guides/BACKUP_RESTORE_RUNBOOK.md")
INCIDENT_RESPONSE_MATRIX_GUIDE = Path("docs/guides/INCIDENT_RESPONSE_MATRIX.md")
RELEASE_BUILDER = Path("tools/build_release_artifact.py")
SBOM_BUILDER = Path("tools/build_sbom_artifact.py")
VULNERABILITY_SCAN_REPORT_WRITER = Path("tools/write_vulnerability_scan_report.py")
PYTHON_VULNERABILITY_SCAN_RUNNER = Path("tools/run_python_vulnerability_scan.py")
CONTAINER_VULNERABILITY_SCAN_RUNNER = Path("tools/run_container_vulnerability_scan.py")
VULNERABILITY_EXCEPTION_REPORT_BUILDER = Path(
    "tools/build_vulnerability_exception_report.py"
)
VULNERABILITY_REMEDIATION_REPORT_BUILDER = Path(
    "tools/build_vulnerability_remediation_report.py"
)
SECURITY_RELEASE_PREFLIGHT_RUNNER = Path("tools/run_security_release_preflight.py")
BACKUP_RESTORE_REHEARSAL_RUNNER = Path("tools/run_backup_restore_rehearsal.py")
SECURITY_POSTURE_BUILDER = Path("tools/build_security_posture_report.py")
COLLECT_RELEASE_EVIDENCE = Path("tools/collect_release_evidence.py")
ROS2_WORKSPACE_README = Path("hardware/ros2_ws/README.md")
CORE_API_README = Path("agi_walker/core/api/README.md")
PRODUCTION_RUNBOOK = Path("PRODUCTION_DEPLOYMENT_RUNBOOK.md")
COMPOSE_ENV_EXAMPLE = Path("deployment/compose.env.example")
WEB_PANEL_ENV_EXAMPLE = Path("deployment/web_panel.env.example")
WEB_PANEL_DOCKERFILE = Path("deployment/Dockerfile.web_panel")
GODOT_STUDIO_SETUP = Path("godot_studio_agent/setup.bat")
GODOT_STUDIO_MAIN = Path("godot_studio_agent/main.py")
DOCTOR_MODULE = Path("agi_walker/utils/doctor.py")
DISTRIBUTED_COMPOSE = Path("deployment/docker-compose.yml")
DISTRIBUTED_RUNTIME_DOCKERFILE = Path("deployment/Dockerfile.distributed_runtime")
DISTRIBUTED_DOCKERFILE = Path("deployment/Dockerfile")
DISTIBUTED_RUNTIME_REQUIREMENTS = Path(
    "deployment/requirements.distributed_runtime.txt"
)
DISTIBUTED_SIDECAR = Path("agi_walker/core/distributed/sidecar.py")
HEADLESS_SMOKE_TEST = Path("tests/test_godot_headless_smoke.py")
RUN_SMOKE_TESTS = Path("tests/run_smoke_tests.py")
CI_WORKFLOW = Path(".github/workflows/ci.yml")
TESTING_GUIDE = Path("docs/guides/TESTING_GUIDE.md")
INSTRUCTION_CONTROL_DEMO_RUNBOOK = Path(
    "docs/guides/INSTRUCTION_CONTROL_DEMO_RUNBOOK.md"
)
NEXT_STAGE_EXECUTION_PLAN = Path("docs/guides/NEXT_STAGE_EXECUTION_PLAN_20260426.md")
OPERATOR_HISTORY_TIMELINE = Path("web_panel/static/operator-history-timeline.html")
WEB_BROWSER_MANUAL_VALIDATION_CHECKLIST = Path(
    "docs/guides/WEB_BROWSER_MANUAL_VALIDATION_CHECKLIST_20260426.md"
)

OPT_IN_CI_TRIGGER = (
    "github.event_name == 'workflow_dispatch' || github.event_name == 'schedule'"
)
OPT_IN_LIVE_CI_JOB_CONTRACTS = {
    "distributed-smoke": {
        "artifact_name": "distributed-smoke-artifacts",
        "artifact_path": "test_env/distributed_smoke",
        "retention_days": 7,
        "required_fragments": [
            "docker version",
            "docker compose version",
            "tests/run_distributed_smoke.py --build --stop-after",
            "test_env/distributed_smoke/distributed_smoke_report.json",
        ],
    },
    "godot-headless-smoke": {
        "artifact_name": "godot-headless-smoke-artifacts",
        "artifact_path": "test_env/godot_headless_smoke",
        "retention_days": 7,
        "env": {
            "AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE": "1",
            "AGI_WALKER_GODOT_HEADLESS_ARTIFACT_DIR": "test_env/godot_headless_smoke",
        },
        "required_fragments": [
            "tests/test_godot_headless_smoke.py",
            '"integration and live"',
        ],
    },
    "dynamic-godot-live-verification": {
        "artifact_name": "dynamic-godot-live-verification-artifacts",
        "artifact_path": "test_env/dynamic_godot_live",
        "retention_days": 14,
        "env": {
            "AGI_WALKER_DYNAMIC_GODOT_LIVE_ARTIFACT_ROOT": "test_env/dynamic_godot_live",
        },
        "required_fragments": [
            "--dry-run-discovery",
            "--live-profile $profile",
            "GODOT_EXECUTABLE",
            "tools/build_dynamic_godot_release_readiness.py",
        ],
    },
    "ros2-bridge-smoke": {
        "artifact_name": "ros2-bridge-smoke-artifacts",
        "artifact_path": "test_env/ros2_bridge_smoke",
        "retention_days": 7,
        "env": {
            "AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE": "1",
            "AGI_WALKER_ROS2_BRIDGE_SMOKE_ARTIFACT_DIR": "test_env/ros2_bridge_smoke",
        },
        "required_fragments": [
            "ros-humble-ros-base",
            "tests/test_ros2_bridge_smoke.py",
            '"integration and live"',
        ],
    },
}


def _workflow_jobs() -> dict:
    return yaml.safe_load(CI_WORKFLOW.read_text(encoding="utf-8"))["jobs"]


def _joined_step_runs(job: dict) -> str:
    return "\n".join(
        str(step.get("run", ""))
        for step in job.get("steps", [])
        if isinstance(step, dict)
    )


def _artifact_upload_step(job: dict, artifact_name: str) -> dict:
    for step in job.get("steps", []):
        if not isinstance(step, dict):
            continue
        step_with = step.get("with", {})
        if isinstance(step_with, dict) and step_with.get("name") == artifact_name:
            return step
    raise AssertionError(f"missing artifact upload step: {artifact_name}")


def test_dockerfile_does_not_reference_removed_source_layout() -> None:
    content = DOCKERFILE.read_text(encoding="utf-8")

    assert "COPY requirements.txt ." not in content
    assert "COPY python_api/ ./python_api/" not in content
    assert "COPY python_controller/ ./python_controller/" not in content
    assert "COPY pyproject.toml ." in content
    assert "COPY agi_walker/ ./agi_walker/" in content
    assert "COPY web_panel/ ./web_panel/" in content
    assert "RUN pip install --no-cache-dir ." in content


def test_root_version_matches_pyproject_release_metadata() -> None:
    version = ROOT_VERSION.read_text(encoding="utf-8").strip()
    pyproject = tomllib.loads(Path("pyproject.toml").read_text(encoding="utf-8"))

    assert version
    assert version == pyproject["project"]["version"]


def test_contributing_uses_current_source_directories() -> None:
    content = CONTRIBUTING.read_text(encoding="utf-8")

    assert "flake8 python_api/ python_controller/" not in content
    assert "black --check python_api/ python_controller/" not in content
    assert "flake8 agi_walker/ web_panel/ tests/" in content
    assert "black --check agi_walker/ web_panel/ tests/" in content


def test_skill_batch_scripts_do_not_use_brittle_repo_root_traversal() -> None:
    urdf_content = URDF_BATCH_CONVERT.read_text(encoding="utf-8")
    optimizer_content = PARAMETER_BATCH_OPTIMIZE.read_text(encoding="utf-8")

    hardcoded_root = (
        "sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))"
    )

    assert hardcoded_root not in urdf_content
    assert hardcoded_root not in optimizer_content
    assert "Path(__file__).resolve().parents" in urdf_content
    assert "Path(__file__).resolve().parents" in optimizer_content


def test_root_scripts_install_from_pyproject_instead_of_missing_requirements_files() -> (
    None
):
    quick_start_sh = QUICK_START_SH.read_text(encoding="utf-8")
    quick_start_bat = QUICK_START_BAT.read_text(encoding="utf-8")
    install_sh = INSTALL_SH.read_text(encoding="utf-8")
    install_bat = INSTALL_BAT.read_text(encoding="utf-8")

    assert "requirements.txt" not in install_sh
    assert "requirements.txt" not in install_bat
    assert "requirements-dev.txt" not in quick_start_sh
    assert "requirements.txt" not in quick_start_bat
    assert 'pip install -q -e ".[dev]"' in quick_start_sh
    assert 'python -m pip install -q -e ".[dev]"' in quick_start_bat
    assert "pip install -e ." in install_sh
    assert "python -m pip install -e ." in install_bat
    assert '"pyproject.toml"' in quick_start_sh


def test_active_docs_do_not_reintroduce_removed_root_requirements_or_python_api_layout() -> (
    None
):
    release_notes = ROOT_RELEASE_NOTES.read_text(encoding="utf-8")
    ros2_readme = ROS2_WORKSPACE_README.read_text(encoding="utf-8")
    core_api_readme = CORE_API_README.read_text(encoding="utf-8")

    assert "pip install -r requirements.txt" not in release_notes
    assert "pip install -r requirements.txt" not in ros2_readme
    assert "cd python_api" not in core_api_readme
    assert "pip install -r requirements.txt" not in core_api_readme
    assert 'pip install -e ".[dev]"' in release_notes
    assert "pip install -e ." in ros2_readme
    assert "pip install -e ." in core_api_readme
    assert (
        "from agi_walker.core.api.godot_robot_env import GodotRobotEnv"
        in core_api_readme
    )


def test_root_release_notes_track_current_release_gate_and_evidence() -> None:
    release_notes = ROOT_RELEASE_NOTES.read_text(encoding="utf-8")

    assert "发布摘要：" in release_notes
    assert "release_gate_status=ready" in release_notes
    assert "test_env/release/release_manifest.json" in release_notes
    assert "distributed_runtime_live=passed" in release_notes
    assert "godot_headless_live=passed" in release_notes
    assert "ros2_bridge_live=passed" in release_notes
    assert "release_approval.status=approved" in release_notes
    assert "release_source" in release_notes
    assert "Industrial Titan" not in release_notes
    assert "Enterprise Ready" not in release_notes


def test_gitignore_covers_runtime_and_generated_cleanup_noise() -> None:
    content = GITIGNORE.read_text(encoding="utf-8")

    assert ".agi_data/sessions/" in content
    assert ".agi_data/trajectories/" in content
    assert ".agi_data/workflows/artifacts/" in content
    assert "agi_walker.db" in content
    assert "configs/generated/" in content
    assert "knowledge/test_index/" in content
    assert "godot_project/.godot_agent_index.json" in content
    assert "agi_walker/core/api/parts/**/__pycache__/" in content


def test_godot_studio_agent_setup_targets_real_cli_entrypoint() -> None:
    setup_content = GODOT_STUDIO_SETUP.read_text(encoding="utf-8")
    main_content = GODOT_STUDIO_MAIN.read_text(encoding="utf-8")

    assert "cd api_server" not in setup_content
    assert "http://localhost:8000/ui" not in setup_content
    assert "python main.py" in setup_content
    assert "from agent_system import GodotStudioRouter" in main_content
    assert "def main() -> int:" in main_content


def test_doctor_checks_current_runtime_dependencies_and_ports() -> None:
    content = DOCTOR_MODULE.read_text(encoding="utf-8")

    assert '"mcp": "mcp"' in content
    assert '"sqlalchemy": "sqlalchemy"' in content
    assert '"aiosqlite": "aiosqlite"' in content
    assert '"PyJWT": "jwt"' in content
    assert '"python-jose": "jose"' not in content
    assert (
        '"prometheus-fastapi-instrumentator": "prometheus_fastapi_instrumentator"'
        in content
    )
    assert '"python-json-logger": "pythonjsonlogger"' in content
    assert '"lxml"' not in content
    assert "ports = [8000, 9999]" in content


def test_distributed_runtime_uses_current_package_entrypoints() -> None:
    compose_content = DISTRIBUTED_COMPOSE.read_text(encoding="utf-8")
    runtime_dockerfile = DISTRIBUTED_RUNTIME_DOCKERFILE.read_text(encoding="utf-8")
    distributed_dockerfile = DISTRIBUTED_DOCKERFILE.read_text(encoding="utf-8")
    runtime_requirements = DISTIBUTED_RUNTIME_REQUIREMENTS.read_text(encoding="utf-8")
    sidecar_content = DISTIBUTED_SIDECAR.read_text(encoding="utf-8")

    assert "distributed/run_learner.py" not in compose_content
    assert "distributed/sidecar.py" not in compose_content
    assert "agi_walker.core.distributed.run_learner" in compose_content
    assert "agi_walker.core.distributed.sidecar" in compose_content
    assert "--godot-host" in compose_content
    assert "--godot-port" in compose_content
    assert "WEB_PANEL_APT_PACKAGES: build-essential" not in compose_content
    web_panel_dockerfile = WEB_PANEL_DOCKERFILE.read_text(encoding="utf-8")
    assert "ARG WEB_PANEL_BASE_IMAGE=python:3.11-alpine" in web_panel_dockerfile
    assert "FROM ${WEB_PANEL_BASE_IMAGE}" in web_panel_dockerfile
    assert "command -v apt-get" in web_panel_dockerfile
    assert "command -v apk" in web_panel_dockerfile
    assert 'ARG WEB_PANEL_APK_PACKAGES="libgcc"' in web_panel_dockerfile
    assert 'ARG WEB_PANEL_APK_BUILD_PACKAGES="build-base"' in web_panel_dockerfile
    assert "apk add --no-cache --virtual .web-panel-build-deps" in web_panel_dockerfile
    assert "apk del .web-panel-build-deps" in web_panel_dockerfile
    assert "rm -rf /root/.cache /tmp/*" in web_panel_dockerfile
    assert "WEB_PANEL_APK_PACKAGES: ${AGI_WALKER_WEB_PANEL_APK_PACKAGES:-libgcc}" in compose_content
    assert (
        "WEB_PANEL_APK_BUILD_PACKAGES: ${AGI_WALKER_WEB_PANEL_APK_BUILD_PACKAGES:-build-base}"
        in compose_content
    )
    assert "AGI_WALKER_WEB_PANEL_BASE_IMAGE:-python:3.11-alpine" in compose_content

    assert "distributed/run_learner.py" not in runtime_dockerfile
    assert "distributed/run_learner.py" not in distributed_dockerfile
    assert "agi_walker.core.distributed.run_learner" in runtime_dockerfile
    assert "agi_walker.core.distributed.run_learner" in distributed_dockerfile
    assert "pip install --no-cache-dir ." in runtime_dockerfile
    assert "import yaml; from agi_walker.skills_loader import SkillsLoader" in runtime_dockerfile
    assert "distributed_runtime_import_ok" in runtime_dockerfile
    assert "pyyaml" in distributed_dockerfile
    assert "import yaml; from agi_walker.skills_loader import SkillsLoader" in distributed_dockerfile
    assert "distributed_base_import_ok" in distributed_dockerfile
    assert "build-essential" not in runtime_dockerfile
    assert "python -m pip install --upgrade pip setuptools wheel" in runtime_dockerfile
    assert (
        "python -m pip install --upgrade pip setuptools wheel" in distributed_dockerfile
    )
    assert "psutil" in runtime_requirements
    assert "pyyaml" in runtime_requirements
    assert '"--godot-host"' in sidecar_content
    assert "AGI_WALKER_GODOT_HOST" in sidecar_content


def test_godot_headless_smoke_wrapper_keeps_live_and_integration_markers() -> None:
    content = HEADLESS_SMOKE_TEST.read_text(encoding="utf-8").replace("\r\n", "\n")

    assert "@pytest.mark.asyncio" not in content
    assert "@pytest.mark.live\n@pytest.mark.integration" in content
    assert (
        "@pytest.mark.integration\ndef test_godot_headless_smoke_lifecycle() -> None:\n"
        in content
    )
    assert "asyncio.run(_run_godot_headless_smoke_lifecycle())" in content


def test_smoke_runner_exposes_opt_in_ros2_bridge_smoke() -> None:
    content = RUN_SMOKE_TESTS.read_text(encoding="utf-8")

    assert "AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE" in content
    assert "tests/test_ros2_bridge_smoke.py" in content
    assert '"integration and live"' in content
    assert "ros2_bridge_smoke_report.json" in content


def test_distributed_smoke_ci_uploads_structured_report() -> None:
    content = CI_WORKFLOW.read_text(encoding="utf-8")

    command = (
        "python tests/run_distributed_smoke.py --build --stop-after --report-file "
        "test_env/distributed_smoke/distributed_smoke_report.json"
    )
    assert "distributed-smoke:" in content
    assert "timeout-minutes: 25" in content
    assert "docker version" in content
    assert "docker compose version" in content
    assert command in content
    assert "name: distributed-smoke-artifacts" in content
    assert "path: test_env/distributed_smoke" in content


def test_ros2_bridge_smoke_ci_job_uploads_structured_report() -> None:
    content = CI_WORKFLOW.read_text(encoding="utf-8")

    assert "ros2-bridge-smoke:" in content
    assert 'runs-on: ubuntu-22.04' in content
    assert 'AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE: "1"' in content
    assert 'AGI_WALKER_ROS2_BRIDGE_SMOKE_ARTIFACT_DIR: "test_env/ros2_bridge_smoke"' in content
    assert 'PYTEST_DISABLE_PLUGIN_AUTOLOAD: "1"' in content
    assert "ros-humble-ros-base" in content
    assert (
        'python -m pytest tests/test_ros2_bridge_smoke.py -q -m "integration and live" --tb=short -vv'
        in content
    )
    assert "name: ros2-bridge-smoke-artifacts" in content
    assert "path: test_env/ros2_bridge_smoke" in content


def test_opt_in_live_external_ci_jobs_keep_auditable_contracts() -> None:
    jobs = _workflow_jobs()

    for job_name, contract in OPT_IN_LIVE_CI_JOB_CONTRACTS.items():
        job = jobs[job_name]
        assert job["if"] == OPT_IN_CI_TRIGGER

        for key, value in contract.get("env", {}).items():
            assert job.get("env", {}).get(key) == value

        step_runs = _joined_step_runs(job)
        for fragment in contract["required_fragments"]:
            assert fragment in step_runs

        upload_step = _artifact_upload_step(job, contract["artifact_name"])
        upload_with = upload_step["with"]
        assert upload_with["path"] == contract["artifact_path"]
        assert upload_with["if-no-files-found"] == "ignore"
        assert upload_with["retention-days"] == contract["retention_days"]


def test_capability_matrix_is_exposed_through_current_docs_and_routes() -> None:
    api_reference = API_REFERENCE.read_text(encoding="utf-8")
    mcp_guide = MCP_GUIDE.read_text(encoding="utf-8")
    web_panel_guide = WEB_PANEL_GUIDE.read_text(encoding="utf-8")

    assert "GET /api/capabilities/matrix" in api_reference
    assert "GET /api/capabilities/matrix" in web_panel_guide
    assert "`capability_matrix_get`" in mcp_guide


def test_release_guide_uses_current_release_builder_and_active_docs() -> None:
    release_guide = RELEASE_GUIDE.read_text(encoding="utf-8")
    release_builder = RELEASE_BUILDER.read_text(encoding="utf-8")
    smoke_runner = RUN_SMOKE_TESTS.read_text(encoding="utf-8")

    assert "python tools/build_release_artifact.py" in release_guide
    assert "release_manifest.json" in release_guide
    assert "docs/archive_and_reports/" in release_guide
    assert "release_policy" in release_guide
    assert "release_source" in release_guide
    assert "worktree" in release_guide
    assert "--approval-status" in release_guide
    assert "Git HEAD" in release_guide
    assert "v{version}" in release_guide
    assert "python tools/check_release_readiness.py" in release_guide
    assert "python tools/check_industrial_release_readiness.py" in release_guide
    assert "--security-preflight-report" in release_guide
    assert "customer_delivery_surface" in release_guide
    assert "extension_support_surface" in release_guide
    assert "extension_execution_plan" in release_guide
    assert "extension_execution_evidence" in release_guide
    assert "extension_execution_instance" in release_guide
    assert "extension_execution_schedule" in release_guide
    assert "extension_execution_actuals" in release_guide
    assert "exception_review_due_at" in release_guide
    assert "approval_identity_source_path" in release_guide
    assert "approval_identity_reference" in release_guide
    assert "archive_target_binding_type" in release_guide
    assert "archive_target_binding_reference_base" in release_guide
    assert "due_trigger_binding_type" in release_guide
    assert "due_trigger_binding_reference_base" in release_guide
    assert "due_trigger_checked_at" in release_guide
    assert "closure_archive/index.json" in release_guide
    assert "runbook_entrypoints" in release_guide
    assert "execution_template" in release_guide
    assert "watch_actions" in release_guide
    assert "on_call_handoff_records" in release_guide
    assert "extension_on_call_rehearsal" in release_guide
    assert "residual_risk_handoff_steps" in release_guide
    assert "exception_review_steps" in release_guide
    assert "extension_exception_review_schedule" in release_guide
    assert "incident_escalation_steps" in release_guide
    assert "escalation_closure_steps" in release_guide
    assert "extension_escalation_closure" in release_guide
    assert "signoff_checkpoints" in release_guide
    assert "rollback_evidence_archive_steps" in release_guide
    assert "deployment_commands" in release_guide
    assert "rollback_prerequisites" in release_guide
    assert "industrial_delivery_gate" in release_guide
    assert "python tools/build_worktree_cleanup_report.py" in release_guide
    assert "python tools/build_tracked_artifact_review_report.py" in release_guide
    assert "python tools/build_stable_promotion_checklist.py" in release_guide
    assert "python tools/build_industrial_promotion_checklist.py" in release_guide
    assert "python tools/build_extension_execution_evidence.py" in release_guide
    assert "python tools/build_extension_execution_instance.py" in release_guide
    assert "python tools/build_extension_execution_schedule.py" in release_guide
    assert "python tools/build_extension_execution_actuals.py" in release_guide
    assert "security_release_preflight` step" in release_guide
    assert "确认 Phase E 客户交付文档集齐备" in release_guide
    assert "确认 industrial delivery gate 已闭合" in release_guide
    assert "python tools/run_release_rehearsal.py" in release_guide
    assert "python tools/run_clean_checkout_smoke.py" in release_guide
    assert "python tools/build_customer_acceptance_bundle.py" in release_guide
    assert "clean_checkout_smoke_report.json" in release_guide
    assert "customer_acceptance_bundle.json" in release_guide
    assert "docs/guides/SUPPORT_MATRIX.md" in release_guide
    assert "docs/guides/CAPACITY_AND_SCALE.md" in release_guide
    assert "docs/guides/CUSTOMER_ACCEPTANCE_CHECKLIST.md" in release_guide
    assert "docs/guides/KNOWN_LIMITATIONS.md" in release_guide
    assert "def main(argv: list[str] | None = None) -> int:" in release_builder
    assert "--release-summary" in release_builder
    assert "--approval-status" in release_builder
    assert "--source-root" in release_builder
    assert "tools/check_release_readiness.py" in smoke_runner
    assert "tools/check_industrial_release_readiness.py" in smoke_runner
    assert "tools/build_worktree_cleanup_report.py" in smoke_runner
    assert "tools/build_tracked_artifact_review_report.py" in smoke_runner
    assert "tools/build_stable_promotion_checklist.py" in smoke_runner
    assert "tools/build_industrial_promotion_checklist.py" in smoke_runner
    assert "tools/run_release_rehearsal.py" in smoke_runner
    assert "tools/build_customer_acceptance_bundle.py" in smoke_runner
    assert "tools/build_release_artifact.py" in smoke_runner
    assert "release_gate_status=" in smoke_runner


def test_phase_d_security_docs_and_tools_use_current_runtime_paths() -> None:
    release_guide = RELEASE_GUIDE.read_text(encoding="utf-8")
    security_baseline = SECURITY_BASELINE_GUIDE.read_text(encoding="utf-8")
    audit_policy = AUDIT_TRAIL_POLICY_GUIDE.read_text(encoding="utf-8")
    backup_restore_runbook = BACKUP_RESTORE_RUNBOOK_GUIDE.read_text(encoding="utf-8")
    incident_matrix = INCIDENT_RESPONSE_MATRIX_GUIDE.read_text(encoding="utf-8")
    smoke_runner = RUN_SMOKE_TESTS.read_text(encoding="utf-8")
    sbom_builder = SBOM_BUILDER.read_text(encoding="utf-8")
    vulnerability_writer = VULNERABILITY_SCAN_REPORT_WRITER.read_text(
        encoding="utf-8"
    )
    python_vulnerability_scan_runner = (
        PYTHON_VULNERABILITY_SCAN_RUNNER.read_text(encoding="utf-8")
    )
    container_vulnerability_scan_runner = (
        CONTAINER_VULNERABILITY_SCAN_RUNNER.read_text(encoding="utf-8")
    )
    vulnerability_exception_report_builder = (
        VULNERABILITY_EXCEPTION_REPORT_BUILDER.read_text(encoding="utf-8")
    )
    vulnerability_remediation_report_builder = (
        VULNERABILITY_REMEDIATION_REPORT_BUILDER.read_text(encoding="utf-8")
    )
    security_release_preflight_runner = (
        SECURITY_RELEASE_PREFLIGHT_RUNNER.read_text(encoding="utf-8")
    )
    backup_restore_rehearsal_runner = BACKUP_RESTORE_REHEARSAL_RUNNER.read_text(
        encoding="utf-8"
    )
    security_posture_builder = SECURITY_POSTURE_BUILDER.read_text(encoding="utf-8")
    collect_release_evidence = COLLECT_RELEASE_EVIDENCE.read_text(encoding="utf-8")

    assert "python tools/build_sbom_artifact.py" in release_guide
    assert "python tools/write_vulnerability_scan_report.py" in release_guide
    assert "python tools/run_python_vulnerability_scan.py" in release_guide
    assert "--include-optional-group training" in release_guide
    assert "python tools/run_container_vulnerability_scan.py" in release_guide
    assert "python tools/build_vulnerability_exception_report.py" in release_guide
    assert "python tools/build_vulnerability_remediation_report.py" in release_guide
    assert "python tools/run_security_release_preflight.py" in release_guide
    assert "deployment-zenoh-router" in release_guide
    assert "eclipse/zenoh:1.9.0" in release_guide
    assert "持续监控真实 `pip-audit` / `trivy` 数据库漂移" in release_guide
    assert "当前受管 no-fix exceptions 为空" in release_guide
    assert "security-preflight` 会 fail closed" in release_guide
    assert "python tools/run_backup_restore_rehearsal.py" in release_guide
    assert "python tools/build_security_posture_report.py" in release_guide
    assert "--python-vuln-raw-report" in release_guide
    assert "--container-vuln-raw-report" in release_guide
    assert "--run-python-vuln-scan" in release_guide
    assert "--run-container-vuln-scan" in release_guide
    assert "docs/guides/SECURITY_BASELINE.md" in release_guide
    assert "docs/guides/AUDIT_TRAIL_POLICY.md" in release_guide
    assert "docs/guides/BACKUP_RESTORE_RUNBOOK.md" in release_guide
    assert "docs/guides/INCIDENT_RESPONSE_MATRIX.md" in release_guide

    assert "AGI_WALKER_SECRET_KEY" in security_baseline
    assert "AGI_WALKER_REQUIRE_EXPLICIT_SECRET" in security_baseline
    assert "deployment/compose.env" in security_baseline
    assert "deployment/web_panel.env" in security_baseline
    assert "/var/lib/agi_walker/backups" in security_baseline
    assert "test_env/security/security_posture_report.json" in security_baseline
    assert "pip-audit-json" in security_baseline
    assert "trivy-json" in security_baseline
    assert "python tools/run_python_vulnerability_scan.py" in security_baseline
    assert "--include-optional-group training" in security_baseline
    assert "AGI_WALKER_PIP_AUDIT_TMPDIR" in security_baseline
    assert "python tools/run_container_vulnerability_scan.py" in security_baseline
    assert "python tools/build_vulnerability_exception_report.py" in security_baseline
    assert "python tools/build_vulnerability_remediation_report.py" in security_baseline
    assert "python tools/run_security_release_preflight.py" in security_baseline

    assert "release_approval" in audit_policy
    assert "release_source" in audit_policy
    assert "workflow_runs" in audit_policy
    assert "workflow_archive" in audit_policy

    assert "AGI_WALKER_RUNTIME_ROOT" in backup_restore_runbook
    assert "docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml up -d --build zenoh-router web-panel" in backup_restore_runbook
    assert "RPO" in backup_restore_runbook
    assert "RTO" in backup_restore_runbook
    assert "恢复演练报告" in backup_restore_runbook

    assert "P1" in incident_matrix
    assert "P2" in incident_matrix
    assert "P3" in incident_matrix
    assert "30 分钟内" in incident_matrix
    assert "4 小时内" in incident_matrix

    assert "tools/build_sbom_artifact.py" in smoke_runner
    assert "tools/write_vulnerability_scan_report.py" in smoke_runner
    assert "tools/run_backup_restore_rehearsal.py" in smoke_runner
    assert "tools/build_security_posture_report.py" in smoke_runner
    assert "tools/build_vulnerability_exception_report.py" in smoke_runner
    assert "tools/build_vulnerability_remediation_report.py" in smoke_runner
    assert "tools/run_security_release_preflight.py" in smoke_runner
    assert "security_posture_status=ready" in smoke_runner
    assert "backup_restore_rehearsal_status=passed" in smoke_runner
    assert "security_release_preflight_status=passed" in smoke_runner
    assert "sbom.json" in smoke_runner
    assert "backup_restore_rehearsal_report.json" in smoke_runner
    assert "vulnerability_exception_report.json" in smoke_runner
    assert "security_posture_report.json" in smoke_runner
    assert "security_release_preflight_report.json" in smoke_runner

    assert "def main(argv: list[str] | None = None) -> int:" in sbom_builder
    assert "def main(argv: list[str] | None = None) -> int:" in vulnerability_writer
    assert "def main(argv: list[str] | None = None) -> int:" in python_vulnerability_scan_runner
    assert "def _resolve_pip_audit_temp_root(cache_dir: Path) -> Path:" in python_vulnerability_scan_runner
    assert "def main(argv: list[str] | None = None) -> int:" in container_vulnerability_scan_runner
    assert "def main(argv: list[str] | None = None) -> int:" in vulnerability_exception_report_builder
    assert "def main(argv: list[str] | None = None) -> int:" in vulnerability_remediation_report_builder
    assert "def main(argv: list[str] | None = None) -> int:" in security_release_preflight_runner
    assert "def main(argv: list[str] | None = None) -> int:" in backup_restore_rehearsal_runner
    assert "def main(argv: list[str] | None = None) -> int:" in security_posture_builder
    assert "--python-vuln-raw-report" in collect_release_evidence
    assert "--container-vuln-raw-report" in collect_release_evidence
    assert "--run-python-vuln-scan" in collect_release_evidence
    assert "--run-container-vuln-scan" in collect_release_evidence
    assert "--vulnerability-exception-report-source" in collect_release_evidence
    assert "--vulnerability-exception-input-source" in collect_release_evidence
    assert "--external-bindings-config-source" in collect_release_evidence
    assert "--release-ops-execution-report-source" in collect_release_evidence
    assert "deployment/customer_delivery.external_bindings.json" in release_guide
    assert "--vulnerability-exception-input-source" in security_release_preflight_runner


def test_security_preflight_ci_job_uses_current_runner_and_artifacts() -> None:
    content = CI_WORKFLOW.read_text(encoding="utf-8")
    security_job = content.split("  security-preflight:", 1)[1].split(
        "  # -----------------------------------------------------------------------------",
        1,
    )[0]

    assert "security-preflight:" in content
    assert (
        "if: github.event_name == 'workflow_dispatch' || github.event_name == 'schedule'"
        not in security_job
    )
    assert "pip install pip-audit" in content
    assert "trivy" in content
    assert 'AGI_WALKER_ZENOH_BASE_IMAGE: "eclipse/zenoh:1.9.0"' in content
    assert 'AGI_WALKER_ZENOH_IMAGE: "deployment-zenoh-router"' in content
    assert "docker compose -f deployment/docker-compose.yml --profile distributed build zenoh-router web-panel-distributed" in content
    assert "docker pull ${{ env.AGI_WALKER_ZENOH_BASE_IMAGE }}" in content
    assert "python tools/run_security_release_preflight.py --security-only --output-root test_env/release_evidence_ci --run-python-vuln-scan --run-container-vuln-scan --container-image-ref ${{ env.AGI_WALKER_ZENOH_IMAGE }} --container-image-ref deployment-web-panel-distributed" in content
    assert "name: security-preflight-artifacts" in content
    assert "path: test_env/release_evidence_ci" in content


def test_next_stage_readiness_ci_job_archives_blocked_or_ready_evidence() -> None:
    jobs = _workflow_jobs()
    job = jobs["next-stage-readiness"]

    assert job["needs"] == "smoke"
    step_runs = _joined_step_runs(job)
    default_command = (
        "python tools/build_next_stage_readiness_report.py "
        "--output test_env/next_stage/next_stage_readiness_report.json"
    )
    assert default_command in step_runs
    assert f"{default_command} --expected-status blocked" in step_runs
    assert "||" in step_runs
    checklist_command = (
        "python tools/build_next_stage_external_evidence_checklist.py "
        "--readiness-report test_env/next_stage/next_stage_readiness_report.json "
        "--output test_env/next_stage/next_stage_external_evidence_checklist.json"
    )
    assert checklist_command in step_runs
    assert "next_stage_external_evidence_checklist.json || true" in step_runs

    upload_step = _artifact_upload_step(job, "next-stage-readiness-artifacts")
    upload_with = upload_step["with"]
    assert upload_with["path"] == "test_env/next_stage"
    assert upload_with["if-no-files-found"] == "ignore"
    assert upload_with["retention-days"] == 14


def test_production_runbook_uses_current_compose_entrypoints() -> None:
    runbook = PRODUCTION_RUNBOOK.read_text(encoding="utf-8")

    assert "deployment/docker-compose.yml" in runbook
    assert "deployment/compose.env.example" in runbook
    assert "deployment/web_panel.env.example" in runbook
    assert "extension_execution_plan" in runbook
    assert "extension_execution_evidence" in runbook
    assert "extension_execution_instance" in runbook
    assert "extension_execution_schedule" in runbook
    assert "extension_execution_actuals" in runbook
    assert "window_trigger_recorded_by" in runbook
    assert "approval_identity_source_path" in runbook
    assert "archive_target_binding_type" in runbook
    assert "due_trigger_binding_type" in runbook
    assert "due_trigger_checked_at" in runbook
    assert "execution_template" in runbook
    assert "watch_actions" in runbook
    assert "on_call_handoff_records" in runbook
    assert "residual_risk_handoff_steps" in runbook
    assert "exception_review_steps" in runbook
    assert "extension_on_call_rehearsal_report.json" in runbook
    assert "extension_exception_review_schedule_report.json" in runbook
    assert "extension_escalation_closure_report.json" in runbook
    assert "extension_execution_instance.json" in runbook
    assert "extension_execution_schedule.json" in runbook
    assert "extension_execution_actuals.json" in runbook
    assert "approval_identity_source.json" in runbook
    assert "window_trigger.json" in runbook
    assert "signoff.json" in runbook
    assert "exception_review.json" in runbook
    assert "residual_risk_review.json" in runbook
    assert "due_trigger_check.json" in runbook
    assert "archive_target.json" in runbook
    assert "closure_archive/index.json" in runbook
    assert "closure_manifest.json" in runbook
    assert "incident_escalation_steps" in runbook
    assert "escalation_closure_steps" in runbook
    assert "handoff_checkpoints" in runbook
    assert "signoff_checkpoints" in runbook
    assert "rollback_evidence_archive_steps" in runbook
    assert "docs/guides/DISTRIBUTED_GUIDE.md" in runbook
    assert "docs/ros2/ROS2_QUICK_START.md" in runbook
    assert "docs/guides/GODOT_TESTING_GUIDE.md" in runbook
    assert (
        "docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml"
        in runbook
    )
    assert "helm/agi-walker" in runbook
    assert "docker-compose.prod.yml" in runbook
    assert "[待填充]" not in runbook
    assert "kubectl cluster-info" not in runbook
    assert "helm lint ./helm/agi-walker" not in runbook
    assert "docker compose -f docker-compose.prod.yml" not in runbook


def test_testing_guide_references_instruction_control_validation_runner() -> None:
    content = TESTING_GUIDE.read_text(encoding="utf-8")

    assert "Godot instruction-set smoke" in content
    assert "ROS2 instruction-set smoke" in content
    assert "simulated circuit replay smoke" in content
    assert "python tools/run_instruction_control_validation.py" in content
    assert "instruction_control_validation_report.json" in content


def test_instruction_control_demo_runbook_is_referenced_and_actionable() -> None:
    readme = ROOT_RELEASE_NOTES.parent.joinpath("README.md").read_text(encoding="utf-8")
    godot_guide = GODOT_TESTING_GUIDE.read_text(encoding="utf-8")
    runbook = INSTRUCTION_CONTROL_DEMO_RUNBOOK.read_text(encoding="utf-8")

    assert "docs/guides/INSTRUCTION_CONTROL_DEMO_RUNBOOK.md" in readme
    assert "docs/guides/INSTRUCTION_CONTROL_DEMO_RUNBOOK.md" in godot_guide
    assert "python tools/run_instruction_control_validation.py" in runbook
    assert "instruction_control_validation_report.json" in runbook
    assert "godot_instruction_smoke_report.json" in runbook
    assert "ros2_instruction_smoke_report.json" in runbook
    assert "simulated_circuit_replay_smoke_report.json" in runbook


def test_readme_references_next_stage_execution_plan() -> None:
    readme = ROOT_RELEASE_NOTES.parent.joinpath("README.md").read_text(encoding="utf-8")
    plan = NEXT_STAGE_EXECUTION_PLAN.read_text(encoding="utf-8")

    assert "docs/guides/NEXT_STAGE_EXECUTION_PLAN_20260426.md" in readme
    assert "真实硬件联调" in plan
    assert "tools/build_hardware_live_diagnostics_checklist.py" in plan
    assert "deployment/hardware/imc22_live_transport.template.json" in plan
    assert "tools/build_vendor_fault_data_review.py" in plan
    assert "tools/build_vendor_data_promotion_checklist.py" in plan
    assert "deployment/hardware/imc22_fault_telemetry_fields.json" in plan
    assert "deployment/hardware/imc22_vendor_fault_samples.template.json" in plan
    assert "Web 硬件操作面深化" in plan
    assert "ROS2 标准化与生态接入" in plan
    assert "vendor 数据沉淀" in plan
    assert "交付级 live evidence" in plan
    assert "浏览器手工验证" in plan


def test_operator_history_timeline_is_linked_from_active_surfaces() -> None:
    readme = ROOT_RELEASE_NOTES.parent.joinpath("README.md").read_text(encoding="utf-8")
    guide = WEB_PANEL_GUIDE.read_text(encoding="utf-8")
    timeline = OPERATOR_HISTORY_TIMELINE.read_text(encoding="utf-8")

    assert "/static/operator-history-timeline.html" in readme
    assert "/static/operator-history-timeline.html" in guide
    assert "Operator History Timeline" in timeline
    assert 'id="timeline-note-filter"' in timeline
    assert "/api/godot/history/summary" in timeline


def test_web_browser_manual_validation_checklist_is_linked() -> None:
    readme = ROOT_RELEASE_NOTES.parent.joinpath("README.md").read_text(encoding="utf-8")
    guide = WEB_PANEL_GUIDE.read_text(encoding="utf-8")
    plan = NEXT_STAGE_EXECUTION_PLAN.read_text(encoding="utf-8")
    checklist = WEB_BROWSER_MANUAL_VALIDATION_CHECKLIST.read_text(encoding="utf-8")

    assert "docs/guides/WEB_BROWSER_MANUAL_VALIDATION_CHECKLIST_20260426.md" in readme
    assert "docs/guides/WEB_BROWSER_MANUAL_VALIDATION_CHECKLIST_20260426.md" in guide
    assert "docs/guides/WEB_BROWSER_MANUAL_VALIDATION_CHECKLIST_20260426.md" in plan
    assert "deployment/web_browser_manual_validation.template.json" in readme
    assert "deployment/web_browser_manual_validation.template.json" in plan
    assert "tools/run_web_browser_playwright_smoke.py" in readme
    assert "tools/run_web_browser_playwright_smoke.py" in plan
    assert "tools/build_web_browser_manual_validation_report.py" in readme
    assert "tools/build_web_browser_manual_validation_report.py" in plan
    assert "tools/build_web_browser_validation_closeout.py" in readme
    assert "tools/build_web_browser_validation_closeout.py" in plan
    assert "manual_report_missing" in plan
    assert "web_browser_validation_closeout.json` 为 `status=passed" in plan
    assert "/static/instruction-control.html" in checklist
    assert "/static/operator-history.html" in checklist
    assert "/static/operator-history-timeline.html" in checklist
    assert "recovery plan" in checklist
    assert "note_exact" in checklist


def test_customer_deployment_docs_and_compose_defaults_use_current_single_path() -> None:
    compose = DISTRIBUTED_COMPOSE.read_text(encoding="utf-8")
    compose_env = COMPOSE_ENV_EXAMPLE.read_text(encoding="utf-8")
    web_env = WEB_PANEL_ENV_EXAMPLE.read_text(encoding="utf-8")
    web_panel_dockerfile = WEB_PANEL_DOCKERFILE.read_text(encoding="utf-8")
    deployment_matrix = DEPLOYMENT_MATRIX_GUIDE.read_text(encoding="utf-8")
    installation_guide = CUSTOMER_INSTALLATION_GUIDE.read_text(encoding="utf-8")
    support_matrix = SUPPORT_MATRIX_GUIDE.read_text(encoding="utf-8")
    capacity_and_scale = CAPACITY_AND_SCALE_GUIDE.read_text(encoding="utf-8")
    acceptance_checklist = CUSTOMER_ACCEPTANCE_CHECKLIST_GUIDE.read_text(
        encoding="utf-8"
    )
    known_limitations = KNOWN_LIMITATIONS_GUIDE.read_text(encoding="utf-8")
    readme = ROOT_RELEASE_NOTES.parent.joinpath("README.md").read_text(encoding="utf-8")

    assert "${AGI_WALKER_COMPOSE_WEB_ENV_FILE:-./web_panel.env.example}" in compose
    assert "${AGI_WALKER_WEB_PORT:-8080}:8000" in compose
    assert "${AGI_WALKER_WEB_DISTRIBUTED_PORT:-8081}:8000" in compose
    assert "dockerfile: deployment/Dockerfile.zenoh_router" in compose
    assert "ZENOH_BASE_IMAGE: ${AGI_WALKER_ZENOH_BASE_IMAGE:-eclipse/zenoh:1.9.0}" in compose
    assert "${AGI_WALKER_ZENOH_IMAGE:-deployment-zenoh-router}" in compose
    assert "${AGI_WALKER_RUNTIME_ROOT:-./runtime}/db:/var/lib/agi_walker/db" in compose
    assert (
        "${AGI_WALKER_RUNTIME_ROOT:-./runtime}/workflow_runs:/var/lib/agi_walker/workflow_runs"
        in compose
    )
    assert (
        "${AGI_WALKER_RUNTIME_ROOT:-./runtime}/workflow_archive:/var/lib/agi_walker/workflow_archive"
        in compose
    )
    assert (
        "${AGI_WALKER_RUNTIME_ROOT:-./runtime}/backups:/var/lib/agi_walker/backups"
        in compose
    )

    assert "AGI_WALKER_COMPOSE_WEB_ENV_FILE=./web_panel.env" in compose_env
    assert "AGI_WALKER_RUNTIME_ROOT=./runtime" in compose_env
    assert "AGI_WALKER_WEB_PORT=8080" in compose_env
    assert "AGI_WALKER_WEB_DISTRIBUTED_PORT=8081" in compose_env
    assert "AGI_WALKER_ZENOH_TCP_PORT=7447" in compose_env
    assert "AGI_WALKER_ZENOH_REST_PORT=8000" in compose_env
    assert "AGI_WALKER_ZENOH_BASE_IMAGE=eclipse/zenoh:1.9.0" in compose_env
    assert "AGI_WALKER_ZENOH_IMAGE=deployment-zenoh-router" in compose_env
    assert "AGI_WALKER_WEB_PANEL_BASE_IMAGE=python:3.11-alpine" in compose_env
    assert "AGI_WALKER_WEB_PANEL_APK_PACKAGES=libgcc" in compose_env
    assert "AGI_WALKER_WEB_PANEL_APK_BUILD_PACKAGES=build-base" in compose_env

    assert "AGI_WALKER_DATABASE_URL=sqlite+aiosqlite:////var/lib/agi_walker/db/agi_walker.db" in compose_env
    assert "AGI_WALKER_WEB_OUTPUT_ROOT=/var/lib/agi_walker/workflow_runs" in compose_env
    assert "AGI_WALKER_WEB_ARCHIVE_ROOT=/var/lib/agi_walker/workflow_archive" in compose_env
    assert "AGI_WALKER_SECRET_KEY=change-me-before-production" in web_env
    assert "AGI_WALKER_ZENOH_ENDPOINT=tcp/zenoh-router:7447" in web_env
    assert "apt-get dist-upgrade -y" in web_panel_dockerfile
    assert "apt-get clean" in web_panel_dockerfile
    assert "rm -rf /var/lib/apt/lists/*" in web_panel_dockerfile

    assert "Docker Compose" in deployment_matrix
    assert "deployment/compose.env.example" in deployment_matrix
    assert "deployment/web_panel.env.example" in deployment_matrix
    assert "Helm / Kubernetes" in deployment_matrix
    assert "当前不支持" in deployment_matrix

    assert "安装前检查" in installation_guide
    assert "首次启动" in installation_guide
    assert "健康检查" in installation_guide
    assert "升级" in installation_guide
    assert "回滚" in installation_guide
    assert "卸载" in installation_guide
    assert "extension_execution_plan" in installation_guide
    assert "extension_execution_evidence" in installation_guide
    assert "extension_execution_instance" in installation_guide
    assert "extension_execution_schedule" in installation_guide
    assert "extension_execution_actuals" in installation_guide
    assert "window_trigger_recorded_by" in installation_guide
    assert "approval_identity_source_path" in installation_guide
    assert "approval_identity_reference" in installation_guide
    assert "archive_target_binding_type" in installation_guide
    assert "archive_target_binding_reference_base" in installation_guide
    assert "due_trigger_binding_type" in installation_guide
    assert "due_trigger_binding_reference_base" in installation_guide
    assert "due_trigger_checked_at" in installation_guide
    assert "execution_template" in installation_guide
    assert "watch_actions" in installation_guide
    assert "on_call_handoff_records" in installation_guide
    assert "residual_risk_handoff_steps" in installation_guide
    assert "exception_review_steps" in installation_guide
    assert "extension_on_call_rehearsal_report.json" in installation_guide
    assert "extension_exception_review_schedule_report.json" in installation_guide
    assert "extension_escalation_closure_report.json" in installation_guide
    assert "extension_execution_instance.profiles[*]" in installation_guide
    assert "extension_execution_schedule.profiles[*]" in installation_guide
    assert "extension_execution_actuals.profiles[*]" in installation_guide
    assert "exception_review_record_path" in installation_guide
    assert "closure_index_path" in installation_guide
    assert "due_trigger_check.json" in installation_guide
    assert "archive_target.json" in installation_guide
    assert "incident_escalation_owner_role" in installation_guide
    assert "escalation_closure_steps" in installation_guide
    assert "signoff_checkpoints" in installation_guide
    assert "rollback_evidence_owner_role" in installation_guide
    assert "deployment/compose.env.example" in installation_guide
    assert "deployment/web_panel.env.example" in installation_guide
    assert (
        "docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml up -d --build zenoh-router web-panel"
        in installation_guide
    )
    assert "Docker Compose" in support_matrix
    assert "3.10" in support_matrix
    assert "3.11" in support_matrix
    assert "ROS2 Humble" in support_matrix
    assert "Chromium" in support_matrix
    assert "Helm / Kubernetes" in support_matrix
    assert "单机评估版" in capacity_and_scale
    assert "小规模团队" in capacity_and_scale
    assert "分布式实验环境" in capacity_and_scale
    assert "高可用" in capacity_and_scale
    assert "互联网规模" in capacity_and_scale
    assert "功能验收" in acceptance_checklist
    assert "部署验收" in acceptance_checklist
    assert "安全验收" in acceptance_checklist
    assert "性能与运维验收" in acceptance_checklist
    assert "回滚验收" in acceptance_checklist
    assert "CAPACITY_AND_SCALE.md" in acceptance_checklist
    assert "customer_acceptance_bundle.extension_execution_evidence" in acceptance_checklist
    assert "customer_acceptance_bundle.extension_execution_instance" in acceptance_checklist
    assert "customer_acceptance_bundle.extension_execution_schedule" in acceptance_checklist
    assert "customer_acceptance_bundle.extension_execution_actuals" in acceptance_checklist
    assert "window_trigger_recorded_by" in acceptance_checklist
    assert "approval_identity_source_path" in acceptance_checklist
    assert "approval_identity_reference" in acceptance_checklist
    assert "archive_target_binding_type" in acceptance_checklist
    assert "archive_target_binding_reference_base" in acceptance_checklist
    assert "due_trigger_binding_type" in acceptance_checklist
    assert "due_trigger_binding_reference_base" in acceptance_checklist
    assert "due_trigger_checked_at" in acceptance_checklist
    assert "closure_archive_due_at" in acceptance_checklist
    assert "execution_template" in acceptance_checklist
    assert "handoff_owner_role" in acceptance_checklist
    assert "watch_owner_role" in acceptance_checklist
    assert "on_call_handoff_owner_role" in acceptance_checklist
    assert "residual_risk_owner_role" in acceptance_checklist
    assert "exception_review_owner_role" in acceptance_checklist
    assert "incident_escalation_owner_role" in acceptance_checklist
    assert "escalation_closure_owner_role" in acceptance_checklist
    assert "extension_on_call_rehearsal_report.json" in acceptance_checklist
    assert "extension_exception_review_schedule_report.json" in acceptance_checklist
    assert "extension_escalation_closure_report.json" in acceptance_checklist
    assert "extension_execution_instance.json" in acceptance_checklist
    assert "extension_execution_schedule.json" in acceptance_checklist
    assert "extension_execution_actuals.json" in acceptance_checklist
    assert "approval_identity_source.json" in acceptance_checklist
    assert "window_trigger.json" in acceptance_checklist
    assert "signoff.json" in acceptance_checklist
    assert "exception_review.json" in acceptance_checklist
    assert "residual_risk_review.json" in acceptance_checklist
    assert "due_trigger_check.json" in acceptance_checklist
    assert "archive_target.json" in acceptance_checklist
    assert "closure_archive/index.json" in acceptance_checklist
    assert "closure_manifest.json" in acceptance_checklist
    assert "rollback_evidence_archive_steps" in acceptance_checklist
    assert "deployment/security/vulnerability_exceptions.input.json" in acceptance_checklist
    assert "Helm / Kubernetes" in known_limitations
    assert "SQLite" in known_limitations
    assert "stdio" in known_limitations
    assert "deployment-web-panel-distributed" in known_limitations
    assert "0 findings" in known_limitations
    assert "0 active exceptions" in known_limitations
    assert "security-preflight` 会 fail closed" in known_limitations
    assert "104" not in known_limitations
    assert "31 条 active no-fix exceptions" not in known_limitations
    assert "CAPACITY_AND_SCALE.md" in known_limitations
    assert "生产部署 Runbook" in readme
    assert "客户安装指南" in readme
    assert "部署矩阵" in readme
    assert "支持矩阵" in readme
    assert "容量与规模声明" in readme
    assert "客户验收清单" in readme
    assert "已知限制" in readme
    assert "extension_execution_evidence" in readme
    assert "extension_execution_instance" in readme
    assert "extension_execution_schedule" in readme
    assert "extension_execution_actuals" in readme
    assert "approval_identity_source_path" in readme
    assert "archive_target_binding_type" in readme
    assert "due_trigger_binding_type" in readme
    assert "due_trigger_checked_at" in readme
    assert "closure_archive/index.json" in readme
