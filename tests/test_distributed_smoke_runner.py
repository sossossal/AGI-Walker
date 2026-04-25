import importlib.util
import json
import subprocess
import sys
from pathlib import Path


def _load_runner_module():
    script_path = Path(__file__).with_name("run_distributed_smoke.py")
    spec = importlib.util.spec_from_file_location("run_distributed_smoke", script_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_log_diagnostics_detect_removed_entrypoint_paths() -> None:
    runner = _load_runner_module()
    logs = "\n".join(
        [
            "sidecar-1-1  | python: can't open file '/app/distributed/sidecar.py': [Errno 2] No such file or directory",
            "learner-1     | python: can't open file '/app/distributed/run_learner.py': [Errno 2] No such file or directory",
        ]
    )

    diagnostics = runner._build_log_diagnostics(logs, "actor_docker_1")

    hint_codes = {hint["code"] for hint in diagnostics["hints"]}
    assert "container_entrypoint_path" in hint_codes
    assert "actor_id_missing_from_logs" in hint_codes
    assert diagnostics["services"]["sidecar-1"]["line_count"] == 1
    assert diagnostics["services"]["learner"]["line_count"] == 1


def test_log_diagnostics_detect_docker_environment_failures() -> None:
    runner = _load_runner_module()
    logs = "\n".join(
        [
            r"open C:\Users\user\.docker\buildx\.lock: Access is denied.",
            "failed to connect to the docker API at npipe:////./pipe/dockerDesktopLinuxEngine",
        ]
    )

    diagnostics = runner._build_log_diagnostics(logs, "actor_docker_1")

    hint_codes = {hint["code"] for hint in diagnostics["hints"]}
    assert "docker_buildx_lock" in hint_codes
    assert "docker_daemon_unavailable" in hint_codes
    assert "actor_id_missing_from_logs" not in hint_codes


def test_log_diagnostics_detect_port_allocation_failures() -> None:
    runner = _load_runner_module()
    logs = "Bind for 0.0.0.0:8000 failed: port is already allocated"

    diagnostics = runner._build_log_diagnostics(logs, "actor_docker_1")

    hint_codes = {hint["code"] for hint in diagnostics["hints"]}
    assert "docker_port_allocated" in hint_codes


def test_log_diagnostics_detect_missing_pyyaml_in_distributed_runtime() -> None:
    runner = _load_runner_module()
    logs = "\n".join(
        [
            "sidecar-1-1   |   File \"/app/agi_walker/skills_loader.py\", line 11, in <module>",
            "sidecar-1-1   |     import yaml",
            "sidecar-1-1   | ModuleNotFoundError: No module named 'yaml'",
            "learner-1     | ModuleNotFoundError: No module named 'yaml'",
        ]
    )

    diagnostics = runner._build_log_diagnostics(logs, "actor_docker_1")

    hint_codes = {hint["code"] for hint in diagnostics["hints"]}
    assert "distributed_runtime_missing_pyyaml" in hint_codes
    assert diagnostics["services"]["sidecar-1"]["line_count"] == 3
    assert diagnostics["services"]["learner"]["line_count"] == 1


def test_compose_env_uses_smoke_safe_zenoh_host_ports() -> None:
    runner = _load_runner_module()

    env = runner._compose_env("actor-ci")

    assert env["AGI_WALKER_SIDECAR_ACTOR_ID"] == "actor-ci"
    assert env["AGI_WALKER_FORCE_OFFLOAD"] == "1"
    assert env["AGI_WALKER_ZENOH_TCP_PORT"] == runner.SMOKE_ZENOH_HOST_TCP_PORT
    assert env["AGI_WALKER_ZENOH_REST_PORT"] == runner.SMOKE_ZENOH_HOST_REST_PORT


def test_parse_compose_ps_output_accepts_json_array_and_json_lines() -> None:
    runner = _load_runner_module()
    json_array = json.dumps(
        [
            {
                "Service": "learner",
                "Name": "agi-learner-1",
                "State": "running",
                "ExitCode": 0,
            }
        ]
    )
    json_lines = "\n".join(
        [
            json.dumps(
                {
                    "Service": "sidecar-1",
                    "Name": "agi-sidecar-1",
                    "State": "running",
                    "ExitCode": 0,
                }
            ),
            json.dumps(
                {
                    "Service": "mock-godot",
                    "Name": "agi-mock-godot-1",
                    "State": "running",
                    "ExitCode": 0,
                }
            ),
        ]
    )

    assert runner._parse_compose_ps_output(json_array)["learner"]["state"] == "running"
    parsed_lines = runner._parse_compose_ps_output(json_lines)
    assert parsed_lines["sidecar-1"]["name"] == "agi-sidecar-1"
    assert parsed_lines["mock-godot"]["exit_code"] == 0


def test_distributed_smoke_success_writes_machine_readable_report(
    monkeypatch, tmp_path: Path
) -> None:
    runner = _load_runner_module()
    report_path = tmp_path / "distributed-smoke-report.json"
    actor_id = "actor-ci"

    def fake_run_compose(
        compose_file,
        args,
        *,
        actor_id,
        capture_output=False,
    ):
        if args[:2] == ["ps", "--format"]:
            stdout = json.dumps(
                [
                    {"Service": "learner", "Name": "learner-1", "State": "running"},
                    {
                        "Service": "sidecar-1",
                        "Name": "sidecar-1-1",
                        "State": "running",
                    },
                ]
            )
        elif args and args[0] == "logs":
            stdout = "\n".join(
                [
                    f"mock-godot-1  | listening on 0.0.0.0:9000 for actor {actor_id}",
                    "mock-godot-1  | received step #1 with 12 actions",
                    f"sidecar-1-1   | published observation #1 to ag/{actor_id}/obs",
                ]
            )
        else:
            stdout = ""
        return subprocess.CompletedProcess(args, 0, stdout=stdout, stderr="")

    def fake_try_fetch_json(url: str):
        if url.endswith("/api/system/status"):
            return {
                "status": "running",
                "distributed_monitor": {
                    "schema_version": "1.0",
                    "monitor_active": True,
                    "endpoint": runner.SMOKE_ZENOH_ENDPOINT,
                },
            }
        if url.endswith("/api/distributed/status"):
            return {
                "schema_version": "1.0",
                "actors": {
                    actor_id: {
                        "id": actor_id,
                        "status": "active",
                        "last_seen": "2026-04-11T00:00:00",
                        "data": {},
                    }
                },
                "monitor": {"monitor_active": True},
            }
        return None

    monkeypatch.setattr(runner, "_run_compose", fake_run_compose)
    monkeypatch.setattr(runner, "_try_fetch_json", fake_try_fetch_json)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "run_distributed_smoke.py",
            "--build",
            "--actor-id",
            actor_id,
            "--report-file",
            str(report_path),
        ],
    )

    assert runner.main() == 0
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["schema_version"] == "1.0"
    assert report["status"] == "passed"
    assert report["actor_id"] == actor_id
    assert report["zenoh_endpoint"] == runner.SMOKE_ZENOH_ENDPOINT
    assert {check["name"] for check in report["checks"]} == {
        "compose_build",
        "compose_up",
        "web_panel_status",
        "distributed_monitor",
        "sidecar_start",
        "actor_discovery",
        "learner_action_loop",
    }
