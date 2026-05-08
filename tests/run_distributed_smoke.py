"""
End-to-end distributed smoke test for the Docker deployment.

This runner starts the following compose services:
- zenoh-router
- learner
- web-panel-distributed
- mock-godot
- sidecar-1

Validation succeeds when:
- web-panel-distributed reports an active distributed monitor
- the expected actor appears in /api/distributed/status
- mock-godot logs show that at least one step action was received
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_COMPOSE_FILE = PROJECT_ROOT / "deployment" / "docker-compose.yml"
DEFAULT_WEB_PANEL_URL = "http://127.0.0.1:8081"
DEFAULT_ACTOR_ID = "actor_docker_1"
DEFAULT_TIMEOUT_SECONDS = 180.0
DEFAULT_POLL_INTERVAL_SECONDS = 1.0
SMOKE_REPORT_SCHEMA_VERSION = "1.0"
SMOKE_ZENOH_ENDPOINT = "tcp/zenoh-router:7447"
SMOKE_ZENOH_HOST_TCP_PORT = "17447"
SMOKE_ZENOH_HOST_REST_PORT = "18000"
SMOKE_SERVICES = [
    "zenoh-router",
    "learner",
    "web-panel-distributed",
    "mock-godot",
    "sidecar-1",
]
LOG_HINT_PATTERNS = [
    (
        "docker_buildx_lock",
        re.compile(r"\.docker[\\/]+buildx[\\/]+\.lock: Access is denied"),
        "Docker buildx lock is not accessible; fix user Docker config permissions or run the smoke with a Docker-enabled account.",
    ),
    (
        "docker_daemon_unavailable",
        re.compile(
            r"(dockerDesktopLinuxEngine|failed to connect to the docker API|request returned 500 Internal Server Error)"
        ),
        "Docker Desktop Linux engine is unavailable or unhealthy; start Docker Desktop and verify `docker version` before rerunning.",
    ),
    (
        "docker_port_allocated",
        re.compile(r"Bind for 0\.0\.0\.0:\d+ failed: port is already allocated"),
        "A required host port is already allocated; stop the conflicting container/process or override the compose port environment variables.",
    ),
    (
        "container_entrypoint_path",
        re.compile(r"can't open file '.*/distributed/(sidecar|run_learner)\.py'"),
        "Compose is still pointing at removed /app/distributed/*.py entrypoints.",
    ),
    (
        "python_module_entrypoint",
        re.compile(r"No module named .*agi_walker\.core\.distributed"),
        "The distributed runtime image cannot import agi_walker.core.distributed modules.",
    ),
    (
        "distributed_runtime_missing_pyyaml",
        re.compile(r"ModuleNotFoundError: No module named 'yaml'"),
        "The distributed runtime image is missing PyYAML; verify deployment/requirements.distributed_runtime.txt and the Dockerfile import sanity check.",
    ),
    (
        "web_panel_missing_gymnasium",
        re.compile(r"ModuleNotFoundError: No module named 'gymnasium'"),
        "The web panel image is missing gymnasium; verify deployment/requirements.web_panel.txt is used by deployment/Dockerfile.web_panel.",
    ),
    (
        "zenoh_connection",
        re.compile(
            r"(Zenoh|zenoh|connect/endpoints|Failed to init|Connection refused)"
        ),
        "Check zenoh-router health and AGI_WALKER_ZENOH_ENDPOINT/ZENOH_ROUTER.",
    ),
    (
        "godot_tcp_connection",
        re.compile(r"(Connection failed|Connected to Godot|client disconnected)"),
        "Check mock-godot sidecar host/port and AGI_WALKER_GODOT_HOST.",
    ),
]
SMOKE_BOOTSTRAP_SERVICES = [
    "zenoh-router",
    "learner",
    "web-panel-distributed",
    "mock-godot",
]


if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")


def _compose_env(actor_id: str) -> dict[str, str]:
    env = os.environ.copy()
    env["AGI_WALKER_GODOT_HOST"] = "mock-godot"
    env["AGI_WALKER_SIDECAR_ACTOR_ID"] = actor_id
    env["AGI_WALKER_FORCE_OFFLOAD"] = "1"
    env.setdefault("AGI_WALKER_ZENOH_TCP_PORT", SMOKE_ZENOH_HOST_TCP_PORT)
    env.setdefault("AGI_WALKER_ZENOH_REST_PORT", SMOKE_ZENOH_HOST_REST_PORT)
    return env


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _new_report(
    compose_file: Path, web_panel_url: str, actor_id: str
) -> dict[str, Any]:
    return {
        "schema_version": SMOKE_REPORT_SCHEMA_VERSION,
        "status": "running",
        "created_at": _utc_now(),
        "finished_at": None,
        "compose_file": str(compose_file),
        "web_panel_url": web_panel_url,
        "actor_id": actor_id,
        "zenoh_endpoint": SMOKE_ZENOH_ENDPOINT,
        "host_ports": {
            "zenoh_tcp": SMOKE_ZENOH_HOST_TCP_PORT,
            "zenoh_rest": SMOKE_ZENOH_HOST_REST_PORT,
            "web_panel": "8081",
        },
        "sidecar": {
            "force_offload": True,
        },
        "services": SMOKE_SERVICES,
        "checks": [],
        "diagnostics": {},
    }


def _record_check(
    report: dict[str, Any],
    name: str,
    status: str,
    details: dict[str, Any] | None = None,
) -> None:
    report["checks"].append(
        {
            "name": name,
            "status": status,
            "timestamp": _utc_now(),
            "details": details or {},
        }
    )


def _run_compose(
    compose_file: Path,
    args: list[str],
    *,
    actor_id: str,
    capture_output: bool = False,
) -> subprocess.CompletedProcess[str]:
    command = [
        "docker",
        "compose",
        "-f",
        str(compose_file),
        "--profile",
        "distributed",
        "--profile",
        "smoke",
        *args,
    ]
    return subprocess.run(
        command,
        cwd=str(PROJECT_ROOT),
        env=_compose_env(actor_id),
        capture_output=capture_output,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )


def _fetch_json(url: str) -> dict[str, Any]:
    with urllib.request.urlopen(url, timeout=10) as response:
        return json.loads(response.read().decode("utf-8"))


def _try_fetch_json(url: str) -> dict[str, Any] | None:
    try:
        return _fetch_json(url)
    except Exception:
        return None


def _wait_for_condition(
    timeout_seconds: float,
    predicate,
    *,
    poll_interval_seconds: float = DEFAULT_POLL_INTERVAL_SECONDS,
) -> Any:
    deadline = time.time() + timeout_seconds
    last_value = None
    while time.time() < deadline:
        last_value = predicate()
        if last_value:
            return last_value
        time.sleep(poll_interval_seconds)
    return last_value


def _collect_logs(compose_file: Path, actor_id: str) -> str:
    result = _run_compose(
        compose_file,
        [
            "logs",
            "learner",
            "sidecar-1",
            "web-panel-distributed",
            "mock-godot",
            "--tail",
            "200",
        ],
        actor_id=actor_id,
        capture_output=True,
    )
    output = "\n".join(
        part for part in [result.stdout.strip(), result.stderr.strip()] if part
    )
    return output or "no compose logs captured"


def _parse_compose_ps_output(output: str) -> dict[str, dict[str, Any]]:
    stripped = output.strip()
    if not stripped:
        return {}

    documents: list[dict[str, Any]]
    try:
        parsed = json.loads(stripped)
        documents = parsed if isinstance(parsed, list) else [parsed]
    except json.JSONDecodeError:
        documents = [
            parsed_line
            for line in stripped.splitlines()
            if line.strip()
            for parsed_line in [json.loads(line)]
        ]

    services: dict[str, dict[str, Any]] = {}
    for item in documents:
        service = item.get("Service") or item.get("Name")
        if not service:
            continue
        services[str(service)] = {
            "name": item.get("Name"),
            "state": item.get("State") or item.get("Status"),
            "status": item.get("Status"),
            "exit_code": item.get("ExitCode"),
        }
    return services


def _collect_service_statuses(compose_file: Path, actor_id: str) -> dict[str, Any]:
    result = _run_compose(
        compose_file,
        ["ps", "--format", "json"],
        actor_id=actor_id,
        capture_output=True,
    )
    if result.returncode != 0:
        return {"error": (result.stderr or result.stdout).strip()}
    try:
        return _parse_compose_ps_output(result.stdout)
    except Exception as exc:
        return {"error": f"failed to parse docker compose ps output: {exc}"}


def _tail_lines(lines: list[str], limit: int = 20) -> list[str]:
    return lines[-limit:]


def _build_log_diagnostics(logs: str, actor_id: str) -> dict[str, Any]:
    lines = logs.splitlines()
    service_logs: dict[str, list[str]] = {service: [] for service in SMOKE_SERVICES}
    for line in lines:
        header = line.split("|", 1)[0]
        for service in SMOKE_SERVICES:
            if service in header:
                service_logs[service].append(line)
                break

    hints = [
        {
            "code": code,
            "message": message,
        }
        for code, pattern, message in LOG_HINT_PATTERNS
        if pattern.search(logs)
    ]
    has_service_logs = any(service_lines for service_lines in service_logs.values())
    if actor_id not in logs and has_service_logs:
        hints.append(
            {
                "code": "actor_id_missing_from_logs",
                "message": "The expected actor id was not observed in compose logs.",
            }
        )

    return {
        "log_line_count": len(lines),
        "hints": hints,
        "services": {
            service: {
                "line_count": len(service_lines),
                "tail": _tail_lines(service_lines),
            }
            for service, service_lines in service_logs.items()
        },
        "tail": _tail_lines(lines, limit=40),
    }


def _emit_report(report: dict[str, Any], report_file: str | None) -> None:
    report["finished_at"] = report.get("finished_at") or _utc_now()
    encoded = json.dumps(report, ensure_ascii=False, indent=2)
    print("distributed smoke report:")
    print(encoded)
    if report_file:
        path = Path(report_file)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(encoded + "\n", encoding="utf-8")


def _fail(
    report: dict[str, Any],
    *,
    check_name: str,
    label: str,
    detail: str,
    compose_file: Path,
    actor_id: str,
    report_file: str | None,
    stdout: str = "",
    stderr: str = "",
    context: dict[str, Any] | None = None,
) -> int:
    logs = _collect_logs(compose_file, actor_id)
    report["status"] = "failed"
    report["failed_check"] = check_name
    report["diagnostics"] = {
        "detail": detail,
        "stdout": stdout,
        "stderr": stderr,
        "compose_services": _collect_service_statuses(compose_file, actor_id),
        "logs": _build_log_diagnostics(logs, actor_id),
        **(context or {}),
    }
    _record_check(report, check_name, "fail", {"detail": detail})
    print(f"{label}: FAIL")
    if stdout:
        print(stdout)
    if stderr:
        print(stderr)
    _emit_report(report, report_file)
    print(logs)
    return 1


def _mock_godot_received_step(compose_file: Path, actor_id: str) -> bool:
    logs = _collect_logs(compose_file, actor_id)
    return "received step #" in logs


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run the distributed Docker smoke test."
    )
    parser.add_argument(
        "--compose-file",
        default=str(DEFAULT_COMPOSE_FILE),
        help="Path to deployment/docker-compose.yml",
    )
    parser.add_argument(
        "--web-panel-url",
        default=DEFAULT_WEB_PANEL_URL,
        help="Base URL of the distributed web panel service.",
    )
    parser.add_argument(
        "--actor-id",
        default=DEFAULT_ACTOR_ID,
        help="Actor ID expected from sidecar-1.",
    )
    parser.add_argument(
        "--timeout-seconds",
        type=float,
        default=DEFAULT_TIMEOUT_SECONDS,
        help="Overall timeout for the smoke validation.",
    )
    parser.add_argument(
        "--stop-after",
        action="store_true",
        help="Stop the smoke services after the run completes.",
    )
    parser.add_argument(
        "--build",
        action="store_true",
        help="Rebuild learner, sidecar-1, mock-godot, and web-panel-distributed before running the smoke test.",
    )
    parser.add_argument(
        "--report-file",
        default=None,
        help="Optional path for a machine-readable distributed smoke report.",
    )
    args = parser.parse_args()

    compose_file = Path(args.compose_file).resolve()
    web_panel_url = args.web_panel_url.rstrip("/")
    report = _new_report(compose_file, web_panel_url, args.actor_id)

    print("AGI-Walker distributed smoke test")
    print(f"compose file: {compose_file}")
    print(f"web panel url: {web_panel_url}")
    print(f"actor id: {args.actor_id}")
    print()

    try:
        _run_compose(
            compose_file,
            ["stop", "sidecar-1"],
            actor_id=args.actor_id,
            capture_output=True,
        )

        if args.build:
            build_result = _run_compose(
                compose_file,
                [
                    "build",
                    "learner",
                    "sidecar-1",
                    "mock-godot",
                    "web-panel-distributed",
                ],
                actor_id=args.actor_id,
                capture_output=True,
            )
            if build_result.returncode != 0:
                return _fail(
                    report,
                    check_name="compose_build",
                    label="compose build",
                    detail="docker compose build returned a non-zero exit code",
                    compose_file=compose_file,
                    actor_id=args.actor_id,
                    report_file=args.report_file,
                    stdout=build_result.stdout,
                    stderr=build_result.stderr,
                )
            _record_check(report, "compose_build", "pass")
            print("compose build: PASS")

            runtime_dependency_result = _run_compose(
                compose_file,
                [
                    "run",
                    "--rm",
                    "--no-deps",
                    "learner",
                    "python",
                    "-c",
                    "import yaml; from agi_walker.skills_loader import SkillsLoader; print('distributed_runtime_import_ok')",
                ],
                actor_id=args.actor_id,
                capture_output=True,
            )
            if runtime_dependency_result.returncode != 0:
                return _fail(
                    report,
                    check_name="runtime_dependency_check",
                    label="runtime dependency check",
                    detail="distributed runtime image cannot import required Python dependencies",
                    compose_file=compose_file,
                    actor_id=args.actor_id,
                    report_file=args.report_file,
                    stdout=runtime_dependency_result.stdout,
                    stderr=runtime_dependency_result.stderr,
                )
            _record_check(
                report,
                "runtime_dependency_check",
                "pass",
                {"stdout": runtime_dependency_result.stdout.strip()},
            )
            print("runtime dependency check: PASS")

        up_args = ["up", "-d", "--force-recreate", *SMOKE_BOOTSTRAP_SERVICES]
        result = _run_compose(
            compose_file,
            up_args,
            actor_id=args.actor_id,
            capture_output=True,
        )
        if result.returncode != 0:
            return _fail(
                report,
                check_name="compose_up",
                label="compose up",
                detail="docker compose up returned a non-zero exit code",
                compose_file=compose_file,
                actor_id=args.actor_id,
                report_file=args.report_file,
                stdout=result.stdout,
                stderr=result.stderr,
            )
        _record_check(report, "compose_up", "pass")
        print("compose up: PASS")

        system_status = _wait_for_condition(
            args.timeout_seconds,
            lambda: _try_fetch_json(f"{web_panel_url}/api/system/status"),
        )
        if not system_status or system_status.get("status") != "running":
            return _fail(
                report,
                check_name="web_panel_status",
                label="web panel status",
                detail="web panel did not report status=running before timeout",
                compose_file=compose_file,
                actor_id=args.actor_id,
                report_file=args.report_file,
                context={"system_status": system_status},
            )
        _record_check(report, "web_panel_status", "pass", system_status)

        distributed_ready = _wait_for_condition(
            args.timeout_seconds,
            lambda: (
                status
                if (
                    (status := _try_fetch_json(f"{web_panel_url}/api/system/status"))
                    and status.get("distributed_monitor", {}).get("monitor_active")
                )
                else None
            ),
        )
        if not distributed_ready:
            return _fail(
                report,
                check_name="distributed_monitor",
                label="distributed monitor",
                detail="distributed monitor did not become active before timeout",
                compose_file=compose_file,
                actor_id=args.actor_id,
                report_file=args.report_file,
                context={"system_status": system_status},
            )
        _record_check(
            report,
            "distributed_monitor",
            "pass",
            distributed_ready.get("distributed_monitor", {}),
        )

        sidecar_result = _run_compose(
            compose_file,
            ["up", "-d", "--force-recreate", "sidecar-1"],
            actor_id=args.actor_id,
            capture_output=True,
        )
        if sidecar_result.returncode != 0:
            return _fail(
                report,
                check_name="sidecar_start",
                label="sidecar start",
                detail="sidecar-1 failed to start",
                compose_file=compose_file,
                actor_id=args.actor_id,
                report_file=args.report_file,
                stdout=sidecar_result.stdout,
                stderr=sidecar_result.stderr,
            )
        _record_check(report, "sidecar_start", "pass")
        print("sidecar start: PASS")

        actor_status = _wait_for_condition(
            args.timeout_seconds,
            lambda: (
                status
                if (
                    (
                        status := _try_fetch_json(
                            f"{web_panel_url}/api/distributed/status"
                        )
                    )
                    and args.actor_id in status.get("actors", {})
                )
                else None
            ),
        )
        if not actor_status:
            latest_status = _try_fetch_json(f"{web_panel_url}/api/distributed/status")
            return _fail(
                report,
                check_name="actor_discovery",
                label="actor discovery",
                detail=f"actor {args.actor_id!r} did not appear before timeout",
                compose_file=compose_file,
                actor_id=args.actor_id,
                report_file=args.report_file,
                context={"distributed_status": latest_status},
            )
        _record_check(
            report,
            "actor_discovery",
            "pass",
            {
                "actor_id": args.actor_id,
                "actor_ids": sorted(actor_status.get("actors", {})),
            },
        )

        step_seen = _wait_for_condition(
            max(30.0, min(args.timeout_seconds, 90.0)),
            lambda: (
                True if _mock_godot_received_step(compose_file, args.actor_id) else None
            ),
        )
        if not step_seen:
            return _fail(
                report,
                check_name="learner_action_loop",
                label="learner action loop",
                detail="mock-godot did not receive a learner step action before timeout",
                compose_file=compose_file,
                actor_id=args.actor_id,
                report_file=args.report_file,
                context={"actor_status": actor_status},
            )
        _record_check(report, "learner_action_loop", "pass")

        print("distributed smoke summary: PASS")
        report["status"] = "passed"
        report["system_status"] = distributed_ready
        report["actor_snapshot"] = actor_status["actors"][args.actor_id]
        report["compose_services"] = _collect_service_statuses(
            compose_file, args.actor_id
        )
        print(json.dumps(distributed_ready, ensure_ascii=False, indent=2))
        print(
            json.dumps(
                actor_status["actors"][args.actor_id], ensure_ascii=False, indent=2
            )
        )
        _emit_report(report, args.report_file)
        return 0
    finally:
        if args.stop_after:
            _run_compose(
                compose_file,
                ["stop", *SMOKE_SERVICES],
                actor_id=args.actor_id,
                capture_output=True,
            )


if __name__ == "__main__":
    raise SystemExit(main())
