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
import subprocess
import sys
import time
import urllib.request
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_COMPOSE_FILE = PROJECT_ROOT / "deployment" / "docker-compose.yml"
DEFAULT_WEB_PANEL_URL = "http://127.0.0.1:8081"
DEFAULT_ACTOR_ID = "actor_docker_1"
DEFAULT_TIMEOUT_SECONDS = 180.0
DEFAULT_POLL_INTERVAL_SECONDS = 1.0
SMOKE_SERVICES = [
    "zenoh-router",
    "learner",
    "web-panel-distributed",
    "mock-godot",
    "sidecar-1",
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
    return env


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
    args = parser.parse_args()

    compose_file = Path(args.compose_file).resolve()
    web_panel_url = args.web_panel_url.rstrip("/")

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
                print("compose build: FAIL")
                print(build_result.stdout)
                print(build_result.stderr)
                return 1
            print("compose build: PASS")

        result = _run_compose(
            compose_file,
            ["up", "-d", *SMOKE_BOOTSTRAP_SERVICES],
            actor_id=args.actor_id,
            capture_output=True,
        )
        if result.returncode != 0:
            print("compose up: FAIL")
            print(result.stdout)
            print(result.stderr)
            return 1
        print("compose up: PASS")

        system_status = _wait_for_condition(
            args.timeout_seconds,
            lambda: _try_fetch_json(f"{web_panel_url}/api/system/status"),
        )
        if not system_status or system_status.get("status") != "running":
            print("web panel status: FAIL")
            print(_collect_logs(compose_file, args.actor_id))
            return 1

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
            print("distributed monitor: FAIL")
            print(json.dumps(system_status, ensure_ascii=False, indent=2))
            print(_collect_logs(compose_file, args.actor_id))
            return 1

        sidecar_result = _run_compose(
            compose_file,
            ["up", "-d", "sidecar-1"],
            actor_id=args.actor_id,
            capture_output=True,
        )
        if sidecar_result.returncode != 0:
            print("sidecar start: FAIL")
            print(sidecar_result.stdout)
            print(sidecar_result.stderr)
            return 1
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
            print("actor discovery: FAIL")
            print(_collect_logs(compose_file, args.actor_id))
            return 1

        step_seen = _wait_for_condition(
            max(30.0, min(args.timeout_seconds, 90.0)),
            lambda: (
                True if _mock_godot_received_step(compose_file, args.actor_id) else None
            ),
        )
        if not step_seen:
            print("learner action loop: FAIL")
            print(_collect_logs(compose_file, args.actor_id))
            return 1

        print("distributed smoke summary: PASS")
        print(json.dumps(distributed_ready, ensure_ascii=False, indent=2))
        print(
            json.dumps(
                actor_status["actors"][args.actor_id], ensure_ascii=False, indent=2
            )
        )
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
