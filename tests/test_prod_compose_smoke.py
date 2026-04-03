from __future__ import annotations

import json
import os
import shutil
import socket
import subprocess
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
from typing import Any

import pytest


ENABLE_ENV_VAR = "AGI_WALKER_ENABLE_PROD_COMPOSE_SMOKE"
DEFAULT_TIMEOUT_SECONDS = 240.0


def _reserve_tcp_port() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(("127.0.0.1", 0))
    _, port = sock.getsockname()
    sock.close()
    return port


def _compose_env(web_port: int, prometheus_port: int, grafana_port: int) -> dict[str, str]:
    env = os.environ.copy()
    env.update(
        {
            "AGI_WALKER_SECRET_KEY": f"prod-compose-smoke-{uuid.uuid4().hex}",
            "AGI_WALKER_DB_PASSWORD": f"db-pass-{uuid.uuid4().hex[:12]}",
            "AGI_WALKER_WEB_PORT": str(web_port),
            "AGI_WALKER_PROMETHEUS_PORT": str(prometheus_port),
            "AGI_WALKER_GRAFANA_PORT": str(grafana_port),
        }
    )
    return env


def _run_compose(project_name: str, env: dict[str, str], *args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    command = ["docker", "compose", "-f", "docker-compose.prod.yml", "-p", project_name, *args]
    completed = subprocess.run(
        command,
        cwd=".",
        env=env,
        check=False,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    if check and completed.returncode != 0:
        raise AssertionError(
            "docker compose command failed:\n"
            f"command: {' '.join(command)}\n"
            f"stdout:\n{completed.stdout}\n"
            f"stderr:\n{completed.stderr}"
        )
    return completed


def _http_request(
    url: str,
    *,
    method: str = "GET",
    json_body: dict[str, Any] | None = None,
    form_body: dict[str, str] | None = None,
    headers: dict[str, str] | None = None,
    timeout: float = 10.0,
) -> tuple[int, bytes]:
    request_headers = dict(headers or {})
    body = None
    if json_body is not None:
        body = json.dumps(json_body).encode("utf-8")
        request_headers["Content-Type"] = "application/json"
    elif form_body is not None:
        body = urllib.parse.urlencode(form_body).encode("utf-8")
        request_headers["Content-Type"] = "application/x-www-form-urlencoded"

    request = urllib.request.Request(url, data=body, method=method, headers=request_headers)
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return response.status, response.read()
    except urllib.error.HTTPError as exc:
        return exc.code, exc.read()


def _wait_for_http_ready(base_url: str, *, timeout_seconds: float = DEFAULT_TIMEOUT_SECONDS) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_seconds
    last_error: str | None = None
    while time.monotonic() < deadline:
        try:
            status, payload = _http_request(f"{base_url}/api/system/status", timeout=5.0)
            if status == 200:
                return json.loads(payload.decode("utf-8"))
            last_error = f"unexpected status {status}: {payload.decode('utf-8', errors='ignore')}"
        except Exception as exc:  # pragma: no cover - exercised only on startup races
            last_error = str(exc)
        time.sleep(2.0)
    raise AssertionError(f"web-api did not become ready within {timeout_seconds}s: {last_error}")


def _wait_for_status_code(url: str, *, expected_status: int = 200, timeout_seconds: float = 120.0) -> bytes:
    """Wait until one HTTP endpoint consistently returns the expected status code."""
    deadline = time.monotonic() + timeout_seconds
    last_error: str | None = None
    while time.monotonic() < deadline:
        try:
            status, payload = _http_request(url, timeout=5.0)
            if status == expected_status:
                return payload
            last_error = f"unexpected status {status}: {payload.decode('utf-8', errors='ignore')}"
        except Exception as exc:  # pragma: no cover - exercised only on startup races
            last_error = str(exc)
        time.sleep(2.0)
    raise AssertionError(f"endpoint {url} did not return {expected_status} within {timeout_seconds}s: {last_error}")


def _wait_for_terminal_run(base_url: str, run_id: str, *, timeout_seconds: float = DEFAULT_TIMEOUT_SECONDS) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_seconds
    last_payload: dict[str, Any] | None = None
    while time.monotonic() < deadline:
        status, payload = _http_request(f"{base_url}/api/workflows/runs/{run_id}/status", timeout=5.0)
        if status != 200:
            time.sleep(2.0)
            continue
        last_payload = json.loads(payload.decode("utf-8"))
        run = last_payload["run"]
        if run["status"] in {"completed", "failed", "cancelled", "timed_out"}:
            return run
        time.sleep(2.0)
    raise AssertionError(f"workflow run did not reach a terminal state: {last_payload}")


@pytest.mark.integration
def test_prod_compose_stack_runs_authenticated_workflow() -> None:
    if os.getenv(ENABLE_ENV_VAR) != "1":
        pytest.skip(f"set {ENABLE_ENV_VAR}=1 to enable the production compose smoke test")
    if shutil.which("docker") is None:
        pytest.skip("docker CLI is not available in PATH")

    project_name = f"agiwalker-prod-{uuid.uuid4().hex[:8]}"
    web_port = _reserve_tcp_port()
    prometheus_port = _reserve_tcp_port()
    grafana_port = _reserve_tcp_port()
    env = _compose_env(web_port, prometheus_port, grafana_port)
    base_url = f"http://127.0.0.1:{web_port}"

    try:
        _run_compose(project_name, env, "up", "-d", "--build")

        system_status = _wait_for_http_ready(base_url)
        assert system_status["status"] == "running"

        _wait_for_status_code(f"http://127.0.0.1:{prometheus_port}/-/healthy")
        grafana_payload = _wait_for_status_code(f"http://127.0.0.1:{grafana_port}/api/health")
        assert json.loads(grafana_payload.decode("utf-8"))["database"] == "ok"

        username = f"prod_user_{uuid.uuid4().hex[:8]}"
        password = "prod-pass-1234"

        register_status, register_payload = _http_request(
            f"{base_url}/api/auth/register",
            method="POST",
            form_body={"username": username, "password": password},
            timeout=10.0,
        )
        assert register_status == 200, register_payload.decode("utf-8", errors="ignore")

        login_status, login_payload = _http_request(
            f"{base_url}/api/auth/login",
            method="POST",
            form_body={"username": username, "password": password},
            timeout=10.0,
        )
        assert login_status == 200, login_payload.decode("utf-8", errors="ignore")
        token = json.loads(login_payload.decode("utf-8"))["access_token"]

        run_status, run_payload = _http_request(
            f"{base_url}/api/workflows/robot_creation_pipeline/run",
            method="POST",
            json_body={
                "use_real": True,
                "execution_strategy": "force",
            },
            headers={"Authorization": f"Bearer {token}"},
            timeout=15.0,
        )
        assert run_status == 202, run_payload.decode("utf-8", errors="ignore")
        run_response = json.loads(run_payload.decode("utf-8"))
        run_id = run_response["run"]["run_id"]

        terminal_run = _wait_for_terminal_run(base_url, run_id)
        assert terminal_run["status"] == "completed", terminal_run
        assert terminal_run["worker_pid"] is None

        detail_status, detail_payload = _http_request(f"{base_url}/api/workflows/runs/{run_id}", timeout=10.0)
        assert detail_status == 200, detail_payload.decode("utf-8", errors="ignore")
        detail = json.loads(detail_payload.decode("utf-8"))["run"]
        assert detail["status"] == "completed"
        assert detail["artifacts"], detail
        assert detail["log_download_url"], detail

        artifact_status, artifact_payload = _http_request(
            f"{base_url}{detail['artifacts'][0]['download_url']}",
            timeout=10.0,
        )
        assert artifact_status == 200
        assert artifact_payload

        log_status, log_payload = _http_request(
            f"{base_url}{detail['log_download_url']}",
            timeout=10.0,
        )
        assert log_status == 200
        assert log_payload
    finally:
        _run_compose(project_name, env, "down", "-v", "--remove-orphans", check=False)
