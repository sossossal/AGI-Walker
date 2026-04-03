from __future__ import annotations

import json
import os
import time
import urllib.parse
import urllib.request
from datetime import datetime
from typing import Any, Callable, Dict, List


DEFAULT_API_BASE = "https://api.github.com"
DEFAULT_WORKFLOW_FILE = ".github/workflows/ci.yml"
DEFAULT_TRACKED_JOBS = ("smoke", "distributed-smoke", "godot-headless-smoke")
JOB_METADATA = {
    "smoke": {
        "label": "Smoke",
        "artifact_name": "smoke-artifacts",
        "artifact_path": "test_env/smoke_ci/",
        "local_repro_command": "python tests/run_smoke_tests.py --output-root test_env/smoke_runs/manual_nightly_repro",
        "owner_area": "Core / Workflow",
        "artifact_paths": [
            "test_env/smoke_ci/robot_creation_real/workflow_artifacts/",
            "test_env/smoke_ci/robot_creation_mock/workflow_artifacts/",
        ],
    },
    "distributed-smoke": {
        "label": "Distributed Smoke",
        "artifact_name": "distributed-smoke-artifacts",
        "artifact_path": "test_env/distributed_smoke/",
        "local_repro_command": "python tests/run_distributed_smoke.py --build",
        "owner_area": "Distributed Runtime",
    },
    "godot-headless-smoke": {
        "label": "Godot Headless Smoke",
        "artifact_name": "godot-headless-smoke-artifacts",
        "artifact_path": "test_env/",
        "local_repro_command": "python -m pytest tests/test_godot_headless_smoke.py -q -m integration",
        "owner_area": "Web / Godot Integration",
        "artifact_paths": [
            "test_env/godot_headless_smoke/headless_smoke_report.json",
            "test_env/godot_headless_smoke/",
        ],
    },
}


def _env_int(name: str, default: int) -> int:
    raw = os.getenv(name, "").strip()
    if not raw:
        return default
    try:
        return int(raw)
    except ValueError:
        return default


def _parse_timestamp(value: str | None) -> datetime:
    if not value:
        return datetime.min
    normalized = value.replace("Z", "+00:00")
    try:
        return datetime.fromisoformat(normalized)
    except ValueError:
        return datetime.min


class NightlyStatusProvider:
    def __init__(
        self,
        repo: str | None = None,
        workflow_file: str = DEFAULT_WORKFLOW_FILE,
        tracked_jobs: tuple[str, ...] = DEFAULT_TRACKED_JOBS,
        token: str | None = None,
        api_base: str = DEFAULT_API_BASE,
        cache_ttl_seconds: int = 300,
        fetch_json: Callable[[str, Dict[str, str]], Dict[str, Any]] | None = None,
    ) -> None:
        self.repo = (repo or "").strip()
        self.workflow_file = workflow_file.strip() or DEFAULT_WORKFLOW_FILE
        self.tracked_jobs = tracked_jobs or DEFAULT_TRACKED_JOBS
        self.token = (token or "").strip() or None
        self.api_base = api_base.rstrip("/")
        self.cache_ttl_seconds = max(cache_ttl_seconds, 0)
        self._fetch_json = fetch_json or self._default_fetch_json
        self._cached_dashboard: Dict[str, Any] | None = None
        self._cached_limit: int = 0
        self._cached_at: float = 0.0

    @classmethod
    def from_env(cls) -> "NightlyStatusProvider":
        tracked_jobs_raw = os.getenv("AGI_WALKER_NIGHTLY_TRACKED_JOBS", "")
        tracked_jobs = tuple(
            job.strip() for job in tracked_jobs_raw.split(",") if job.strip()
        ) or DEFAULT_TRACKED_JOBS
        return cls(
            repo=os.getenv("AGI_WALKER_GITHUB_REPO"),
            workflow_file=os.getenv(
                "AGI_WALKER_GITHUB_WORKFLOW_FILE",
                DEFAULT_WORKFLOW_FILE,
            ),
            tracked_jobs=tracked_jobs,
            token=os.getenv("AGI_WALKER_GITHUB_TOKEN"),
            api_base=os.getenv("AGI_WALKER_GITHUB_API_BASE", DEFAULT_API_BASE),
            cache_ttl_seconds=_env_int("AGI_WALKER_NIGHTLY_STATUS_CACHE_SECONDS", 300),
        )

    def snapshot(self) -> Dict[str, Any]:
        dashboard = self.dashboard(limit_runs=1)
        snapshot = {
            "status": dashboard["status"],
            "message": dashboard["message"],
            "repo": dashboard["repo"],
            "workflow_file": dashboard["workflow_file"],
            "tracked_jobs": dashboard["tracked_jobs"],
            "fetched_at": dashboard["fetched_at"],
            "summary": dashboard["summary"],
            "jobs": dashboard["jobs"],
            "latest_run": dashboard["latest_run"],
            "configured": dashboard["configured"],
            "cache_state": dashboard["cache_state"],
        }
        if "stale_reason" in dashboard:
            snapshot["stale_reason"] = dashboard["stale_reason"]
        return snapshot

    def dashboard(self, limit_runs: int = 5) -> Dict[str, Any]:
        limit_runs = max(limit_runs, 1)
        now = time.monotonic()
        if (
            self._cached_dashboard is not None
            and self.cache_ttl_seconds > 0
            and (now - self._cached_at) < self.cache_ttl_seconds
            and self._cached_limit >= limit_runs
        ):
            cached = self._trim_dashboard(self._cached_dashboard, limit_runs)
            cached["cache_state"] = "hit"
            return cached

        if not self.repo:
            dashboard = self._base_dashboard(
                status="not_configured",
                message="Set AGI_WALKER_GITHUB_REPO to enable nightly regression status.",
            )
            dashboard["configured"] = False
            dashboard["cache_state"] = "disabled"
            return self._trim_dashboard(dashboard, limit_runs)

        try:
            dashboard = self._fetch_dashboard(limit_runs=limit_runs)
            dashboard["cache_state"] = "miss"
            self._cached_dashboard = dashboard
            self._cached_limit = limit_runs
            self._cached_at = now
            return self._trim_dashboard(dashboard, limit_runs)
        except Exception as exc:  # pragma: no cover - defensive fallback
            if self._cached_dashboard is not None:
                stale = self._trim_dashboard(self._cached_dashboard, limit_runs)
                stale["status"] = "stale"
                stale["message"] = f"Using cached nightly status: {exc}"
                stale["stale_reason"] = str(exc)
                stale["cache_state"] = "stale"
                return stale
            dashboard = self._base_dashboard(
                status="error",
                message=f"Failed to load nightly regression status: {exc}",
            )
            dashboard["cache_state"] = "error"
            return self._trim_dashboard(dashboard, limit_runs)

    def _fetch_dashboard(self, limit_runs: int) -> Dict[str, Any]:
        recent_runs = self._fetch_recent_runs(limit_runs)
        if not recent_runs:
            dashboard = self._base_dashboard(
                status="empty",
                message="No nightly or manual specialized workflow runs found yet.",
            )
            dashboard["configured"] = True
            return dashboard

        latest_run = recent_runs[0]
        dashboard = self._base_dashboard(
            status=latest_run["status"],
            message=f"Latest specialized run #{latest_run.get('run_number', latest_run.get('id'))}",
        )
        dashboard["configured"] = True
        dashboard["latest_run"] = self._strip_run_summary(latest_run)
        dashboard["jobs"] = latest_run["jobs"]
        dashboard["summary"] = latest_run["summary"]
        dashboard["recent_runs"] = recent_runs
        return dashboard

    def _fetch_recent_runs(self, limit_runs: int) -> List[Dict[str, Any]]:
        candidate_runs: List[Dict[str, Any]] = []
        for event_name in ("schedule", "workflow_dispatch"):
            runs_payload = self._get_workflow_runs(event_name=event_name, per_page=limit_runs)
            for run in runs_payload.get("workflow_runs", []):
                candidate_runs.append(run)

        unique_runs: Dict[int, Dict[str, Any]] = {}
        for run in candidate_runs:
            run_id = run.get("id")
            if run_id is not None:
                unique_runs[run_id] = run

        sorted_runs = sorted(
            unique_runs.values(),
            key=lambda item: _parse_timestamp(item.get("run_started_at") or item.get("created_at")),
            reverse=True,
        )

        recent_runs: List[Dict[str, Any]] = []
        for run in sorted_runs[:limit_runs]:
            jobs = self._get_jobs(run["id"])
            job_map = self._build_job_map(jobs)
            if not any(job.get("present") for job in job_map.values()):
                continue
            recent_runs.append(self._compose_run_record(run, job_map))
        return recent_runs

    def _compose_run_record(
        self,
        run: Dict[str, Any],
        job_map: Dict[str, Dict[str, Any]],
    ) -> Dict[str, Any]:
        summary = self._summarize_jobs(job_map)
        status = "healthy"
        if summary["failed_jobs"] > 0:
            status = "degraded"
        elif summary["running_jobs"] > 0:
            status = "running"
        elif summary["missing_jobs"] == len(self.tracked_jobs):
            status = "empty"

        return {
            "id": run.get("id"),
            "run_number": run.get("run_number"),
            "event": run.get("event"),
            "status": status,
            "workflow_status": run.get("status"),
            "conclusion": run.get("conclusion"),
            "created_at": run.get("created_at"),
            "updated_at": run.get("updated_at"),
            "run_started_at": run.get("run_started_at"),
            "html_url": run.get("html_url"),
            "head_branch": run.get("head_branch"),
            "jobs": job_map,
            "summary": summary,
        }

    def _summarize_jobs(self, job_map: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
        summary = {
            "tracked_jobs": len(self.tracked_jobs),
            "passed_jobs": 0,
            "failed_jobs": 0,
            "running_jobs": 0,
            "missing_jobs": 0,
            "skipped_jobs": 0,
            "artifact_paths_total": 0,
            "jobs_with_artifacts": 0,
        }
        for job in job_map.values():
            if not job.get("present"):
                summary["missing_jobs"] += 1
                continue
            if job.get("status") != "completed":
                summary["running_jobs"] += 1
                continue

            conclusion = (job.get("conclusion") or "").lower()
            if conclusion == "success":
                summary["passed_jobs"] += 1
            elif conclusion == "skipped":
                summary["skipped_jobs"] += 1
            elif conclusion:
                summary["failed_jobs"] += 1
            else:
                summary["running_jobs"] += 1
            paths = job.get("artifact_paths") or []
            if paths:
                summary["jobs_with_artifacts"] += 1
                summary["artifact_paths_total"] += len(paths)
        return summary

    def _strip_run_summary(self, run_record: Dict[str, Any]) -> Dict[str, Any]:
        return {
            "id": run_record.get("id"),
            "run_number": run_record.get("run_number"),
            "event": run_record.get("event"),
            "status": run_record.get("workflow_status"),
            "conclusion": run_record.get("conclusion"),
            "created_at": run_record.get("created_at"),
            "updated_at": run_record.get("updated_at"),
            "run_started_at": run_record.get("run_started_at"),
            "html_url": run_record.get("html_url"),
            "head_branch": run_record.get("head_branch"),
        }

    def _build_job_map(self, jobs_payload: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
        jobs = jobs_payload.get("jobs", [])
        indexed = {job.get("name"): job for job in jobs}
        result: Dict[str, Dict[str, Any]] = {}
        for job_name in self.tracked_jobs:
            meta = JOB_METADATA.get(job_name, {})
            job = indexed.get(job_name)
            if job is None:
                result[job_name] = {
                    "name": job_name,
                    "present": False,
                    "status": "missing",
                    "conclusion": None,
                    "html_url": None,
                    "started_at": None,
                    "completed_at": None,
                    **meta,
                }
                continue

            result[job_name] = {
                "name": job_name,
                "present": True,
                "status": job.get("status"),
                "conclusion": job.get("conclusion"),
                "html_url": job.get("html_url"),
                "started_at": job.get("started_at"),
                "completed_at": job.get("completed_at"),
                **meta,
            }
        return result

    def _get_workflow_runs(self, event_name: str, per_page: int) -> Dict[str, Any]:
        workflow_path = urllib.parse.quote(self.workflow_file, safe="")
        query = urllib.parse.urlencode({"event": event_name, "per_page": per_page})
        url = f"{self.api_base}/repos/{self.repo}/actions/workflows/{workflow_path}/runs?{query}"
        return self._fetch_json(url, self._headers())

    def _get_jobs(self, run_id: int) -> Dict[str, Any]:
        query = urllib.parse.urlencode({"per_page": 100})
        url = f"{self.api_base}/repos/{self.repo}/actions/runs/{run_id}/jobs?{query}"
        return self._fetch_json(url, self._headers())

    def _headers(self) -> Dict[str, str]:
        headers = {
            "Accept": "application/vnd.github+json",
            "User-Agent": "AGI-Walker-nightly-status",
        }
        if self.token:
            headers["Authorization"] = f"Bearer {self.token}"
        return headers

    def _base_dashboard(self, status: str, message: str) -> Dict[str, Any]:
        return {
            "status": status,
            "message": message,
            "repo": self.repo or None,
            "workflow_file": self.workflow_file,
            "tracked_jobs": list(self.tracked_jobs),
            "fetched_at": datetime.now().isoformat(),
            "summary": {
                "tracked_jobs": len(self.tracked_jobs),
                "passed_jobs": 0,
                "failed_jobs": 0,
                "running_jobs": 0,
                "missing_jobs": len(self.tracked_jobs),
                "skipped_jobs": 0,
            },
            "jobs": {},
            "latest_run": None,
            "recent_runs": [],
            "job_catalog": {
                job_name: {"name": job_name, **JOB_METADATA.get(job_name, {})}
                for job_name in self.tracked_jobs
            },
        }

    def _trim_dashboard(self, dashboard: Dict[str, Any], limit_runs: int) -> Dict[str, Any]:
        trimmed = dict(dashboard)
        trimmed["recent_runs"] = list(dashboard.get("recent_runs", []))[:limit_runs]
        return trimmed

    @staticmethod
    def _default_fetch_json(url: str, headers: Dict[str, str]) -> Dict[str, Any]:
        request = urllib.request.Request(url, headers=headers)
        with urllib.request.urlopen(request, timeout=10) as response:
            return json.load(response)


def get_nightly_regression_status(app: Any) -> Dict[str, Any]:
    provider = getattr(app.state, "nightly_status_provider", None)
    if provider is None:
        provider = NightlyStatusProvider.from_env()
        app.state.nightly_status_provider = provider
    return provider.snapshot()


def get_nightly_regression_dashboard(app: Any, limit_runs: int = 5) -> Dict[str, Any]:
    provider = getattr(app.state, "nightly_status_provider", None)
    if provider is None:
        provider = NightlyStatusProvider.from_env()
        app.state.nightly_status_provider = provider
    return provider.dashboard(limit_runs=limit_runs)
