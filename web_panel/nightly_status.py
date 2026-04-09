from __future__ import annotations

import json
import os
import time
import urllib.request
from datetime import datetime, timezone
from typing import Any, Callable, Dict, List, Optional


DEFAULT_API_BASE = "https://api.github.com"
DEFAULT_WORKFLOW_FILE = ".github/workflows/ci.yml"
DEFAULT_TRACKED_JOBS = [
    "smoke",
    "distributed-smoke",
    "godot-headless-smoke",
]


def _parse_timestamp(ts_str: Optional[str]) -> datetime:
    if not ts_str:
        return datetime.min.replace(tzinfo=timezone.utc)
    return datetime.fromisoformat(ts_str.replace("Z", "+00:00"))


class NightlyStatusProvider:
    """Provide cached nightly regression status snapshots for the Web panel."""

    def __init__(
        self,
        repo: Optional[str] = None,
        token: Optional[str] = None,
        workflow_file: str = DEFAULT_WORKFLOW_FILE,
        cache_ttl_seconds: int = 300,
        fetch_json: Optional[Callable[[str, Dict[str, str]], Any]] = None,
    ):
        self.repo = repo
        self.token = token
        self.workflow_file = workflow_file
        self.cache_ttl_seconds = cache_ttl_seconds
        self.fetch_json = fetch_json
        self.tracked_jobs = list(DEFAULT_TRACKED_JOBS)
        self._cached_dashboard: Optional[Dict[str, Any]] = None
        self._cached_requested_limit: int = 0
        self._cached_at: float = 0.0

    @classmethod
    def from_env(cls) -> NightlyStatusProvider:
        return cls(
            repo=os.getenv("AGI_WALKER_GITHUB_REPO"),
            token=os.getenv("AGI_WALKER_GITHUB_TOKEN"),
            workflow_file=os.getenv(
                "AGI_WALKER_GITHUB_WORKFLOW_FILE", DEFAULT_WORKFLOW_FILE
            ),
            cache_ttl_seconds=int(
                os.getenv("AGI_WALKER_NIGHTLY_STATUS_CACHE_SECONDS", "300")
            ),
        )

    def snapshot(self) -> Dict[str, Any]:
        return self.dashboard(limit_runs=5)

    def dashboard(self, limit_runs: int = 5) -> Dict[str, Any]:
        limit_runs = max(limit_runs, 1)
        now = time.monotonic()

        if (
            self._cached_dashboard is not None
            and self.cache_ttl_seconds > 0
            and (now - self._cached_at) < self.cache_ttl_seconds
            and self._cached_requested_limit == limit_runs
        ):
            cached = self._trim_dashboard(self._cached_dashboard, limit_runs)
            cached["cache_state"] = "hit"
            return cached

        if not self.repo:
            dashboard = self._base_dashboard(
                status="not_configured",
                message="Missing repo",
                configured=False,
            )
            dashboard["summary"] = {
                "tracked_jobs": len(self.tracked_jobs),
                "passed_jobs": 0,
                "failed_jobs": 0,
                "running_jobs": 0,
                "missing_jobs": len(self.tracked_jobs),
                "skipped_jobs": 0,
            }
            dashboard["cache_state"] = "miss"
            return dashboard

        try:
            fetch_limit = limit_runs if self.fetch_json else max(limit_runs, 10)
            dashboard = self._fetch_dashboard(limit_runs=fetch_limit)
            dashboard["trends"] = self._calculate_trends(
                dashboard.get("recent_runs", [])
            )
            dashboard["cache_state"] = "miss"
            self._cached_dashboard = dashboard
            self._cached_requested_limit = limit_runs
            self._cached_at = now
            return self._trim_dashboard(dashboard, limit_runs)
        except Exception as exc:
            return self._base_dashboard(status="error", message=str(exc))

    def _job_catalog(self) -> Dict[str, Dict[str, str]]:
        return {
            "smoke": {
                "artifact_name": "smoke-artifacts",
                "local_repro_command": "python tests/run_smoke_tests.py --output-root test_env/smoke_runs/manual_nightly_repro",
            },
            "distributed-smoke": {
                "artifact_name": "distributed-smoke-artifacts",
                "local_repro_command": "python tests/run_distributed_smoke.py --build",
            },
            "godot-headless-smoke": {
                "artifact_name": "godot-headless-smoke-artifacts",
                "local_repro_command": "python -m pytest tests/test_godot_headless_smoke.py -q -m integration",
            },
        }

    def _headers(self) -> Dict[str, str]:
        headers = {
            "Accept": "application/vnd.github+json",
            "User-Agent": "AGI-Walker-NightlyStatus/1.0",
        }
        if self.token:
            headers["Authorization"] = f"Bearer {self.token}"
        return headers

    def _request_json(self, url: str) -> Any:
        headers = self._headers()
        if self.fetch_json is not None:
            return self.fetch_json(url, headers)

        request = urllib.request.Request(url, headers=headers)
        with urllib.request.urlopen(request, timeout=15) as response:
            return json.loads(response.read().decode("utf-8"))

    def _calculate_trends(self, runs: List[Dict[str, Any]]) -> Dict[str, Any]:
        if not runs:
            return {}

        success_trend = []
        for run in reversed(runs):
            summary = run.get("summary", {})
            total = (
                summary.get("tracked_jobs", 0)
                - summary.get("missing_jobs", 0)
                - summary.get("running_jobs", 0)
            )
            if total <= 0:
                continue
            rate = (summary.get("passed_jobs", 0) / total) * 100
            success_trend.append({"run": run.get("run_number"), "rate": round(rate, 1)})
        return {"success_rate": success_trend}

    def _base_dashboard(
        self, status: str, message: str, configured: bool = True
    ) -> Dict[str, Any]:
        return {
            "status": "healthy" if status == "ok" else status,
            "message": message,
            "updated_at": datetime.now(timezone.utc).isoformat(),
            "repo": self.repo,
            "workflow_file": self.workflow_file,
            "configured": configured,
            "tracked_jobs": list(self.tracked_jobs),
            "job_catalog": self._job_catalog(),
            "recent_runs": [],
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
        }

    def _trim_dashboard(self, dashboard: Dict[str, Any], limit: int) -> Dict[str, Any]:
        snapshot = dict(dashboard)
        runs = list(dashboard.get("recent_runs", []))
        snapshot["recent_runs"] = runs[:limit]
        if snapshot["recent_runs"]:
            latest_run = snapshot["recent_runs"][0]
            snapshot["latest_run"] = latest_run
            snapshot["jobs"] = latest_run.get("jobs", {})
            snapshot["summary"] = latest_run.get("summary", {})
            snapshot["status"] = latest_run.get("status", dashboard.get("status"))
        return snapshot

    def _build_jobs_summary(
        self, jobs_payload: List[Dict[str, Any]]
    ) -> tuple[Dict[str, Any], Dict[str, int]]:
        raw_jobs = {job.get("name"): job for job in jobs_payload if job.get("name")}
        job_catalog = self._job_catalog()
        jobs: Dict[str, Any] = {}
        summary = {
            "tracked_jobs": len(self.tracked_jobs),
            "passed_jobs": 0,
            "failed_jobs": 0,
            "running_jobs": 0,
            "missing_jobs": 0,
            "skipped_jobs": 0,
        }

        for name in self.tracked_jobs:
            raw = raw_jobs.get(name)
            payload = {
                "name": name,
                "artifact_name": job_catalog[name]["artifact_name"],
                "local_repro_command": job_catalog[name]["local_repro_command"],
            }
            if raw is None:
                payload.update(
                    {
                        "present": False,
                        "status": "missing",
                        "conclusion": None,
                    }
                )
                summary["missing_jobs"] += 1
                jobs[name] = payload
                continue

            payload.update(
                {
                    "present": True,
                    "status": raw.get("status"),
                    "conclusion": raw.get("conclusion"),
                    "html_url": raw.get("html_url"),
                    "started_at": raw.get("started_at"),
                    "completed_at": raw.get("completed_at"),
                }
            )
            status = raw.get("status")
            conclusion = raw.get("conclusion")
            if status != "completed":
                summary["running_jobs"] += 1
            elif conclusion == "success":
                summary["passed_jobs"] += 1
            elif conclusion == "skipped":
                summary["skipped_jobs"] += 1
            else:
                summary["failed_jobs"] += 1
            jobs[name] = payload

        return jobs, summary

    def _run_status(
        self, run: Dict[str, Any], summary: Dict[str, int], jobs: Dict[str, Any]
    ) -> str:
        if summary["failed_jobs"] > 0:
            return "degraded"
        if run.get("conclusion") not in {None, "success"}:
            return "degraded"
        if any(job.get("status") == "in_progress" for job in jobs.values()):
            return "running"
        return "healthy"

    def _fetch_dashboard(self, limit_runs: int) -> Dict[str, Any]:
        dashboard = self._base_dashboard("ok", "Success")
        base = (
            f"{DEFAULT_API_BASE}/repos/{self.repo}/actions/workflows/{self.workflow_file}/runs"
        )
        runs: List[Dict[str, Any]] = []
        for event in ("schedule", "workflow_dispatch"):
            payload = self._request_json(f"{base}?event={event}&per_page={limit_runs}")
            runs.extend(payload.get("workflow_runs", []))

        runs.sort(
            key=lambda run: _parse_timestamp(
                run.get("run_started_at") or run.get("created_at")
            ),
            reverse=True,
        )

        recent_runs = []
        for run in runs[:limit_runs]:
            jobs_payload = self._request_json(
                f"{DEFAULT_API_BASE}/repos/{self.repo}/actions/runs/{run['id']}/jobs"
            ).get("jobs", [])
            jobs, summary = self._build_jobs_summary(jobs_payload)
            recent_runs.append(
                {
                    "id": run.get("id"),
                    "run_number": run.get("run_number"),
                    "event": run.get("event"),
                    "workflow_status": run.get("status"),
                    "conclusion": run.get("conclusion"),
                    "status": self._run_status(run, summary, jobs),
                    "created_at": run.get("created_at"),
                    "updated_at": run.get("updated_at"),
                    "run_started_at": run.get("run_started_at"),
                    "html_url": run.get("html_url"),
                    "head_branch": run.get("head_branch"),
                    "jobs": jobs,
                    "summary": summary,
                }
            )

        dashboard["recent_runs"] = recent_runs
        return dashboard


def get_nightly_regression_dashboard(app: Any, limit_runs: int = 5) -> Dict[str, Any]:
    return get_provider(app).dashboard(limit_runs=limit_runs)


def get_nightly_regression_status(app: Any) -> Dict[str, Any]:
    return get_provider(app).snapshot()


def get_provider(app: Any) -> NightlyStatusProvider:
    provider = getattr(app.state, "nightly_status_provider", None)
    if provider is None:
        provider = NightlyStatusProvider.from_env()
        app.state.nightly_status_provider = provider
    return provider
