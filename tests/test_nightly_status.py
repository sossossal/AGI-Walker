from web_panel.nightly_status import NightlyStatusProvider


def test_nightly_status_provider_returns_not_configured_without_repo():
    provider = NightlyStatusProvider(repo="")
    snapshot = provider.snapshot()

    assert snapshot["status"] == "not_configured"
    assert snapshot["configured"] is False
    assert snapshot["summary"]["missing_jobs"] == 3


def test_nightly_status_provider_parses_github_runs_and_jobs():
    calls = {"count": 0}

    def fake_fetch(url, headers):
        calls["count"] += 1
        if "event=schedule" in url:
            return {
                "workflow_runs": [
                    {
                        "id": 101,
                        "run_number": 88,
                        "event": "schedule",
                        "status": "completed",
                        "conclusion": "success",
                        "created_at": "2026-04-01T02:00:00Z",
                        "updated_at": "2026-04-01T02:12:00Z",
                        "run_started_at": "2026-04-01T02:01:00Z",
                        "html_url": "https://example.invalid/run/101",
                        "head_branch": "main",
                    }
                ]
            }
        if "event=workflow_dispatch" in url:
            return {"workflow_runs": []}
        if "/actions/runs/101/jobs" in url:
            return {
                "jobs": [
                    {
                        "name": "smoke",
                        "status": "completed",
                        "conclusion": "success",
                        "html_url": "https://example.invalid/job/smoke",
                        "started_at": "2026-04-01T02:01:00Z",
                        "completed_at": "2026-04-01T02:03:00Z",
                    },
                    {
                        "name": "distributed-smoke",
                        "status": "completed",
                        "conclusion": "success",
                        "html_url": "https://example.invalid/job/dist",
                        "started_at": "2026-04-01T02:04:00Z",
                        "completed_at": "2026-04-01T02:06:00Z",
                    },
                    {
                        "name": "godot-headless-smoke",
                        "status": "completed",
                        "conclusion": "success",
                        "html_url": "https://example.invalid/job/godot",
                        "started_at": "2026-04-01T02:07:00Z",
                        "completed_at": "2026-04-01T02:12:00Z",
                    },
                ]
            }
        raise AssertionError(f"Unexpected URL: {url}")

    provider = NightlyStatusProvider(
        repo="demo/agi-walker",
        fetch_json=fake_fetch,
        cache_ttl_seconds=300,
    )

    first = provider.snapshot()
    second = provider.snapshot()

    assert first["status"] == "healthy"
    assert first["configured"] is True
    assert first["latest_run"]["run_number"] == 88
    assert first["jobs"]["smoke"]["conclusion"] == "success"
    assert first["summary"]["passed_jobs"] == 3
    assert first["summary"]["failed_jobs"] == 0
    assert first["cache_state"] == "miss"
    assert second["cache_state"] == "hit"
    assert calls["count"] == 3

    dashboard = provider.dashboard(limit_runs=3)
    assert dashboard["cache_state"] == "miss"
    assert len(dashboard["recent_runs"]) == 1
    assert (
        dashboard["recent_runs"][0]["jobs"]["smoke"]["artifact_name"]
        == "smoke-artifacts"
    )
    assert (
        dashboard["job_catalog"]["distributed-smoke"]["artifact_name"]
        == "distributed-smoke-artifacts"
    )
    assert calls["count"] == 6


def test_nightly_status_provider_marks_degraded_when_job_fails():
    def fake_fetch(url, headers):
        if "event=schedule" in url:
            return {
                "workflow_runs": [
                    {
                        "id": 202,
                        "run_number": 89,
                        "event": "schedule",
                        "status": "completed",
                        "conclusion": "failure",
                        "created_at": "2026-04-01T02:00:00Z",
                        "updated_at": "2026-04-01T02:05:00Z",
                        "run_started_at": "2026-04-01T02:01:00Z",
                        "html_url": "https://example.invalid/run/202",
                        "head_branch": "main",
                    }
                ]
            }
        if "event=workflow_dispatch" in url:
            return {"workflow_runs": []}
        if "/actions/runs/202/jobs" in url:
            return {
                "jobs": [
                    {"name": "smoke", "status": "completed", "conclusion": "success"},
                    {
                        "name": "distributed-smoke",
                        "status": "completed",
                        "conclusion": "failure",
                    },
                    {
                        "name": "godot-headless-smoke",
                        "status": "completed",
                        "conclusion": "success",
                    },
                ]
            }
        raise AssertionError(f"Unexpected URL: {url}")

    provider = NightlyStatusProvider(repo="demo/agi-walker", fetch_json=fake_fetch)
    snapshot = provider.snapshot()

    assert snapshot["status"] == "degraded"
    assert snapshot["summary"]["failed_jobs"] == 1
    assert snapshot["jobs"]["distributed-smoke"]["conclusion"] == "failure"
    dashboard = provider.dashboard(limit_runs=2)
    assert dashboard["recent_runs"][0]["status"] == "degraded"
