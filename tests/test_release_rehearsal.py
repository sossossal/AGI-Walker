from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from agi_walker.core.api.release_contracts import validate_release_manifest_artifact


PROJECT_ROOT = Path(__file__).resolve().parent.parent


def test_run_release_rehearsal_script_generates_ready_stable_manifest(
    tmp_path: Path,
) -> None:
    output_root = tmp_path / "release_rehearsal"
    report_path = output_root / "release_rehearsal_report.json"

    result = subprocess.run(
        [
            sys.executable,
            "tools/run_release_rehearsal.py",
            "--version",
            "2026.04.12-rehearsal",
            "--build-id",
            "release-rehearsal-test",
            "--output-root",
            str(output_root),
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )

    assert result.returncode == 0, result.stderr
    assert "release_rehearsal_written=" in result.stdout
    assert "release_rehearsal_gate=ready" in result.stdout
    assert report_path.exists()

    report_payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert report_payload["status"] == "passed"
    assert report_payload["tag"] == "2026.04.12-rehearsal"
    assert any(
        item["name"] == "stable_gate_ready" and item["status"] == "pass"
        for item in report_payload["checks"]
    )

    manifest_path = Path(report_payload["manifest_path"])
    assert manifest_path.exists()
    manifest_payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert validate_release_manifest_artifact(manifest_payload) == []
    assert manifest_payload["channel"] == "stable"
    assert manifest_payload["release_gate_status"] == "ready"
    assert manifest_payload["release_source"]["version_tag_matches"] is True


def test_run_release_rehearsal_script_is_repeatable_for_same_output_root(
    tmp_path: Path,
) -> None:
    output_root = tmp_path / "release_rehearsal"

    for _ in range(2):
        result = subprocess.run(
            [
                sys.executable,
                "tools/run_release_rehearsal.py",
                "--version",
                "2026.04.12-rehearsal",
                "--build-id",
                "release-rehearsal-repeatable",
                "--output-root",
                str(output_root),
            ],
            cwd=str(PROJECT_ROOT),
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
        )

        assert result.returncode == 0, result.stderr
        assert "release_rehearsal_gate=ready" in result.stdout
