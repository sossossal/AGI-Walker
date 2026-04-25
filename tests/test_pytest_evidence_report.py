from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from agi_walker.core.api.release_contracts import validate_release_evidence_report


PROJECT_ROOT = Path(__file__).resolve().parent.parent


def test_write_pytest_evidence_report_script_writes_valid_report(
    tmp_path: Path,
) -> None:
    output_path = tmp_path / "non_live_gate_report.json"
    result = subprocess.run(
        [
            sys.executable,
            "tools/write_pytest_evidence_report.py",
            "--name",
            "docs_utf8_check",
            "--output",
            str(output_path),
            "--",
            "tests/test_docs_utf8.py",
            "-q",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "release_evidence_report_written=" in result.stdout
    assert "release_evidence_status=passed" in result.stdout
    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert validate_release_evidence_report(payload) == []
    assert payload["evidence_name"] == "docs_utf8_check"
    assert payload["status"] == "passed"
    assert payload["metrics"]["passed"] >= 1
    assert Path(payload["stdout_path"]).is_file()
    assert Path(payload["stderr_path"]).is_file()
