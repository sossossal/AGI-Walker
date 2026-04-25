from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent


def _make_source_root(tmp_path: Path) -> Path:
    source_root = tmp_path / "source_root"
    source_root.mkdir(parents=True, exist_ok=True)
    (source_root / "README.md").write_text("# clean checkout smoke\n", encoding="utf-8")
    return source_root


def test_clean_checkout_smoke_report_passes_when_custom_command_keeps_checkout_clean(
    tmp_path: Path,
) -> None:
    source_root = _make_source_root(tmp_path)
    output_root = tmp_path / "clean_checkout_smoke"
    report_path = output_root / "clean_checkout_smoke_report.json"
    command_script = (
        "from pathlib import Path; "
        "run_root = Path(r'{run_output_root}'); "
        "run_root.mkdir(parents=True, exist_ok=True); "
        "(run_root / 'status.txt').write_text('ok', encoding='utf-8'); "
        "print('clean_checkout_smoke_ok')"
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/run_clean_checkout_smoke.py",
            "--source-root",
            str(source_root),
            "--output-root",
            str(output_root),
            "--version",
            "1.2.3",
            "--runs",
            "2",
            "--",
            sys.executable,
            "-c",
            command_script,
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "clean_checkout_smoke_status=passed" in result.stdout
    payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert payload["status"] == "passed"
    assert payload["runs"] == 2
    assert len(payload["run_reports"]) == 2
    assert all(item["status"] == "passed" for item in payload["run_reports"])
    assert all(item["worktree_clean"] is True for item in payload["run_reports"])


def test_clean_checkout_smoke_report_blocks_when_command_dirties_checkout(
    tmp_path: Path,
) -> None:
    source_root = _make_source_root(tmp_path)
    output_root = tmp_path / "dirty_checkout_smoke"
    report_path = output_root / "clean_checkout_smoke_report.json"
    command_script = (
        "from pathlib import Path; "
        "readme = Path(r'{checkout_root}') / 'README.md'; "
        "readme.write_text('# dirty\\n', encoding='utf-8'); "
        "print('dirty_checkout_smoke_ok')"
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/run_clean_checkout_smoke.py",
            "--source-root",
            str(source_root),
            "--output-root",
            str(output_root),
            "--version",
            "1.2.4",
            "--runs",
            "1",
            "--",
            sys.executable,
            "-c",
            command_script,
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode == 1
    assert "clean_checkout_smoke_status=blocked" in result.stdout
    payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"
    assert "dirtied the checkout" in (payload["failure_reason"] or "")
    assert payload["run_reports"][0]["worktree_clean"] is False
    assert any(
        entry.endswith("README.md") for entry in payload["run_reports"][0]["dirty_paths"]
    )


def test_clean_checkout_smoke_ignores_transient_workspace_directories(
    tmp_path: Path,
) -> None:
    source_root = _make_source_root(tmp_path)
    preserved_module = (
        source_root
        / "agi_walker"
        / "integrations"
        / "godot_agent"
        / "godot_agent_adapter.py"
    )
    preserved_module.parent.mkdir(parents=True, exist_ok=True)
    preserved_module.write_text("class Preserved:\n    pass\n", encoding="utf-8")
    transient_dirs = [
        "codex_extbind_abcd1234",
        "pytest-cache-files-abcd1234",
        "godot_agent_adapter_abcd1234",
        "godot_agent_factory_abcd1234",
        "runtime_tmp",
        "runtime_pytest_tmp",
    ]
    for name in transient_dirs:
        directory = source_root / name
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "marker.txt").write_text(name, encoding="utf-8")

    output_root = tmp_path / "ignored_transient_checkout_smoke"
    report_path = output_root / "clean_checkout_smoke_report.json"
    command_script = "print('transient_dirs_ignored')"

    result = subprocess.run(
        [
            sys.executable,
            "tools/run_clean_checkout_smoke.py",
            "--source-root",
            str(source_root),
            "--output-root",
            str(output_root),
            "--version",
            "1.2.5",
            "--runs",
            "1",
            "--",
            sys.executable,
            "-c",
            command_script,
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode == 0, result.stderr
    payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert payload["status"] == "passed"
    checkout_root = output_root / "checkout"
    for name in transient_dirs:
        assert not (checkout_root / name).exists()
    assert (
        checkout_root
        / "agi_walker"
        / "integrations"
        / "godot_agent"
        / "godot_agent_adapter.py"
    ).is_file()


def test_clean_checkout_smoke_seeds_structured_release_evidence_when_present(
    tmp_path: Path,
) -> None:
    source_root = _make_source_root(tmp_path)
    seeded_paths = [
        source_root
        / "test_env"
        / "release_evidence"
        / "security"
        / "vulnerability_exception_review_report.json",
        source_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "external_mainline_execution_plan.json",
    ]
    for path in seeded_paths:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('{"status":"passed"}\n', encoding="utf-8")

    output_root = tmp_path / "seeded_evidence_checkout_smoke"
    report_path = output_root / "clean_checkout_smoke_report.json"

    result = subprocess.run(
        [
            sys.executable,
            "tools/run_clean_checkout_smoke.py",
            "--source-root",
            str(source_root),
            "--output-root",
            str(output_root),
            "--version",
            "1.2.6",
            "--runs",
            "1",
            "--",
            sys.executable,
            "-c",
            "print('seeded_evidence_ok')",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode == 0, result.stderr
    payload = json.loads(report_path.read_text(encoding="utf-8"))
    seeded_evidence_paths = set(payload["seeded_evidence_paths"])
    assert (
        "test_env/release_evidence/security/vulnerability_exception_review_report.json"
        in seeded_evidence_paths
    )
    assert (
        "test_env/release_evidence/operations/external_mainline_execution_plan.json"
        in seeded_evidence_paths
    )
