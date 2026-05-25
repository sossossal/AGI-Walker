from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "test_env" / "local_acceptance_report.json"


def run_command(name: str, command: list[str], cwd: Path) -> dict[str, Any]:
    completed = subprocess.run(command, cwd=cwd, capture_output=True, text=True, check=False)
    return {
        "name": name,
        "command": command,
        "exit_code": completed.returncode,
        "stdout": completed.stdout.strip(),
        "stderr": completed.stderr.strip(),
        "status": "passed" if completed.returncode == 0 else "failed",
    }


def maybe_godot_command() -> list[str] | None:
    configured = os.environ.get("GODOT_EXE")
    candidates = [configured] if configured else []
    candidates.extend([r"D:\迅雷下载\Godot\Godot.exe"])
    for candidate in candidates:
        if candidate and Path(candidate).is_file():
            return [candidate, "--headless", "--path", str(ROOT / "godot"), "--quit-after", "240"]
    return None


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False)
        handle.write("\n")


def main() -> int:
    parser = argparse.ArgumentParser(description="Run biped_robot folder-scoped acceptance checks.")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    commands = [
        (
            "python_syntax",
            [
                sys.executable,
                "-m",
                "py_compile",
                str(ROOT / "tools" / "simulate_biped.py"),
                str(ROOT / "tools" / "validate_biped_workspace.py"),
                str(ROOT / "tools" / "check_contact_stability.py"),
                str(ROOT / "tools" / "build_hardware_gap_report.py"),
                str(ROOT / "tools" / "simulate_actuator_physics.py"),
                str(ROOT / "tools" / "build_component_parameter_log.py"),
                str(ROOT / "tools" / "validate_godot_io.py"),
                str(ROOT / "tools" / "build_retention_manifest.py"),
                str(ROOT / "tools" / "run_local_acceptance.py"),
            ],
        ),
        ("workspace_contract", [sys.executable, str(ROOT / "tools" / "validate_biped_workspace.py")]),
        (
            "deterministic_simulation",
            [
                sys.executable,
                str(ROOT / "tools" / "simulate_biped.py"),
                "--output",
                str(ROOT / "test_env" / "biped_sim_report.json"),
                "--trace-output",
                str(ROOT / "test_env" / "biped_sim_trace.json"),
            ],
        ),
        ("contact_stability", [sys.executable, str(ROOT / "tools" / "check_contact_stability.py")]),
        ("actuator_physics", [sys.executable, str(ROOT / "tools" / "simulate_actuator_physics.py")]),
        ("component_parameter_log", [sys.executable, str(ROOT / "tools" / "build_component_parameter_log.py")]),
        ("hardware_gap_report", [sys.executable, str(ROOT / "tools" / "build_hardware_gap_report.py")]),
    ]
    godot = maybe_godot_command()
    if godot:
        commands.append(("godot_headless_load", godot))
        commands.append(("godot_io_validation", [sys.executable, str(ROOT / "tools" / "validate_godot_io.py")]))
    commands.append(("retention_manifest", [sys.executable, str(ROOT / "tools" / "build_retention_manifest.py")]))

    results = [run_command(name, command, ROOT.parent) for name, command in commands]
    if not godot:
        results.append(
            {
                "name": "godot_headless_load",
                "command": [],
                "exit_code": None,
                "stdout": "",
                "stderr": "Godot executable not found. Set GODOT_EXE to run this check.",
                "status": "not_run",
            }
        )

    failed = [result for result in results if result["status"] == "failed"]
    report = {
        "status": "passed" if not failed else "failed",
        "scope": "biped_robot only",
        "outside_project_files_touched": False,
        "results": results,
    }
    write_json(args.output, report)
    print(json.dumps({"status": report["status"], "output": str(args.output)}, ensure_ascii=False))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
