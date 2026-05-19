"""Build static Godot node-tree evidence artifacts in one command."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
REPORT_TOOL = ROOT / "tools" / "build_dynamic_robot_generation_report.py"
GATE_VALIDATOR_TOOL = ROOT / "tools" / "validate_delivery_acceptance_gate.py"
DEFAULT_INPUTS = (
    ROOT / "tests" / "fixtures" / "robot_dynamic_fixed_pair.json",
    ROOT / "tests" / "fixtures" / "robot_dynamic_biped.json",
    ROOT / "tests" / "fixtures" / "robot_dynamic_quadruped.json",
)
MISMATCH_KINDS = (
    "missing",
    "unexpected",
    "value_mismatch",
    "duplicate",
    "root_mismatch",
)


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def _read_json_object(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    payload = json.loads(path.read_text(encoding="utf-8"))
    return payload if isinstance(payload, dict) else {}


def _run(args: list[str]) -> dict[str, Any]:
    result = subprocess.run(
        args,
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    return {
        "args": args,
        "returncode": result.returncode,
        "stdout": result.stdout[-4000:],
        "stderr": result.stderr[-4000:],
    }


def _validator_args(
    *,
    manifest_dir: Path,
    input_count: int,
    validation_summary_path: Path,
) -> list[str]:
    args = [
        sys.executable,
        str(GATE_VALIDATOR_TOOL),
        str(manifest_dir),
        "--fail-on-invalid-node-tree-manifest-sidecar",
        "--fail-on-node-tree-manifest-sidecar-validation-error",
        "--fail-on-node-tree-manifest-sidecar-path-incomplete",
        "--fail-on-node-tree-manifest-sidecar-path-map-mismatch",
        "--expect-node-tree-manifest-sidecar-count",
        str(input_count),
        "--expect-node-tree-manifest-sidecar-complete-count",
        str(input_count),
        "--expect-node-tree-manifest-sidecar-incomplete-count",
        "0",
        "--expect-node-tree-manifest-sidecar-valid-count",
        str(input_count),
        "--expect-node-tree-manifest-sidecar-invalid-count",
        "0",
        "--expect-node-tree-manifest-sidecar-validation-error-count",
        "0",
        "--expect-node-tree-manifest-sidecar-path-incomplete-count",
        "0",
        "--expect-node-tree-manifest-sidecar-path-map-mismatch-count",
        "0",
    ]
    for kind in MISMATCH_KINDS:
        args.extend(
            [
                "--expect-node-tree-manifest-sidecar-path-map-mismatch-kind",
                f"{kind}=0",
            ]
        )
    args.extend(["--summary-output", str(validation_summary_path)])
    return args


def _closeout_payload(
    *,
    inputs: list[Path],
    output_root: Path,
    manifest_dir: Path,
    report_path: Path,
    gate_path: Path,
    validation_summary_path: Path,
    report_command: dict[str, Any],
    validation_command: dict[str, Any],
) -> dict[str, Any]:
    report = _read_json_object(report_path)
    gate = _read_json_object(gate_path)
    validation_summary = _read_json_object(validation_summary_path)
    errors = []
    if report_command["returncode"] != 0:
        errors.append("static report generation failed")
    if validation_command.get("returncode") not in (0, None):
        errors.append("static sidecar validation failed")
    errors.extend(
        error
        for error in validation_summary.get("errors", [])
        if isinstance(error, str)
    )
    return {
        "status": "error" if errors else "success",
        "acceptance_level": "static_only",
        "live_godot_smoke_run": False,
        "residual_risks": [
            "Live Godot load, runtime node-tree readback, and motion simulation were not run."
        ],
        "inputs": [str(path) for path in inputs],
        "input_count": len(inputs),
        "output_root": str(output_root),
        "artifacts": {
            "report": str(report_path),
            "gate": str(gate_path),
            "manifest_dir": str(manifest_dir),
            "validation_summary": str(validation_summary_path),
        },
        "report_status": report.get("status"),
        "gate_passed": gate.get("passed"),
        "validation_status": validation_summary.get("status"),
        "node_tree_manifest_sidecar_count": validation_summary.get(
            "node_tree_manifest_sidecar_count", 0
        ),
        "node_tree_manifest_sidecar_valid_count": validation_summary.get(
            "node_tree_manifest_sidecar_valid_count", 0
        ),
        "node_tree_manifest_sidecar_invalid_count": validation_summary.get(
            "node_tree_manifest_sidecar_invalid_count", 0
        ),
        "node_tree_manifest_sidecar_validation_error_count": validation_summary.get(
            "node_tree_manifest_sidecar_validation_error_count", 0
        ),
        "node_tree_manifest_sidecar_path_map_mismatch_count": validation_summary.get(
            "node_tree_manifest_sidecar_path_map_mismatch_count", 0
        ),
        "node_tree_manifest_sidecar_path_map_mismatch_kind_counts": (
            validation_summary.get(
                "node_tree_manifest_sidecar_path_map_mismatch_kind_counts", {}
            )
        ),
        "errors": errors,
        "commands": {
            "report": report_command,
            "validation": validation_command,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Build static Godot node-tree report, sidecars, validation summary, and closeout evidence."
    )
    parser.add_argument(
        "inputs",
        nargs="*",
        type=Path,
        help="Robot JSON configs. Defaults to the fixed-pair and biped fixtures.",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=ROOT / "test_env" / "static_godot_node_tree_manifest_ci",
        help="Directory for report, gate, sidecars, validation summary, and closeout.",
    )
    parser.add_argument(
        "--manifest-dir",
        type=Path,
        default=None,
        help="Optional sidecar output directory. Defaults under --output-root.",
    )
    parser.add_argument(
        "--closeout-output",
        type=Path,
        default=None,
        help="Optional closeout JSON path. Defaults under --output-root.",
    )
    args = parser.parse_args()

    inputs = [path.resolve() for path in (args.inputs or DEFAULT_INPUTS)]
    output_root = args.output_root.resolve()
    manifest_dir = (
        args.manifest_dir.resolve()
        if args.manifest_dir
        else output_root / "node_tree_manifests"
    )
    report_path = output_root / "report.json"
    gate_path = output_root / "gate.json"
    validation_summary_path = output_root / "validation_summary.json"
    closeout_path = (
        args.closeout_output.resolve()
        if args.closeout_output
        else output_root / "static_godot_node_tree_evidence_closeout.json"
    )

    report_command = _run(
        [
            sys.executable,
            str(REPORT_TOOL),
            *[str(path) for path in inputs],
            "--require-static-node-tree-manifest-output",
            "--static-node-tree-manifest-dir",
            str(manifest_dir),
            "--output",
            str(report_path),
            "--gate-output",
            str(gate_path),
        ]
    )
    validation_command: dict[str, Any] = {"returncode": None, "args": []}
    if report_command["returncode"] == 0:
        validation_command = _run(
            _validator_args(
                manifest_dir=manifest_dir,
                input_count=len(inputs),
                validation_summary_path=validation_summary_path,
            )
        )

    closeout = _closeout_payload(
        inputs=inputs,
        output_root=output_root,
        manifest_dir=manifest_dir,
        report_path=report_path,
        gate_path=gate_path,
        validation_summary_path=validation_summary_path,
        report_command=report_command,
        validation_command=validation_command,
    )
    closeout["artifacts"]["closeout"] = str(closeout_path)
    _write_json(closeout_path, closeout)
    print(json.dumps(closeout, indent=2, ensure_ascii=False))
    return 0 if closeout["status"] == "success" else 1


if __name__ == "__main__":
    raise SystemExit(main())
