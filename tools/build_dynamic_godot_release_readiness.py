"""Build a release/readiness summary from dynamic Godot generation evidence."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

SUMMARY_VERSION = "dynamic_godot_release_readiness_summary.v1"
ARTIFACT_TYPE = "dynamic_godot_release_readiness_summary"
GATE_CONTRACT_VERSION = "delivery_acceptance_gate.v1"
LEVEL_RANKS = {
    "incomplete": 0,
    "static_only": 1,
    "godot_load_verified": 2,
    "godot_verified": 3,
}
LEVEL_LABELS = {
    "incomplete": "No complete dynamic Godot generation evidence was found.",
    "static_only": "Static manifest, report, and gate evidence are complete.",
    "godot_load_verified": "A Web/session Godot load gate is complete.",
    "godot_verified": "A full Godot smoke-motion gate is complete.",
}


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def _read_json_object(path: Path) -> tuple[dict[str, Any], str | None]:
    if not path.exists():
        return {}, "file does not exist"
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        return {}, f"{exc.msg} at line {exc.lineno} column {exc.colno}"
    if not isinstance(payload, dict):
        return {}, "top-level JSON value is not an object"
    return payload, None


def _looks_like_gate(payload: dict[str, Any]) -> bool:
    return payload.get("contract_version") == GATE_CONTRACT_VERSION


def _extract_gate(payload: dict[str, Any]) -> dict[str, Any] | None:
    if _looks_like_gate(payload):
        return payload
    gate = payload.get("delivery_acceptance_gate")
    if isinstance(gate, dict) and _looks_like_gate(gate):
        return gate
    return None


def _gate_evidence(path: Path, gate: dict[str, Any]) -> dict[str, Any]:
    level = str(gate.get("level") or "incomplete")
    complete = gate.get("complete") is True
    passed = gate.get("passed") is True
    proven = complete and passed and level in LEVEL_RANKS and level != "incomplete"
    return {
        "path": str(path),
        "kind": "delivery_acceptance_gate",
        "source": gate.get("source"),
        "verification_scope": gate.get("verification_scope"),
        "acceptance_profile": gate.get("acceptance_profile"),
        "level": level,
        "proven": proven,
        "passed": gate.get("passed"),
        "complete": gate.get("complete"),
        "exit_code": gate.get("exit_code"),
        "reason_codes": gate.get("reason_codes", []),
    }


def _static_closeout_evidence(path: Path, payload: dict[str, Any]) -> dict[str, Any] | None:
    if payload.get("acceptance_level") != "static_only":
        return None
    status = payload.get("status")
    proven = status == "success"
    return {
        "path": str(path),
        "kind": "static_godot_node_tree_evidence_closeout",
        "source": "static_godot_node_tree_evidence",
        "verification_scope": "static_manifest",
        "acceptance_profile": "static_godot_node_tree_evidence",
        "level": "static_only",
        "proven": proven,
        "passed": proven,
        "complete": proven,
        "status": status,
        "residual_risks": payload.get("residual_risks", []),
    }


def _evidence_from_payload(path: Path, payload: dict[str, Any]) -> list[dict[str, Any]]:
    evidence: list[dict[str, Any]] = []
    static_evidence = _static_closeout_evidence(path, payload)
    if static_evidence is not None:
        evidence.append(static_evidence)
    gate = _extract_gate(payload)
    if gate is not None:
        evidence.append(_gate_evidence(path, gate))
    return evidence


def _best_level(evidence: list[dict[str, Any]]) -> str:
    best = "incomplete"
    for item in evidence:
        level = str(item.get("level") or "incomplete")
        if item.get("proven") is True and LEVEL_RANKS.get(level, 0) > LEVEL_RANKS[best]:
            best = level
    return best


def _residual_risks(level: str, evidence: list[dict[str, Any]]) -> list[str]:
    risks: list[str] = []
    if level == "incomplete":
        risks.append("No complete static, Godot-load, or full motion evidence was found.")
    if LEVEL_RANKS[level] < LEVEL_RANKS["godot_load_verified"]:
        risks.append("Godot load through Web/session delivery is not proven by the supplied evidence.")
    if LEVEL_RANKS[level] < LEVEL_RANKS["godot_verified"]:
        risks.append("Full live Godot smoke-motion verification is not proven by the supplied evidence.")
    for item in evidence:
        for risk in item.get("residual_risks", []):
            if isinstance(risk, str) and risk not in risks:
                risks.append(risk)
    return risks


def build_readiness_summary(inputs: list[Path]) -> dict[str, Any]:
    evidence: list[dict[str, Any]] = []
    input_errors: list[dict[str, str]] = []
    for path in inputs:
        payload, error = _read_json_object(path)
        if error is not None:
            input_errors.append({"path": str(path), "error": error})
            continue
        extracted = _evidence_from_payload(path, payload)
        if not extracted:
            input_errors.append({"path": str(path), "error": "no recognized dynamic Godot evidence"})
            continue
        evidence.extend(extracted)

    proven_level = _best_level(evidence)
    levels_found = sorted(
        {
            str(item.get("level"))
            for item in evidence
            if item.get("proven") is True and str(item.get("level")) in LEVEL_RANKS
        },
        key=lambda level: LEVEL_RANKS[level],
    )
    status = "ready" if proven_level != "incomplete" and not input_errors else "blocked"
    return {
        "summary_version": SUMMARY_VERSION,
        "artifact_type": ARTIFACT_TYPE,
        "status": status,
        "proven_level": proven_level,
        "proven_level_rank": LEVEL_RANKS[proven_level],
        "proven_level_label": LEVEL_LABELS[proven_level],
        "levels_found": levels_found,
        "input_count": len(inputs),
        "evidence_count": len(evidence),
        "evidence": evidence,
        "input_errors": input_errors,
        "residual_risks": _residual_risks(proven_level, evidence),
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Summarize dynamic Godot generation readiness from existing evidence JSON."
    )
    parser.add_argument(
        "inputs",
        nargs="+",
        type=Path,
        help="Evidence JSON files: static closeout, report, gate, or Web delivery record.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional path for the readiness summary JSON.",
    )
    args = parser.parse_args()

    summary = build_readiness_summary([path.resolve() for path in args.inputs])
    if args.output:
        _write_json(args.output.resolve(), summary)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if summary["status"] == "ready" else 1


if __name__ == "__main__":
    raise SystemExit(main())
