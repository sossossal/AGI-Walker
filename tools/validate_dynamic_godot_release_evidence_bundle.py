"""Validate a dynamic Godot release evidence bundle index."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

BUNDLE_VERSION = "dynamic_godot_release_evidence_bundle.v1"
BUNDLE_ARTIFACT_TYPE = "dynamic_godot_release_evidence_bundle"
VALIDATION_VERSION = "dynamic_godot_release_evidence_bundle_validation.v1"
READINESS_VERSION = "dynamic_godot_release_readiness_summary.v1"
READINESS_ARTIFACT_TYPE = "dynamic_godot_release_readiness_summary"
GATE_CONTRACT_VERSION = "delivery_acceptance_gate.v1"
LEVEL_RANKS = {
    "incomplete": 0,
    "static_only": 1,
    "godot_load_verified": 2,
    "godot_verified": 3,
}
REQUIRED_ARTIFACT_KEYS = ("static_closeout", "delivery_gate", "readiness_summary")
REQUIRED_DOC_ROLES = (
    "static_workflow",
    "live_workflow",
    "web_workflow",
    "readiness_workflow",
)


def read_json_object(path: Path) -> tuple[dict[str, Any], str | None]:
    if not path.exists():
        return {}, "file does not exist"
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        return {}, f"{exc.msg} at line {exc.lineno} column {exc.colno}"
    if not isinstance(payload, dict):
        return {}, "top-level JSON value is not an object"
    return payload, None


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _entry_path(bundle_root: Path, entry: dict[str, Any]) -> Path:
    bundle_path = Path(str(entry.get("bundle_path") or ""))
    return bundle_path if bundle_path.is_absolute() else bundle_root / bundle_path


def _validate_entry(
    entry: Any,
    *,
    bundle_root: Path,
    collection: str,
    errors: list[str],
) -> dict[str, Any] | None:
    if not isinstance(entry, dict):
        errors.append(f"{collection} entries must be objects")
        return None
    key = entry.get("key") or entry.get("role")
    if not isinstance(key, str) or not key.strip():
        errors.append(f"{collection} entry key/role must be a non-empty string")
    path = _entry_path(bundle_root, entry)
    if not path.exists():
        errors.append(f"{collection}.{key} bundle_path does not exist: {path}")
        return entry
    size_bytes = path.stat().st_size
    if entry.get("size_bytes") != size_bytes:
        errors.append(f"{collection}.{key} size_bytes must equal {size_bytes}")
    checksum = sha256_file(path)
    if entry.get("sha256") != checksum:
        errors.append(f"{collection}.{key} sha256 must equal {checksum}")
    return entry


def _extract_gate(payload: dict[str, Any]) -> dict[str, Any] | None:
    if payload.get("contract_version") == GATE_CONTRACT_VERSION:
        return payload
    gate = payload.get("delivery_acceptance_gate")
    if isinstance(gate, dict) and gate.get("contract_version") == GATE_CONTRACT_VERSION:
        return gate
    return None


def _validate_readiness(
    readiness: dict[str, Any],
    index: dict[str, Any],
    errors: list[str],
) -> None:
    if readiness.get("summary_version") != READINESS_VERSION:
        errors.append(f"readiness_summary.summary_version must be {READINESS_VERSION!r}")
    if readiness.get("artifact_type") != READINESS_ARTIFACT_TYPE:
        errors.append(f"readiness_summary.artifact_type must be {READINESS_ARTIFACT_TYPE!r}")
    level = readiness.get("proven_level")
    if level not in LEVEL_RANKS:
        errors.append("readiness_summary.proven_level must be a known evidence level")
        return
    if index.get("evidence_level") != level:
        errors.append("bundle evidence_level must match readiness_summary.proven_level")
    if index.get("evidence_level_rank") != LEVEL_RANKS[level]:
        errors.append("bundle evidence_level_rank must match readiness summary rank")
    if readiness.get("status") != "ready":
        errors.append("readiness_summary.status must be 'ready'")


def validate_bundle_index(index_path: Path) -> dict[str, Any]:
    index_path = index_path.resolve()
    bundle_root = index_path.parent
    index, read_error = read_json_object(index_path)
    errors: list[str] = []
    if read_error is not None:
        errors.append(f"bundle index: {read_error}")
        index = {}
    if index.get("bundle_version") != BUNDLE_VERSION:
        errors.append(f"bundle_version must be {BUNDLE_VERSION!r}")
    if index.get("artifact_type") != BUNDLE_ARTIFACT_TYPE:
        errors.append(f"artifact_type must be {BUNDLE_ARTIFACT_TYPE!r}")

    artifacts = index.get("artifacts")
    if not isinstance(artifacts, list):
        errors.append("artifacts must be a list")
        artifacts = []
    docs = index.get("documentation")
    if not isinstance(docs, list):
        errors.append("documentation must be a list")
        docs = []

    artifact_entries = [
        entry
        for entry in (
            _validate_entry(item, bundle_root=bundle_root, collection="artifacts", errors=errors)
            for item in artifacts
        )
        if isinstance(entry, dict)
    ]
    doc_entries = [
        entry
        for entry in (
            _validate_entry(item, bundle_root=bundle_root, collection="documentation", errors=errors)
            for item in docs
        )
        if isinstance(entry, dict)
    ]
    artifact_by_key = {
        str(entry.get("key")): entry
        for entry in artifact_entries
        if isinstance(entry.get("key"), str)
    }
    missing_keys = [key for key in REQUIRED_ARTIFACT_KEYS if key not in artifact_by_key]
    for key in missing_keys:
        errors.append(f"required artifact {key!r} is missing")
    doc_roles = {
        str(entry.get("role"))
        for entry in doc_entries
        if isinstance(entry.get("role"), str)
    }
    for role in REQUIRED_DOC_ROLES:
        if role not in doc_roles:
            errors.append(f"required documentation role {role!r} is missing")

    readiness_payload: dict[str, Any] = {}
    for key in REQUIRED_ARTIFACT_KEYS:
        entry = artifact_by_key.get(key)
        if entry is None:
            continue
        payload, error = read_json_object(_entry_path(bundle_root, entry))
        if error is not None:
            errors.append(f"{key}: {error}")
            continue
        if key == "static_closeout":
            if payload.get("status") != "success":
                errors.append("static_closeout.status must be 'success'")
            if payload.get("acceptance_level") != "static_only":
                errors.append("static_closeout.acceptance_level must be 'static_only'")
        elif key == "delivery_gate" and _extract_gate(payload) is None:
            errors.append("delivery_gate must contain delivery_acceptance_gate.v1")
        elif key == "readiness_summary":
            readiness_payload = payload
    if readiness_payload:
        _validate_readiness(readiness_payload, index, errors)

    return {
        "validation_version": VALIDATION_VERSION,
        "artifact_type": "dynamic_godot_release_evidence_bundle_validation",
        "status": "ready" if not errors else "invalid",
        "bundle_index": str(index_path),
        "bundle_root": str(bundle_root),
        "artifact_count": len(artifact_entries),
        "documentation_count": len(doc_entries),
        "required_artifacts": list(REQUIRED_ARTIFACT_KEYS),
        "required_documentation_roles": list(REQUIRED_DOC_ROLES),
        "evidence_level": index.get("evidence_level"),
        "errors": errors,
    }


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate a dynamic Godot release evidence bundle index."
    )
    parser.add_argument("bundle_index", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    report = validate_bundle_index(args.bundle_index)
    if args.output:
        _write_json(args.output.resolve(), report)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report["status"] == "ready" else 1


if __name__ == "__main__":
    raise SystemExit(main())
