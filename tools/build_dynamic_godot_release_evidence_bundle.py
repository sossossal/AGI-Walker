"""Build a self-validating dynamic Godot release evidence bundle."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

TOOLS_DIR = Path(__file__).resolve().parent
ROOT = TOOLS_DIR.parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from validate_dynamic_godot_release_evidence_bundle import (  # noqa: E402
    BUNDLE_ARTIFACT_TYPE,
    BUNDLE_VERSION,
    LEVEL_RANKS,
    REQUIRED_DOC_ROLES,
    sha256_file,
    validate_bundle_index,
)

DEFAULT_DOC_PATH = ROOT / "docs" / "guides" / "DYNAMIC_GODOT_ROBOT_GENERATION.md"


def _mtime_iso(path: Path) -> str:
    return datetime.fromtimestamp(path.stat().st_mtime, UTC).isoformat()


def _read_json_object(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    return payload if isinstance(payload, dict) else {}


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def _copy_entry(
    *,
    source: Path,
    output_root: Path,
    target_dir: str,
    key: str,
    role: str,
    required: bool,
) -> dict[str, Any]:
    source = source.resolve()
    target = output_root / target_dir / f"{key}{source.suffix or '.json'}"
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)
    return {
        "key": key,
        "role": role,
        "required": required,
        "source_path": str(source),
        "bundle_path": str(target.relative_to(output_root)),
        "size_bytes": target.stat().st_size,
        "sha256": sha256_file(target),
        "source_modified_at": _mtime_iso(source),
        "bundle_modified_at": _mtime_iso(target),
    }


def _doc_entries(output_root: Path, docs: list[tuple[str, Path]]) -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []
    for role, path in docs:
        entries.append(
            _copy_entry(
                source=path,
                output_root=output_root,
                target_dir="docs",
                key=role,
                role=role,
                required=role in REQUIRED_DOC_ROLES,
            )
        )
    return entries


def _artifact_entries(args: argparse.Namespace, output_root: Path) -> list[dict[str, Any]]:
    specs = [
        ("static_closeout", "static_closeout", args.static_closeout, True),
        ("delivery_gate", "delivery_gate", args.delivery_gate, True),
        ("readiness_summary", "readiness_summary", args.readiness_summary, True),
        ("live_smoke", "live_smoke", args.live_smoke, False),
        ("web_delivery_record", "web_delivery_record", args.web_delivery_record, False),
    ]
    return [
        _copy_entry(
            source=Path(path),
            output_root=output_root,
            target_dir="artifacts",
            key=key,
            role=role,
            required=required,
        )
        for key, role, path, required in specs
        if path is not None
    ]


def _parse_doc(value: str) -> tuple[str, Path]:
    if "=" not in value:
        path = Path(value)
        return path.stem, path
    role, path = value.split("=", 1)
    return role.strip(), Path(path)


def _default_docs() -> list[tuple[str, Path]]:
    return [(role, DEFAULT_DOC_PATH) for role in REQUIRED_DOC_ROLES]


def build_bundle(args: argparse.Namespace) -> dict[str, Any]:
    output_root = Path(args.output_root).resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    readiness = _read_json_object(Path(args.readiness_summary).resolve())
    level = str(readiness.get("proven_level") or "incomplete")
    docs = [_parse_doc(value) for value in args.doc] if args.doc else _default_docs()
    index = {
        "bundle_version": BUNDLE_VERSION,
        "artifact_type": BUNDLE_ARTIFACT_TYPE,
        "generated_at": datetime.now(UTC).isoformat(),
        "bundle_root": str(output_root),
        "evidence_level": level,
        "evidence_level_rank": LEVEL_RANKS.get(level, 0),
        "readiness_status": readiness.get("status"),
        "residual_risks": readiness.get("residual_risks", []),
        "artifacts": _artifact_entries(args, output_root),
        "documentation": _doc_entries(output_root, docs),
    }
    index_path = output_root / "bundle_index.json"
    validation_path = output_root / "bundle_validation.json"
    _write_json(index_path, index)
    validation = validate_bundle_index(index_path)
    _write_json(validation_path, validation)
    index["validation_report"] = str(validation_path.relative_to(output_root))
    index["validation_status"] = validation["status"]
    _write_json(index_path, index)
    return index


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Build a dynamic Godot release evidence bundle from existing artifacts."
    )
    parser.add_argument("--static-closeout", required=True, type=Path)
    parser.add_argument("--delivery-gate", required=True, type=Path)
    parser.add_argument("--readiness-summary", required=True, type=Path)
    parser.add_argument("--live-smoke", type=Path)
    parser.add_argument("--web-delivery-record", type=Path)
    parser.add_argument(
        "--doc",
        action="append",
        default=[],
        help="Documentation entry as role=path. Defaults cover static/live/Web/readiness.",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=ROOT / "test_env" / "dynamic_godot_release_evidence_bundle",
    )
    args = parser.parse_args()

    index = build_bundle(args)
    print(json.dumps(index, indent=2, ensure_ascii=False))
    return 0 if index["validation_status"] == "ready" else 1


if __name__ == "__main__":
    raise SystemExit(main())
