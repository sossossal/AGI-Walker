from __future__ import annotations

import argparse
import hashlib
import json
from datetime import UTC, datetime
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
TEST_ENV = ROOT / "test_env"
ACTUATOR_CONFIG = ROOT / "config" / "actuators.json"
DEFAULT_OUTPUT = TEST_ENV / "retention_manifest.json"


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def detect_schema(path: Path) -> str:
    if path.suffix == ".json":
        try:
            payload = load_json(path)
        except json.JSONDecodeError:
            return "unknown-json"
        if isinstance(payload, dict):
            return str(payload.get("schema_version", path.stem))
        if isinstance(payload, list):
            return f"{path.stem}.array"
        return f"{path.stem}.{type(payload).__name__}"
    if path.suffix == ".jsonl":
        return f"{path.stem}.jsonl"
    return path.suffix.lstrip(".") or "unknown"


def build_manifest(output: Path) -> dict[str, Any]:
    retention = load_json(ACTUATOR_CONFIG)["retention"]
    artifacts = []
    for path in sorted(TEST_ENV.glob("*")):
        if path.is_file() and path != output:
            artifacts.append(
                {
                    "path": str(path.relative_to(ROOT)).replace("\\", "/"),
                    "bytes": path.stat().st_size,
                    "sha256": sha256(path),
                    "schema": detect_schema(path),
                }
            )
    return {
        "schema_version": "biped-retention-manifest.v1",
        "created_utc": datetime.now(UTC).isoformat(),
        "scope": "biped_robot/test_env",
        "retention_days": retention["retention_days"],
        "artifact_count": len(artifacts),
        "artifacts": artifacts,
    }


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False)
        handle.write("\n")


def main() -> int:
    parser = argparse.ArgumentParser(description="Build a retention manifest for biped_robot generated evidence.")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    manifest = build_manifest(args.output)
    write_json(args.output, manifest)
    print(json.dumps({"status": "passed", "output": str(args.output), "artifact_count": manifest["artifact_count"]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
