from __future__ import annotations

import json
import logging
import os
import shutil
import sys
import uuid
from contextlib import contextmanager
from pathlib import Path

from fastapi.testclient import TestClient


logger = logging.getLogger(__name__)
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from web_panel.server import app


def _configure_runtime() -> None:
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    if hasattr(sys.stderr, "reconfigure"):
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")


@contextmanager
def _pushd(target: Path):
    previous = Path.cwd()
    os.chdir(target)
    try:
        yield
    finally:
        os.chdir(previous)


def _prepare_temp_parts_workspace(temp_root: Path) -> tuple[str, str, Path]:
    source_root = PROJECT_ROOT / "parts_library"
    runtime_root = temp_root / "parts_library"
    runtime_root.mkdir(parents=True, exist_ok=True)

    db_path = runtime_root / "complete_parts_database.json"
    shutil.copy2(source_root / "complete_parts_database.json", db_path)

    market_root = runtime_root / "cloud_repo_mock"
    market_parts_dir = market_root / "parts" / "motors"
    market_parts_dir.mkdir(parents=True, exist_ok=True)

    manifest = {
        "name": "agi-walker-parts",
        "description": "Temporary mock parts index for verification",
        "maintainers": ["codex"],
        "license": "MIT",
        "parts_index": "parts/",
    }
    (market_root / "manifest.json").write_text(
        json.dumps(manifest, indent=4, ensure_ascii=False),
        encoding="utf-8",
    )

    target_part = {
        "id": "MT-C01",
        "name": "Vesta V1 High-Torque Motor",
        "type": "brushless_custom",
        "specs": {
            "power": "800W",
            "voltage": "48V",
            "max_torque": "8.5 Nm",
            "max_speed": "2500 RPM",
            "weight": "0.9 kg",
        },
        "price": 150.0,
        "supplier": "Vesta Robotics",
        "applications": ["Heavy Duty Joint"],
        "stock": "available",
    }
    category = "motors"
    (market_parts_dir / "mt_c01.json").write_text(
        json.dumps(target_part, indent=4, ensure_ascii=False),
        encoding="utf-8",
    )

    local_db = json.loads(db_path.read_text(encoding="utf-8"))
    category_parts = local_db.setdefault("parts", {}).setdefault(category, [])
    local_db["parts"][category] = [
        part for part in category_parts if part.get("id") != target_part["id"]
    ]
    db_path.write_text(
        json.dumps(local_db, indent=4, ensure_ascii=False),
        encoding="utf-8",
    )

    return target_part["id"], category, db_path


def verify_parts_store() -> bool:
    logger.info("=== Parts Store Verification ===")

    test_env_root = PROJECT_ROOT / "test_env"
    test_env_root.mkdir(parents=True, exist_ok=True)
    runtime_root = test_env_root / f"verify_parts_store_runtime_{uuid.uuid4().hex[:8]}"

    try:
        runtime_root.mkdir(parents=True, exist_ok=False)
        part_id, category, db_path = _prepare_temp_parts_workspace(runtime_root)

        with _pushd(runtime_root):
            with TestClient(app) as client:
                logger.info("[1/4] Fetching parts market...")
                market_response = client.get("/api/parts/market")
                if market_response.status_code != 200:
                    raise AssertionError(
                        f"GET /api/parts/market failed: {market_response.status_code}"
                    )

                market_payload = market_response.json()
                if market_payload.get("status") != "success":
                    raise AssertionError(f"Invalid market payload: {market_payload}")

                parts = market_payload.get("parts", [])
                if not any(part.get("id") == part_id for part in parts):
                    raise AssertionError(f"Target part {part_id} missing from market")

                logger.info("[2/4] Importing part through API...")
                import_response = client.post(
                    "/api/parts/import",
                    json={"part_id": part_id, "category": category},
                )
                if import_response.status_code != 200:
                    raise AssertionError(
                        f"POST /api/parts/import failed: {import_response.text}"
                    )

                import_payload = import_response.json()
                if import_payload.get("status") != "success":
                    raise AssertionError(f"Unexpected import payload: {import_payload}")

                logger.info("[3/4] Verifying local database update...")
                local_db = json.loads(db_path.read_text(encoding="utf-8"))
                imported = [
                    part
                    for part in local_db.get("parts", {}).get(category, [])
                    if part.get("id") == part_id
                ]
                if len(imported) != 1:
                    raise AssertionError(
                        f"Expected exactly one imported part {part_id}, found {len(imported)}"
                    )

                logger.info("[4/4] Verifying duplicate import handling...")
                duplicate_response = client.post(
                    "/api/parts/import",
                    json={"part_id": part_id, "category": category},
                )
                if duplicate_response.status_code != 200:
                    raise AssertionError(
                        f"Duplicate import failed: {duplicate_response.text}"
                    )
                duplicate_payload = duplicate_response.json()
                if duplicate_payload.get("status") != "skipped":
                    raise AssertionError(
                        f"Expected duplicate import to be skipped: {duplicate_payload}"
                    )

        logger.info("PASS: Parts store verification completed")
        return True
    except Exception as exc:
        logger.exception("FAIL: %s", exc)
        return False
    finally:
        shutil.rmtree(runtime_root, ignore_errors=True)


if __name__ == "__main__":
    _configure_runtime()
    raise SystemExit(0 if verify_parts_store() else 1)
