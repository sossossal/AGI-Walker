import logging
from typing import Any, Optional, Dict, List, Tuple
logger = logging.getLogger(__name__)
import requests
import json
import os
import sys

# Ensure we can import from the project root if needed, but here we just use requests
BASE_URL = "http://localhost:8000"


def test_fetch_market() -> None:
    logger.info("Testing GET /api/parts/market...")
    try:
        res = requests.get(f"{BASE_URL}/api/parts/market")
        if res.status_code != 200:
            logger.info(f"❌ Failed: Status Code {res.status_code}")
            return False

        data = res.json()
        if data.get("status") != "success":
            logger.info(f"❌ Failed: API status is {data.get('status')}")
            return False

        parts = data.get("parts", [])
        logger.info(f"✅ Market fetched successfully. Found {len(parts)} parts.")
        for p in parts:
            logger.info(f"   - {p['name']} (${p['price']})")

        return True
    except Exception as e:
        logger.info(f"❌ Exception: {e}")
        return False


def test_import_part() -> None:
    logger.info("\nTesting POST /api/parts/import...")
    payload = {"part_id": "MT-C01", "category": "motors"}
    try:
        res = requests.post(f"{BASE_URL}/api/parts/import", json=payload)
        if res.status_code != 200:
            logger.info(f"❌ Failed: Status Code {res.status_code}")
            logger.info(res.text)
            return False

        data = res.json()
        if data.get("status") in ["success", "skipped"]:
            logger.info(f"✅ Import successful: {data.get('message')}")
            return True
        else:
            logger.info(f"❌ Failed: API status is {data.get('status')}")
            return False
    except Exception as e:
        logger.info(f"❌ Exception: {e}")
        return False


def verify_local_db():
    logger.info("\nVerifying local database update...")
    db_path = os.path.join("parts_library", "complete_parts_database.json")
    try:
        with open(db_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        motors = data.get("parts", {}).get("motors", [])
        found = False
        for m in motors:
            if m["id"] == "MT-C01":
                found = True
                logger.info(f"✅ Found MT-C01 in local database: {m['name']}")
                break

        if not found:
            logger.info("❌ MT-C01 not found in local database!")
            return False
        return True

    except Exception as e:
        logger.info(f"❌ Exception reading DB: {e}")
        return False


if __name__ == "__main__":
    logger.info("🚀 Starting Parts Store Verification")

    if test_fetch_market() and test_import_part() and verify_local_db():
        logger.info("\n✨ All tests passed!")
        sys.exit(0)
    else:
        logger.error("\n💥 Some tests failed.")
        sys.exit(1)
