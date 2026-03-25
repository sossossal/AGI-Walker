import logging
from typing import Any, Optional, Dict, List, Tuple
logger = logging.getLogger(__name__)
import pytest
pytest.importorskip("requests")

import requests
import json
import time


def test_generation() -> None:
    url = "http://localhost:8000/api/generate_robot"
    payload = {
        "name": "web_test_bot",
        "type": "quadruped",
        "scenario": "custom",
        "height": 0.35,
        "mass": 4.5,  # Custom override
        "material": "carbon_fiber",  # Custom override
    }

    logger.info(f"Sending request to {url}...")
    try:
        start_time = time.time()
        response = requests.post(url, json=payload)
        end_time = time.time()

        logger.info(f"Status Code: {response.status_code}")
        logger.info(f"Time Taken: {end_time - start_time:.2f}s")

        if response.status_code == 200:
            data = response.json()
            logger.info("\nResponse Data:")
            logger.info(json.dumps(data, indent=2, ensure_ascii=False))

            if data["status"] == "success":
                logger.info("\n✅ Web Generation Test PASSED")
            else:
                logger.error("\n❌ Web Generation Test FAILED (Logic Error)")
        else:
            logger.info(f"\n❌ Web Generation Test FAILED (HTTP {response.status_code})")
            logger.info(response.text)

    except Exception as e:
        logger.info(f"\n❌ Connection Failed: {e}")
        logger.info("Make sure server.py is running!")


if __name__ == "__main__":
    test_generation()
