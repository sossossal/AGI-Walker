"""
Verification script for the legacy Web-Godot integration flow.

Exercises:
1. Agent command parsing via /api/agent/parse-command
2. Legacy Godot connect/load/start/stop/disconnect routes
3. End-to-end command delivery to a real MockGodotServer
"""

import logging
import sys
import time
from pathlib import Path

from fastapi.testclient import TestClient


logger = logging.getLogger(__name__)
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agi_walker.core.api.comm.godot_client import MockGodotServer
from web_panel.godot_controller import godot_controller
from web_panel.server import app


def verify_web_integration() -> bool:
    logger.info("=== Web-Godot Integration Verification ===")

    session_id = "verify-web-integration"
    server = MockGodotServer(port=0)
    logger.info("[1/7] Starting Mock Godot Server...")
    if not server.start():
        logger.error("FAIL: Could not start mock server")
        return False
    port = server.port

    try:
        time.sleep(0.5)
        with TestClient(app) as client:
            logger.info("[2/7] Testing Agent Command Parsing...")
            parse_response = client.post(
                "/api/agent/parse-command",
                json={"command": "create quadruped"},
            )
            if parse_response.status_code != 200:
                logger.error("FAIL: Parse command API failed: %s", parse_response.text)
                return False

            parse_payload = parse_response.json()
            config = parse_payload.get("config", {})
            if (
                parse_payload.get("status") != "success"
                or config.get("metadata", {}).get("type") != "quadruped"
                or len(config.get("parts", [])) == 0
            ):
                logger.error("FAIL: Invalid parse response: %s", parse_payload)
                return False
            logger.info("PASS: Agent command parsed successfully")

            logger.info("[3/7] Testing Connect API...")
            connect_response = client.post(
                f"/api/godot/connect?session_id={session_id}",
                json={"host": "127.0.0.1", "port": port},
            )
            if (
                connect_response.status_code != 200
                or connect_response.json().get("status") != "connected"
            ):
                logger.error("FAIL: Connect API failed: %s", connect_response.text)
                return False
            logger.info("PASS: Connected to Godot via Web API")

            logger.info("[4/7] Testing Status API...")
            status_response = client.get(f"/api/godot/status?session_id={session_id}")
            if (
                status_response.status_code != 200
                or status_response.json().get("connected") is not True
            ):
                logger.error("FAIL: Status API failed: %s", status_response.text)
                return False
            logger.info("PASS: Status API reports connected state")

            logger.info("[5/7] Testing Load Robot API...")
            load_response = client.post(
                f"/api/godot/load-robot?session_id={session_id}",
                json=config,
            )
            if (
                load_response.status_code != 200
                or load_response.json().get("status") != "success"
            ):
                logger.error("FAIL: Load Robot API failed: %s", load_response.text)
                return False
            logger.info("PASS: Robot loaded via Web API")

            logger.info("[6/7] Testing Start/Stop APIs...")
            start_response = client.post(
                f"/api/godot/start?session_id={session_id}",
                json={"physics": {"gravity": 9.8}},
            )
            if (
                start_response.status_code != 200
                or start_response.json().get("status") != "started"
            ):
                logger.error("FAIL: Start Simulation API failed: %s", start_response.text)
                return False

            stop_response = client.post(f"/api/godot/stop?session_id={session_id}")
            if (
                stop_response.status_code != 200
                or stop_response.json().get("status") != "stopped"
            ):
                logger.error("FAIL: Stop Simulation API failed: %s", stop_response.text)
                return False
            logger.info("PASS: Simulation lifecycle completed via Web API")

            logger.info("[7/7] Testing Disconnect API...")
            disconnect_response = client.post(
                f"/api/godot/disconnect?session_id={session_id}"
            )
            if (
                disconnect_response.status_code != 200
                or disconnect_response.json().get("status") != "disconnected"
            ):
                logger.error(
                    "FAIL: Disconnect API failed: %s", disconnect_response.text
                )
                return False
            logger.info("PASS: Disconnect API completed")

        logger.info("\nPASS: Web Integration Verification Passed")
        return True
    except Exception as exc:
        logger.exception("FAIL: Exception during verification: %s", exc)
        return False
    finally:
        try:
            godot_controller.release_session(session_id)
        except Exception:
            pass
        server.stop()


if __name__ == "__main__":
    raise SystemExit(0 if verify_web_integration() else 1)
