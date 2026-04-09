import json
import logging
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from fastapi.testclient import TestClient


logger = logging.getLogger(__name__)
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from web_panel.server import app


ROBOT_NAME = "verified_walker_01_runtime_probe"
TARGET_HEIGHT = 0.45
EXPECTED_LINKS = [
    "torso_1",
    "thigh_left_2",
    "shin_left_3",
    "thigh_right_4",
    "shin_right_5",
]
EXPECTED_JOINTS = ["hip_left", "knee_left", "hip_right", "knee_right"]


def _cleanup_generated_files(robot_name: str) -> tuple[Path, Path]:
    config_path = PROJECT_ROOT / "configs" / "generated" / f"{robot_name}.json"
    urdf_path = PROJECT_ROOT / "exports" / f"{robot_name}.urdf"
    for path in (config_path, urdf_path):
        if path.exists():
            path.unlink()
    return config_path, urdf_path


def _validate_urdf_structure(urdf_path: Path) -> None:
    logger.info(f"\n[2/3] Validating URDF Structure: {urdf_path}...")

    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")

    tree = ET.parse(urdf_path)
    root = tree.getroot()

    links = [link.attrib["name"] for link in root.findall("link")]
    logger.info(f"   Found Links: {len(links)}")
    missing_links = [
        expected_link
        for expected_link in EXPECTED_LINKS
        if not any(expected_link in found for found in links)
    ]
    if missing_links:
        raise AssertionError(f"Missing critical links: {missing_links}")
    logger.info("   PASS: Link hierarchy verified")

    joints = [joint.attrib["name"] for joint in root.findall("joint")]
    logger.info(f"   Found Joints: {len(joints)}")
    missing_joints = [
        expected_joint
        for expected_joint in EXPECTED_JOINTS
        if not any(expected_joint in found for found in joints)
    ]
    if missing_joints:
        raise AssertionError(f"Missing critical joints: {missing_joints}")
    logger.info("   PASS: Joint hierarchy verified")

    cylinders = root.findall(".//cylinder")
    if len(cylinders) < 8:
        raise AssertionError(
            f"Expected at least 8 cylinder segments, found {len(cylinders)}"
        )
    logger.info(f"   PASS: Geometry verified ({len(cylinders)} cylinder segments)")


def _validate_json_metadata(config_path: Path) -> None:
    logger.info(f"\n[3/3] Validating Advanced Metadata: {config_path}...")

    if not config_path.exists():
        raise FileNotFoundError(f"JSON config not found: {config_path}")

    data = json.loads(config_path.read_text(encoding="utf-8"))
    metadata = data.get("metadata", {})
    advanced_dynamics = metadata.get("advanced_dynamics", {})

    if "stiffness" not in advanced_dynamics or "backlash" not in advanced_dynamics:
        raise AssertionError("Missing advanced_dynamics stiffness/backlash metadata")

    logger.info("   PASS: Advanced dynamics found")
    logger.info(f"      - Stiffness: {advanced_dynamics['stiffness']}")
    logger.info(f"      - Backlash:  {advanced_dynamics['backlash']}")
    logger.info(
        f"      - Thermal:   {advanced_dynamics.get('thermal_resistance', 'N/A')}"
    )

    warnings = metadata.get("design_warnings", [])
    if warnings:
        logger.info(f"   WARN: Design warnings generated: {len(warnings)}")
        logger.info(f"      Example: {warnings[0]}")
    else:
        logger.info("   PASS: No design warnings")


def verify_pipeline() -> bool:
    logger.info("=== Web Generation Pipeline Verification ===")
    client = TestClient(app)
    config_path, urdf_path = _cleanup_generated_files(ROBOT_NAME)

    try:
        logger.info("\n[1/3] Sending generation request to /api/generate_robot...")
        response = client.post(
            "/api/generate_robot",
            json={
                "name": ROBOT_NAME,
                "type": "quadruped",
                "scenario": "performance",
                "height": TARGET_HEIGHT,
            },
        )
        response.raise_for_status()
        data = response.json()

        if data["status"] != "success":
            raise AssertionError(f"API Error: {data}")

        config_path = PROJECT_ROOT / data["config_path"]
        urdf_path = PROJECT_ROOT / data["urdf_path"]
        logger.info(f"   PASS: Generated config at {config_path}")

        _validate_urdf_structure(urdf_path)
        _validate_json_metadata(config_path)
        logger.info("\nPASS: Pipeline verification completed")
        return True
    except Exception as exc:
        logger.exception("FAIL: %s", exc)
        return False
    finally:
        _cleanup_generated_files(ROBOT_NAME)


if __name__ == "__main__":
    raise SystemExit(0 if verify_pipeline() else 1)
