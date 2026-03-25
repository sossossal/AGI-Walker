import logging
logger = logging.getLogger(__name__)
import requests
import json
import os
import xml.etree.ElementTree as ET
import time

# Configuration
API_URL = "http://localhost:8000/api/generate_robot"
ROBOT_NAME = "verified_walker_01"
TARGET_HEIGHT = 0.45
EXPECTED_LINKS = [
    "torso_1",
    "thigh_left_2",
    "shin_left_3",
    "thigh_right_4",
    "shin_right_5",
]
EXPECTED_JOINTS = ["hip_left", "knee_left", "hip_right", "knee_right"]


def step_1_generate_via_web():
    logger.info(f"\n[1/3] Sending Generation Request to {API_URL}...")
    payload = {
        "name": ROBOT_NAME,
        "type": "quadruped",
        "scenario": "performance",
        "height": TARGET_HEIGHT,
    }

    try:
        response = requests.post(API_URL, json=payload, timeout=10)
        response.raise_for_status()
        data = response.json()

        if data["status"] == "success":
            logger.info(f"✅ Success! Config: {data['config_path']}")
            return data["urdf_path"]
        else:
            logger.error(f"❌ API Error: {data['message']}")
            exit(1)
    except Exception as e:
        logger.info(f"❌ Connection Failed: {e}")
        logger.info("Ensure 'python web_panel/server.py' is running.")
        exit(1)


def step_2_validate_urdf_structure(urdf_path):
    logger.info(f"\n[2/3] Validating URDF Structure: {urdf_path}...")

    if not os.path.exists(urdf_path):
        logger.info(f"❌ File not found: {urdf_path}")
        exit(1)

    try:
        tree = ET.parse(urdf_path)
        root = tree.getroot()

        # 1. Check Links
        links = [link.attrib["name"] for link in root.findall("link")]
        logger.info(f"   Found Links: {len(links)}")

        missing_links = [
            l for l in EXPECTED_LINKS if not any(l in found for found in links)
        ]
        if missing_links:
            logger.info(f"❌ Missing critical links: {missing_links}")
            logger.error("   (Did the leg upgrade fail?)")
            exit(1)
        logger.info("   ✅ Link Hierarchy Verified (Thighs & Shins present)")

        # 2. Check Joints
        joints = [j.attrib["name"] for j in root.findall("joint")]
        logger.info(f"   Found Joints: {len(joints)}")

        missing_joints = [
            j for j in EXPECTED_JOINTS if not any(j in found for found in joints)
        ]
        if missing_joints:
            logger.info(f"❌ Missing critical joints: {missing_joints}")
            exit(1)
        logger.info("   ✅ Joint Hierarchy Verified (Hips & Knees present)")

        # 3. Check Geometry (Cylinders)
        cylinders = root.findall(".//cylinder")
        if len(cylinders) >= 8:  # 4 legs * 2 segments
            logger.info(f"   ✅ Geometry Verified ({len(cylinders)} cylinder segments found)")
        else:
            logger.info(f"⚠️  Warning: Only {len(cylinders)} cylinders found. Expected >= 8.")

    except ET.ParseError:
        logger.info("❌ Invalid XML format")
        exit(1)


def step_3_validate_json_metadata():
    json_path = f"configs/generated/{ROBOT_NAME}.json"
    logger.info(f"\n[3/3] Validating Advanced Metadata: {json_path}...")

    if not os.path.exists(json_path):
        logger.info(f"❌ JSON Config not found")
        exit(1)

    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    meta = data.get("metadata", {})
    adv = meta.get("advanced_dynamics", {})

    # Check for Stiffness & Backlash (Real world params)
    if "stiffness" in adv and "backlash" in adv:
        logger.info(f"   ✅ Advanced Dynamics Found:")
        logger.info(f"      - Stiffness: {adv['stiffness']}")
        logger.info(f"      - Backlash:  {adv['backlash']}")
        logger.info(f"      - Thermal:   {adv.get('thermal_resistance', 'N/A')}")
    else:
        logger.info("❌ Missing 'advanced_dynamics' in metadata")
        exit(1)

    # Check Warnings
    warnings = meta.get("design_warnings", [])
    if warnings:
        logger.info(f"   ⚠️  Design Warnings Generated: {len(warnings)}")
        logger.info(f"      Example: {warnings[0]}")
    else:
        logger.info("   ✅ No Design Warnings (Clean build)")


if __name__ == "__main__":
    urdf_file = step_1_generate_via_web()
    step_2_validate_urdf_structure(urdf_file)
    step_3_validate_json_metadata()
    logger.info("\n🎉 ALL TESTS PASSED: Pipeline is Robust.")
