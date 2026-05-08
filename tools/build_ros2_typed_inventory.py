from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Sequence


DEFAULT_OUTPUT = "test_env/ros2_typed_idl_cutover/typed_inventory.json"
MSG_DIR = Path("hardware/ros2_ws/src/agi_walker_msgs/msg")
SRV_DIR = Path("hardware/ros2_ws/src/agi_walker_msgs/srv")
BRIDGE_NODE = Path("hardware/ros2_ws/src/agi_walker_ros2/agi_walker_ros2/bridge_node.py")
SCHEMA_VERSION = "1.0"


SURFACE_REQUIREMENTS = [
    {
        "name": "instruction_set",
        "messages": ["InstructionSet.msg", "InstructionStep.msg"],
        "services": ["ApplyInstructionSet.srv"],
        "routes": ["/instruction_set", "/instruction_set/apply"],
    },
    {
        "name": "simulated_circuit",
        "messages": ["SimulatedCircuit.msg"],
        "services": ["ConfigureSimulatedCircuit.srv"],
        "routes": ["/simulated_circuit", "/simulated_circuit/configure"],
    },
    {
        "name": "hardware_recovery",
        "messages": [
            "HardwareFault.msg",
            "HardwareRecoveryAction.msg",
            "HardwareRecoveryStatus.msg",
        ],
        "services": ["HardwareRecovery.srv"],
        "routes": ["/hardware/recovery"],
    },
    {
        "name": "behavior_navigation_perception",
        "messages": [
            "BehaviorCommand.msg",
            "NavigationGoal.msg",
            "PerceptionSnapshot.msg",
        ],
        "services": [],
        "routes": [
            "/behavior_command",
            "/navigation_goal",
            "/perception_snapshot",
        ],
    },
]


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a local code inventory for ROS2 typed IDL surfaces."
    )
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def _surface_status(requirement: dict[str, Any], bridge_text: str) -> str:
    messages_ready = all(MSG_DIR.joinpath(name).exists() for name in requirement["messages"])
    services_ready = all(SRV_DIR.joinpath(name).exists() for name in requirement["services"])
    routes_ready = all(route in bridge_text for route in requirement["routes"])
    return "ready" if messages_ready and services_ready and routes_ready else "blocked"


def build_ros2_typed_inventory() -> dict[str, Any]:
    bridge_text = BRIDGE_NODE.read_text(encoding="utf-8") if BRIDGE_NODE.exists() else ""
    surfaces = [
        {
            "name": requirement["name"],
            "status": _surface_status(requirement, bridge_text),
            "messages": [
                f"agi_walker_msgs/msg/{Path(name).stem}"
                for name in requirement["messages"]
            ],
            "services": [
                f"agi_walker_msgs/srv/{Path(name).stem}"
                for name in requirement["services"]
            ],
            "routes": requirement["routes"],
        }
        for requirement in SURFACE_REQUIREMENTS
    ]
    return {
        "schema_version": SCHEMA_VERSION,
        "status": "ready"
        if all(surface["status"] == "ready" for surface in surfaces)
        else "blocked",
        "scope": "local_code_inventory",
        "note": (
            "Static code-level inventory for typed IDL surfaces; does not replace "
            "target ROS2 Humble live cutover evidence."
        ),
        "typed_surfaces": surfaces,
    }


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    payload = build_ros2_typed_inventory()
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return 0 if payload["status"] == "ready" else 1


if __name__ == "__main__":
    raise SystemExit(main())
