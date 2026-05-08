import json
from pathlib import Path

from tools.build_ros2_typed_inventory import build_ros2_typed_inventory, main


def test_ros2_typed_inventory_tracks_local_code_surfaces() -> None:
    payload = build_ros2_typed_inventory()

    assert payload["schema_version"] == "1.0"
    assert payload["status"] == "ready"
    surfaces = {surface["name"]: surface for surface in payload["typed_surfaces"]}
    assert set(surfaces) == {
        "instruction_set",
        "simulated_circuit",
        "hardware_recovery",
        "behavior_navigation_perception",
    }
    assert surfaces["instruction_set"]["status"] == "ready"
    assert "agi_walker_msgs/msg/InstructionSet" in surfaces["instruction_set"]["messages"]
    assert "/instruction_set/apply" in surfaces["instruction_set"]["routes"]


def test_ros2_typed_inventory_writes_output(tmp_path: Path) -> None:
    output = tmp_path / "typed_inventory.json"

    exit_code = main(["--output", str(output)])

    assert exit_code == 0
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "ready"
