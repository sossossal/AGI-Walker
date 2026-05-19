"""Validate and optionally normalize robot JSON for dynamic Godot assembly."""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
from typing import Any


def _load_robot_schema_module() -> Any:
    repo_root = Path(__file__).resolve().parents[1]
    module_path = repo_root / "agi_walker" / "core" / "api" / "robot_schema.py"
    spec = importlib.util.spec_from_file_location("robot_schema", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load robot schema module from {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate robot JSON for dynamic Godot assembly."
    )
    parser.add_argument("input", help="Path to a robot config JSON file.")
    parser.add_argument(
        "--input-kind",
        choices=["robot", "node-tree-manifest", "auto"],
        default="robot",
        help=(
            "Input payload kind. Use node-tree-manifest to validate a standalone "
            "static Godot node-tree manifest sidecar, or auto to detect it from "
            "manifest_version."
        ),
    )
    parser.add_argument(
        "--write-normalized",
        help="Optional output path for the normalized Godot-ready JSON.",
    )
    parser.add_argument(
        "--write-node-tree-manifest",
        help="Optional output path for the static Godot node-tree manifest JSON.",
    )
    args = parser.parse_args()

    input_path = Path(args.input)
    payload = json.loads(input_path.read_text(encoding="utf-8"))
    robot_schema = _load_robot_schema_module()
    input_kind = args.input_kind
    if (
        input_kind == "auto"
        and isinstance(payload, dict)
        and payload.get("manifest_version")
        == robot_schema.GODOT_NODE_TREE_MANIFEST_VERSION
    ):
        input_kind = "node-tree-manifest"
    if input_kind == "node-tree-manifest":
        node_tree_manifest_errors = (
            robot_schema.validate_godot_node_tree_manifest(payload)
        )
        node_tree_manifest_path_map_mismatches = (
            robot_schema.build_godot_node_tree_manifest_path_map_mismatches(payload)
        )
        result = {
            "status": "error" if node_tree_manifest_errors else "success",
            "input": str(input_path),
            "input_kind": "node-tree-manifest",
            "manifest_version": payload.get("manifest_version")
            if isinstance(payload, dict)
            else None,
            "robot_name": payload.get("robot_name")
            if isinstance(payload, dict)
            else None,
            "node_tree_manifest": payload,
            "node_tree_manifest_errors": node_tree_manifest_errors,
            "node_tree_manifest_path_map_mismatches": (
                node_tree_manifest_path_map_mismatches
            ),
        }
        print(json.dumps(result, indent=2, ensure_ascii=False))
        return 1 if node_tree_manifest_errors else 0

    normalized = robot_schema.normalize_robot_config_for_godot(payload)
    errors = robot_schema.validate_godot_robot_config(normalized)
    topology_summary = robot_schema.build_mechanical_topology_summary(normalized)
    node_tree_manifest = robot_schema.build_godot_node_tree_manifest(normalized)
    node_tree_manifest_errors = (
        robot_schema.validate_godot_node_tree_manifest(node_tree_manifest)
    )
    node_tree_manifest_path_map_mismatches = (
        robot_schema.build_godot_node_tree_manifest_path_map_mismatches(
            node_tree_manifest
        )
    )
    all_errors = [*errors, *node_tree_manifest_errors]

    result = {
        "status": "error" if all_errors else "success",
        "input": str(input_path),
        "input_kind": "robot",
        "schema_version": normalized.get("schema_version"),
        "parts_count": len(normalized.get("parts", [])),
        "connections_count": len(normalized.get("connections", [])),
        "topology_summary": topology_summary,
        "node_tree_manifest": node_tree_manifest,
        "node_tree_manifest_errors": node_tree_manifest_errors,
        "node_tree_manifest_path_map_mismatches": (
            node_tree_manifest_path_map_mismatches
        ),
        "errors": errors,
    }

    if args.write_normalized:
        output_path = Path(args.write_normalized)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(normalized, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        result["normalized_output"] = str(output_path)

    if args.write_node_tree_manifest:
        manifest_path = Path(args.write_node_tree_manifest)
        manifest_path.parent.mkdir(parents=True, exist_ok=True)
        manifest_path.write_text(
            json.dumps(node_tree_manifest, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        result["node_tree_manifest_output"] = str(manifest_path)

    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 1 if all_errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
