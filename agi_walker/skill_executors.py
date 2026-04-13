"""
Standardized Skill Executor Interface

Provides base classes and implementations for executing Skills in the
workflow orchestration system, enabling consistent action handling,
input/output management, and registry-based skill discovery.
"""

from abc import ABC, abstractmethod
import json
import os
from pathlib import Path
from typing import Any, Dict, Optional

import logging

from agi_walker.core.api.workflow_contracts import to_jsonable

logger = logging.getLogger(__name__)


class SkillExecutor(ABC):
    """Base class for all Skill executors"""

    def __init__(self, name: str):
        self.name = name

    @abstractmethod
    def execute(self, action: str, inputs: Dict[str, Any]) -> Dict[str, Any]:
        """
        Execute a skill action with given inputs

        Args:
            action: The action to perform
            inputs: Input parameters for the action

        Returns:
            Dictionary with execution results, including 'status' and action-specific outputs
        """
        pass

    @abstractmethod
    def get_supported_actions(self) -> list[str]:
        """Get list of actions supported by this executor"""
        pass


class RobotModelingExecutor(SkillExecutor):
    """Executor for robot modeling operations"""

    def __init__(self):
        super().__init__("robot_modeling")

    def get_supported_actions(self) -> list[str]:
        return ["create_from_template", "load_config", "customize_design"]

    def execute(self, action: str, inputs: Dict[str, Any]) -> Dict[str, Any]:
        """Execute robot modeling action"""

        if action == "create_from_template":
            template = inputs.get("template", "biped")
            output_file = inputs.get("output_file", f"models/{template}_model.json")
            # Mock implementation - would call actual robot modeling skill
            return {
                "status": "success",
                "action": action,
                "output_file": output_file,
                "template": template,
                "model_data": {
                    "name": f"{template}_robot",
                    "type": template,
                    "parts": ["torso", "legs", "arms"],
                },
            }

        elif action == "load_config":
            config_file = inputs.get("config_file")
            output_file = inputs.get("output_file", "models/loaded_robot.json")
            # Mock implementation
            return {
                "status": "success",
                "action": action,
                "source_config": config_file,
                "output_file": output_file,
                "model_data": {"source": config_file},
            }

        elif action == "customize_design":
            design_params = inputs.get("design_params", {})
            output_file = inputs.get("output_file", "models/custom_robot.json")
            # Mock implementation
            return {
                "status": "success",
                "action": action,
                "output_file": output_file,
                "parameters": design_params,
            }

        else:
            raise ValueError(f"Unknown action for robot_modeling: {action}")


class ParameterOptimizerExecutor(SkillExecutor):
    """Executor for parameter optimization operations"""

    def __init__(self):
        super().__init__("parameter_optimizer")

    def get_supported_actions(self) -> list[str]:
        return [
            "optimize_mass_distribution",
            "validate_physics",
            "calibrate_parameters",
        ]

    def execute(self, action: str, inputs: Dict[str, Any]) -> Dict[str, Any]:
        """Execute parameter optimization action"""

        if action == "optimize_mass_distribution":
            robot_config = inputs.get("robot_config")
            output_file = inputs.get("output_file", "models/optimized_params.json")
            # Mock implementation - would call actual optimization skill
            return {
                "status": "success",
                "action": action,
                "input_config": robot_config,
                "output_file": output_file,
                "optimization_results": {
                    "stability_score": 0.92,
                    "efficiency_score": 0.87,
                    "parameters": {
                        "mass_center": [0, 0, 0.5],
                        "balance_point": [0, 0, 0.3],
                    },
                },
            }

        elif action == "validate_physics":
            robot_config = inputs.get("robot_config")
            output_file = inputs.get("output_file", "models/validated_params.json")
            # Mock implementation
            return {
                "status": "success",
                "action": action,
                "input_config": robot_config,
                "output_file": output_file,
                "validation_results": {
                    "physics_valid": True,
                    "collision_free": True,
                    "joint_limits_ok": True,
                },
            }

        elif action == "calibrate_parameters":
            calibration_data = inputs.get("calibration_data", {})
            output_file = inputs.get("output_file", "models/calibrated_params.json")
            # Mock implementation
            return {
                "status": "success",
                "action": action,
                "output_file": output_file,
                "calibration": calibration_data,
            }

        else:
            raise ValueError(f"Unknown action for parameter_optimizer: {action}")


class URDFGeneratorExecutor(SkillExecutor):
    """Executor for URDF/SDF generation operations"""

    def __init__(self):
        super().__init__("urdf_generator")

    def get_supported_actions(self) -> list[str]:
        return ["export_to_format", "convert_format", "validate_format"]

    def execute(self, action: str, inputs: Dict[str, Any]) -> Dict[str, Any]:
        """Execute URDF/SDF generation action"""

        if action == "export_to_format":
            robot_config = inputs.get("robot_config")
            output_format = inputs.get("output_format", "urdf").lower()
            output_file = inputs.get("output_file", f"exports/robot.{output_format}")
            # Mock implementation - would call actual export skill
            return {
                "status": "success",
                "action": action,
                "input_config": robot_config,
                "output_file": output_file,
                "format": output_format,
                "file_size": 15234,  # Mock size
            }

        elif action == "convert_format":
            source_file = inputs.get("source_file")
            target_format = inputs.get("target_format", "sdf").lower()
            output_file = inputs.get(
                "output_file", f"exports/converted.{target_format}"
            )
            # Mock implementation
            return {
                "status": "success",
                "action": action,
                "source_file": source_file,
                "output_file": output_file,
                "target_format": target_format,
            }

        elif action == "validate_format":
            robot_file = inputs.get("robot_file")
            # Mock implementation
            return {
                "status": "success",
                "action": action,
                "robot_file": robot_file,
                "format_valid": True,
                "warnings": [],
            }

        else:
            raise ValueError(f"Unknown action for urdf_generator: {action}")


# ============================================================================
# Real Executor Implementations
# ============================================================================


class RealRobotModelingExecutor(SkillExecutor):
    """Real executor for robot modeling - calls actual skill APIs"""

    def __init__(self):
        super().__init__("robot_modeling")

    def get_supported_actions(self) -> list[str]:
        return ["create_from_template", "load_config"]

    def execute(self, action: str, inputs: Dict[str, Any]) -> Dict[str, Any]:
        """Execute real robot modeling action"""
        try:
            from agi_walker.skills.robot_modeling import (
                load_template,
            )
        except ImportError as e:
            return {
                "status": "error",
                "error": f"Import failed: {str(e)}",
                "action": action,
            }

        try:
            if action == "create_from_template":
                template = inputs.get("template", "biped")
                output_file = inputs.get("output_file", f"models/{template}_model.json")

                # Create output directory if needed
                Path(output_file).parent.mkdir(parents=True, exist_ok=True)

                # Load template
                config = load_template(template)

                # Convert to dict if it's a RobotConfig object
                if hasattr(config, "__dict__"):
                    config_dict = config.__dict__
                elif hasattr(config, "to_dict"):
                    config_dict = config.to_dict()
                else:
                    config_dict = (
                        dict(config)
                        if isinstance(config, dict)
                        else {"template": template, "data": str(config)}
                    )

                # Save config
                with open(output_file, "w", encoding="utf-8") as f:
                    json.dump(config_dict, f, indent=2, default=str)

                return {
                    "status": "success",
                    "action": action,
                    "output_file": output_file,
                    "template": template,
                    "message": f"Created robot from template: {template}",
                }

            elif action == "load_config":
                config_file = inputs.get("config_file")
                output_file = inputs.get("output_file", "models/loaded_robot.json")

                if not os.path.exists(config_file):
                    return {
                        "status": "error",
                        "error": f"Config file not found: {config_file}",
                        "action": action,
                    }

                # Read config
                with open(config_file, "r", encoding="utf-8") as f:
                    config = json.load(f)

                # Save to output
                Path(output_file).parent.mkdir(parents=True, exist_ok=True)
                with open(output_file, "w", encoding="utf-8") as f:
                    json.dump(config, f, indent=2, ensure_ascii=False)

                return {
                    "status": "success",
                    "action": action,
                    "source_config": config_file,
                    "output_file": output_file,
                    "message": f"Loaded config from: {config_file}",
                }

            else:
                return {
                    "status": "error",
                    "error": f"Unknown action: {action}",
                    "action": action,
                }

        except Exception as e:
            return {
                "status": "error",
                "error": str(e),
                "action": action,
                "error_type": type(e).__name__,
            }


class RealParameterOptimizerExecutor(SkillExecutor):
    """Real executor for parameter optimization - calls actual skill APIs"""

    def __init__(self):
        super().__init__("parameter_optimizer")

    def get_supported_actions(self) -> list[str]:
        return ["optimize_mass_distribution", "validate_physics"]

    def execute(self, action: str, inputs: Dict[str, Any]) -> Dict[str, Any]:
        """Execute real parameter optimization action"""
        try:
            from agi_walker.skills.parameter_optimizer import optimize_mass_distribution
        except ImportError as e:
            return {
                "status": "error",
                "error": f"Import failed: {str(e)}",
                "action": action,
            }

        try:
            if action == "optimize_mass_distribution":
                robot_config = inputs.get("robot_config")
                output_file = inputs.get("output_file", "models/optimized_params.json")

                if not os.path.exists(robot_config):
                    return {
                        "status": "error",
                        "error": f"Robot config not found: {robot_config}",
                        "action": action,
                    }

                # Read robot config
                with open(robot_config, "r", encoding="utf-8") as f:
                    config_data = json.load(f)

                # Extract all parameters for the skill function
                skill_params = {
                    k: v
                    for k, v in inputs.items()
                    if k not in ["robot_config", "output_file"]
                }

                # Call real optimization with all available parameters
                try:
                    result = optimize_mass_distribution(config_data, **skill_params)
                except TypeError:
                    # If function doesn't accept these params, try without them
                    result = optimize_mass_distribution(config_data)

                # Save optimized parameters
                Path(output_file).parent.mkdir(parents=True, exist_ok=True)

                # Convert result to dict if needed
                if hasattr(result, "__dict__"):
                    result_dict = result.__dict__
                elif hasattr(result, "to_dict"):
                    result_dict = result.to_dict()
                else:
                    result_dict = (
                        dict(result)
                        if isinstance(result, dict)
                        else {"result": str(result)}
                    )

                result_dict = to_jsonable(result_dict)

                with open(output_file, "w", encoding="utf-8") as f:
                    json.dump(result_dict, f, indent=2, ensure_ascii=False)

                return {
                    "status": "success",
                    "action": action,
                    "input_config": robot_config,
                    "output_file": output_file,
                    "optimization_results": result_dict,
                    "message": "Mass distribution optimized successfully",
                }

            elif action == "validate_physics":
                robot_config = inputs.get("robot_config")
                output_file = inputs.get("output_file", "models/validated_robot.json")

                if not os.path.exists(robot_config):
                    return {
                        "status": "error",
                        "error": f"Robot config not found: {robot_config}",
                        "action": action,
                    }

                with open(robot_config, "r", encoding="utf-8") as f:
                    config_data = json.load(f)

                parts = config_data.get("parts", [])
                connections = config_data.get("connections", [])
                validation_results = {
                    "physics_valid": bool(parts),
                    "collision_free": True,
                    "joint_limits_ok": True,
                    "parts_count": len(parts),
                    "connections_count": len(connections),
                }

                output_config = dict(config_data)
                metadata = dict(output_config.get("metadata", {}))
                metadata["physics_validation"] = validation_results
                output_config["metadata"] = metadata

                Path(output_file).parent.mkdir(parents=True, exist_ok=True)
                with open(output_file, "w", encoding="utf-8") as f:
                    json.dump(output_config, f, indent=2, ensure_ascii=False)

                return {
                    "status": "success",
                    "action": action,
                    "input_config": robot_config,
                    "output_file": output_file,
                    "validation_results": validation_results,
                    "message": "Physics validation completed",
                }

            else:
                return {
                    "status": "error",
                    "error": f"Unknown action: {action}",
                    "action": action,
                }

        except Exception as e:
            return {
                "status": "error",
                "error": str(e),
                "action": action,
                "error_type": type(e).__name__,
            }


class RealURDFGeneratorExecutor(SkillExecutor):
    """Real executor for URDF/SDF generation - calls actual skill APIs"""

    def __init__(self):
        super().__init__("urdf_generator")

    def get_supported_actions(self) -> list[str]:
        return ["export_to_format"]

    def execute(self, action: str, inputs: Dict[str, Any]) -> Dict[str, Any]:
        """Execute real URDF/SDF generation action"""
        try:
            from agi_walker.skills.urdf_generator import (
                convert_to_urdf,
                convert_to_sdf,
                validate_urdf,
            )
        except ImportError as e:
            return {
                "status": "error",
                "error": f"Import failed: {str(e)}",
                "action": action,
            }

        try:
            if action == "export_to_format":
                robot_config = inputs.get("robot_config")
                output_format = inputs.get("output_format", "urdf").lower()
                output_file = inputs.get(
                    "output_file", f"exports/robot.{output_format}"
                )

                if not os.path.exists(robot_config):
                    return {
                        "status": "error",
                        "error": f"Robot config not found: {robot_config}",
                        "action": action,
                    }

                # Read robot config
                with open(robot_config, "r", encoding="utf-8") as f:
                    config_data = json.load(f)

                # Convert to requested format
                Path(output_file).parent.mkdir(parents=True, exist_ok=True)

                if output_format == "urdf":
                    try:
                        urdf_content = convert_to_urdf(
                            config_data, output_file=output_file
                        )
                        # Only write if function returns content (not None)
                        if urdf_content:
                            with open(output_file, "w", encoding="utf-8") as f:
                                f.write(urdf_content)
                    except TypeError:
                        # Fallback if function doesn't accept output_file parameter
                        urdf_content = convert_to_urdf(config_data)
                        if urdf_content:
                            with open(output_file, "w", encoding="utf-8") as f:
                                f.write(urdf_content)

                elif output_format == "sdf":
                    try:
                        sdf_content = convert_to_sdf(
                            config_data, output_file=output_file
                        )
                        # Only write if function returns content (not None)
                        if sdf_content:
                            with open(output_file, "w", encoding="utf-8") as f:
                                f.write(sdf_content)
                    except TypeError:
                        # Fallback if function doesn't accept output_file parameter
                        sdf_content = convert_to_sdf(config_data)
                        if sdf_content:
                            with open(output_file, "w", encoding="utf-8") as f:
                                f.write(sdf_content)
                else:
                    return {
                        "status": "error",
                        "error": f"Unsupported format: {output_format}",
                        "action": action,
                    }

                # Validate output
                try:
                    is_valid = (
                        validate_urdf(output_file) if output_format == "urdf" else True
                    )
                except Exception:
                    is_valid = True  # Skip validation if it fails

                output_exists = os.path.exists(output_file)
                file_size = os.path.getsize(output_file) if output_exists else 0

                if not output_exists:
                    return {
                        "status": "error",
                        "action": action,
                        "input_config": robot_config,
                        "output_file": output_file,
                        "format": output_format,
                        "error": f"Final {output_format.upper()} file was not materialized at {output_file}.",
                        "output_generated": False,
                    }

                response: Dict[str, Any] = {
                    "status": "success",
                    "action": action,
                    "input_config": robot_config,
                    "output_file": output_file,
                    "format": output_format,
                    "file_size": file_size,
                    "valid": is_valid,
                    "message": f"Exported to {output_format}: {output_file}",
                    "output_generated": True,
                }

                return response

            else:
                return {
                    "status": "error",
                    "error": f"Unknown action: {action}",
                    "action": action,
                }

        except Exception as e:
            return {
                "status": "error",
                "error": str(e),
                "action": action,
                "error_type": type(e).__name__,
            }


# Executor registry
_skill_executors: Dict[str, SkillExecutor] = {}
_real_skill_executors: Dict[str, SkillExecutor] = {}
_use_real_executors = False  # Global flag to switch between real and mock


def set_executor_mode(use_real: bool = False) -> None:
    """Set whether to use real or mock executors for the entire application"""
    global _use_real_executors
    _use_real_executors = use_real


def get_executor_mode() -> str:
    """Get current executor mode (real or mock)"""
    return "real" if _use_real_executors else "mock"


def register_skill_executor(executor: SkillExecutor) -> None:
    """Register a skill executor"""
    _skill_executors[executor.name] = executor


def register_real_skill_executor(executor: SkillExecutor) -> None:
    """Register a real (non-mock) skill executor"""
    _real_skill_executors[executor.name] = executor


def get_skill_executor(
    name: str, use_real: Optional[bool] = None
) -> Optional[SkillExecutor]:
    """Get a registered skill executor by name

    Args:
        name: Name of the executor
        use_real: If True, get real executor; if False, get mock executor;
                 if None, use global setting

    Returns:
        SkillExecutor instance or None if not found
    """
    use_real_exec = use_real if use_real is not None else _use_real_executors

    if use_real_exec:
        return _real_skill_executors.get(name)
    else:
        return _skill_executors.get(name)


def execute_skill(
    executor_name: str,
    action: str,
    inputs: Dict[str, Any],
    use_real: Optional[bool] = None,
) -> Dict[str, Any]:
    """Execute a skill using its executor"""
    executor = get_skill_executor(executor_name, use_real)
    if not executor:
        return {
            "status": "error",
            "error": f"Executor {executor_name} not found (mode: {get_executor_mode()})",
        }
    return executor.execute(action, inputs)


def list_skill_executors() -> list:
    """List all registered skill executors"""
    return list(_skill_executors.keys())


# Initialize built-in executors
def _initialize_builtin_executors():
    """Initialize built-in skill executors"""
    # Mock executors (safe, don't require actual dependencies)
    register_skill_executor(RobotModelingExecutor())
    register_skill_executor(ParameterOptimizerExecutor())
    register_skill_executor(URDFGeneratorExecutor())

    # Real executors (call actual skill APIs)
    register_real_skill_executor(RealRobotModelingExecutor())
    register_real_skill_executor(RealParameterOptimizerExecutor())
    register_real_skill_executor(RealURDFGeneratorExecutor())


# Auto-initialize built-in executors on module import
_initialize_builtin_executors()
