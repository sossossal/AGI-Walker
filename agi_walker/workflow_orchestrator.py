"""
Workflow Orchestration Engine for AGI-Walker Skills System

Provides workflow templates, orchestration, and execution management for
coordinating complex multi-step Skills workflows with state tracking,
error handling, and context-based variable resolution.
"""

import logging
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
import json
import os
import time
from typing import Any, Callable, Dict, List, Optional

logger = logging.getLogger(__name__)

_ARTIFACT_REQUIRED_FIELDS = {
    "workflow",
    "step",
    "executor",
    "action",
    "status",
    "inputs",
    "output",
    "mode",
}


class WorkflowStatus(Enum):
    """Workflow execution states"""

    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class StepStatus(Enum):
    """Individual step execution states"""

    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    SKIPPED = "skipped"


@dataclass
class WorkflowStep:
    """Represents a single step in a workflow"""

    name: str
    skill_executor: str  # Name of the executor to use
    action: str  # Action to perform
    inputs: Dict[str, Any] = field(default_factory=dict)
    status: StepStatus = StepStatus.PENDING
    output: Dict[str, Any] = field(default_factory=dict)
    error: Optional[str] = None
    artifact_path: Optional[str] = None
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None

    @property
    def duration(self) -> float:
        """Get step execution duration in seconds"""
        if self.start_time and self.end_time:
            return (self.end_time - self.start_time).total_seconds()
        return 0.0


@dataclass
class WorkflowResult:
    """Result of a workflow execution"""

    workflow_name: str
    status: WorkflowStatus
    steps: List[WorkflowStep] = field(default_factory=list)
    error_message: Optional[str] = None
    start_time: datetime = field(default_factory=datetime.now)
    end_time: Optional[datetime] = None
    log_path: Optional[str] = None

    @property
    def duration(self) -> float:
        """Get total workflow duration in seconds"""
        if self.end_time:
            return (self.end_time - self.start_time).total_seconds()
        return 0.0

    @property
    def total_steps(self) -> int:
        """Get total number of workflow steps"""
        return len(self.steps)

    def _count_steps(self, *statuses: StepStatus) -> int:
        """Count steps matching any of the provided statuses"""
        return sum(1 for step in self.steps if step.status in statuses)

    @property
    def completed_steps(self) -> int:
        """Get number of completed steps"""
        return self._count_steps(StepStatus.COMPLETED)

    @property
    def skipped_steps(self) -> int:
        """Get number of skipped steps"""
        return self._count_steps(StepStatus.SKIPPED)

    @property
    def failed_steps(self) -> int:
        """Get number of failed steps"""
        return self._count_steps(StepStatus.FAILED)

    @property
    def successful_steps(self) -> int:
        """Get number of successful terminal steps"""
        return self._count_steps(StepStatus.COMPLETED, StepStatus.SKIPPED)

    @property
    def success_rate(self) -> float:
        """Get percentage of steps that reached a non-failed terminal state"""
        if not self.total_steps:
            return 0.0
        return (self.successful_steps / self.total_steps) * 100

    def to_dict(self) -> Dict[str, Any]:
        """Convert result to a serializable dictionary"""
        return {
            "workflow_name": self.workflow_name,
            "status": self.status.value,
            "duration": self.duration,
            "total_steps": self.total_steps,
            "completed_steps": self.completed_steps,
            "skipped_steps": self.skipped_steps,
            "failed_steps": self.failed_steps,
            "successful_steps": self.successful_steps,
            "success_rate": self.success_rate,
            "error_message": self.error_message,
            "log_path": self.log_path,
            "start_time": self.start_time.isoformat(),
            "end_time": self.end_time.isoformat() if self.end_time else None,
            "steps": [
                {
                    "name": s.name,
                    "executor": s.skill_executor,
                    "action": s.action,
                    "status": s.status.value,
                    "duration": s.duration,
                    "output": s.output,
                    "error": s.error,
                    "start_time": s.start_time.isoformat() if s.start_time else None,
                    "end_time": s.end_time.isoformat() if s.end_time else None,
                    "artifact_path": s.artifact_path,
                }
                for s in self.steps
            ],
        }


class WorkflowOrchestrator:
    """Main orchestration engine for Skills workflows"""

    DEFAULT_EXECUTION_STRATEGY = "resume"
    VALID_EXECUTION_STRATEGIES = {"resume", "force"}

    def __init__(self, use_real_executors: bool = False):
        self.workflows: Dict[str, Dict[str, Any]] = {}
        self._skill_executors: Dict[str, Any] = {}
        self._real_skill_executors: Dict[str, Any] = {}
        self._execution_context: Dict[str, Any] = {}
        self._progress_callback: Optional[Callable[[Dict[str, Any]], None]] = None
        self._use_real_executors = use_real_executors

        # Register built-in workflows
        self._register_builtin_workflows()
        # Register built-in Skill executors
        self._register_builtin_executors()

    def _register_builtin_workflows(self):
        """Register built-in workflow templates"""

        # Robot Creation Pipeline: Model -> Optimize -> Export
        self.workflows["robot_creation_pipeline"] = {
            "name": "robot_creation_pipeline",
            "description": "Create a robot model, optimize parameters, and export to URDF",
            "steps": [
                {
                    "name": "create_model",
                    "skill_executor": "robot_modeling",
                    "action": "create_from_template",
                    "inputs": {
                        "template": "biped_basic",
                        "output_file": ".output/created_robot.json",
                    },
                },
                {
                    "name": "optimize_params",
                    "skill_executor": "parameter_optimizer",
                    "action": "optimize_mass_distribution",
                    "inputs": {
                        "robot_config": "{create_model.output_file}",
                        "output_file": ".output/optimized_robot.json",
                        "target_com_height": 0.4,
                    },
                },
                {
                    "name": "export_urdf",
                    "skill_executor": "urdf_generator",
                    "action": "export_to_format",
                    "inputs": {
                        "robot_config": "{optimize_params.output_file}",
                        "output_format": "urdf",
                        "output_file": "exports/robot.urdf",
                    },
                },
            ],
        }

        # Simulation Ready Pipeline: Model -> Validate -> Export
        self.workflows["simulation_ready_robot"] = {
            "name": "simulation_ready_robot",
            "description": "Prepare a robot for simulation with validation",
            "steps": [
                {
                    "name": "load_model",
                    "skill_executor": "robot_modeling",
                    "action": "load_config",
                    "inputs": {
                        "config_file": "configs/tutorial_01_biped.json",
                        "output_file": ".output/sim_robot.json",
                    },
                },
                {
                    "name": "validate_physics",
                    "skill_executor": "parameter_optimizer",
                    "action": "validate_physics",
                    "inputs": {
                        "robot_config": "{load_model.output_file}",
                        "output_file": ".output/validated_robot.json",
                    },
                },
                {
                    "name": "export_for_sim",
                    "skill_executor": "urdf_generator",
                    "action": "export_to_format",
                    "inputs": {
                        "robot_config": "{validate_physics.output_file}",
                        "output_format": "sdf",
                        "output_file": "exports/robot_sim.sdf",
                    },
                },
            ],
        }

    def _register_builtin_executors(self):
        """Register built-in Skill executors"""
        try:
            from agi_walker.skill_executors import get_skill_executor

            # Load mock executors (always available)
            for executor_name in [
                "robot_modeling",
                "parameter_optimizer",
                "urdf_generator",
            ]:
                try:
                    executor = get_skill_executor(executor_name, use_real=False)
                    self._skill_executors[executor_name] = executor
                except Exception as e:
                    logger.info(
                        f"Warning: Could not load mock executor {executor_name}: {e}"
                    )

            # Load real executors (may fail if dependencies missing)
            for executor_name in [
                "robot_modeling",
                "parameter_optimizer",
                "urdf_generator",
            ]:
                try:
                    executor = get_skill_executor(executor_name, use_real=True)
                    self._real_skill_executors[executor_name] = executor
                except Exception as e:
                    logger.info(
                        f"Warning: Could not load real executor {executor_name}: {e}"
                    )

        except ImportError:
            logger.warning(
                "skill_executors module not available - using fallback mock executors"
            )
            self._setup_mock_executors()

    def set_executor_mode(self, use_real: bool) -> None:
        """Switch between real and mock executors"""
        self._use_real_executors = use_real

    def get_executor_mode(self) -> str:
        """Get current executor mode"""
        return "real" if self._use_real_executors else "mock"

    def _get_executor(self, name: str):
        """Get executor by name, respecting current mode"""
        if self._use_real_executors:
            return self._real_skill_executors.get(name) or self._skill_executors.get(
                name
            )
        else:
            return self._skill_executors.get(name)

    def _setup_mock_executors(self):
        """Setup mock executors for testing"""

        class MockExecutor:
            def __init__(self, name):
                self.name = name

            def execute(self, action, inputs):
                return {
                    "status": "success",
                    "action": action,
                    "output_file": inputs.get(
                        "output_file", f"output_{self.name}.json"
                    ),
                }

        self._skill_executors["robot_modeling"] = MockExecutor("robot_modeling")
        self._skill_executors["parameter_optimizer"] = MockExecutor(
            "parameter_optimizer"
        )
        self._skill_executors["urdf_generator"] = MockExecutor("urdf_generator")

    def list_workflows(self) -> List[str]:
        """List available workflows"""
        return list(self.workflows.keys())

    def get_workflow(self, name: str) -> Optional[Dict[str, Any]]:
        """Get workflow definition by name"""
        return self.workflows.get(name)

    def validate_workflow(self, name: str) -> tuple[bool, str]:
        """Validate workflow definition before execution"""
        workflow = self.get_workflow(name)
        if not workflow:
            return False, f"Workflow '{name}' not found"

        if "steps" not in workflow or not workflow["steps"]:
            return False, f"Workflow '{name}' has no steps defined"

        for i, step in enumerate(workflow["steps"]):
            step_id = f"Step {i} ({step.get('name', 'unnamed')})"

            if "name" not in step:
                return False, f"{step_id} missing 'name' field"
            if "skill_executor" not in step:
                return False, f"{step_id} missing 'skill_executor' field"
            if "action" not in step:
                return False, f"{step_id} missing 'action' field"

            executor_name = step["skill_executor"]
            action = step["action"]

            # Check if executor exists
            executor = self._skill_executors.get(executor_name)
            if not executor:
                return False, f"{step_id}: Executor '{executor_name}' not registered"

            # Check if action is supported (Mandatory check)
            if hasattr(executor, "get_supported_actions"):
                supported_actions = executor.get_supported_actions()
                if action not in supported_actions:
                    return False, (
                        f"{step_id}: Action '{action}' is not supported by executor '{executor_name}'. "
                        f"Supported actions are: {', '.join(supported_actions)}"
                    )
            else:
                return (
                    False,
                    f"{step_id}: Executor '{executor_name}' does not provide a list of supported actions",
                )

        return True, "Workflow is valid"

    def create_custom_workflow(
        self, name: str, steps: List[Dict[str, Any]], description: str = ""
    ) -> bool:
        """Create and register a custom workflow"""
        workflow = {"name": name, "description": description, "steps": steps}

        # Validate before registering
        self.workflows[name] = workflow  # Temporarily add for validation
        is_valid, message = self.validate_workflow(name)

        if not is_valid:
            del self.workflows[name]  # Remove if invalid
            return False

        return True

    def _get_execution_strategy(self) -> str:
        """Resolve the workflow execution strategy from the current context"""
        strategy = self._execution_context.get("execution_strategy")

        if strategy is None:
            if "skip_if_exists" in self._execution_context:
                return (
                    "resume" if self._execution_context["skip_if_exists"] else "force"
                )
            return self.DEFAULT_EXECUTION_STRATEGY

        normalized = str(strategy).strip().lower()
        if normalized not in self.VALID_EXECUTION_STRATEGIES:
            valid = ", ".join(sorted(self.VALID_EXECUTION_STRATEGIES))
            raise ValueError(
                f"Invalid execution_strategy '{strategy}'. Expected one of: {valid}"
            )
        return normalized

    def _get_output_root(self) -> Optional[str]:
        """Return the configured output root, if any"""
        output_root = self._execution_context.get("output_root")
        if output_root in (None, ""):
            return None
        return os.path.normpath(os.fspath(output_root))

    def _resolve_output_file_path(self, output_file: Any) -> Any:
        """Rewrite relative output paths under output_root when configured"""
        if not isinstance(output_file, str) or not output_file:
            return output_file

        output_root = self._get_output_root()
        normalized_output = os.path.normpath(output_file)
        if not output_root or os.path.isabs(normalized_output):
            return normalized_output

        return os.path.normpath(os.path.join(output_root, normalized_output))

    def _serialize_step_for_progress(self, step: WorkflowStep) -> Dict[str, Any]:
        """Build a compact step payload for live progress reporting."""
        output = step.output if isinstance(step.output, dict) else {}
        return {
            "name": step.name,
            "executor": step.skill_executor,
            "action": step.action,
            "status": step.status.value,
            "duration": step.duration,
            "error": step.error,
            "start_time": step.start_time.isoformat() if step.start_time else None,
            "end_time": step.end_time.isoformat() if step.end_time else None,
            "output_file": output.get("output_file"),
            "output_keys": list(output.keys()),
        }

    def _emit_progress(
        self,
        result: WorkflowResult,
        event: str,
        *,
        current_step: Optional[WorkflowStep] = None,
        step_index: Optional[int] = None,
        total_steps: Optional[int] = None,
    ) -> None:
        """Send a best-effort progress update to the registered callback."""
        if not self._progress_callback:
            return

        steps_snapshot = [
            self._serialize_step_for_progress(step) for step in result.steps
        ]
        if current_step is not None:
            steps_snapshot.append(self._serialize_step_for_progress(current_step))

        completed_steps = sum(
            1 for step in steps_snapshot if step["status"] == StepStatus.COMPLETED.value
        )
        skipped_steps = sum(
            1 for step in steps_snapshot if step["status"] == StepStatus.SKIPPED.value
        )
        failed_steps = sum(
            1 for step in steps_snapshot if step["status"] == StepStatus.FAILED.value
        )
        total_step_count = (
            total_steps
            if total_steps is not None
            else max(len(steps_snapshot), result.total_steps)
        )
        success_rate = 0.0
        if total_step_count:
            success_rate = ((completed_steps + skipped_steps) / total_step_count) * 100

        payload = {
            "event": event,
            "workflow_name": result.workflow_name,
            "workflow_status": result.status.value,
            "timestamp": datetime.now().isoformat(),
            "started_at": result.start_time.isoformat(),
            "finished_at": result.end_time.isoformat() if result.end_time else None,
            "step_index": step_index,
            "total_steps": total_step_count,
            "completed_steps": completed_steps,
            "skipped_steps": skipped_steps,
            "failed_steps": failed_steps,
            "success_rate": success_rate,
            "error_message": result.error_message,
            "current_step": (
                self._serialize_step_for_progress(current_step)
                if current_step
                else None
            ),
            "steps": steps_snapshot,
        }

        try:
            self._progress_callback(payload)
        except Exception as exc:
            logger.warning(f"Workflow progress callback failed: {exc}")

    def _write_step_artifact(
        self,
        workflow_name: str,
        step: WorkflowStep,
        resolved_inputs: Dict[str, Any],
        step_index: int,
    ) -> Optional[str]:
        """Persist step inputs/outputs for real executor runs under .output."""
        if self.get_executor_mode() != "real":
            return None

        output_root = self._get_output_root()
        base_dir = output_root or ".output"
        artifact_dir = os.path.join(
            base_dir, "workflow_artifacts", workflow_name.replace("/", "_")
        )
        os.makedirs(artifact_dir, exist_ok=True)

        safe_step_name = "".join(
            c if c.isalnum() or c in ("-", "_") else "_"
            for c in step.name.replace(" ", "_")
        )
        filename = f"{step_index:02d}_{safe_step_name}.json"
        artifact_path = os.path.join(artifact_dir, filename)

        payload = {
            "workflow": workflow_name,
            "step": step.name,
            "executor": step.skill_executor,
            "action": step.action,
            "status": step.status.value,
            "inputs": resolved_inputs,
            "output": step.output,
            "error": step.error,
            "artifact_index": step_index,
            "mode": self.get_executor_mode(),
            "started_at": step.start_time.isoformat() if step.start_time else None,
            "ended_at": step.end_time.isoformat() if step.end_time else None,
        }

        if not self._validate_artifact_payload(payload):
            return None

        try:
            with open(artifact_path, "w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2, ensure_ascii=False)
            return artifact_path
        except Exception as exc:
            logger.warning(f"无法写入步骤产物 {artifact_path}: {exc}")
            return None

    def _validate_artifact_payload(self, payload: Dict[str, Any]) -> bool:
        """Return True if artifact payload satisfies the minimal schema."""
        missing = [
            field
            for field in _ARTIFACT_REQUIRED_FIELDS
            if field not in payload or payload[field] is None
        ]
        if missing:
            logger.error(
                "Artifact payload missing required fields: %s", ", ".join(missing)
            )
            return False

        if payload["status"] not in {status.value for status in StepStatus}:
            logger.error("Artifact status '%s' is not a valid step state", payload["status"])
            return False

        if not isinstance(payload["workflow"], str) or not payload["workflow"].strip():
            logger.error("Artifact workflow name is invalid: %s", payload["workflow"])
            return False

        if not isinstance(payload["step"], str) or not payload["step"].strip():
            logger.error("Artifact step name is invalid: %s", payload["step"])
            return False

        if not isinstance(payload["inputs"], dict):
            logger.error("Artifact inputs must be a dict, but received %s", type(payload["inputs"]))
            return False

        if not isinstance(payload["output"], (dict, list)):
            logger.error("Artifact output must be dict or list, but received %s", type(payload["output"]))
            return False

        if not isinstance(payload["mode"], str):
            logger.error("Artifact mode must be a string, got %s", type(payload["mode"]))
            return False

        return True
    def execute_workflow(
        self,
        name: str,
        parameters: Optional[Dict[str, Any]] = None,
        use_real: Optional[bool] = None,
        progress_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
    ) -> WorkflowResult:
        """Execute a workflow by name with optional parameters

        Args:
            name: Name of the workflow to execute
            parameters: Parameters to pass to the workflow
            use_real: If specified, temporarily switch to real/mock executors
            progress_callback: Optional callback to receive live progress events

        Returns:
            WorkflowResult object with execution details
        """
        # Temporarily switch mode if requested
        original_mode = self._use_real_executors
        original_progress_callback = self._progress_callback
        self._progress_callback = progress_callback
        if use_real is not None:
            self._use_real_executors = use_real

        result = None
        try:
            workflow = self.get_workflow(name)
            if not workflow:
                result = WorkflowResult(
                    workflow_name=name,
                    status=WorkflowStatus.FAILED,
                    error_message=f"Workflow '{name}' not found",
                )
                result.end_time = datetime.now()
                return result

            # Validate workflow
            is_valid, message = self.validate_workflow(name)
            if not is_valid:
                result = WorkflowResult(
                    workflow_name=name,
                    status=WorkflowStatus.FAILED,
                    error_message=f"Workflow validation failed: {message}",
                )
                result.end_time = datetime.now()
                return result

            # Initialize execution context
            self._execution_context = parameters or {}
            try:
                self._execution_context["execution_strategy"] = (
                    self._get_execution_strategy()
                )
            except ValueError as exc:
                result = WorkflowResult(
                    workflow_name=name,
                    status=WorkflowStatus.FAILED,
                    error_message=str(exc),
                )
                result.end_time = datetime.now()
                return result
            output_root = self._get_output_root()
            if output_root:
                self._execution_context["output_root"] = output_root

            # Create result object
            result = WorkflowResult(workflow_name=name, status=WorkflowStatus.RUNNING)
            total_steps = len(workflow["steps"])
            self._emit_progress(result, "workflow_started", total_steps=total_steps)

            # Execute steps
            for step_index, step_def in enumerate(workflow["steps"], start=1):
                step = self._execute_step(
                    step_def,
                    result,
                    workflow_name=name,
                    step_index=step_index,
                    total_steps=total_steps,
                )
                result.steps.append(step)

                # Stop if step failed
                if step.status == StepStatus.FAILED:
                    result.status = WorkflowStatus.FAILED
                    result.error_message = (
                        f"Workflow failed at step '{step.name}': {step.error}"
                    )
                    self._mark_remaining_steps_skipped(
                        workflow,
                        result,
                        workflow_name=name,
                        start_index=step_index + 1,
                    )
                    break

            # Mark workflow as completed if all steps succeeded
            if result.status == WorkflowStatus.RUNNING:
                result.status = WorkflowStatus.COMPLETED

            result.end_time = datetime.now()
            self._emit_progress(result, "workflow_finished", total_steps=total_steps)
            return result

        finally:
            # Save logs to disk
            if result:
                self._save_workflow_log(result)

            # Restore original mode
            self._use_real_executors = original_mode
            self._progress_callback = original_progress_callback

    def _save_workflow_log(self, result: WorkflowResult) -> None:
        """Save workflow execution log to .output/ directory"""
        try:
            timestamp = result.start_time.strftime("%Y%m%d_%H%M%S")
            log_filename = f"workflow_log_{result.workflow_name}_{timestamp}.json"
            log_dir = ".output"
            output_root = self._get_output_root()
            if output_root:
                log_dir = os.path.join(output_root, ".output")
            log_path = os.path.join(log_dir, log_filename)

            # Ensure output log directory exists
            os.makedirs(log_dir, exist_ok=True)

            result_dict = result.to_dict()
            artifacts = [
                {"step": step.name, "artifact_path": step.artifact_path}
                for step in result.steps
                if step.artifact_path
            ]
            if artifacts:
                result_dict["artifacts"] = artifacts

            with open(log_path, "w", encoding="utf-8") as f:
                json.dump(result_dict, f, indent=2, ensure_ascii=False)
            result.log_path = log_path
            logger.info(f"Workflow execution log saved to: {log_path}")
        except Exception as e:
            logger.error(f"Failed to save workflow log: {e}")

    def _execute_step(
        self,
        step_def: Dict[str, Any],
        result: WorkflowResult,
        *,
        workflow_name: str,
        step_index: Optional[int] = None,
        total_steps: Optional[int] = None,
    ) -> WorkflowStep:
        """Execute a single step in the workflow with support for skipping if output exists"""
        step = WorkflowStep(
            name=step_def["name"],
            skill_executor=step_def["skill_executor"],
            action=step_def["action"],
            inputs=step_def.get("inputs", {}),
        )

        step.status = StepStatus.RUNNING
        step.start_time = datetime.now()
        self._emit_progress(
            result,
            "step_started",
            current_step=step,
            step_index=step_index,
            total_steps=total_steps,
        )

        resolved_inputs: Dict[str, Any] = dict(step.inputs)

        try:
            # Resolve variable references in inputs
            resolved_inputs = self._resolve_variables(step.inputs, result)
            if "output_file" in resolved_inputs:
                resolved_inputs["output_file"] = self._resolve_output_file_path(
                    resolved_inputs["output_file"]
                )

            fail_at_step = self._execution_context.get("fail_at")
            if isinstance(fail_at_step, str) and fail_at_step == step.name:
                raise RuntimeError("Simulated failure")

            # Check for existing output file to support resuming/skipping
            output_file = resolved_inputs.get("output_file")
            execution_strategy = self._get_execution_strategy()
            skip_if_exists = execution_strategy == "resume"

            if (
                skip_if_exists
                and output_file
                and os.path.exists(output_file)
                and os.path.getsize(output_file) > 0
            ):
                logger.info(
                    f"Step '{step.name}' output already exists at {output_file}. Skipping step."
                )
                step.status = StepStatus.SKIPPED
                step.output = {
                    "status": "success",
                    "action": step.action,
                    "output_file": output_file,
                    "message": f"Skipped: Output already exists at {output_file}",
                    "skipped": True,
                }
                # Try to add more metadata if it's a JSON config
                if output_file.endswith(".json"):
                    try:
                        with open(output_file, "r", encoding="utf-8") as f:
                            data = json.load(f)
                            # Propagate keys that might be needed by downstream steps
                            for k in ["template", "robot_name", "format"]:
                                if k in data:
                                    step.output[k] = data[k]
                    except Exception:
                        pass

                step.end_time = datetime.now()
                return step

            # Get executor and execute
            executor = self._get_executor(step.skill_executor)
            if not executor:
                raise ValueError(
                    f"Executor '{step.skill_executor}' not found in {self.get_executor_mode()} mode"
                )

            # Execute the skill
            output = executor.execute(step.action, resolved_inputs)

            # Check if execution failed
            if isinstance(output, dict) and output.get("status") == "error":
                step.status = StepStatus.FAILED
                step.error = output.get("error", "Unknown error")
                step.output = output
            else:
                step.output = output
                step.status = StepStatus.COMPLETED

        except Exception as e:
            step.status = StepStatus.FAILED
            step.error = str(e)

        finally:
            step.end_time = datetime.now()
            artifact_index = step_index or result.total_steps or 0
            artifact_path = self._write_step_artifact(
                workflow_name, step, resolved_inputs, artifact_index
            )
            if artifact_path:
                step.artifact_path = artifact_path
            self._emit_progress(
                result,
                "step_finished",
                current_step=step,
                step_index=step_index,
                total_steps=total_steps,
            )

        return step

    def _resolve_variables(
        self, inputs: Dict[str, Any], result: WorkflowResult
    ) -> Dict[str, Any]:
        """Resolve variable references in step inputs, including from execution context"""
        resolved = {}

        for key, value in inputs.items():
            # First, check if there's an override in execution context
            if key in self._execution_context:
                resolved[key] = self._execution_context[key]
                continue

            if isinstance(value, str):
                # Check for {step_name.output_key} references
                if value.startswith("{") and value.endswith("}"):
                    ref = value[1:-1]  # Remove braces
                    if "." in ref:
                        step_name, output_key = ref.split(".", 1)
                        # Find the step in result
                        for completed_step in result.steps:
                            if completed_step.name == step_name:
                                if output_key in completed_step.output:
                                    resolved[key] = completed_step.output[output_key]
                                else:
                                    # Fallback: use the value as-is if key not found
                                    resolved[key] = value
                                break
                        else:
                            # Step not found, use value as-is
                            resolved[key] = value
                    else:
                        resolved[key] = value
                else:
                    resolved[key] = value
            elif isinstance(value, dict):
                resolved[key] = self._resolve_variables(value, result)
            elif isinstance(value, list):
                resolved[key] = [
                    self._resolve_variables(v, result) if isinstance(v, dict) else v
                    for v in value
                ]
            else:
                resolved[key] = value

        return resolved


    def _mark_remaining_steps_skipped(
        self,
        workflow: Dict[str, Any],
        result: WorkflowResult,
        *,
        workflow_name: str,
        start_index: int,
    ) -> None:
        """Mark the remaining workflow steps as skipped after a failure"""

        total_steps = len(workflow.get("steps", []))
        for index in range(start_index, total_steps + 1):
            step_def = workflow["steps"][index - 1]
            skipped_step = WorkflowStep(
                name=step_def["name"],
                skill_executor=step_def["skill_executor"],
                action=step_def["action"],
                inputs=step_def.get("inputs", {}),
            )
            skipped_step.status = StepStatus.SKIPPED
            skipped_step.start_time = datetime.now()
            skipped_step.end_time = datetime.now()
            skipped_step.output = {
                "status": "skipped",
                "message": "Skipped due to previous failure",
                "action": skipped_step.action,
            }
            skipped_step.error = "Skipped after failure"
            artifact_path = self._write_step_artifact(
                workflow_name, skipped_step, skipped_step.inputs, index
            )
            if artifact_path:
                skipped_step.artifact_path = artifact_path
            result.steps.append(skipped_step)
            self._emit_progress(
                result,
                "step_skipped",
                current_step=skipped_step,
                step_index=index,
                total_steps=total_steps,
            )

# Global orchestrator instance
_orchestrator_instance: Optional[WorkflowOrchestrator] = None


def get_workflow_orchestrator() -> WorkflowOrchestrator:
    """Get or create the global workflow orchestrator instance"""
    global _orchestrator_instance
    if _orchestrator_instance is None:
        _orchestrator_instance = WorkflowOrchestrator()
    return _orchestrator_instance
