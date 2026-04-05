"""
Workflow Orchestration Engine for AGI-Walker Skills System

Provides workflow templates, orchestration, and execution management for
coordinating complex multi-step Skills workflows with state tracking,
error handling, and context-based variable resolution.
"""

import logging
import concurrent.futures
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
import json
import os
import time
from typing import Any, Callable, Dict, List, Optional

# V2.1 TaskGraph Integration
from agi_walker.core.api.task_planning import TaskGraph, TaskNode, TaskNodeStatus

logger = logging.getLogger(__name__)

# --- V2.1 Industrial Hardening Exceptions ---

class WorkflowBaseError(Exception):
    """Base class for workflow-related errors"""
    pass

class EnvironmentalError(WorkflowBaseError):
    """Errors caused by external environment. Retryable."""
    pass

class LogicError(WorkflowBaseError):
    """Errors caused by invalid input or algorithm logic. Non-retryable."""
    pass

class WorkflowTimeoutError(WorkflowBaseError):
    """Errors raised when a step execution exceeds its allowed time."""
    pass

class WorkflowStateStore:
    """Handles persistence of workflow execution states"""
    def __init__(self, base_dir: str = ".output/workflow_states"):
        self.base_dir = base_dir
        os.makedirs(base_dir, exist_ok=True)

    def save_state(self, run_id: str, result: "WorkflowResult"):
        path = os.path.join(self.base_dir, f"{run_id}.json")
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(result.to_dict(), f, indent=2, ensure_ascii=False)
        except Exception as e:
            logger.warning(f"Failed to save workflow state for {run_id}: {e}")

    def load_state(self, run_id: str) -> Optional[Dict[str, Any]]:
        path = os.path.join(self.base_dir, f"{run_id}.json")
        if os.path.exists(path):
            try:
                with open(path, "r", encoding="utf-8") as f:
                    return json.load(f)
            except Exception:
                return None
        return None

@dataclass
class StepPolicy:
    """Execution policy for a single workflow step"""
    timeout: Optional[float] = None
    max_retries: int = 0
    retry_delay: float = 1.0
    retry_backoff: float = 2.0

_ARTIFACT_REQUIRED_FIELDS = {"workflow", "step", "executor", "action", "status", "inputs", "output", "mode"}

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
    skill_executor: str
    action: str
    inputs: Dict[str, Any] = field(default_factory=dict)
    status: StepStatus = StepStatus.PENDING
    output: Dict[str, Any] = field(default_factory=dict)
    error: Optional[str] = None
    error_type: Optional[str] = None
    attempts: int = 0
    policy: StepPolicy = field(default_factory=StepPolicy)
    artifact_path: Optional[str] = None
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None

    @property
    def duration(self) -> float:
        if self.start_time and self.end_time:
            return (self.end_time - self.start_time).total_seconds()
        return 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "executor": self.skill_executor,
            "action": self.action,
            "status": self.status.value,
            "duration": self.duration,
            "output": self.output,
            "error": self.error,
            "error_type": self.error_type,
            "attempts": self.attempts,
            "start_time": self.start_time.isoformat() if self.start_time else None,
            "end_time": self.end_time.isoformat() if self.end_time else None,
            "artifact_path": self.artifact_path,
        }

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
    graph_data: Optional[Dict[str, Any]] = None

    @property
    def duration(self) -> float:
        if self.end_time:
            return (self.end_time - self.start_time).total_seconds()
        return 0.0

    @property
    def total_steps(self) -> int: return len(self.steps)
    @property
    def completed_steps(self) -> int: return sum(1 for s in self.steps if s.status == StepStatus.COMPLETED)
    @property
    def skipped_steps(self) -> int: return sum(1 for s in self.steps if s.status == StepStatus.SKIPPED)
    @property
    def failed_steps(self) -> int: return sum(1 for s in self.steps if s.status == StepStatus.FAILED)
    @property
    def successful_steps(self) -> int: return self.completed_steps + self.skipped_steps
    @property
    def success_rate(self) -> float:
        if not self.total_steps: return 0.0
        return (self.successful_steps / self.total_steps) * 100

    def to_dict(self) -> Dict[str, Any]:
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
            "start_time": self.start_time.isoformat(),
            "end_time": self.end_time.isoformat() if self.end_time else None,
            "graph_data": self.graph_data,
            "steps": [s.to_dict() for s in self.steps],
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
        self._progress_callback: Optional[Callable] = None
        self._use_real_executors = use_real_executors
        self._state_store = WorkflowStateStore()
        self._register_builtin_workflows()
        self._register_builtin_executors()

    def _register_builtin_workflows(self):
        self.workflows["robot_creation_pipeline"] = {
            "name": "robot_creation_pipeline",
            "steps": [
                {"name": "create_model", "skill_executor": "robot_modeling", "action": "create_from_template", "inputs": {"template": "biped_basic", "output_file": ".output/created_robot.json"}},
                {"name": "optimize_params", "skill_executor": "parameter_optimizer", "action": "optimize_mass_distribution", "inputs": {"robot_config": "{create_model.output_file}", "output_file": ".output/optimized_robot.json", "target_com_height": 0.4}},
                {"name": "export_urdf", "skill_executor": "urdf_generator", "action": "export_to_format", "inputs": {"robot_config": "{optimize_params.output_file}", "output_format": "urdf", "output_file": "exports/robot.urdf"}},
            ]
        }
        self.workflows["simulation_ready_robot"] = {
            "name": "simulation_ready_robot",
            "steps": [
                {"name": "load_model", "skill_executor": "robot_modeling", "action": "load_config", "inputs": {"config_file": "configs/tutorial_01_biped.json", "output_file": ".output/sim_robot.json"}},
                {"name": "validate_physics", "skill_executor": "parameter_optimizer", "action": "validate_physics", "inputs": {"robot_config": "{load_model.output_file}", "output_file": ".output/validated_robot.json"}},
                {"name": "export_for_sim", "skill_executor": "urdf_generator", "action": "export_to_format", "inputs": {"robot_config": "{validate_physics.output_file}", "output_format": "sdf", "output_file": "exports/robot_sim.sdf"}},
            ]
        }

    def _register_builtin_executors(self):
        try:
            from agi_walker.skill_executors import get_skill_executor
            for name in ["robot_modeling", "parameter_optimizer", "urdf_generator"]:
                try: self._skill_executors[name] = get_skill_executor(name, use_real=False)
                except Exception: pass
                try: self._real_skill_executors[name] = get_skill_executor(name, use_real=True)
                except Exception: pass
        except ImportError: self._setup_mock_executors()

    def _setup_mock_executors(self):
        class MockExecutor:
            def __init__(self, name): self.name = name
            def execute(self, action, inputs):
                if inputs.get("simulate_timeout"): time.sleep(inputs["simulate_timeout"] + 1)
                if inputs.get("simulate_env_error"): return {"status": "error", "error": f"Simulated env error: {inputs['simulate_env_error']}"}
                return {"status": "success", "action": action, "output_file": inputs.get("output_file", f"output_{self.name}.json")}
        for name in ["robot_modeling", "parameter_optimizer", "urdf_generator"]:
            self._skill_executors[name] = MockExecutor(name)

    def list_workflows(self) -> List[str]: return list(self.workflows.keys())
    def get_workflow(self, name: str) -> Optional[Dict[str, Any]]: return self.workflows.get(name)

    def validate_workflow(self, name: str) -> tuple[bool, str]:
        workflow = self.get_workflow(name)
        if not workflow: return False, f"Workflow '{name}' not found"
        return True, "Workflow is valid"

    def create_custom_workflow(self, name: str, steps: List[Dict], description: str = "") -> bool:
        self.workflows[name] = {"name": name, "description": description, "steps": steps}
        return True

    def get_executor_mode(self) -> str: return "real" if self._use_real_executors else "mock"
    def _get_executor(self, name: str):
        if self._use_real_executors: return self._real_skill_executors.get(name) or self._skill_executors.get(name)
        return self._skill_executors.get(name)

    def _get_execution_strategy(self) -> str:
        strategy = self._execution_context.get("execution_strategy")
        if strategy is None:
            if "skip_if_exists" in self._execution_context:
                return "resume" if self._execution_context["skip_if_exists"] else "force"
            return self.DEFAULT_EXECUTION_STRATEGY
        normalized = str(strategy).strip().lower()
        if normalized not in self.VALID_EXECUTION_STRATEGIES:
            raise ValueError(f"Invalid execution_strategy '{strategy}'")
        return normalized

    def _resolve_output_file_path(self, output_file: Any) -> Any:
        if not isinstance(output_file, str) or not output_file: return output_file
        root = self._execution_context.get("output_root")
        if not root or os.path.isabs(output_file): return output_file
        return os.path.normpath(os.path.join(root, output_file))

    def _emit_progress(self, result: WorkflowResult, event: str, **kwargs) -> None:
        if not self._progress_callback: return
        try:
            payload = {
                "event": event,
                "workflow_name": result.workflow_name,
                "workflow_status": result.status.value,
                "timestamp": datetime.now().isoformat(),
                "completed_steps": result.completed_steps,
                "skipped_steps": result.skipped_steps,
                "failed_steps": result.failed_steps,
                "success_rate": result.success_rate,
                "steps": [s.to_dict() for s in result.steps],
                **kwargs
            }
            self._progress_callback(payload)
        except Exception: pass

    def _write_step_artifact(self, workflow_name: str, step: WorkflowStep, inputs: Dict, index: int) -> Optional[str]:
        if self.get_executor_mode() != "real": return None
        base_dir = self._execution_context.get("output_root") or ".output"
        artifact_dir = os.path.join(base_dir, "workflow_artifacts", workflow_name.replace("/", "_"))
        os.makedirs(artifact_dir, exist_ok=True)
        path = os.path.join(artifact_dir, f"{index:02d}_{step.name}.json")
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump({"workflow": workflow_name, "step": step.name, "inputs": inputs, "output": step.output}, f, indent=2)
            return path
        except Exception: return None

    def execute_workflow(self, name: str, parameters: Optional[Dict] = None, use_real: Optional[bool] = None, progress_callback: Optional[Callable] = None) -> WorkflowResult:
        orig_mode = self._use_real_executors
        if use_real is not None: self._use_real_executors = use_real
        self._progress_callback, self._execution_context = progress_callback, parameters or {}
        try: self._get_execution_strategy()
        except ValueError as exc:
            res = WorkflowResult(name, WorkflowStatus.FAILED, error_message=str(exc))
            res.end_time = datetime.now()
            return res
        workflow = self.get_workflow(name)
        if not workflow: return WorkflowResult(name, WorkflowStatus.FAILED, error_message="Workflow not found")
        result = WorkflowResult(workflow_name=name, status=WorkflowStatus.RUNNING)
        total_steps = len(workflow["steps"])
        self._emit_progress(result, "workflow_started", total_steps=total_steps)
        for i, step_def in enumerate(workflow["steps"], 1):
            step = self._execute_step(step_def, result, name, i, total_steps)
            result.steps.append(step)
            self._emit_progress(result, "step_finished", current_step=step.to_dict(), step_index=i, total_steps=total_steps)
            if step.status == StepStatus.FAILED:
                result.status = WorkflowStatus.FAILED
                break
        if result.status == WorkflowStatus.RUNNING: result.status = WorkflowStatus.COMPLETED
        result.end_time = datetime.now()
        self._emit_progress(result, "workflow_finished")
        self._use_real_executors = orig_mode
        return result

    def _execute_step(self, step_def: Dict, result: WorkflowResult, wf_name: str, index: int, total: int) -> WorkflowStep:
        policy = StepPolicy(**step_def.get("policy", {}))
        step = WorkflowStep(name=step_def["name"], skill_executor=step_def["skill_executor"], action=step_def["action"], inputs=step_def.get("inputs", {}), policy=policy)
        step.status, step.start_time = StepStatus.RUNNING, datetime.now()
        self._emit_progress(result, "step_started", current_step=step.to_dict(), step_index=index, total_steps=total)
        resolved = self._resolve_variables(step.inputs, result)
        if "output_file" in resolved: resolved["output_file"] = self._resolve_output_file_path(resolved["output_file"])
        if self._get_execution_strategy() == "resume" and resolved.get("output_file") and os.path.exists(resolved["output_file"]):
            step.status, step.output = StepStatus.SKIPPED, {"status": "success", "skipped": True}
            step.end_time = datetime.now()
            return step
        max_att = policy.max_retries + 1
        for att in range(1, max_att + 1):
            step.attempts = att
            try:
                executor = self._get_executor(step.skill_executor)
                with concurrent.futures.ThreadPoolExecutor(max_workers=1) as pool:
                    output = pool.submit(executor.execute, step.action, resolved).result(timeout=policy.timeout)
                if isinstance(output, dict) and output.get("status") == "error": raise EnvironmentalError(output.get("error"))
                step.output, step.status = output, StepStatus.COMPLETED
                break
            except Exception as e:
                step.error = str(e)
                if att == max_att: step.status = StepStatus.FAILED
        step.end_time = datetime.now()
        step.artifact_path = self._write_step_artifact(wf_name, step, resolved, index)
        return step

    def _resolve_variables(self, inputs: Any, result: WorkflowResult) -> Any:
        if isinstance(inputs, dict): return {k: self._resolve_variables(v, result) for k, v in inputs.items()}
        if isinstance(inputs, list): return [self._resolve_variables(v, result) for v in inputs]
        if isinstance(inputs, str) and inputs.startswith("{") and inputs.endswith("}"):
            ref = inputs[1:-1]
            if "." in ref:
                step_name, key = ref.split(".", 1)
                for s in result.steps:
                    if s.name == step_name and key in s.output: return s.output[key]
        return inputs

    def execute_task_graph(self, graph: TaskGraph, parameters: Optional[Dict] = None, use_real: Optional[bool] = None, progress_callback: Optional[Callable] = None) -> WorkflowResult:
        orig_mode = self._use_real_executors
        if use_real is not None: self._use_real_executors = use_real
        self._progress_callback, self._execution_context = progress_callback, parameters or {}
        result = WorkflowResult(workflow_name="task_graph", status=WorkflowStatus.RUNNING)
        total_nodes = len(graph.nodes)
        self._emit_progress(result, "workflow_started", total_steps=total_nodes)
        try:
            with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
                while True:
                    runnable = graph.get_runnable_nodes()
                    if not runnable:
                        # V2.1 Refinement: Determine if we finished successfully or stalled
                        executed_steps = len(result.steps)
                        any_failed = any(n.status == TaskNodeStatus.FAILURE for n in graph.nodes.values())
                        
                        # Mark untaken branches as SKIPPED for clearer auditing
                        for n in graph.nodes.values():
                            if n.status == TaskNodeStatus.PENDING:
                                n.status = TaskNodeStatus.SKIPPED

                        if executed_steps == 0 and len(graph.nodes) > 0:
                            # Not a single node could be started -> Deadlock or Cycle detected
                            result.status = WorkflowStatus.FAILED
                            result.error_message = "TaskGraph stalled: no nodes are runnable. Check for cycles."
                        elif any_failed:
                            result.status = WorkflowStatus.FAILED
                        else:
                            result.status = WorkflowStatus.COMPLETED
                        break
                    futures = {pool.submit(self._execute_step, {"name": n.name, "skill_executor": n.skill, "action": n.action, "inputs": n.params}, result, "graph", 0, total_nodes): n for n in runnable}
                    for f in concurrent.futures.as_completed(futures):
                        node, step = futures[f], f.result()
                        node.status = TaskNodeStatus.SUCCESS if step.status == StepStatus.COMPLETED else TaskNodeStatus.FAILURE
                        result.steps.append(step)
                        self._emit_progress(result, "step_finished", current_step=step.to_dict())
            result.end_time = datetime.now()
            self._emit_progress(result, "workflow_finished")
            return result
        finally: self._use_real_executors = orig_mode

_orchestrator_instance = None
def get_workflow_orchestrator():
    global _orchestrator_instance
    if _orchestrator_instance is None: _orchestrator_instance = WorkflowOrchestrator()
    return _orchestrator_instance
