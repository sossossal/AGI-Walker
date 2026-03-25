"""
Workflow Orchestration Engine for AGI-Walker Skills System

Provides workflow templates, orchestration, and execution management for
coordinating complex multi-step Skills workflows with state tracking,
error handling, and context-based variable resolution.
"""

import logging
logger = logging.getLogger(__name__)
from enum import Enum
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any
import time
from datetime import datetime
import json


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
    
    @property
    def duration(self) -> float:
        """Get total workflow duration in seconds"""
        if self.end_time:
            return (self.end_time - self.start_time).total_seconds()
        return 0.0
    
    @property
    def success_rate(self) -> float:
        """Get percentage of successfully completed steps"""
        if not self.steps:
            return 0.0
        completed = sum(1 for s in self.steps if s.status == StepStatus.COMPLETED)
        return (completed / len(self.steps)) * 100


class WorkflowOrchestrator:
    """Main orchestration engine for Skills workflows"""
    
    def __init__(self, use_real_executors: bool = False):
        self.workflows: Dict[str, Dict[str, Any]] = {}
        self._skill_executors: Dict[str, Any] = {}
        self._real_skill_executors: Dict[str, Any] = {}
        self._execution_context: Dict[str, Any] = {}
        self._use_real_executors = use_real_executors
        
        # Register built-in workflows
        self._register_builtin_workflows()
        # Register built-in Skill executors
        self._register_builtin_executors()
    
    def _register_builtin_workflows(self):
        """Register built-in workflow templates"""
        
        # Robot Creation Pipeline: Model -> Optimize -> Export
        self.workflows['robot_creation_pipeline'] = {
            'name': 'robot_creation_pipeline',
            'description': 'Create a robot model, optimize parameters, and export to URDF',
            'steps': [
                {
                    'name': 'create_model',
                    'skill_executor': 'robot_modeling',
                    'action': 'create_from_template',
                    'inputs': {
                        'template': 'biped_basic',
                        'output_file': '.output/created_robot.json'
                    }
                },
                {
                    'name': 'optimize_params',
                    'skill_executor': 'parameter_optimizer',
                    'action': 'optimize_mass_distribution',
                    'inputs': {
                        'robot_config': '{create_model.output_file}',
                        'output_file': '.output/optimized_robot.json',
                        'target_com_height': 0.4
                    }
                },
                {
                    'name': 'export_urdf',
                    'skill_executor': 'urdf_generator',
                    'action': 'export_to_format',
                    'inputs': {
                        'robot_config': '{optimize_params.output_file}',
                        'output_format': 'urdf',
                        'output_file': 'exports/robot.urdf'
                    }
                }
            ]
        }
        
        # Simulation Ready Pipeline: Model -> Validate -> Export
        self.workflows['simulation_ready_robot'] = {
            'name': 'simulation_ready_robot',
            'description': 'Prepare a robot for simulation with validation',
            'steps': [
                {
                    'name': 'load_model',
                    'skill_executor': 'robot_modeling',
                    'action': 'load_config',
                    'inputs': {
                        'config_file': 'configs/tutorial_01_biped.json',
                        'output_file': '.output/sim_robot.json'
                    }
                },
                {
                    'name': 'validate_physics',
                    'skill_executor': 'parameter_optimizer',
                    'action': 'validate_physics',
                    'inputs': {
                        'robot_config': '{load_model.output_file}',
                        'output_file': '.output/validated_robot.json'
                    }
                },
                {
                    'name': 'export_for_sim',
                    'skill_executor': 'urdf_generator',
                    'action': 'export_to_format',
                    'inputs': {
                        'robot_config': '{validate_physics.output_file}',
                        'output_format': 'sdf',
                        'output_file': 'exports/robot_sim.sdf'
                    }
                }
            ]
        }
    
    def _register_builtin_executors(self):
        """Register built-in Skill executors"""
        try:
            from agi_walker.skill_executors import get_skill_executor
            
            # Load mock executors (always available)
            for executor_name in ['robot_modeling', 'parameter_optimizer', 'urdf_generator']:
                try:
                    executor = get_skill_executor(executor_name, use_real=False)
                    self._skill_executors[executor_name] = executor
                except Exception as e:
                    logger.info(f"Warning: Could not load mock executor {executor_name}: {e}")
            
            # Load real executors (may fail if dependencies missing)
            for executor_name in ['robot_modeling', 'parameter_optimizer', 'urdf_generator']:
                try:
                    executor = get_skill_executor(executor_name, use_real=True)
                    self._real_skill_executors[executor_name] = executor
                except Exception as e:
                    logger.info(f"Warning: Could not load real executor {executor_name}: {e}")
                    
        except ImportError:
            logger.warning("skill_executors module not available - using fallback mock executors")
            self._setup_mock_executors()
    
    def set_executor_mode(self, use_real: bool) -> None:
        """Switch between real and mock executors"""
        self._use_real_executors = use_real
    
    def get_executor_mode(self) -> str:
        """Get current executor mode"""
        return 'real' if self._use_real_executors else 'mock'
    
    def _get_executor(self, name: str):
        """Get executor by name, respecting current mode"""
        if self._use_real_executors:
            return self._real_skill_executors.get(name) or self._skill_executors.get(name)
        else:
            return self._skill_executors.get(name)
    
    def _setup_mock_executors(self):
        """Setup mock executors for testing"""
        class MockExecutor:
            def __init__(self, name):
                self.name = name
            
            def execute(self, action, inputs):
                return {
                    'status': 'success',
                    'action': action,
                    'output_file': inputs.get('output_file', f'output_{self.name}.json')
                }
        
        self._skill_executors['robot_modeling'] = MockExecutor('robot_modeling')
        self._skill_executors['parameter_optimizer'] = MockExecutor('parameter_optimizer')
        self._skill_executors['urdf_generator'] = MockExecutor('urdf_generator')
    
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
        
        if 'steps' not in workflow or not workflow['steps']:
            return False, f"Workflow '{name}' has no steps defined"
        
        for i, step in enumerate(workflow['steps']):
            step_id = f"Step {i} ({step.get('name', 'unnamed')})"
            
            if 'name' not in step:
                return False, f"{step_id} missing 'name' field"
            if 'skill_executor' not in step:
                return False, f"{step_id} missing 'skill_executor' field"
            if 'action' not in step:
                return False, f"{step_id} missing 'action' field"
            
            executor_name = step['skill_executor']
            action = step['action']
            
            # Check if executor exists
            executor = self._skill_executors.get(executor_name)
            if not executor:
                return False, f"{step_id}: Executor '{executor_name}' not registered"
            
            # Check if action is supported (Mandatory check)
            if hasattr(executor, 'get_supported_actions'):
                supported_actions = executor.get_supported_actions()
                if action not in supported_actions:
                    return False, (
                        f"{step_id}: Action '{action}' is not supported by executor '{executor_name}'. "
                        f"Supported actions are: {', '.join(supported_actions)}"
                    )
            else:
                return False, f"{step_id}: Executor '{executor_name}' does not provide a list of supported actions"
        
        return True, "Workflow is valid"
    
    def create_custom_workflow(self, name: str, steps: List[Dict[str, Any]],
                              description: str = "") -> bool:
        """Create and register a custom workflow"""
        workflow = {
            'name': name,
            'description': description,
            'steps': steps
        }
        
        # Validate before registering
        self.workflows[name] = workflow  # Temporarily add for validation
        is_valid, message = self.validate_workflow(name)
        
        if not is_valid:
            del self.workflows[name]  # Remove if invalid
            return False
        
        return True
    
    def execute_workflow(self, name: str, parameters: Optional[Dict[str, Any]] = None, use_real: Optional[bool] = None) -> WorkflowResult:
        """Execute a workflow by name with optional parameters
        
        Args:
            name: Name of the workflow to execute
            parameters: Parameters to pass to the workflow
            use_real: If specified, temporarily switch to real/mock executors
            
        Returns:
            WorkflowResult object with execution details
        """
        # Temporarily switch mode if requested
        original_mode = self._use_real_executors
        if use_real is not None:
            self._use_real_executors = use_real
        
        try:
            workflow = self.get_workflow(name)
            if not workflow:
                result = WorkflowResult(
                    workflow_name=name,
                    status=WorkflowStatus.FAILED,
                    error_message=f"Workflow '{name}' not found"
                )
                result.end_time = datetime.now()
                return result
            
            # Validate workflow
            is_valid, message = self.validate_workflow(name)
            if not is_valid:
                result = WorkflowResult(
                    workflow_name=name,
                    status=WorkflowStatus.FAILED,
                    error_message=f"Workflow validation failed: {message}"
                )
                result.end_time = datetime.now()
                return result
            
            # Initialize execution context
            self._execution_context = parameters or {}
            
            # Create result object
            result = WorkflowResult(workflow_name=name, status=WorkflowStatus.RUNNING)
            
            # Execute steps
            for step_def in workflow['steps']:
                step = self._execute_step(step_def, result)
                result.steps.append(step)
                
                # Stop if step failed
                if step.status == StepStatus.FAILED:
                    result.status = WorkflowStatus.FAILED
                    result.error_message = f"Workflow failed at step '{step.name}': {step.error}"
                    break
            
            # Mark workflow as completed if all steps succeeded
            if result.status == WorkflowStatus.RUNNING:
                result.status = WorkflowStatus.COMPLETED
            
            result.end_time = datetime.now()
            return result
        
        finally:
            # Restore original mode
            self._use_real_executors = original_mode
    
    def _execute_step(self, step_def: Dict[str, Any], result: WorkflowResult) -> WorkflowStep:
        """Execute a single step in the workflow"""
        step = WorkflowStep(
            name=step_def['name'],
            skill_executor=step_def['skill_executor'],
            action=step_def['action'],
            inputs=step_def.get('inputs', {})
        )
        
        step.status = StepStatus.RUNNING
        step.start_time = datetime.now()
        
        try:
            # Resolve variable references in inputs
            resolved_inputs = self._resolve_variables(step.inputs, result)
            
            # Get executor and execute
            executor = self._get_executor(step.skill_executor)
            if not executor:
                raise ValueError(f"Executor '{step.skill_executor}' not found in {self.get_executor_mode()} mode")
            
            # Execute the skill
            output = executor.execute(step.action, resolved_inputs)
            
            # Check if execution failed
            if isinstance(output, dict) and output.get('status') == 'error':
                step.status = StepStatus.FAILED
                step.error = output.get('error', 'Unknown error')
                step.output = output
            else:
                step.output = output
                step.status = StepStatus.COMPLETED
            
        except Exception as e:
            step.status = StepStatus.FAILED
            step.error = str(e)
        
        finally:
            step.end_time = datetime.now()
        
        return step
    
    def _resolve_variables(self, inputs: Dict[str, Any], result: WorkflowResult) -> Dict[str, Any]:
        """Resolve variable references in step inputs, including from execution context"""
        resolved = {}
        
        for key, value in inputs.items():
            # First, check if there's an override in execution context
            if key in self._execution_context:
                resolved[key] = self._execution_context[key]
                continue
            
            if isinstance(value, str):
                # Check for {step_name.output_key} references
                if value.startswith('{') and value.endswith('}'):
                    ref = value[1:-1]  # Remove braces
                    if '.' in ref:
                        step_name, output_key = ref.split('.', 1)
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


# Global orchestrator instance
_orchestrator_instance: Optional[WorkflowOrchestrator] = None


def get_workflow_orchestrator() -> WorkflowOrchestrator:
    """Get or create the global workflow orchestrator instance"""
    global _orchestrator_instance
    if _orchestrator_instance is None:
        _orchestrator_instance = WorkflowOrchestrator()
    return _orchestrator_instance
