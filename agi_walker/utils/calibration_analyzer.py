"""
Sim2Real Calibration Analyzer for AGI-Walker

Performs sensitivity analysis by perturbing physical parameters
and observing their impact on robot stability and workflow results.
"""

import os
import json
import logging
import copy
from typing import Dict, List, Any
from agi_walker.workflow_orchestrator import get_workflow_orchestrator

logger = logging.getLogger(__name__)


class CalibrationAnalyzer:
    def __init__(self):
        self.orchestrator = get_workflow_orchestrator()
        self.base_config_path = "configs/tutorial_01_biped.json"

    def run_sensitivity_test(self, param_name: str, perturbations: List[float]):
        """
        Perturb a parameter and run validation workflow.

        Args:
            param_name: Name of the parameter in config (e.g., 'mass')
            perturbations: List of multipliers (e.g., [0.9, 1.0, 1.1])
        """
        if not os.path.exists(self.base_config_path):
            logger.error(f"Base config not found: {self.base_config_path}")
            return

        with open(self.base_config_path, "r", encoding="utf-8") as f:
            base_config = json.load(f)

        results = []

        print(f"\nSensitivity Analysis for: {param_name}")
        print(f"{'Value':<10} | {'Status':<10} | {'Score':<10}")
        print("-" * 35)

        for p in perturbations:
            # Create perturbed config
            test_config = copy.deepcopy(base_config)

            # Apply perturbation to all parts (simplified for now)
            for part in test_config.get("parts", []):
                if param_name in part.get("params", {}):
                    part["params"][param_name] *= p

            # Save temporary test config
            test_path = f".output/calib_test_{param_name}_{p}.json"
            os.makedirs(".output", exist_ok=True)
            with open(test_path, "w", encoding="utf-8") as f:
                json.dump(test_config, f, indent=2)

            # Run simulation/validation workflow using this specific config
            # We override the input file in the workflow context
            wf_result = self.orchestrator.execute_workflow(
                "simulation_ready_robot",
                parameters={"config_file": test_path},
                use_real=True,
            )

            # Extract score (if available from parameter_optimizer)
            score = 0.0
            for step in wf_result.steps:
                if step.name == "validate_physics":
                    score = step.output.get("validation_results", {}).get(
                        "parts_count", 0
                    )

            print(f"{p:<10.2f} | {wf_result.status.value:<10} | {score:<10.2f}")
            results.append(
                {"perturbation": p, "status": wf_result.status.value, "score": score}
            )

        return results


if __name__ == "__main__":
    analyzer = CalibrationAnalyzer()
    # Test mass sensitivity
    analyzer.run_sensitivity_test("mass", [0.5, 0.8, 1.0, 1.2, 1.5])
