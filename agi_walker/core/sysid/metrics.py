import numpy as np
from typing import Dict, Any


class Sim2RealScorer:
    """
    Calculates a 0-100 score representing the fidelity of the simulation
    compared to the real world (Sim2Real gap).
    """

    def __init__(self, sensitivity: float = 0.5):
        """
        sensitivity: Controls how fast the score drops with error.
                     Higher = stricter.
        """
        self.sensitivity = sensitivity

    def calculate_score(
        self, real_data: Dict[str, Any], sim_data: Dict[str, Any]
    ) -> float:
        """
        Computes the Sim2Real score.

        Args:
            real_data: Dictionary containing 'measurements' list.
            sim_data: Dictionary containing 'measurements' list.

        Returns:
            float: Score between 0.0 and 100.0
        """
        mse, _ = self._compute_error(real_data, sim_data)

        # Normalize MSE logic:
        # If signals are approx range [-1, 1], MSE of 1.0 is huge.
        # We want Score = 100 * exp(-lambda * MSE)

        score = 100.0 * np.exp(-self.sensitivity * mse)
        return float(np.clip(score, 0.0, 100.0))

    def generate_report(
        self, real_data: Dict[str, Any], sim_data: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        Generates a detailed report of the Sim2Real comparison.
        """
        mse, joint_errors = self._compute_error(real_data, sim_data)
        score = self.calculate_score(real_data, sim_data)

        return {
            "overall_score": score,
            "overall_mse": mse,
            "joint_errors": joint_errors,
            "sensitivity_used": self.sensitivity,
        }

    def _compute_error(self, real_data, sim_data):
        # Extract trajectories
        real_measurements = real_data.get("measurements", [])
        sim_measurements = sim_data.get("measurements", [])

        # Align lengths
        min_len = min(len(real_measurements), len(sim_measurements))
        if min_len == 0:
            return 1e9, {}  # Infinite error if empty

        # We look at joint positions for now
        # Assumes structure: m['state']['joints']['name']

        joint_names = real_measurements[0]["state"]["joints"].keys()

        total_sq_error = 0.0
        count = 0
        joint_errors = {}

        for name in joint_names:
            r_vals = np.array(
                [m["state"]["joints"][name] for m in real_measurements[:min_len]]
            )
            s_vals = np.array(
                [m["state"]["joints"][name] for m in sim_measurements[:min_len]]
            )

            # Mean Squared Error for this joint
            mse_joint = np.mean((r_vals - s_vals) ** 2)
            joint_errors[name] = float(mse_joint)

            total_sq_error += mse_joint
            count += 1

        avg_mse = total_sq_error / max(1, count)
        return avg_mse, joint_errors
