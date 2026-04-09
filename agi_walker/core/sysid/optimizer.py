import numpy as np
import os
from scipy.optimize import minimize
import json
from agi_walker.core.sysid.simulator_interface import MockSimulator


class SysIDOptimizer:
    def __init__(self, simulator_interface, real_data_path):
        self.sim = simulator_interface
        self.real_data = self._load_data(real_data_path)

    def _load_data(self, path):
        with open(path, "r") as f:
            return json.load(f)

    def objective_function(self, params):
        """
        params: [friction, damping]
        """
        # 1. Update Simulation Parameters
        self.sim.set_params(friction=params[0], damping=params[1])

        # 2. Run Simulation
        sim_data = self.sim.run_trajectory(
            duration=self.real_data["metadata"]["duration"]
        )

        # 3. Calculate Error (MSE between trajectories)
        # Simplistic error: match final state or average velocity?
        # Ideally match full trajectory points.
        # For this mock, we assume sim_data structure matches real_data["measurements"]

        # Extract arrays for faster compute
        real_pos = np.array(
            [m["state"]["joints"]["hip_left"] for m in self.real_data["measurements"]]
        )
        sim_pos = np.array(
            [m["state"]["joints"]["hip_left"] for m in sim_data["measurements"]]
        )

        # Truncate to matching length
        min_len = min(len(real_pos), len(sim_pos))
        mse = np.mean((real_pos[:min_len] - sim_pos[:min_len]) ** 2)

        print(f"Iter: params={params}, error={mse:.6f}")
        return mse

    def optimize(self, initial_guess=[0.5, 0.1]):
        print(f"🚀 Starting Optimization. Initial Guess: {initial_guess}")

        # Bounds: Friction [0.0, 5.0], Damping [0.0, 2.0]
        bounds = [(0.0, 5.0), (0.0, 2.0)]

        result = minimize(
            self.objective_function,
            initial_guess,
            method="L-BFGS-B",
            bounds=bounds,
            options={"maxiter": 20, "disp": True},
        )

        print("✅ Optimization Complete.")
        print(
            f"   Optimal Params: Friction={result.x[0]:.4f}, Damping={result.x[1]:.4f}"
        )
        print(f"   Final Error: {result.fun:.6f}")
        return result.x


if __name__ == "__main__":
    # Create dummy data file representing "Real" world
    mock_real_sim = MockSimulator()
    # Set to "Real" values (Target)
    mock_real_sim.set_params(1.2, 0.5)

    # We need to adapt the mock call because MockSimulator.run_trajectory returns dict directly
    # In optimizer, we expect simulator_interface to return data
    ground_truth_data = mock_real_sim.run_trajectory()

    if not os.path.exists("sysid/data"):
        os.makedirs("sysid/data")
    with open("sysid/data/mock_real.json", "w") as f:
        json.dump(
            {
                "metadata": {"duration": 1.0},
                "measurements": ground_truth_data["measurements"],
            },
            f,
        )

    # Run Optimizer
    # Initialize with WRONG guess (0.1, 0.1)
    opt = SysIDOptimizer(MockSimulator(), "sysid/data/mock_real.json")
    opt.optimize(initial_guess=[0.1, 0.1])
