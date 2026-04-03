"""
Compatibility wrapper for the parameter-optimizer skill.
"""

import logging
from agi_walker.skills._compat import load_skill_module

logger = logging.getLogger(__name__)


_module = load_skill_module(__name__, "parameter-optimizer")

OptimizationResult = _module.OptimizationResult
PIDGains = _module.PIDGains
MassOptimizationResult = _module.MassOptimizationResult
ParameterOptimizer = _module.ParameterOptimizer
MassDistributionOptimizer = _module.MassDistributionOptimizer
PIDTuner = _module.PIDTuner
optimize_mass_distribution = _module.optimize_mass_distribution
tune_pid_controller = _module.tune_pid_controller
batch_optimize_pid = _module.batch_optimize_pid

__all__ = [
    "OptimizationResult",
    "PIDGains",
    "MassOptimizationResult",
    "ParameterOptimizer",
    "MassDistributionOptimizer",
    "PIDTuner",
    "optimize_mass_distribution",
    "tune_pid_controller",
    "batch_optimize_pid",
]
