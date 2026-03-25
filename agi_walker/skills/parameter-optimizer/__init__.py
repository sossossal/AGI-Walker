"""
Parameter Optimizer Skill - 机器人参数优化工具

提供质量分布优化、PID调优、多目标优化等功能。
"""

import logging
logger = logging.getLogger(__name__)
import json
import numpy as np
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
from scipy.optimize import minimize, differential_evolution


@dataclass
class OptimizationResult:
    """优化结果"""

    success: bool
    iterations: int
    final_value: float
    parameters: Dict[str, float]
    message: str


@dataclass
class PIDGains:
    """PID增益"""

    kp: float
    ki: float
    kd: float


@dataclass
class MassOptimizationResult:
    """质量优化结果"""

    mass_distribution: Dict[str, float]
    com_position: np.ndarray
    com_error: float
    iterations: int
    success: bool


class ParameterOptimizer:
    """参数优化器基类"""

    def __init__(self, robot_config: Any) -> None:
        """初始化优化器

        Args:
            robot_config: 机器人配置 (字典或RobotConfig对象)
        """
        if isinstance(robot_config, (str, Path)):
            with open(robot_config, "r", encoding="utf-8") as f:
                self.config = json.load(f)
        elif isinstance(robot_config, dict):
            self.config = robot_config
        else:
            # 假设是RobotConfig对象
            self.config = robot_config.to_dict()

        self.parts = self.config.get("parts", [])
        self.connections = self.config.get("connections", [])
        self.metadata = self.config.get("metadata", {})

    def _build_position_map(self) -> Dict[str, np.ndarray]:
        """构建部件位置映射

        根据连接关系计算每个部件的相对位置。

        Returns:
            部件ID到位置的映射
        """
        positions = {}

        # 找到根部件 (torso)
        root_parts = [p for p in self.parts if "torso" in p["id"].lower()]
        if root_parts:
            root_id = root_parts[0]["id"]
            positions[root_id] = np.array([0.0, 0.0, 0.0])
        else:
            # 如果没有torso,使用第一个部件作为根
            positions[self.parts[0]["id"]] = np.array([0.0, 0.0, 0.0])

        # 基于连接关系计算其他部件位置
        for conn in self.connections:
            parent_id = conn["from"]
            child_id = conn["to"]

            # 简化模型: 根据部件类型估计偏移
            if parent_id in positions:
                parent_pos = positions[parent_id]

                # 查找子部件信息
                child_part = next((p for p in self.parts if p["id"] == child_id), None)
                if child_part:
                    # 根据部件类型和参数估计位置
                    part_type = child_part["type"]
                    params = child_part["params"]

                    if "leg" in part_type:
                        # 腿部件: 向下偏移
                        leg_length = params.get("thigh_length", 0.3) + params.get(
                            "shin_length", 0.3
                        )
                        offset = np.array([0.0, 0.0, -leg_length / 2])
                    elif "arm" in part_type:
                        # 手臂: 水平偏移
                        arm_length = params.get("upper_arm_length", 0.2) + params.get(
                            "forearm_length", 0.2
                        )
                        offset = np.array([arm_length / 2, 0.0, 0.0])
                    else:
                        # 其他部件: 小偏移
                        offset = np.array([0.0, 0.0, -0.1])

                    positions[child_id] = parent_pos + offset

        # 为未连接的部件分配默认位置
        for part in self.parts:
            if part["id"] not in positions:
                positions[part["id"]] = np.array([0.0, 0.0, 0.0])

        return positions

    def _calculate_com(self, masses: Dict[str, float]) -> np.ndarray:
        """计算重心位置

        Args:
            masses: 部件ID到质量的映射

        Returns:
            重心位置 (x, y, z)
        """
        total_mass = 0.0
        com = np.zeros(3)

        # 构建位置映射
        positions = self._build_position_map()

        for part in self.parts:
            part_id = part["id"]
            mass = masses.get(part_id, part["params"].get("mass", 1.0))
            position = positions.get(part_id, np.array([0.0, 0.0, 0.0]))

            com += mass * position
            total_mass += mass

        if total_mass > 0:
            com /= total_mass

        return com


class MassDistributionOptimizer(ParameterOptimizer):
    """质量分布优化器"""

    def optimize(
        self,
        target_com_height: float,
        max_iterations: int = 100,
        method: str = "gradient",
    ) -> MassOptimizationResult:
        """优化质量分布

        Args:
            target_com_height: 期望重心高度
            max_iterations: 最大迭代次数
            method: 优化方法 ("gradient" 或 "genetic")

        Returns:
            MassOptimizationResult对象
        """
        # 提取初始质量
        initial_masses = {}
        for part in self.parts:
            part_id = part["id"]
            initial_masses[part_id] = part["params"].get("mass", 1.0)

        if len(initial_masses) == 0:
            # 没有部件,返回空结果
            return MassOptimizationResult(
                mass_distribution={},
                com_position=np.zeros(3),
                com_error=0.0,
                iterations=0,
                success=False,
            )

        # 定义目标函数
        def objective(mass_vec) -> None:
            masses = dict(zip(initial_masses.keys(), mass_vec))
            com = self._calculate_com(masses)
            error = (com[2] - target_com_height) ** 2
            # 添加正则化: 惩罚质量变化过大
            mass_change = sum((m - initial_masses[k]) ** 2 for k, m in masses.items())
            return error + 0.01 * mass_change

        # 优化边界 (质量范围: 0.1kg - 20kg)
        bounds = [(0.1, 20.0) for _ in initial_masses]

        # 执行优化
        try:
            if method == "gradient":
                result = minimize(
                    objective,
                    x0=list(initial_masses.values()),
                    method="L-BFGS-B",
                    bounds=bounds,
                    options={"maxiter": max_iterations, "ftol": 1e-6},
                )
            else:  # genetic
                result = differential_evolution(
                    objective,
                    bounds=bounds,
                    maxiter=max_iterations,
                    seed=42,
                    atol=1e-6,
                    tol=1e-6,
                )

            # 构建结果
            optimized_masses = dict(zip(initial_masses.keys(), result.x))
            final_com = self._calculate_com(optimized_masses)

            return MassOptimizationResult(
                mass_distribution=optimized_masses,
                com_position=final_com,
                com_error=abs(final_com[2] - target_com_height),
                iterations=result.nit if hasattr(result, "nit") else max_iterations,
                success=result.success if hasattr(result, "success") else True,
            )

        except Exception as e:
            logger.info(f"优化过程中出错: {e}")
            # 返回初始质量作为fallback
            initial_com = self._calculate_com(initial_masses)
            return MassOptimizationResult(
                mass_distribution=initial_masses,
                com_position=initial_com,
                com_error=abs(initial_com[2] - target_com_height),
                iterations=0,
                success=False,
            )


class PIDTuner(ParameterOptimizer):
    """PID参数调优器"""

    def tune(
        self, joint_name: str, method: str = "ziegler_nichols", **kwargs
    ) -> PIDGains:
        """调优PID增益

        Args:
            joint_name: 关节名称
            method: 调优方法 ("ziegler_nichols" 或 "genetic")
            **kwargs: 额外参数

        Returns:
            PIDGains对象
        """
        if method == "ziegler_nichols":
            return self._ziegler_nichols_tuning(joint_name)
        elif method == "genetic":
            return self._genetic_tuning(joint_name, **kwargs)
        else:
            raise ValueError(f"未知调优方法: {method}")

    def _ziegler_nichols_tuning(self, joint_name: str) -> PIDGains:
        """Ziegler-Nichols调优法

        基于系统特性快速计算PID增益。
        这是一个简化实现,实际应进行阶跃响应实验。
        """
        # 简化: 使用经验公式
        # 实际应通过仿真获取系统响应曲线

        # 假设临界增益和周期 (这里使用经验值)
        ku = 10.0  # 临界增益
        tu = 0.5  # 临界周期

        # Ziegler-Nichols公式 (PID)
        kp = 0.6 * ku
        ki = 1.2 * ku / tu
        kd = 0.075 * ku * tu

        logger.info(f"[Ziegler-Nichols] {joint_name}: Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.2f}")

        return PIDGains(kp=kp, ki=ki, kd=kd)

    def _genetic_tuning(
        self,
        joint_name: str,
        population_size: int = 50,
        generations: int = 100,
        fitness_metric: str = "ise",
    ) -> PIDGains:
        """遗传算法调优

        Args:
            joint_name: 关节名称
            population_size: 种群大小
            generations: 迭代代数
            fitness_metric: 适应度指标 (ise/iae/itae)
        """

        # 定义适应度函数 (需要仿真环境)
        def fitness(gains) -> None:
            kp, ki, kd = gains
            # TODO: 实际应运行仿真并计算误差积分
            # 这里使用简化的启发式评分
            error = abs(kp - 6.0) + abs(ki - 2.0) + abs(kd - 0.5)
            return error

        # 优化边界
        bounds = [(0.1, 20.0), (0.01, 10.0), (0.01, 5.0)]

        # 执行遗传算法
        result = differential_evolution(
            fitness,
            bounds=bounds,
            maxiter=generations,
            popsize=population_size,
            seed=42,
        )

        kp, ki, kd = result.x
        logger.info(f"[Genetic] {joint_name}: Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.2f}")

        return PIDGains(kp=kp, ki=ki, kd=kd)


# 便捷函数


def optimize_mass_distribution(
    robot_config: Any,
    target_com_height: float,
    max_iterations: int = 100,
    method: str = "gradient",
) -> MassOptimizationResult:
    """优化机器人质量分布

    Args:
        robot_config: 机器人配置文件路径或配置对象
        target_com_height: 目标重心高度 (米)
        max_iterations: 最大迭代次数
        method: 优化方法 ("gradient" 或 "genetic")

    Returns:
        MassOptimizationResult对象

    Example:
        >>> result = optimize_mass_distribution(
        ...     "configs/my_robot.json",
        ...     target_com_height=0.25,
        ...     max_iterations=200
        ... )
        >>> logger.info(f"COM error: {result.com_error:.4f} m")
    """
    optimizer = MassDistributionOptimizer(robot_config)
    return optimizer.optimize(target_com_height, max_iterations, method)


def tune_pid_controller(
    robot_config: Any, joint_name: str, method: str = "ziegler_nichols", **kwargs
) -> PIDGains:
    """调优PID控制器增益

    Args:
        robot_config: 机器人配置
        joint_name: 关节名称
        method: 调优方法 ("ziegler_nichols" 或 "genetic")
        **kwargs: 额外参数

    Returns:
        PIDGains对象

    Example:
        >>> gains = tune_pid_controller(
        ...     "configs/my_robot.json",
        ...     joint_name="hip_flex",
        ...     method="genetic",
        ...     population_size=50,
        ...     generations=100
        ... )
        >>> logger.info(f"Kp={gains.kp:.2f}")
    """
    tuner = PIDTuner(robot_config)
    return tuner.tune(joint_name, method, **kwargs)


def batch_optimize_pid(
    robot_config: Any, joint_names: List[str], method: str = "ziegler_nichols"
) -> Dict[str, PIDGains]:
    """批量优化多个关节的PID增益

    Args:
        robot_config: 机器人配置
        joint_names: 关节名称列表
        method: 调优方法

    Returns:
        关节名到PIDGains的映射
    """
    tuner = PIDTuner(robot_config)
    results = {}

    logger.info(f"\n开始批量优化 {len(joint_names)} 个关节...\n")

    for joint in joint_names:
        gains = tuner.tune(joint, method)
        results[joint] = gains

    logger.info("\nBatch optimization completed")
    return results
