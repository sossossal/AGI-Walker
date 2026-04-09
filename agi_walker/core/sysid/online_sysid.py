import numpy as np
import logging
from typing import List

logger = logging.getLogger(__name__)


class ParameterEstimator:
    """
    AGI-Walker V2.0 Online System Identification (OnlineSysID).
    Uses Recursive Least Squares (RLS) with forgetting factor to estimate
    physical drift (friction, damping, payload) in real-time.
    """

    def __init__(
        self,
        num_params: int = 3,
        lambda_factor: float = 0.99,
        initial_p: float = 1000.0,
    ):
        # 遗忘因子 (Forgetting Factor)，用于跟踪时变参数
        self.lambda_factor = lambda_factor
        # 参数向量 (Theta)
        self.theta = np.zeros((num_params, 1))
        # 协方差矩阵 (P)
        self.P = np.eye(num_params) * initial_p

    def update(self, y: float, phi: List[float]):
        """
        根据单次观测更新参数估计。
        y: 实际观测输出 (标量)
        phi: 回归向量 (长度为 num_params 的列表)
        """
        phi_vec = np.array(phi).reshape(-1, 1)

        # 1. 计算增益向量 (K)
        den = self.lambda_factor + phi_vec.T @ self.P @ phi_vec
        K = (self.P @ phi_vec) / den

        # 2. 计算误差 (e)
        e = y - (phi_vec.T @ self.theta).item()

        # 3. 更新参数 (Theta)
        self.theta = self.theta + K * e

        # 4. 更新协方差 (P)
        self.P = (self.P - K @ phi_vec.T @ self.P) / self.lambda_factor

    def get_params(self) -> np.ndarray:
        return self.theta.flatten()


class GroundFrictionEstimator:
    """
    针对地面摩擦力和阻尼的专门估算器。
    y = b*v + f*sign(v)
    """

    def __init__(self):
        self.rls = ParameterEstimator(num_params=2)  # 2个参数: 阻尼, 静摩擦

    def step(self, measured_torque: float, velocity: float):
        # 构造回归向量: [速度, 速度方向]
        phi = [velocity, np.sign(velocity)]
        self.rls.update(measured_torque, phi)

        params = self.rls.get_params()
        return {"damping_b": params[0], "friction_f": params[1]}

    def report(self):
        params = self.rls.get_params()
        logger.info(
            f"SysID Estimate: Damping={params[0]:.4f} | Friction={params[1]:.4f}"
        )
