"""
MuJoCo 物理后端
提供高精度物理仿真,可替代 Godot 物理引擎
"""

import numpy as np
from typing import Optional, Dict, Any, Tuple
from dataclasses import dataclass

try:
    import mujoco

    MUJOCO_AVAILABLE = True
except ImportError:
    MUJOCO_AVAILABLE = False
    print("⚠️ MuJoCo 未安装，请运行: pip install mujoco")

try:
    from python_api.comm.zenoh_interface import ZenohInterface

    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False


@dataclass
class MuJoCoConfig:
    """MuJoCo 配置"""

    model_path: str  # XML 模型文件路径
    timestep: float = 0.002  # 仿真时间步长 (2ms)
    sync_to_godot: bool = True  # 是否同步到 Godot 可视化
    zenoh_state_key: str = "rt/mujoco/state"
    zenoh_cmd_key: str = "rt/mujoco/cmd"


class MuJoCoBackend:
    """
    MuJoCo 物理后端

    架构:
        MuJoCo (物理计算) ←→ Zenoh ←→ Godot (可视化)

    优势:
        - 物理精度高 (接触模型精确)
        - 速度快 (比 Godot 快 10x)
        - 学术标准 (论文可复现)

    用法:
        config = MuJoCoConfig(model_path="robot.xml")
        backend = MuJoCoBackend(config)

        for _ in range(1000):
            action = policy.get_action(obs)
            obs, reward, done = backend.step(action)
    """

    def __init__(self, config: MuJoCoConfig):
        if not MUJOCO_AVAILABLE:
            raise ImportError("MuJoCo 未安装")

        self.config = config

        # 加载模型
        self.model = mujoco.MjModel.from_xml_path(config.model_path)
        self.data = mujoco.MjData(self.model)

        # 设置时间步长
        self.model.opt.timestep = config.timestep

        # Zenoh 通信 (可选)
        self.zenoh: Optional[ZenohInterface] = None
        if config.sync_to_godot and ZENOH_AVAILABLE:
            self.zenoh = ZenohInterface()
            self.zenoh.declare_publisher(config.zenoh_state_key)
            self.zenoh.declare_subscriber(config.zenoh_cmd_key, self._on_godot_cmd)

        print("✅ MuJoCo 后端初始化完成")
        print(f"   - 模型: {config.model_path}")
        print(f"   - DoF: {self.model.nv}")
        print(f"   - 时间步长: {config.timestep*1000:.1f}ms")

    def reset(self) -> np.ndarray:
        """重置仿真"""
        mujoco.mj_resetData(self.model, self.data)
        return self._get_observation()

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool]:
        """
        执行一步仿真

        Args:
            action: 控制输入 (关节力矩或位置)

        Returns:
            observation: 观测值
            reward: 奖励 (需要外部定义)
            done: 是否结束
        """
        # 应用控制
        self.data.ctrl[:] = action

        # 物理步进
        mujoco.mj_step(self.model, self.data)

        # 同步到 Godot
        if self.zenoh:
            self._sync_to_godot()

        # 获取观测
        obs = self._get_observation()

        # 计算奖励 (示例: 保持直立)
        reward = self._compute_reward()

        # 判断结束
        done = self._check_done()

        return obs, reward, done

    def _get_observation(self) -> np.ndarray:
        """获取观测值"""
        obs = np.concatenate(
            [
                self.data.qpos,  # 关节位置
                self.data.qvel,  # 关节速度
                self.data.sensordata,  # 传感器数据 (IMU 等)
            ]
        )
        return obs

    def _compute_reward(self) -> float:
        """计算奖励 (示例实现)"""
        # 示例: 保持躯干高度
        trunk_height = self.data.qpos[2]  # 假设 z 是第 3 个坐标
        height_reward = np.exp(-abs(trunk_height - 0.5))

        # 示例: 惩罚过大的力矩
        ctrl_cost = -0.01 * np.sum(self.data.ctrl**2)

        return height_reward + ctrl_cost

    def _check_done(self) -> bool:
        """判断是否结束"""
        # 示例: 躯干倾倒
        trunk_height = self.data.qpos[2]
        if trunk_height < 0.2:
            return True
        return False

    def _sync_to_godot(self):
        """同步状态到 Godot (用于可视化)"""
        state = {
            "positions": self.data.qpos.tolist(),
            "velocities": self.data.qvel.tolist(),
            "timestamp": self.data.time,
        }
        self.zenoh.publish(self.config.zenoh_state_key, state)

    def _on_godot_cmd(self, data):
        """接收 Godot 命令 (可选,用于交互)"""
        if "reset" in data:
            self.reset()

    def render(self, camera_id: int = 0) -> np.ndarray:
        """渲染图像 (用于视觉 RL)"""
        renderer = mujoco.Renderer(self.model, height=480, width=640)
        renderer.update_scene(self.data, camera=camera_id)
        return renderer.render()

    def close(self):
        """清理资源"""
        if self.zenoh:
            self.zenoh.close()
        print("🔌 MuJoCo 后端已关闭")


# ==================== 示例代码 ====================

if __name__ == "__main__":
    print("MuJoCo Backend Demo")

    if not MUJOCO_AVAILABLE:
        print("❌ MuJoCo 未安装，无法运行 demo")
        exit(1)

    # 创建简单的摆锤模型 (用于测试)
    xml = """
    <mujoco>
      <worldbody>
        <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
        <geom type="plane" size="1 1 0.1" rgba=".9 0 0 1"/>
        <body pos="0 0 1">
          <joint type="hinge" axis="1 0 0"/>
          <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5"/>
        </body>
      </worldbody>
    </mujoco>
    """

    # 保存临时模型
    import tempfile

    with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as f:
        f.write(xml)
        model_path = f.name

    # 创建后端
    config = MuJoCoConfig(model_path=model_path, sync_to_godot=False)
    backend = MuJoCoBackend(config)

    # 运行仿真
    obs = backend.reset()
    print(f"\n初始观测: {obs}")

    for i in range(100):
        action = np.array([0.1])  # 施加小扭矩
        obs, reward, done = backend.step(action)

        if i % 20 == 0:
            print(f"Step {i}: reward={reward:.3f}, done={done}")

        if done:
            print("仿真结束")
            break

    backend.close()

    # 清理临时文件
    import os

    os.unlink(model_path)
