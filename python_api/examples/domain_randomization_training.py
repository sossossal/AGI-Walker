# domain_randomization_training.py
# 域随机化训练示例
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

import gymnasium as gym
from godot_robot_env import GodotRobotEnv


class DomainRandomizationWrapper(gym.Wrapper):
    """
    域随机化包装器 - 提高策略泛化能力

    每个 episode 开始时随机化环境参数，使训练的策略
    能够适应更多不同的环境条件。
    """

    def __init__(self, env, randomize_params=None):
        super().__init__(env)

        # 默认随机化参数
        self.randomize_params = randomize_params or {
            "gravity": (7.0, 12.0),  # 地球重力的 ±20%
            "air_density": (0.5, 2.0),
            "temperature": (-20.0, 40.0),
            "ground_materials": ["concrete", "wood", "carpet", "ice", "sand"],
        }

        self.episode_count = 0
        self.current_params = {}

    def reset(self, **kwargs):
        """每个 episode 开始时随机化环境"""
        self._randomize_environment()
        self.episode_count += 1
        return self.env.reset(**kwargs)

    def _randomize_environment(self):
        """随机化环境参数"""
        params = {}

        # 随机重力
        if "gravity" in self.randomize_params:
            g_min, g_max = self.randomize_params["gravity"]
            params["gravity"] = np.random.uniform(g_min, g_max)

        # 随机空气密度
        if "air_density" in self.randomize_params:
            rho_min, rho_max = self.randomize_params["air_density"]
            params["air_density"] = np.random.uniform(rho_min, rho_max)

        # 随机温度
        if "temperature" in self.randomize_params:
            t_min, t_max = self.randomize_params["temperature"]
            params["temperature"] = np.random.uniform(t_min, t_max)

        # 随机地面材质
        if "ground_materials" in self.randomize_params:
            materials = self.randomize_params["ground_materials"]
            params["ground_material"] = np.random.choice(materials)

        # 应用到环境
        try:
            self.env.set_physics_params(params)
            self.current_params = params

            print(
                f"🎲 Episode {self.episode_count}: "
                f"g={params.get('gravity', 9.81):.2f} m/s², "
                f"ρ={params.get('air_density', 1.225):.3f} kg/m³, "
                f"T={params.get('temperature', 25):.1f}°C, "
                f"mat={params.get('ground_material', 'concrete')}"
            )
        except Exception as e:
            print(f"⚠️  Warning: Failed to set physics params: {e}")


def test_domain_randomization():
    """测试域随机化包装器"""
    print("=" * 60)
    print("域随机化测试")
    print("=" * 60)

    # 创建环境
    print("\n1. 创建基础环境...")
    env = GodotRobotEnv()

    # 应用域随机化
    print("2. 应用域随机化包装器...")
    env = DomainRandomizationWrapper(env)

    # 测试多个 episodes
    print("\n3. 测试5个随机化的 episodes:\n")

    for episode in range(5):
        obs, info = env.reset()
        print(f"   Episode {episode + 1} 参数: {env.current_params}")

    print("\n✅ 域随机化测试完成!")
    print("\n提示: 实际训练需要:")
    print("  1. 启动 Godot 仿真器")
    print("  2. 配置 TCP 服务器")
    print("  3. 运行完整训练脚本")


if __name__ == "__main__":
    test_domain_randomization()
