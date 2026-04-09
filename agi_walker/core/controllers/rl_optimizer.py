"""
强化学习策略优化器
基于Stable-Baselines3，集成Godot仿真环境
"""

import time
import json
import argparse
from pathlib import Path
from dataclasses import dataclass
from typing import Callable, Optional
import numpy as np
import logging

logger = logging.getLogger(__name__)

# 延迟导入SB3
sb3 = None
VecEnv = None


def _init_sb3() -> None:
    """延迟初始化Stable-Baselines3"""
    global sb3, VecEnv
    if sb3 is None:
        try:
            import stable_baselines3 as _sb3
            from stable_baselines3.common.vec_env import DummyVecEnv

            sb3 = _sb3
            VecEnv = DummyVecEnv
            logger.info(f"Stable-Baselines3 v{sb3.__version__} loaded")
            return True
        except ImportError:
            logger.warning("Stable-Baselines3 not installed")
            logger.warning("Run: pip install stable-baselines3")
            return False
    return True


@dataclass
class RLConfig:
    """强化学习配置"""

    algorithm: str = "PPO"  # 算法选择
    learning_rate: float = 3e-4  # 学习率
    n_steps: int = 2048  # 每次更新的步数
    batch_size: int = 64  # 批量大小
    n_epochs: int = 10  # 每次更新的epochs
    gamma: float = 0.99  # 折扣因子
    gae_lambda: float = 0.95  # GAE lambda
    clip_range: float = 0.2  # PPO clip范围
    ent_coef: float = 0.01  # 熵系数
    verbose: int = 1  # 日志级别
    tensorboard_log: str = "./tensorboard_logs"


class RLOptimizer:
    """
    强化学习策略优化器

    支持算法：PPO, SAC, TD3, A2C
    集成Godot仿真环境进行自动化优化
    """

    ALGORITHMS = {
        "PPO": "stable_baselines3.PPO",
        "SAC": "stable_baselines3.SAC",
        "TD3": "stable_baselines3.TD3",
        "A2C": "stable_baselines3.A2C",
    }

    def __init__(
        self,
        env,
        config: Optional[RLConfig] = None,
        save_dir: Optional[str] = None,
    ):
        """
        初始化RL优化器

        Args:
            env: Gym环境（GodotRobotEnv）
            config: RL配置
            save_dir: 模型保存目录
        """
        if not _init_sb3():
            raise ImportError("Stable-Baselines3不可用")

        self.config = config or RLConfig()

        # 动态获取项目根目录
        project_root = Path(__file__).resolve().parent.parent
        self.save_dir = Path(save_dir) if save_dir else project_root / "models" / "rl"
        self.save_dir.mkdir(parents=True, exist_ok=True)

        # 包装环境
        self.env = env
        self.vec_env = VecEnv([lambda: env]) if VecEnv else None

        # 创建模型
        self.model = self._create_model()

        # 训练历史
        self.training_history = []
        self.best_reward = float("-inf")

        logger.info("✅ RL优化器初始化完成")
        logger.info(f"   算法: {self.config.algorithm}")
        logger.info(f"   保存目录: {self.save_dir}")

    def _create_model(self) -> None:
        """创建RL模型"""
        algorithm = self.config.algorithm.upper()

        if algorithm not in self.ALGORITHMS:
            raise ValueError(f"不支持的算法: {algorithm}")

        # 动态导入算法
        if algorithm == "PPO":
            model = sb3.PPO(
                "MlpPolicy",
                self.vec_env or self.env,
                learning_rate=self.config.learning_rate,
                n_steps=self.config.n_steps,
                batch_size=self.config.batch_size,
                n_epochs=self.config.n_epochs,
                gamma=self.config.gamma,
                gae_lambda=self.config.gae_lambda,
                clip_range=self.config.clip_range,
                ent_coef=self.config.ent_coef,
                verbose=self.config.verbose,
                tensorboard_log=self.config.tensorboard_log,
            )
        elif algorithm == "SAC":
            model = sb3.SAC(
                "MlpPolicy",
                self.vec_env or self.env,
                learning_rate=self.config.learning_rate,
                batch_size=self.config.batch_size,
                gamma=self.config.gamma,
                verbose=self.config.verbose,
                tensorboard_log=self.config.tensorboard_log,
            )
        elif algorithm == "TD3":
            model = sb3.TD3(
                "MlpPolicy",
                self.vec_env or self.env,
                learning_rate=self.config.learning_rate,
                batch_size=self.config.batch_size,
                gamma=self.config.gamma,
                verbose=self.config.verbose,
                tensorboard_log=self.config.tensorboard_log,
            )
        elif algorithm == "A2C":
            model = sb3.A2C(
                "MlpPolicy",
                self.vec_env or self.env,
                learning_rate=self.config.learning_rate,
                n_steps=self.config.n_steps,
                gamma=self.config.gamma,
                gae_lambda=self.config.gae_lambda,
                ent_coef=self.config.ent_coef,
                verbose=self.config.verbose,
                tensorboard_log=self.config.tensorboard_log,
            )

        return model

    def train(
        self,
        total_timesteps: int = 100000,
        eval_freq: int = 10000,
        n_eval_episodes: int = 5,
        callback: Optional[Callable] = None,
    ) -> dict:
        """
        训练RL代理

        Args:
            total_timesteps: 总训练步数
            eval_freq: 评估频率
            n_eval_episodes: 每次评估的episode数
            callback: 自定义回调

        Returns:
            训练结果统计
        """
        logger.info("\n 开始训练")
        logger.info(f"   总步数: {total_timesteps}")
        logger.info(f"   评估频率: {eval_freq}")

        start_time = time.time()

        # 创建回调
        from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback

        callbacks = []

        # 检查点回调
        checkpoint_callback = CheckpointCallback(
            save_freq=eval_freq,
            save_path=str(self.save_dir / "checkpoints"),
            name_prefix=f"{self.config.algorithm.lower()}_model",
        )
        callbacks.append(checkpoint_callback)

        # 评估回调
        if self.vec_env:
            eval_callback = EvalCallback(
                self.vec_env,
                best_model_save_path=str(self.save_dir / "best_model"),
                log_path=str(self.save_dir / "eval_logs"),
                eval_freq=eval_freq,
                n_eval_episodes=n_eval_episodes,
                deterministic=True,
            )
            callbacks.append(eval_callback)

        if callback:
            callbacks.append(callback)

        # 训练
        self.model.learn(
            total_timesteps=total_timesteps,
            callback=callbacks if callbacks else None,
            progress_bar=True,
        )

        training_time = time.time() - start_time

        # 保存最终模型
        final_path = self.save_dir / f"{self.config.algorithm.lower()}_final.zip"
        self.model.save(str(final_path))
        logger.info(f"✅ 模型已保存: {final_path}")

        # 记录历史
        result = {
            "algorithm": self.config.algorithm,
            "total_timesteps": total_timesteps,
            "training_time": training_time,
            "model_path": str(final_path),
        }
        self.training_history.append(result)

        return result

    def evaluate(self, n_episodes: int = 10) -> dict:
        """评估当前策略"""
        from stable_baselines3.common.evaluation import evaluate_policy

        logger.info(f"\n📊 评估策略 ({n_episodes} episodes)")

        mean_reward, std_reward = evaluate_policy(
            self.model,
            self.vec_env or self.env,
            n_eval_episodes=n_episodes,
            deterministic=True,
        )

        result = {
            "mean_reward": mean_reward,
            "std_reward": std_reward,
            "n_episodes": n_episodes,
        }

        logger.info(f"   平均奖励: {mean_reward:.2f} ± {std_reward:.2f}")

        return result

    def load(self, path: str) -> bool:
        """加载已保存的模型"""
        algorithm = self.config.algorithm.upper()

        if algorithm == "PPO":
            self.model = sb3.PPO.load(path, env=self.vec_env or self.env)
        elif algorithm == "SAC":
            self.model = sb3.SAC.load(path, env=self.vec_env or self.env)
        elif algorithm == "TD3":
            self.model = sb3.TD3.load(path, env=self.vec_env or self.env)
        elif algorithm == "A2C":
            self.model = sb3.A2C.load(path, env=self.vec_env or self.env)

        logger.info(f"✅ 模型已加载: {path}")

    def export_policy_onnx(self, output_path: str) -> None:
        """导出策略为ONNX格式"""
        try:
            import torch

            # 获取策略网络
            policy = self.model.policy

            # 创建dummy输入
            obs_shape = self.env.observation_space.shape
            dummy_obs = torch.zeros((1,) + obs_shape)

            # 导出
            torch.onnx.export(
                policy,
                dummy_obs,
                output_path,
                opset_version=11,
                input_names=["observation"],
                output_names=["action"],
            )

            logger.info(f"✅ 策略已导出为ONNX: {output_path}")

        except Exception as e:
            logger.info(f" ONNX导出失败: {e}")

    def get_action(
        self, observation: np.ndarray, deterministic: bool = True
    ) -> np.ndarray:
        """获取动作"""
        action, _ = self.model.predict(observation, deterministic=deterministic)
        return action

    def get_stats(self) -> dict:
        """获取统计信息"""
        return {
            "algorithm": self.config.algorithm,
            "training_history": self.training_history,
            "model_path": str(self.save_dir),
        }


class DummyEnv:
    """虚拟环境（用于测试）"""

    def __init__(self) -> None:
        import gymnasium as gym

        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(12,))
        self.action_space = gym.spaces.Box(low=-1, high=1, shape=(2,))
        self._step_count = 0

    def reset(self, seed=None, options=None) -> None:
        self._step_count = 0
        return np.zeros(12, dtype=np.float32), {}

    def step(self, action) -> tuple:
        self._step_count += 1
        obs = np.random.randn(12).astype(np.float32) * 0.1
        reward = 1.0 - np.abs(obs[0])  # 简单奖励
        terminated = abs(obs[0]) > 2.0
        truncated = self._step_count > 1000
        return obs, reward, terminated, truncated, {}

    def close(self) -> None:
        pass


def main() -> None:
    """主函数"""
    parser = argparse.ArgumentParser(description="RL策略优化器")

    parser.add_argument(
        "--algorithm",
        default="PPO",
        choices=["PPO", "SAC", "TD3", "A2C"],
        help="RL算法",
    )
    parser.add_argument("--timesteps", type=int, default=100000, help="训练步数")
    parser.add_argument("--eval-freq", type=int, default=10000, help="评估频率")
    parser.add_argument("--learning-rate", type=float, default=3e-4, help="学习率")
    parser.add_argument(
        "--use-godot", action="store_true", help="使用Godot环境（默认使用虚拟环境）"
    )

    args = parser.parse_args()

    # 创建环境
    if args.use_godot:
        try:
            from agi_walker.core.api.godot_robot_env.gym_env import GodotRobotEnv

            env = GodotRobotEnv()
        except ImportError as e:
            logger.error(f"Godot environment not available: {e}")
            logger.warning("Falling back to virtual env...")
            env = DummyEnv()
    else:
        logger.info("Using virtual environment for testing...")
        env = DummyEnv()

    # 创建配置
    config = RLConfig(algorithm=args.algorithm, learning_rate=args.learning_rate)

    # 创建优化器
    optimizer = RLOptimizer(env, config)

    # 训练
    result = optimizer.train(total_timesteps=args.timesteps, eval_freq=args.eval_freq)

    # 评估
    eval_result = optimizer.evaluate(n_episodes=5)

    # 打印结果
    logger.info("\n" + "=" * 50)
    logger.info("训练完成")
    logger.info("=" * 50)
    logger.info(json.dumps(result, indent=2))
    logger.info(json.dumps(eval_result, indent=2))

    env.close()


if __name__ == "__main__":
    main()
