"""
AGI-Walker 离线强化学习模块
使用 d3rlpy 实现 CQL (Conservative Q-Learning)
"""

import logging
import os
import pickle
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict

import gymnasium as gym
import numpy as np

from agi_walker.core.api.training_contracts import (
    build_training_run_artifact,
    write_training_run_artifact,
)

logger = logging.getLogger(__name__)


class ExpertDataCollector:
    """专家数据收集器"""

    def __init__(self, env_id: str, save_dir: str = "offline_data"):
        self.env_id = env_id
        self.save_dir = save_dir
        os.makedirs(save_dir, exist_ok=True)

    def collect_from_policy(
        self, policy, n_episodes: int = 1000, deterministic: bool = True
    ) -> Dict:
        """
        从专家策略收集演示数据

        参数:
            policy: 训练好的策略
            n_episodes: 收集的回合数
            deterministic: 是否使用确定性动作

        返回:
            包含 observations, actions, rewards, terminals 的字典
        """
        env = gym.make(self.env_id)

        observations = []
        actions = []
        rewards = []
        terminals = []

        print(f"开始收集专家数据，目标回合数: {n_episodes}")

        for episode in range(n_episodes):
            obs, _ = env.reset()
            done = False
            episode_reward = 0

            while not done:
                # 获取动作
                action, _ = policy.predict(obs, deterministic=deterministic)

                # 存储状态和动作
                observations.append(obs)
                actions.append(action)

                # 执行动作
                obs, reward, terminated, truncated, _ = env.step(action)
                done = terminated or truncated

                # 存储奖励和终止标志
                rewards.append(reward)
                terminals.append(done)
                episode_reward += reward

            if (episode + 1) % 100 == 0:
                print(
                    f"已收集 {episode + 1}/{n_episodes} 回合, "
                    f"最近回合奖励: {episode_reward:.2f}"
                )

        env.close()

        # 转换为 numpy 数组
        dataset = {
            "observations": np.array(observations),
            "actions": np.array(actions),
            "rewards": np.array(rewards),
            "terminals": np.array(terminals),
        }

        print("\n数据收集完成:")
        print(f"  - 样本总数: {len(observations)}")
        print(f"  - 观测维度: {observations[0].shape}")
        print(f"  - 动作维度: {actions[0].shape}")
        print(f"  - 平均奖励: {np.mean(rewards):.2f}")

        return dataset

    def save_dataset(self, dataset: Dict, filename: str):
        """保存数据集"""
        filepath = os.path.join(self.save_dir, filename)
        with open(filepath, "wb") as f:
            pickle.dump(dataset, f)
        print(f"数据集已保存至: {filepath}")

    def load_dataset(self, filename: str) -> Dict:
        """加载数据集"""
        filepath = os.path.join(self.save_dir, filename)
        with open(filepath, "rb") as f:
            dataset = pickle.load(f)
        print(f"数据集已加载: {filepath}")
        return dataset


class OfflineRLTrainer:
    """离线 RL 训练器（使用 CQL）"""

    def __init__(
        self,
        env_id: str,
        algorithm: str = "cql",
        actor_lr: float = 3e-4,
        critic_lr: float = 3e-4,
        batch_size: int = 256,
        n_critics: int = 2,
    ):
        self.env_id = env_id
        self.algorithm = algorithm

        # 延迟导入 d3rlpy（如果未安装会给出友好提示）
        try:
            from d3rlpy.algos import CQL, SAC
            from d3rlpy.dataset import MDPDataset

            self.CQL = CQL
            self.SAC = SAC
            self.MDPDataset = MDPDataset
        except ImportError:
            logger.error("错误: 未安装 d3rlpy")
            print("请运行: pip install d3rlpy")
            raise

        # CQL 配置
        if algorithm == "cql":
            self.model = self.CQL(
                actor_learning_rate=actor_lr,
                critic_learning_rate=critic_lr,
                batch_size=batch_size,
                n_critics=n_critics,
                conservative_weight=5.0,  # CQL 保守性权重
            )
        else:
            raise ValueError(f"不支持的算法: {algorithm}")

    def prepare_dataset(self, raw_dataset: Dict):
        """将原始数据转换为 d3rlpy MDPDataset"""
        dataset = self.MDPDataset(
            observations=raw_dataset["observations"],
            actions=raw_dataset["actions"],
            rewards=raw_dataset["rewards"],
            terminals=raw_dataset["terminals"],
        )
        return dataset

    def train_offline(
        self,
        dataset,
        n_steps: int = 100000,
        save_interval: int = 10000,
        save_dir: str = "offline_models",
        run_id: str | None = None,
    ):
        """离线训练"""
        save_path = Path(save_dir)
        save_path.mkdir(parents=True, exist_ok=True)
        started_at = datetime.now(timezone.utc).isoformat()
        start_time = time.time()

        print(f"\n开始离线训练 ({self.algorithm.upper()})")
        print(f"训练步数: {n_steps}")

        # 训练
        self.model.fit(
            dataset,
            n_steps=n_steps,
            save_interval=save_interval,
            save_dir=str(save_path),
            verbose=True,
        )

        finished_at = datetime.now(timezone.utc).isoformat()
        duration_seconds = time.time() - start_time
        manifest = build_training_run_artifact(
            run_id=run_id or f"{self.env_id}:offline:{self.algorithm}",
            run_type="offline_dataset_training",
            stage="offline_rl_training",
            status="completed",
            algorithm=self.algorithm,
            environment={"kind": "offline_dataset", "env_id": self.env_id},
            inputs={
                "n_steps": n_steps,
                "save_interval": save_interval,
                "dataset": _dataset_summary(dataset),
            },
            metrics={
                "training_time_seconds": duration_seconds,
                "n_steps": n_steps,
            },
            artifacts=[
                {
                    "name": "offline_model_dir",
                    "path": str(save_path),
                    "artifact_type": "model_directory",
                }
            ],
            hardware_required=False,
            hardware_enabled=False,
            started_at=started_at,
            finished_at=finished_at,
            duration_seconds=duration_seconds,
        )
        manifest_path = write_training_run_artifact(
            manifest, save_path / "training_run_manifest.json"
        )
        manifest["manifest_path"] = str(manifest_path)

        print("\n离线训练完成!")
        return manifest

    def finetune_online(
        self, env, n_steps: int = 10000, save_dir: str = "finetuned_models"
    ):
        """在线 Fine-tuning"""
        os.makedirs(save_dir, exist_ok=True)

        print("\n开始在线 Fine-tuning")
        print(f"训练步数: {n_steps}")

        self.model.fit_online(env, n_steps=n_steps, save_dir=save_dir)

        print("\nFine-tuning 完成!")

    def save(self, filepath: str):
        """保存模型"""
        self.model.save(filepath)
        print(f"模型已保存: {filepath}")

    def load(self, filepath: str):
        """加载模型"""
        self.model.load(filepath)
        print(f"模型已加载: {filepath}")


if __name__ == "__main__":
    print("离线强化学习模块加载成功")
    print("使用示例请查看: examples/offline_rl_demo.py")


def _dataset_summary(dataset: Any) -> Dict[str, Any]:
    if isinstance(dataset, dict):
        return {
            "format": "dict",
            "fields": sorted(str(key) for key in dataset),
            "sample_count": _safe_len(dataset.get("observations")),
            "reward_count": _safe_len(dataset.get("rewards")),
            "terminal_count": _safe_len(dataset.get("terminals")),
        }

    episodes = getattr(dataset, "episodes", None)
    if episodes is not None:
        return {
            "format": dataset.__class__.__name__,
            "episode_count": _safe_len(episodes),
        }

    return {
        "format": dataset.__class__.__name__,
        "sample_count": _safe_len(dataset),
    }


def _safe_len(value: Any) -> int | None:
    try:
        return len(value)
    except TypeError:
        return None
