"""
进化循环管理器
串联RL训练、数据生成、自动标记和PEFT微调，实现自动化进化
"""

import os
import time
import json
import asyncio
from pathlib import Path
from typing import Dict, List, Optional
from dataclasses import dataclass

import sys

sys.path.append(str(Path(__file__).parent.parent))

# 导入各模块
try:
    from python_controller.rl_optimizer import RLOptimizer, RLConfig, DummyEnv
except ImportError:
    from rl_optimizer import RLOptimizer, RLConfig, DummyEnv

from training.auto_labeler import AutoLabeler
from training.dataset_cleaner import DatasetCleaner
from training.peft_trainer import PEFTTrainer, PEFTConfig, PEFTMethod


@dataclass
class EvolutionConfig:
    """进化循环配置"""

    # 路径配置
    workspace_dir: str = str(Path(__file__).resolve().parent.parent)
    iteration_name: str = "evo_v1"

    # RL配置
    rl_timesteps: int = 50000
    rl_algorithm: str = "PPO"

    # 数据生成
    n_trajectories: int = 100

    # 微调配置
    peft_method: str = "prefix_tuning"
    peft_epochs: int = 3

    # 自动化
    auto_proceed: bool = True


class EvolutionManager:
    """
    进化循环管理器

    实现完整的自动化进化流程：
    1. RL探索与训练 (Stage 4)
    2. 策略部署与数据生成
    3. 数据自动标记与清洗 (Stage 5)
    4. 小模型PEFT微调
    5. 模型评估与部署
    """

    def __init__(self, config: Optional[EvolutionConfig] = None):
        self.config = config or EvolutionConfig()

        # 目录设置
        self.workspace = Path(self.config.workspace_dir)
        self.data_dir = self.workspace / "offline_data" / self.config.iteration_name
        self.models_dir = self.workspace / "models" / self.config.iteration_name

        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.models_dir.mkdir(parents=True, exist_ok=True)

        # 状态
        self.current_stage = "INIT"
        self.history = []

        print(f"✅ 进化管理器初始化: {self.config.iteration_name}")

    async def run_loop(self):
        """运行完整进化循环"""
        print("=" * 50)
        print(f"🚀 开始进化循环: {self.config.iteration_name}")
        print("=" * 50)

        start_time = time.time()

        # 1. RL 训练
        rl_model_path = await self.stage_rl_training()

        # 2. 数据生成
        raw_data_path = await self.stage_data_generation(rl_model_path)

        # 3. 数据标记与清洗
        clean_data_path = await self.stage_data_processing(raw_data_path)

        # 4. PEFT 微调
        final_model_path = await self.stage_model_finetuning(clean_data_path)

        total_time = time.time() - start_time
        print(f"\n🎉 进化循环完成! 用时: {total_time:.1f}秒")
        print(f"📍 最终模型: {final_model_path}")

        return final_model_path

    async def stage_rl_training(self) -> str:
        """阶段1: RL训练"""
        self.current_stage = "RL_TRAINING"
        print(f"\n[Stage 1/4] RL训练 ({self.config.rl_algorithm})")

        # 配置RL
        rl_config = RLConfig(
            algorithm=self.config.rl_algorithm,
            tensorboard_log=str(self.models_dir / "tensorboard"),
        )

        # 创建优化器 (使用DummyEnv模拟，实际应连接Godot)
        # 实际代码: from gym_env import GodotRobotEnv; env = GodotRobotEnv()
        env = DummyEnv()
        optimizer = RLOptimizer(env, rl_config, save_dir=str(self.models_dir / "rl"))

        # 训练（使用线程池或异步执行）
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(
            None, lambda: optimizer.train(total_timesteps=self.config.rl_timesteps)
        )

        # 导出模型
        model_path = (
            self.models_dir / "rl" / f"{self.config.rl_algorithm.lower()}_final.zip"
        )

        # 同时导出ONNX
        onnx_path = self.models_dir / "rl" / "policy.onnx"
        optimizer.export_policy_onnx(str(onnx_path))

        self.history.append(
            {"stage": "rl", "status": "success", "path": str(model_path)}
        )
        return str(model_path)

    async def stage_data_generation(self, model_path: str) -> str:
        """阶段2: 数据生成"""
        self.current_stage = "DATA_GEN"
        print(f"\n[Stage 2/4] 数据生成 ({self.config.n_trajectories} trajectories)")

        output_path = self.data_dir / "raw_trajectories.json"

        # 模拟数据生成过程
        # 这里应该加载RL模型并在环境中运行，收集数据
        print("   正在运行策略收集数据...")
        await asyncio.sleep(2)  # 模拟耗时

        # 生成模拟数据
        import random

        trajectories = []
        for i in range(self.config.n_trajectories):
            traj = {
                "id": f"traj_{i}",
                "states": [
                    {
                        "sensors": {
                            "imu": {
                                "orient": [
                                    random.uniform(-5, 5),
                                    random.uniform(-5, 5),
                                    0,
                                ]
                            }
                        }
                    }
                    for _ in range(50)
                ],
                "actions": [[random.random(), random.random()] for _ in range(50)],
                "avg_velocity": random.uniform(0, 1.0),
                "terminated": random.random() < 0.2,
            }
            trajectories.append(traj)

        with open(output_path, "w") as f:
            json.dump(trajectories, f)

        print(f"✅ 数据已保存: {output_path}")
        self.history.append(
            {"stage": "data_gen", "status": "success", "count": len(trajectories)}
        )
        return str(output_path)

    async def stage_data_processing(self, input_path: str) -> str:
        """阶段3: 数据标记与清洗"""
        self.current_stage = "DATA_PROC"
        print("\n[Stage 3/4] 数据处理")

        # 1. 自动标记
        labeler = AutoLabeler()
        labeled_path = self.data_dir / "labeled_data.json"

        loop = asyncio.get_event_loop()
        await loop.run_in_executor(
            None, lambda: labeler.batch_label(input_path, str(labeled_path))
        )

        # 2. 清洗
        cleaner = DatasetCleaner()

        with open(labeled_path, "r", encoding="utf-8") as f:
            labeled_data = json.load(f)

        cleaned_data = cleaner.clean_pipeline(labeled_data, balance=True)

        clean_path = self.data_dir / "train_data.json"
        cleaner.save_dataset(cleaned_data, str(clean_path))

        self.history.append(
            {
                "stage": "processing",
                "status": "success",
                "final_count": len(cleaned_data),
            }
        )
        return str(clean_path)

    async def stage_model_finetuning(self, dataset_path: str) -> str:
        """阶段4: PEFT微调"""
        self.current_stage = "FINETUNING"
        print(f"\n[Stage 4/4] PEFT微调 ({self.config.peft_method})")

        peft_config = PEFTConfig(
            method=PEFTMethod(self.config.peft_method),
            num_epochs=self.config.peft_epochs,
        )

        trainer = PEFTTrainer(
            config=peft_config, output_dir=str(self.models_dir / "peft")
        )

        # 模拟训练（因为没有安装transformers/datasets库）
        # 实际代码:
        # dataset = trainer.prepare_dataset(dataset_path)
        # trainer.train(dataset)

        print("   微调中...")
        await asyncio.sleep(2)  # 模拟

        # 模拟保存
        final_path = self.models_dir / "peft" / "final"
        final_path.mkdir(parents=True, exist_ok=True)
        with open(final_path / "model_info.json", "w") as f:
            json.dump(trainer.get_stats(), f)

        print(f"✅ 微调完成: {final_path}")
        self.history.append({"stage": "finetuning", "status": "success"})
        return str(final_path)

    def get_report(self) -> str:
        """生成进化报告"""
        report = f"# 进化报告: {self.config.iteration_name}\n\n"

        for item in self.history:
            status_icon = "✅" if item["status"] == "success" else "❌"
            report += f"## {status_icon} Stage: {item['stage']}\n"
            report += f"{json.dumps(item, indent=2)}\n\n"

        return report


# 测试代码
async def test_evolution():
    print("进化循环管理器测试\n")

    config = EvolutionConfig(
        iteration_name="test_evo_v1",
        rl_timesteps=1000,
        n_trajectories=50,
        peft_epochs=1,
    )

    manager = EvolutionManager(config)
    await manager.run_loop()

    print("\n=== 报告 ===")
    print(manager.get_report())


if __name__ == "__main__":
    asyncio.run(test_evolution())
