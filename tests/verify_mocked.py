"""
Mock验证脚本
用于在没有安装完整依赖（如Stable-Baselines3）的环境中验证进化循环逻辑
"""

import sys
import unittest
from unittest.mock import MagicMock
from pathlib import Path
import asyncio

# 添加项目路径
sys.path.append(r"d:\新建文件夹\AGI-Walker")

# 1. Mock Stable-Baselines3
mock_sb3 = MagicMock()
mock_sb3.__version__ = "2.0.0"
sys.modules["stable_baselines3"] = mock_sb3
sys.modules["stable_baselines3.common"] = MagicMock()
sys.modules["stable_baselines3.common.vec_env"] = MagicMock()
sys.modules["stable_baselines3.common.callbacks"] = MagicMock()
sys.modules["stable_baselines3.common.evaluation"] = MagicMock()

# Mock PPO
mock_model = MagicMock()
mock_model.predict.return_value = ([0.1, 0.1], None)
mock_sb3.PPO.return_value = mock_model
mock_sb3.PPO.load.return_value = mock_model

# Mock Gymnasium
mock_gym = MagicMock()
sys.modules["gymnasium"] = mock_gym
sys.modules["gymnasium.spaces"] = MagicMock()

# Mock PEFT for Trainer
mock_peft = MagicMock()
sys.modules["peft"] = mock_peft
sys.modules["peft.utils"] = MagicMock()
sys.modules["peft.mapping"] = MagicMock()
sys.modules["peft.peft_model"] = MagicMock()
mock_peft.TaskType.CAUSAL_LM = "CAUSAL_LM"

# Mock Transformers
mock_transformers = MagicMock()
sys.modules["transformers"] = mock_transformers

# Mock Datasets
mock_datasets = MagicMock()
sys.modules["datasets"] = mock_datasets

# 现在导入项目模块
try:
    from python_controller.rl_optimizer import RLOptimizer
    from python_controller.evolution_manager import EvolutionManager, EvolutionConfig
    from training.peft_trainer import PEFTTrainer
except ImportError as e:
    print(f"导入失败: {e}")
    sys.exit(1)

async def run_verification():
    print("🚀 开始Mock验证进化循环...")
    
    # 配置
    config = EvolutionConfig(
        iteration_name="mock_test_v1",
        rl_timesteps=100,
        n_trajectories=10,
        peft_epochs=1,
        workspace_dir=r"d:\新建文件夹\AGI-Walker"
    )
    
    manager = EvolutionManager(config)
    
    try:
        # 运行循环
        print("\n--- 运行 Stage 1: RL Training ---")
        await manager.stage_rl_training()
        print("✅ RL Training 通过")
        
        print("\n--- 运行 Stage 2: Data Generation ---")
        # 手动设置模型路径
        model_path = str(manager.models_dir / "rl" / "ppo_final.zip")
        await manager.stage_data_generation(model_path)
        print("✅ Data Generation 通过")
        
        print("\n--- 运行 Stage 3: Data Processing ---")
        raw_data_path = manager.data_dir / "raw_trajectories.json"
        await manager.stage_data_processing(str(raw_data_path))
        print("✅ Data Processing 通过")
        
        print("\n--- 运行 Stage 4: PEFT Finetuning ---")
        clean_data_path = manager.data_dir / "train_data.json"
        
        # Mock PEFT Trainer behavior specifically
        with unittest.mock.patch('training.peft_trainer.PEFTTrainer.train') as mock_train:
            mock_train.return_value = {"loss": 0.1}
            await manager.stage_model_finetuning(str(clean_data_path))
            
        print("✅ PEFT Finetuning 通过")
        
        print("\n🎉 逻辑验证全部通过！")
        return True
        
    except Exception as e:
        print(f"\n❌ 验证失败: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = asyncio.run(run_verification())
    sys.exit(0 if success else 1)
