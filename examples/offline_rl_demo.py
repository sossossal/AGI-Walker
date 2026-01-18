"""
离线强化学习完整示例
演示如何使用离线 RL 提升训练效率
"""

import gymnasium as gym
from stable_baselines3 import PPO
from python_api.offline_rl import ExpertDataCollector, OfflineRLTrainer


def main():
    # 配置
    ENV_ID = "AGI-Walker/Walker2D-v0"
    EXPERT_MODEL_PATH = "models/ppo_expert.zip"  # 预训练的专家模型
    N_EPISODES = 1000  # 收集的回合数
    
    print("="*60)
    print("离线强化学习完整示例")
    print("="*60)
    
    # ========== 步骤 1: 收集专家数据 ==========
    print("\n[步骤 1/4] 收集专家数据")
    
    # 加载预训练的专家策略
    expert_policy = PPO.load(EXPERT_MODEL_PATH)
    print(f"已加载专家模型: {EXPERT_MODEL_PATH}")
    
    # 收集数据
    collector = ExpertDataCollector(ENV_ID)
    dataset = collector.collect_from_policy(
        expert_policy,
        n_episodes=N_EPISODES,
        deterministic=True
    )
    
    # 保存数据集
    collector.save_dataset(dataset, "expert_demonstrations.pkl")
    
    # ========== 步骤 2: 离线训练 ==========
    print("\n[步骤 2/4] 离线 CQL 训练")
    
    # 创建离线 RL 训练器
    trainer = OfflineRLTrainer(
        env_id=ENV_ID,
        algorithm="cql",
        actor_lr=3e-4,
        critic_lr=3e-4
    )
    
    # 准备数据集
    mdp_dataset = trainer.prepare_dataset(dataset)
    
    # 离线训练
    trainer.train_offline(
        mdp_dataset,
        n_steps=100000,
        save_interval=10000,
        save_dir="offline_models/cql"
    )
    
    # 保存离线训练的模型
    trainer.save("offline_models/cql_final.pt")
    
    # ========== 步骤 3: 在线 Fine-tuning ==========
    print("\n[步骤 3/4] 在线 Fine-tuning")
    
    # 创建环境进行 fine-tuning
    env = gym.make(ENV_ID)
    
    # Fine-tuning
    trainer.finetune_online(
        env,
        n_steps=10000,
        save_dir="finetuned_models"
    )
    
    # 保存 fine-tuned 模型
    trainer.save("finetuned_models/cql_finetuned.pt")
    
    # ========== 步骤 4: 评估性能 ==========
    print("\n[步骤 4/4] 性能评估")
    
    # TODO: 实现评估逻辑
    # - 对比原始 PPO vs. 离线 CQL vs. Fine-tuned CQL
    # - 样本效率对比
    # - 最终性能对比
    
    print("\n" + "="*60)
    print("离线 RL 训练完成!")
    print("="*60)
    print("\n下一步:")
    print("1. 查看训练日志和曲线")
    print("2. 使用 test_offline_policy.py 测试策略")
    print("3. 对比不同方法的性能")


if __name__ == "__main__":
    main()
