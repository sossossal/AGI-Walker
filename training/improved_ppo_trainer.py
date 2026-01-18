"""
AGI-Walker 改进的 PPO 训练器
包含：超参数优化、课程学习、奖励塑形
"""

import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, EvalCallback
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
import torch.nn as nn

class CurriculumScheduler:
    """课程学习调度器"""
    def __init__(self):
        self.stages = [
            {'name': 'easy', 'steps': 100000, 'difficulty': 0.3},
            {'name': 'medium', 'steps': 200000, 'difficulty': 0.6},
            {'name': 'hard', 'steps': 300000, 'difficulty': 1.0},
        ]
        
    def get_difficulty(self, total_steps):
        """根据训练步数返回当前难度"""
        cumulative = 0
        for stage in self.stages:
            cumulative += stage['steps']
            if total_steps < cumulative:
                return stage['difficulty']
        return 1.0  # 最高难度

class ShapedRewardWrapper(gym.Wrapper):
    """奖励塑形包装器"""
    def __init__(self, env):
        super().__init__(env)
        self.prev_distance = None
        self.action_history = []
        
    def reset(self, **kwargs):
        self.prev_distance = None
        self.action_history = []
        return super().reset(**kwargs)
    
    def step(self, action):
        obs, reward, terminated, truncated, info = super().step(action)
        
        # 1. 原始任务奖励
        shaped_reward = reward
        
        # 2. 进度奖励（靠近目标）
        if 'target_position' in info and 'position' in info:
            distance = np.linalg.norm(
                np.array(info['target_position']) - np.array(info['position'])
            )
            if self.prev_distance is not None:
                progress = (self.prev_distance - distance) * 10.0
                shaped_reward += progress
            self.prev_distance = distance
        
        # 3. 能量惩罚（鼓励节能）
        energy_penalty = -0.01 * np.sum(np.square(action))
        shaped_reward += energy_penalty
        
        # 4. 平滑性奖励（鼓励平稳动作）
        self.action_history.append(action)
        if len(self.action_history) > 1:
            smoothness = -0.05 * np.sum(
                np.square(self.action_history[-1] - self.action_history[-2])
            )
            shaped_reward += smoothness
        
        # 限制历史长度
        if len(self.action_history) > 10:
            self.action_history.pop(0)
        
        return obs, shaped_reward, terminated, truncated, info

class CurriculumCallback(BaseCallback):
    """课程学习回调"""
    def __init__(self, scheduler, verbose=0):
        super().__init__(verbose)
        self.scheduler = scheduler
        self.current_difficulty = 0.3
        
    def _on_step(self):
        new_difficulty = self.scheduler.get_difficulty(self.num_timesteps)
        if new_difficulty != self.current_difficulty:
            self.current_difficulty = new_difficulty
            print(f"\n[Curriculum] Difficulty: {new_difficulty:.1f} "
                  f"(Steps: {self.num_timesteps})")
            
            # 更新环境难度
            if hasattr(self.training_env, 'set_attr'):
                self.training_env.set_attr('difficulty', new_difficulty)
        
        return True

def create_improved_ppo_trainer(
    env_id="AGI-Walker/Walker2D-v0",
    total_timesteps=1_000_000,
    use_curriculum=True,
    use_shaped_reward=True,
    save_path="models/improved_ppo"
):
    """
    创建改进的 PPO 训练器
    
    参数:
        env_id: 环境 ID
        total_timesteps: 总训练步数
        use_curriculum: 是否使用课程学习
        use_shaped_reward: 是否使用奖励塑形
        save_path: 模型保存路径
    """
    
    # 1. 创建环境
    def make_env():
        env = gym.make(env_id)
        if use_shaped_reward:
            env = ShapedRewardWrapper(env)
        return env
    
    env = DummyVecEnv([make_env])
    env = VecNormalize(env, norm_obs=True, norm_reward=True)
    
    # 2. 改进的 PPO 配置
    policy_kwargs = dict(
        net_arch=[dict(pi=[256, 256], vf=[256, 256])],
        activation_fn=nn.ReLU,
    )
    
    # 自适应学习率
    def learning_rate_schedule(progress_remaining):
        return 3e-4 * progress_remaining
    
    model = PPO(
        "MlpPolicy",
        env,
        
        # 学习率
        learning_rate=learning_rate_schedule,
        
        # PPO 核心参数
        n_steps=2048,           # 每次更新的步数
        batch_size=64,          # Mini-batch 大小
        n_epochs=10,            # 每次更新的 epoch 数
        gamma=0.99,             # 折扣因子
        gae_lambda=0.95,        # GAE lambda
        
        # 裁剪和正则化
        clip_range=0.2,         # PPO 裁剪范围
        clip_range_vf=None,     # Value function 裁剪
        ent_coef=0.01,          # 熵系数（鼓励探索）
        vf_coef=0.5,            # Value function 系数
        max_grad_norm=0.5,      # 梯度裁剪
        
        # 网络
        policy_kwargs=policy_kwargs,
        
        # 其他
        verbose=1,
        tensorboard_log="./tensorboard_logs/",
    )
    
    # 3. 设置回调
    callbacks = []
    
    # 课程学习
    if use_curriculum:
        scheduler = CurriculumScheduler()
        curriculum_callback = CurriculumCallback(scheduler)
        callbacks.append(curriculum_callback)
    
    # 评估回调
    eval_env = DummyVecEnv([make_env])
    eval_env = VecNormalize(eval_env, norm_obs=True, norm_reward=False)
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=save_path,
        log_path='./logs/',
        eval_freq=10000,
        deterministic=True,
        render=False
    )
    callbacks.append(eval_callback)
    
    # 4. 训练
    print("\n" + "="*50)
    print("开始改进的 PPO 训练")
    print("="*50)
    print(f"环境: {env_id}")
    print(f"总步数: {total_timesteps:,}")
    print(f"课程学习: {use_curriculum}")
    print(f"奖励塑形: {use_shaped_reward}")
    print("="*50 + "\n")
    
    model.learn(
        total_timesteps=total_timesteps,
        callback=callbacks,
        progress_bar=True
    )
    
    # 5. 保存最终模型
    model.save(f"{save_path}/final_model")
    env.save(f"{save_path}/vec_normalize.pkl")
    
    print("\n" + "="*50)
    print("训练完成！")
    print(f"模型保存至: {save_path}")
    print("="*50 + "\n")
    
    return model, env

if __name__ == "__main__":
    # 运行改进的训练
    model, env = create_improved_ppo_trainer(
        env_id="AGI-Walker/Walker2D-v0",
        total_timesteps=500_000,  # 测试用，实际应该更多
        use_curriculum=True,
        use_shaped_reward=True,
        save_path="models/improved_ppo_v1"
    )
    
    print("训练完成！可以使用以下命令测试：")
    print("  python test_improved_policy.py")
