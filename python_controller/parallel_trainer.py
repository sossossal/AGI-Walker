"""
并行训练管理器
支持多环境并行训练
"""

import multiprocessing as mp
from typing import List, Dict, Any, Callable
import numpy as np
import time


class ParallelTrainingManager:
    """
    并行训练管理器
    
    功能:
    - 多进程环境并行
    - 自动负载均衡
    - 结果聚合
    - 资源监控
    """
    
    def __init__(self, num_workers: int = None):
        self.num_workers = num_workers or mp.cpu_count()
        self.workers: List[mp.Process] = []
        self.result_queue = mp.Queue()
        self.task_queue = mp.Queue()
        
        print(f"🚀 并行训练管理器初始化")
        print(f"   - 工作进程数: {self.num_workers}")
    
    def worker_process(self, worker_id: int, env_fn: Callable, policy_fn: Callable):
        """工作进程"""
        print(f"Worker {worker_id} 启动")
        
        # 创建环境
        env = env_fn()
        policy = policy_fn()
        
        while True:
            try:
                # 获取任务
                task = self.task_queue.get(timeout=1)
                
                if task == "STOP":
                    break
                
                # 执行训练
                num_episodes = task.get("num_episodes", 10)
                results = []
                
                for episode in range(num_episodes):
                    obs, info = env.reset()
                    total_reward = 0
                    steps = 0
                    
                    while True:
                        action = policy.get_action(obs)
                        obs, reward, terminated, truncated, info = env.step(action)
                        total_reward += reward
                        steps += 1
                        
                        if terminated or truncated or steps > 1000:
                            break
                    
                    results.append({
                        "episode": episode,
                        "reward": total_reward,
                        "steps": steps
                    })
                
                # 返回结果
                self.result_queue.put({
                    "worker_id": worker_id,
                    "results": results
                })
                
            except Exception as e:
                print(f"Worker {worker_id} 错误: {e}")
                break
        
        print(f"Worker {worker_id} 停止")
    
    def start_workers(self, env_fn: Callable, policy_fn: Callable):
        """启动工作进程"""
        for i in range(self.num_workers):
            p = mp.Process(
                target=self.worker_process,
                args=(i, env_fn, policy_fn)
            )
            p.start()
            self.workers.append(p)
        
        print(f"✅ {self.num_workers} 个工作进程已启动")
    
    def submit_task(self, task: Dict[str, Any]):
        """提交训练任务"""
        self.task_queue.put(task)
    
    def get_results(self, timeout: float = None) -> List[Dict]:
        """获取训练结果"""
        results = []
        
        while not self.result_queue.empty():
            try:
                result = self.result_queue.get(timeout=timeout)
                results.append(result)
            except:
                break
        
        return results
    
    def stop_workers(self):
        """停止所有工作进程"""
        for _ in range(self.num_workers):
            self.task_queue.put("STOP")
        
        for worker in self.workers:
            worker.join(timeout=5)
        
        print("🛑 所有工作进程已停止")


# ==================== 简化的策略类 (用于测试) ====================

class DummyPolicy:
    """虚拟策略 (用于测试)"""
    def get_action(self, obs):
        return np.random.randn(8)


# ==================== 示例代码 ====================

if __name__ == "__main__":
    import gymnasium as gym
    
    print("🧪 并行训练管理器测试\n")
    
    # 环境工厂函数
    def make_env():
        return gym.make('CartPole-v1')
    
    # 策略工厂函数
    def make_policy():
        return DummyPolicy()
    
    # 创建管理器
    manager = ParallelTrainingManager(num_workers=4)
    
    # 启动工作进程
    manager.start_workers(make_env, make_policy)
    
    # 提交任务
    for i in range(4):
        manager.submit_task({"num_episodes": 5})
    
    # 等待结果
    print("\n等待训练完成...")
    time.sleep(10)
    
    # 获取结果
    results = manager.get_results()
    print(f"\n收到 {len(results)} 个结果")
    
    for result in results:
        worker_id = result["worker_id"]
        avg_reward = np.mean([r["reward"] for r in result["results"]])
        print(f"Worker {worker_id}: 平均奖励 = {avg_reward:.2f}")
    
    # 停止
    manager.stop_workers()
