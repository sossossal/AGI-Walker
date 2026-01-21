"""
任务编辑器 (Task Editor)
支持可视化创建、编辑和对比虚拟与现实任务参数
"""

import json
import os
from typing import Dict, Any, Optional
from dataclasses import dataclass, asdict


@dataclass
class TaskConfig:
    """任务配置"""
    name: str
    description: str
    difficulty: str
    robot_type: str
    env_params: Dict[str, Any]
    reward_weights: Dict[str, float]
    termination_conditions: Dict[str, Any]
    observation_config: Dict[str, bool]
    action_config: Dict[str, Any]


class TaskEditor:
    """任务编辑器 - 创建、编辑、对比任务"""
    
    def __init__(self, tasks_dir: str = "examples/tasks"):
        self.tasks_dir = tasks_dir
    
    def create_task(self, name: str, robot_type: str) -> TaskConfig:
        """创建新任务"""
        return TaskConfig(
            name=name,
            description=f"Custom task: {name}",
            difficulty="medium",
            robot_type=robot_type,
            env_params={},
            reward_weights={},
            termination_conditions={},
            observation_config={},
            action_config={}
        )
    
    def set_param(self, task: TaskConfig, param_path: str, value: Any):
        """设置参数"""
        parts = param_path.split('.')
        obj = task
        for part in parts[:-1]:
            obj = getattr(obj, part)
        if isinstance(obj, dict):
            obj[parts[-1]] = value
        else:
            setattr(obj, parts[-1], value)
    
    def save_task(self, task: TaskConfig, filepath: str):
        """保存任务"""
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(asdict(task), f, indent=2, ensure_ascii=False)
        print(f"✅ 任务已保存: {filepath}")
    
    def load_task(self, filepath: str) -> TaskConfig:
        """加载任务"""
        with open(filepath, 'r', encoding='utf-8') as f:
            data = json.load(f)
        return TaskConfig(**data)
    
    def compare_tasks(self, sim_task: TaskConfig, real_task: TaskConfig) -> Dict:
        """对比虚拟与现实任务"""
        diff = {"env_params": {}, "reward_weights": {}}
        
        for key in set(sim_task.env_params.keys()) | set(real_task.env_params.keys()):
            sim_val = sim_task.env_params.get(key)
            real_val = real_task.env_params.get(key)
            if sim_val != real_val:
                diff["env_params"][key] = {"sim": sim_val, "real": real_val}
        
        return diff
