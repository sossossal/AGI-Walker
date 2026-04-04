"""
扩展的单元测试套件
覆盖核心模块的主要功能
"""

import logging
from typing import Any, Optional, Dict, List, Tuple
logger = logging.getLogger(__name__)
import sys
import os
import pytest


def get_np():
    import numpy as np
    return np


class TestZenohInterface:
    """测试 Zenoh 接口"""

    def setup_method(self) -> None:
        try:
            from agi_walker.core.api.comm.zenoh_interface import ZenohInterface, ZENOH_AVAILABLE
            if not ZENOH_AVAILABLE:
                pytest.skip("Zenoh 未安装")
            self.zenoh = ZenohInterface()
        except Exception as e:
            pytest.skip(f"无法在当前环境下创建 Zenoh 会话: {e}")

    def teardown_method(self) -> None:
        if hasattr(self, "zenoh"):
            self.zenoh.close()

    def test_session_creation(self) -> None:
        """测试会话创建"""
        assert self.zenoh.session is not None

    def test_publisher_creation(self) -> None:
        """测试发布者创建"""
        self.zenoh.declare_publisher("test/topic")
        assert "test/topic" in self.zenoh.publishers

    def test_publish(self) -> None:
        """测试发布消息"""
        self.zenoh.declare_publisher("test/data")
        test_data = {"value": 123}
        self.zenoh.publish("test/data", test_data)

    def test_subscriber_creation(self) -> None:
        """测试订阅者创建"""
        def callback(data):
            pass
        self.zenoh.declare_subscriber("test/sub", callback)
        assert "test/sub" in self.zenoh.subscribers


class TestTaskEditor:
    """测试任务编辑器"""

    def setup_method(self) -> None:
        from agi_walker.core.api.task_editor import TaskEditor
        self.editor = TaskEditor()

    def test_create_task(self) -> None:
        """测试创建任务"""
        task = self.editor.create_task("test_task", "quadruped")
        assert task.name == "test_task"
        assert task.robot_type == "quadruped"

    def test_set_param(self) -> None:
        """测试设置参数"""
        task = self.editor.create_task("test", "quadruped")
        task.env_params = {"height": 0.5}
        self.editor.set_param(task, "env_params.height", 0.8)
        assert task.env_params["height"] == 0.8

    def test_save_load_task(self) -> None:
        """测试保存和加载"""
        import tempfile
        task = self.editor.create_task("test", "quadruped")
        task.env_params = {"test": 123}
        with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
            filepath = f.name
        try:
            self.editor.save_task(task, filepath)
            loaded = self.editor.load_task(filepath)
            assert loaded.name == "test"
            assert loaded.env_params["test"] == 123
        finally:
            if os.path.exists(filepath):
                os.unlink(filepath)


class TestPartsManager:
    """测试零件管理器"""

    def setup_method(self) -> None:
        from agi_walker.core.api.parts.parts_manager import PartsManager
        self.pm = PartsManager()

    def test_load_parts(self) -> None:
        """测试零件加载"""
        assert len(self.pm.parts_db) > 0

    def test_get_part(self) -> None:
        """测试获取零件"""
        motor = self.pm.get_part("go_m8010")
        assert motor is not None
        assert motor.specs["max_torque_nm"] == 23.7

    def test_calculate_bom(self) -> None:
        """测试 BOM 计算"""
        bom = self.pm.calculate_bom(["go_m8010", "lipo_4s_5000mah"])
        assert bom["total_cost_usd"] > 0
        assert bom["total_weight_kg"] > 0


class TestTaskEnvironments:
    """测试任务环境"""

    def test_stair_climbing_env(self) -> None:
        """测试楼梯攀爬环境"""
        try:
            import gymnasium as gym
            from examples.tasks.stair_climbing.env import StairClimbingEnv
            # 避免重复注册
            try:
                gym.register(id="StairClimbing-Test", entry_point=StairClimbingEnv)
            except:
                logger.warning("Exception occurred")
            env = gym.make("StairClimbing-Test")
        except Exception as e:
            pytest.skip(f"无法在当前环境下创建 Gym 环境: {e}")

        obs, info = env.reset()
        assert obs.shape == env.observation_space.shape
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        assert isinstance(reward, (int, float))

    def test_object_grasping_env(self) -> None:
        """测试物体抓取环境"""
        try:
            import gymnasium as gym
            from examples.tasks.object_grasping.env import ObjectGraspingEnv
            try:
                gym.register(id="ObjectGrasping-Test", entry_point=ObjectGraspingEnv)
            except:
                logger.warning("Exception occurred")
            env = gym.make("ObjectGrasping-Test")
        except Exception as e:
            pytest.skip(f"无法在当前环境下创建 Gym 环境: {e}")

        obs, info = env.reset()
        assert obs.shape == env.observation_space.shape
        for _ in range(5):
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)
            if terminated or truncated:
                break
