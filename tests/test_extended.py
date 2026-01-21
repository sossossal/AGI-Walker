"""
扩展的单元测试套件
覆盖核心模块的主要功能
"""

import sys
import os
import unittest
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class TestZenohInterface(unittest.TestCase):
    """测试 Zenoh 接口"""
    
    def setUp(self):
        from python_api.zenoh_interface import ZenohInterface, ZENOH_AVAILABLE
        if not ZENOH_AVAILABLE:
            self.skipTest("Zenoh 未安装")
        self.zenoh = ZenohInterface()
    
    def tearDown(self):
        if hasattr(self, 'zenoh'):
            self.zenoh.close()
    
    def test_session_creation(self):
        """测试会话创建"""
        self.assertIsNotNone(self.zenoh.session)
    
    def test_publisher_creation(self):
        """测试发布者创建"""
        self.zenoh.declare_publisher("test/topic")
        self.assertIn("test/topic", self.zenoh.publishers)
    
    def test_publish(self):
        """测试发布消息"""
        self.zenoh.declare_publisher("test/data")
        test_data = {"value": 123}
        # 不应抛出异常
        self.zenoh.publish("test/data", test_data)
    
    def test_subscriber_creation(self):
        """测试订阅者创建"""
        def callback(data):
            pass
        self.zenoh.declare_subscriber("test/sub", callback)
        self.assertIn("test/sub", self.zenoh.subscribers)


class TestTaskEditor(unittest.TestCase):
    """测试任务编辑器"""
    
    def setUp(self):
        from python_api.task_editor import TaskEditor, TaskConfig
        self.editor = TaskEditor()
        self.TaskConfig = TaskConfig
    
    def test_create_task(self):
        """测试创建任务"""
        task = self.editor.create_task("test_task", "quadruped")
        self.assertEqual(task.name, "test_task")
        self.assertEqual(task.robot_type, "quadruped")
    
    def test_set_param(self):
        """测试设置参数"""
        task = self.editor.create_task("test", "quadruped")
        task.env_params = {"height": 0.5}
        self.editor.set_param(task, "env_params.height", 0.8)
        self.assertEqual(task.env_params["height"], 0.8)
    
    def test_save_load_task(self):
        """测试保存和加载"""
        import tempfile
        task = self.editor.create_task("test", "quadruped")
        task.env_params = {"test": 123}
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            filepath = f.name
        
        try:
            self.editor.save_task(task, filepath)
            loaded = self.editor.load_task(filepath)
            self.assertEqual(loaded.name, "test")
            self.assertEqual(loaded.env_params["test"], 123)
        finally:
            os.unlink(filepath)
    
    def test_compare_tasks(self):
        """测试任务对比"""
        task1 = self.editor.create_task("sim", "quadruped")
        task1.env_params = {"height": 0.5}
        
        task2 = self.editor.create_task("real", "quadruped")
        task2.env_params = {"height": 0.6}
        
        diff = self.editor.compare_tasks(task1, task2)
        self.assertIn("height", diff["env_params"])


class TestPartsManager(unittest.TestCase):
    """测试零件管理器"""
    
    def setUp(self):
        from python_api.parts_manager import PartsManager
        self.pm = PartsManager()
    
    def test_load_parts(self):
        """测试零件加载"""
        self.assertGreater(len(self.pm.parts_db), 0)
    
    def test_get_part(self):
        """测试获取零件"""
        motor = self.pm.get_part("go_m8010")
        self.assertIsNotNone(motor)
        self.assertEqual(motor.specs["max_torque_nm"], 23.7)
    
    def test_calculate_bom(self):
        """测试 BOM 计算"""
        bom = self.pm.calculate_bom(["go_m8010", "lipo_4s_5000mah"])
        self.assertGreater(bom["total_cost_usd"], 0)
        self.assertGreater(bom["total_weight_kg"], 0)


class TestTaskEnvironments(unittest.TestCase):
    """测试任务环境"""
    
    def test_stair_climbing_env(self):
        """测试楼梯攀爬环境"""
        import gymnasium as gym
        from examples.tasks.stair_climbing.env import StairClimbingEnv
        
        gym.register(id='StairClimbing-Test', entry_point=StairClimbingEnv)
        env = gym.make('StairClimbing-Test')
        
        obs, info = env.reset()
        self.assertEqual(obs.shape, env.observation_space.shape)
        
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        self.assertIsInstance(reward, float)
    
    def test_object_grasping_env(self):
        """测试物体抓取环境"""
        import gymnasium as gym
        from examples.tasks.object_grasping.env import ObjectGraspingEnv
        
        gym.register(id='ObjectGrasping-Test', entry_point=ObjectGraspingEnv)
        env = gym.make('ObjectGrasping-Test')
        
        obs, info = env.reset()
        self.assertEqual(obs.shape, env.observation_space.shape)
        
        for _ in range(10):
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)
            if terminated or truncated:
                break


def run_tests():
    """运行所有测试"""
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # 添加所有测试类
    suite.addTests(loader.loadTestsFromTestCase(TestZenohInterface))
    suite.addTests(loader.loadTestsFromTestCase(TestTaskEditor))
    suite.addTests(loader.loadTestsFromTestCase(TestPartsManager))
    suite.addTests(loader.loadTestsFromTestCase(TestTaskEnvironments))
    
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # 打印覆盖率报告
    print("\n" + "="*60)
    print("测试总结")
    print("="*60)
    print(f"运行: {result.testsRun}")
    print(f"成功: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"失败: {len(result.failures)}")
    print(f"错误: {len(result.errors)}")
    print(f"跳过: {len(result.skipped)}")
    
    return result.wasSuccessful()


if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)
