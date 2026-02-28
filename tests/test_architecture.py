"""
架构优化测试套件
验证三层模型架构、动态负载均衡和多模态输入
"""

import json
import time
import sys
import unittest
from unittest.mock import Mock, patch, MagicMock

# 确保导入路径正确
sys.path.insert(0, '.')

# 导入测试目标模块
try:
    from medium_model import MediumModel
    from model_orchestrator import ModelOrchestrator, create_orchestrator, ModelTier
    from load_monitor import LoadMonitor, SimplePIDController, ControlMode
    from rag_knowledge_base import PhysicsKnowledgeBase
    MODULES_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ 模块导入失败: {e}")
    MODULES_AVAILABLE = False


class TestLoadMonitor(unittest.TestCase):
    """负载监控器测试"""
    
    def setUp(self):
        """初始化测试环境"""
        self.pid = SimplePIDController(kp=2.0, ki=0.1, kd=0.5)
        self.monitor = LoadMonitor(self.pid)
    
    def test_initial_state(self):
        """测试初始状态"""
        self.assertEqual(self.monitor.current_mode, ControlMode.AI)
        self.assertEqual(self.monitor.total_samples, 0)
    
    def test_latency_recording(self):
        """测试延迟记录"""
        self.monitor.record_latency(15.0)
        self.monitor.record_latency(18.0)
        
        self.assertEqual(self.monitor.total_samples, 2)
        self.assertGreater(self.monitor.ema_latency, 0)
    
    def test_fallback_trigger(self):
        """测试延迟超标触发fallback"""
        # 模拟连续超标
        for _ in range(10):
            self.monitor.record_latency(30.0)  # 超过20ms阈值
        
        self.assertEqual(self.monitor.current_mode, ControlMode.PID)
    
    def test_recovery_from_fallback(self):
        """测试从fallback恢复"""
        # 先触发fallback
        for _ in range(10):
            self.monitor.record_latency(30.0)
        
        self.assertEqual(self.monitor.current_mode, ControlMode.PID)
        
        # 延迟恢复正常
        for _ in range(30):
            self.monitor.record_latency(10.0)
        
        # 应该进入HYBRID或AI模式
        self.assertNotEqual(self.monitor.current_mode, ControlMode.PID)
    
    def test_pid_control_action(self):
        """测试PID控制动作"""
        sensor_data = {
            "sensors": {
                "imu": {"orient": [5.0, -3.0, 0.0]}
            }
        }
        
        # 强制进入PID模式
        self.monitor.current_mode = ControlMode.PID
        
        action = self.monitor.get_control_action(sensor_data)
        
        self.assertIn('motors', action)
        self.assertIn('hip_left', action['motors'])
        self.assertIn('hip_right', action['motors'])


class TestSimplePIDController(unittest.TestCase):
    """简单PID控制器测试"""
    
    def setUp(self):
        self.pid = SimplePIDController(kp=2.0, ki=0.1, kd=0.5)
    
    def test_zero_error(self):
        """测试零误差时输出为零"""
        hip_left, hip_right = self.pid.compute(0.0, 0.0)
        
        self.assertAlmostEqual(hip_left, 0.0, places=1)
        self.assertAlmostEqual(hip_right, 0.0, places=1)
    
    def test_positive_roll(self):
        """测试正向Roll的响应"""
        hip_left, hip_right = self.pid.compute(10.0, 0.0)
        
        # Roll正向时应该产生修正
        self.assertNotEqual(hip_left, hip_right)
    
    def test_output_limits(self):
        """测试输出限幅"""
        hip_left, hip_right = self.pid.compute(90.0, 90.0)
        
        self.assertLessEqual(abs(hip_left), 45.0)
        self.assertLessEqual(abs(hip_right), 45.0)
    
    def test_reset(self):
        """测试重置功能"""
        self.pid.compute(10.0, 10.0)
        self.pid.compute(10.0, 10.0)
        
        self.pid.reset()
        
        self.assertEqual(self.pid.integral_roll, 0.0)
        self.assertEqual(self.pid.integral_pitch, 0.0)


class TestRAGKnowledgeBase(unittest.TestCase):
    """RAG知识库测试"""
    
    def setUp(self):
        self.kb = PhysicsKnowledgeBase(
            index_path="d:/新建文件夹/AGI-Walker/knowledge/test_index",
            use_embeddings=False
        )
    
    def test_knowledge_loading(self):
        """测试知识库加载"""
        self.assertGreater(len(self.kb.entries), 0)
    
    def test_keyword_retrieval(self):
        """测试关键词检索"""
        results = self.kb.retrieve("平衡控制", top_k=3)
        
        self.assertGreater(len(results), 0)
        self.assertIsInstance(results[0][0].title, str)
        self.assertIsInstance(results[0][1], float)
    
    def test_category_filter(self):
        """测试类别过滤"""
        results = self.kb.retrieve("控制", top_k=5, category="balance")
        
        for entry, _ in results:
            self.assertEqual(entry.category, "balance")
    
    def test_prompt_augmentation(self):
        """测试Prompt增强"""
        base_prompt = "请分析机器人状态"
        sensor_data = {
            "sensors": {
                "imu": {"orient": [15.0, -10.0, 0.0]}
            },
            "torso_height": 1.2
        }
        
        augmented = self.kb.augment_prompt(base_prompt, sensor_data)
        
        # 增强后的Prompt应该包含知识
        self.assertGreater(len(augmented), len(base_prompt))
        self.assertIn("物理知识", augmented)
    
    def test_stats(self):
        """测试统计信息"""
        stats = self.kb.get_stats()
        
        self.assertIn("total_entries", stats)
        self.assertIn("categories", stats)
        self.assertGreater(stats["total_entries"], 0)


class TestVisionProcessor(unittest.TestCase):
    """视觉处理器测试"""
    
    def setUp(self):
        try:
            sys.path.insert(0, '../python_api')
            from vision_processor import create_vision_processor, DummyVisionProcessor
            self.processor = create_vision_processor()
            self.DummyProcessor = DummyVisionProcessor
        except ImportError:
            self.skipTest("视觉模块不可用")
    
    def test_processor_creation(self):
        """测试处理器创建"""
        self.assertIsNotNone(self.processor)
    
    def test_dummy_processor(self):
        """测试虚拟处理器"""
        import numpy as np
        
        dummy = self.DummyProcessor()
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        
        result = dummy.process_frame(frame)
        
        self.assertIn("frame_id", result)
        self.assertTrue(result.get("dummy", False))
    
    def test_stats(self):
        """测试统计信息"""
        stats = self.processor.get_stats()
        
        self.assertIn("frames_processed", stats)


class TestMultimodalFusion(unittest.TestCase):
    """多模态融合测试"""
    
    def setUp(self):
        try:
            sys.path.insert(0, '../python_api')
            from multimodal_fusion import create_multimodal_fusion
            self.fusion = create_multimodal_fusion()
        except ImportError:
            self.skipTest("融合模块不可用")
    
    def test_sensor_fusion(self):
        """测试传感器融合"""
        sensor_data = {
            "sensors": {
                "imu": {"orient": [5.0, -3.0, 0.0], "gyro": [0.1, 0.0, 0.0], "accel": [0, 0, -9.8]},
                "joints": {
                    "hip_left": {"angle": 10.0, "velocity": 0.0},
                    "hip_right": {"angle": -8.0, "velocity": 0.0}
                },
                "contacts": {"foot_left": True, "foot_right": True}
            },
            "torso_height": 1.45
        }
        
        fused = self.fusion.fuse_sensors(sensor_data)
        
        self.assertIn("fusion", fused)
        self.assertIn("stability", fused["fusion"])
        self.assertIn("confidence", fused["fusion"])
    
    def test_stability_calculation(self):
        """测试稳定性计算"""
        # 稳定状态
        stable_data = {
            "sensors": {
                "imu": {"orient": [2.0, 1.0, 0.0], "gyro": [0, 0, 0], "accel": [0, 0, -9.8]},
                "joints": {"hip_left": {"angle": 0}, "hip_right": {"angle": 0}},
                "contacts": {"foot_left": True, "foot_right": True}
            }
        }
        
        fused = self.fusion.fuse_sensors(stable_data)
        stability = fused["fusion"]["stability"]
        
        self.assertGreater(stability, 0.8)
        
        # 不稳定状态
        unstable_data = {
            "sensors": {
                "imu": {"orient": [25.0, -20.0, 0.0], "gyro": [1, 0, 0], "accel": [0, 0, -9.8]},
                "joints": {"hip_left": {"angle": 30}, "hip_right": {"angle": -25}},
                "contacts": {"foot_left": True, "foot_right": False}
            }
        }
        
        fused = self.fusion.fuse_sensors(unstable_data)
        stability = fused["fusion"]["stability"]
        
        self.assertLess(stability, 0.8)


class IntegrationTests(unittest.TestCase):
    """集成测试"""
    
    @unittest.skipIf(not MODULES_AVAILABLE, "模块不可用")
    def test_load_monitor_with_pid(self):
        """测试负载监控与PID集成"""
        pid = SimplePIDController()
        monitor = LoadMonitor(pid)
        
        sensor_data = {
            "sensors": {
                "imu": {"orient": [10.0, -5.0, 0.0]}
            }
        }
        
        # 模拟一系列延迟
        latencies = [15, 18, 25, 30, 35, 40, 45, 50, 55, 60]
        
        for lat in latencies:
            monitor.record_latency(lat)
            action = monitor.get_control_action(sensor_data)
            
            if monitor.current_mode == ControlMode.PID:
                self.assertIn('motors', action)


def run_tests():
    """运行所有测试"""
    print("=" * 60)
    print("AGI-Walker 架构优化测试套件")
    print("=" * 60 + "\n")
    
    # 创建测试套件
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # 添加测试类
    test_classes = [
        TestLoadMonitor,
        TestSimplePIDController,
        TestRAGKnowledgeBase,
        TestVisionProcessor,
        TestMultimodalFusion,
        IntegrationTests
    ]
    
    for test_class in test_classes:
        tests = loader.loadTestsFromTestCase(test_class)
        suite.addTests(tests)
    
    # 运行测试
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # 打印摘要
    print("\n" + "=" * 60)
    print("测试摘要")
    print("=" * 60)
    print(f"运行测试: {result.testsRun}")
    print(f"成功: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"失败: {len(result.failures)}")
    print(f"错误: {len(result.errors)}")
    print(f"跳过: {len(result.skipped)}")
    
    return result.wasSuccessful()


if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)
