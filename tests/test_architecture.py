"""
架构优化测试套件
验证三层模型架构、动态负载均衡和多模态输入
"""

import logging

import pytest

# 导入测试目标模块

logger = logging.getLogger(__name__)
try:
    from agi_walker.core.controllers.load_monitor import (
        LoadMonitor,
        SimplePIDController,
        ControlMode,
    )
    from agi_walker.core.controllers.rag_knowledge_base import PhysicsKnowledgeBase

    MODULES_AVAILABLE = True
except ImportError:
    MODULES_AVAILABLE = False


def check_modules_available():
    if not MODULES_AVAILABLE:
        pytest.skip("架构核心模块不可用")


class TestLoadMonitor:
    """负载监控器测试"""

    def setup_method(self) -> None:
        """初始化测试环境"""
        self.pid = SimplePIDController(kp=2.0, ki=0.1, kd=0.5)
        self.monitor = LoadMonitor(self.pid)

    def test_initial_state(self) -> None:
        """测试初始状态"""
        check_modules_available()
        assert self.monitor.current_mode == ControlMode.AI
        assert self.monitor.total_samples == 0

    def test_latency_recording(self) -> None:
        """测试延迟记录"""
        check_modules_available()
        self.monitor.record_latency(15.0)
        self.monitor.record_latency(18.0)
        assert self.monitor.total_samples == 2
        assert self.monitor.ema_latency > 0

    def test_fallback_trigger(self) -> None:
        """测试延迟超标触发fallback"""
        check_modules_available()
        for _ in range(10):
            self.monitor.record_latency(30.0)
        assert self.monitor.current_mode == ControlMode.PID

    def test_recovery_from_fallback(self) -> None:
        """测试从fallback恢复"""
        check_modules_available()
        for _ in range(10):
            self.monitor.record_latency(30.0)
        assert self.monitor.current_mode == ControlMode.PID
        for _ in range(30):
            self.monitor.record_latency(10.0)
        assert self.monitor.current_mode != ControlMode.PID

    def test_pid_control_action(self) -> None:
        """测试PID控制动作"""
        check_modules_available()
        sensor_data = {"sensors": {"imu": {"orient": [5.0, -3.0, 0.0]}}}
        self.monitor.current_mode = ControlMode.PID
        action = self.monitor.get_control_action(sensor_data)
        assert "motors" in action
        assert "hip_left" in action["motors"]


class TestSimplePIDController:
    """简单PID控制器测试"""

    def setup_method(self) -> None:
        check_modules_available()
        self.pid = SimplePIDController(kp=2.0, ki=0.1, kd=0.5)

    def test_zero_error(self) -> None:
        """测试零误差时输出为零"""
        hip_left, hip_right = self.pid.compute(0.0, 0.0)
        assert abs(hip_left) < 0.1
        assert abs(hip_right) < 0.1

    def test_positive_roll(self) -> None:
        """测试正向Roll的响应"""
        hip_left, hip_right = self.pid.compute(10.0, 0.0)
        assert hip_left != hip_right

    def test_output_limits(self) -> None:
        """测试输出限幅"""
        hip_left, hip_right = self.pid.compute(90.0, 90.0)
        assert abs(hip_left) <= 45.0
        assert abs(hip_right) <= 45.0


class TestRAGKnowledgeBase:
    """RAG知识库测试"""

    def setup_method(self) -> None:
        check_modules_available()
        # 依赖于 conftest.py 提供的 sys.path
        from pathlib import Path

        project_root = Path(__file__).resolve().parent.parent
        index_path = project_root / "knowledge" / "test_index"
        self.kb = PhysicsKnowledgeBase(
            index_path=str(index_path),
            use_embeddings=False,
        )

    def test_knowledge_loading(self) -> None:
        """测试知识库加载"""
        assert len(self.kb.entries) > 0

    def test_keyword_retrieval(self) -> None:
        """测试关键词检索"""
        results = self.kb.retrieve("平衡控制", top_k=3)
        assert len(results) > 0

    def test_prompt_augmentation(self) -> None:
        """测试Prompt增强"""
        base_prompt = "请分析机器人状态"
        sensor_data = {
            "sensors": {"imu": {"orient": [15.0, -10.0, 0.0]}},
            "torso_height": 1.2,
        }
        augmented = self.kb.augment_prompt(base_prompt, sensor_data)
        assert len(augmented) > len(base_prompt)
        assert "物理知识" in augmented


def test_vision_processor_mock() -> None:
    """测试视觉处理器 (Mock模式)"""
    try:
        from agi_walker.core.api.vision_processor import create_vision_processor

        processor = create_vision_processor()
        assert processor is not None
    except ImportError:
        pytest.skip("视觉模块不可用")


def test_multimodal_fusion_mock() -> None:
    """测试多模态融合 (Mock模式)"""
    try:
        from agi_walker.core.api.multimodal_fusion import create_multimodal_fusion

        fusion = create_multimodal_fusion()
        assert fusion is not None
    except ImportError:
        pytest.skip("融合模块不可用")


@pytest.mark.integration
def test_load_monitor_integration() -> None:
    """测试负载监控集成"""
    check_modules_available()
    pid = SimplePIDController()
    monitor = LoadMonitor(pid)
    sensor_data = {"sensors": {"imu": {"orient": [10.0, -5.0, 0.0]}}}

    for lat in [15, 25, 35, 45, 55]:
        monitor.record_latency(lat)
        action = monitor.get_control_action(sensor_data)
        if monitor.current_mode == ControlMode.PID:
            assert "motors" in action
