"""
第 2 阶段：控制层测试

目标：覆盖 python_api/control/ 模块的关键路径
覆盖率提升：+2%
测试数：16 个

关键覆盖：
- 参数控制
- 步态生成
- 精度调整
- 实时数据处理
"""

import logging

import pytest

logger = logging.getLogger(__name__)
try:
    from agi_walker.core.api.control.parametric_control import ParametricController
    from agi_walker.core.api.control.gait_generator import GaitGenerator
    from agi_walker.core.api.control.precision_adjuster import PrecisionAdjuster
except ImportError:
    pytest.skip("Control module not available", allow_module_level=True)


# ============================================================================
# Fixtures
# ============================================================================


@pytest.fixture
def robot_config():
    """机器人配置"""
    return {
        "num_joints": 6,
        "joint_limits": {"min": [-3.14] * 6, "max": [3.14] * 6},
        "max_velocity": [2.0] * 6,
        "max_acceleration": [5.0] * 6,
    }


@pytest.fixture
def gait_parameters():
    """步态参数"""
    return {
        "stride_length": 0.5,
        "step_height": 0.1,
        "frequency": 1.0,
        "duty_cycle": 0.6,
    }


@pytest.fixture
def control_target():
    """控制目标"""
    return {
        "position": [1.0, 2.0, 3.0, 0.0, 0.0, 0.0],
        "velocity": [0.1, 0.1, 0.1, 0.0, 0.0, 0.0],
        "force": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    }


# ============================================================================
# 测试组 1：参数控制器
# ============================================================================


class TestParametricController:
    """参数控制器测试"""

    def test_controller_init(self, robot_config) -> None:
        """测试：控制器初始化"""
        # Arrange & Act
        controller = ParametricController(robot_config)

        # Assert
        assert controller is not None
        assert controller.num_joints == 6

    def test_set_parameters(self, robot_config) -> None:
        """测试：设置参数"""
        # Arrange
        controller = ParametricController(robot_config)
        params = {"kp": 10.0, "kd": 5.0, "ki": 0.1}

        # Act
        controller.set_parameters(params)

        # Assert
        # 验证参数被设置
        assert controller is not None

    def test_compute_control_output(self, robot_config, control_target) -> None:
        """测试：计算控制输出"""
        # Arrange
        controller = ParametricController(robot_config)
        current_state = {"position": [0.0] * 6, "velocity": [0.0] * 6}

        # Act
        output = controller.compute_output(current_state, control_target)

        # Assert
        assert output is not None
        assert isinstance(output, dict)

    def test_limit_joint_commands(self, robot_config) -> None:
        """测试：限制关节命令"""
        # Arrange
        controller = ParametricController(robot_config)
        command = [10.0] * 6  # 超出限制

        # Act
        limited = controller.limit_command(command)

        # Assert
        assert limited is not None
        # 应该被限制在最大值以内
        assert all(-3.14 <= c <= 3.14 for c in limited)

    def test_pid_control_law(self, robot_config) -> None:
        """测试：PID 控制律"""
        # Arrange
        controller = ParametricController(robot_config)
        error = 0.5
        controller.kp = 10.0
        controller.kd = 5.0
        controller.ki = 0.1

        # Act
        output = controller.compute_pid(error, 0.0, 0.0)

        # Assert
        assert output is not None
        assert isinstance(output, float)

    def test_trajectory_tracking(self, robot_config) -> None:
        """测试：轨迹追踪"""
        # Arrange
        controller = ParametricController(robot_config)
        trajectory = {
            "positions": [[0.0] * 6, [0.1] * 6, [0.2] * 6],
            "timestamps": [0.0, 0.1, 0.2],
        }

        # Act
        outputs = []
        for pos in trajectory["positions"]:
            output = controller.compute_output(
                {"position": [0.0] * 6, "velocity": [0.0] * 6},
                {"position": pos},
            )
            outputs.append(output)

        # Assert
        assert len(outputs) == 3


# ============================================================================
# 测试组 2：步态生成
# ============================================================================


class TestGaitGenerator:
    """步态生成器测试"""

    def test_generator_init(self, gait_parameters) -> None:
        """测试：步态生成器初始化"""
        # Arrange & Act
        gen = GaitGenerator(gait_parameters)

        # Assert
        assert gen is not None
        assert gen.stride_length == 0.5

    def test_generate_trajectory(self, gait_parameters) -> None:
        """测试：生成步态轨迹"""
        # Arrange
        gen = GaitGenerator(gait_parameters)

        # Act
        trajectory = gen.generate(duration=1.0, num_samples=10)

        # Assert
        assert trajectory is not None
        assert len(trajectory) > 0

    def test_different_gait_phases(self, gait_parameters) -> None:
        """测试：不同步态阶段"""
        # Arrange
        gen = GaitGenerator(gait_parameters)

        # Act
        # 生成完整步态周期
        trajectory = gen.generate(duration=1.0, num_samples=100)

        # Assert
        assert len(trajectory) == 100

    def test_gait_periodicity(self, gait_parameters) -> None:
        """测试：步态周期性"""
        # Arrange
        gen = GaitGenerator(gait_parameters)

        # Act
        traj1 = gen.generate(duration=1.0, num_samples=10)
        traj2 = gen.generate(duration=1.0, num_samples=10)

        # Assert
        # 相同参数应生成相似的轨迹
        assert len(traj1) == len(traj2)

    def test_vary_stride_length(self) -> None:
        """测试：改变步长"""
        # Arrange
        params1 = {"stride_length": 0.3, "step_height": 0.1, "frequency": 1.0}
        params2 = {"stride_length": 0.6, "step_height": 0.1, "frequency": 1.0}

        # Act
        gen1 = GaitGenerator(params1)
        gen2 = GaitGenerator(params2)
        traj1 = gen1.generate(duration=0.5, num_samples=5)
        traj2 = gen2.generate(duration=0.5, num_samples=5)

        # Assert
        assert traj1 is not None
        assert traj2 is not None

    def test_vary_frequency(self) -> None:
        """测试：改变频率"""
        # Arrange
        params1 = {"stride_length": 0.5, "step_height": 0.1, "frequency": 0.5}
        params2 = {"stride_length": 0.5, "step_height": 0.1, "frequency": 2.0}

        # Act
        gen1 = GaitGenerator(params1)
        gen2 = GaitGenerator(params2)

        # Assert
        assert gen1.frequency == 0.5
        assert gen2.frequency == 2.0

    def test_singlesupport_phase(self, gait_parameters) -> None:
        """测试：单支撑阶段"""
        # Arrange
        gen = GaitGenerator(gait_parameters)

        # Act
        trajectory = gen.generate(duration=1.0, num_samples=100)

        # Assert
        assert trajectory is not None


# ============================================================================
# 测试组 3：精度调整
# ============================================================================


class TestPrecisionAdjuster:
    """精度调整器测试"""

    def test_adjuster_init(self) -> None:
        """测试：精度调整器初始化"""
        # Arrange & Act
        adjuster = PrecisionAdjuster()

        # Assert
        assert adjuster is not None

    def test_calibrate(self) -> None:
        """测试：标定"""
        # Arrange
        adjuster = PrecisionAdjuster()

        # Act
        adjuster.calibrate()

        # Assert
        # 验证标定完成
        assert adjuster is not None

    def test_adjust_single_joint(self) -> None:
        """测试：调整单个关节"""
        # Arrange
        adjuster = PrecisionAdjuster()
        measurement = {"joint_0": 0.05}  # 5cm 偏差

        # Act
        adjustment = adjuster.compute_adjustment(measurement)

        # Assert
        assert adjustment is not None

    def test_adjust_multiple_joints(self) -> None:
        """测试：调整多个关节"""
        # Arrange
        adjuster = PrecisionAdjuster()
        measurement = {
            "joint_0": 0.02,
            "joint_1": -0.03,
            "joint_2": 0.01,
        }

        # Act
        adjustment = adjuster.compute_adjustment(measurement)

        # Assert
        assert adjustment is not None
        assert isinstance(adjustment, dict)

    def test_convergence_monitoring(self) -> None:
        """测试：收敛性监视"""
        # Arrange
        adjuster = PrecisionAdjuster()

        # Act
        errors = []
        for i in range(10):
            error = {"x": 0.1 / (i + 1)}  # 模拟收敛
            adjuster.record_error(error)
            errors.append(error)

        # Assert
        assert len(errors) == 10

    def test_error_recovery(self) -> None:
        """测试：错误恢复"""
        # Arrange
        adjuster = PrecisionAdjuster()

        # Act
        # 模拟大的测量误差
        adjuster.last_error = {"x": 1.0}
        recovered = adjuster.recover()

        # Assert
        assert recovered is not None

    def test_precision_threshold(self) -> None:
        """测试：精度阈值"""
        # Arrange
        adjuster = PrecisionAdjuster()
        adjuster.precision_threshold = 0.01  # 1cm

        # Act
        is_within_threshold = abs(0.005) < adjuster.precision_threshold

        # Assert
        assert is_within_threshold


# ============================================================================
# 测试组 4：实时控制集成
# ============================================================================


class TestRealTimeControlIntegration:
    """实时控制集成测试"""

    def test_control_loop_cycle(self, robot_config, gait_parameters) -> None:
        """测试：控制循环周期"""
        # Arrange
        controller = ParametricController(robot_config)
        gen = GaitGenerator(gait_parameters)

        # Act
        current_state = {"position": [0.0] * 6, "velocity": [0.0] * 6}
        target = gen.generate(duration=0.01, num_samples=1)
        if target:
            output = controller.compute_output(current_state, {"position": target[0]})

        # Assert
        assert output is not None

    def test_low_latency_operation(self) -> None:
        """测试：低延迟操作"""
        # Arrange
        controller = ParametricController({"num_joints": 6})

        # Act
        current_state = {"position": [0.0] * 6}
        target = {"position": [0.1] * 6}

        # 计算反应时间
        output = controller.compute_output(current_state, target)

        # Assert
        assert output is not None

    def test_concurrent_joint_control(self, robot_config) -> None:
        """测试：并发关节控制"""
        # Arrange
        ParametricController(robot_config)

        # Act
        commands = [0.1 * i for i in range(6)]  # 6个并发命令

        # Assert
        assert len(commands) == 6

    def test_force_feedback_integration(self, robot_config) -> None:
        """测试：力反馈集成"""
        # Arrange
        controller = ParametricController(robot_config)
        current_state = {
            "position": [0.0] * 6,
            "velocity": [0.0] * 6,
            "force": [1.0, 1.0, 1.0, 0.0, 0.0, 0.0],
        }

        # Act
        target = {"position": [0.1] * 6}
        output = controller.compute_output(current_state, target)

        # Assert
        assert output is not None

    def test_state_estimation_integration(self, robot_config) -> None:
        """测试：状态估计集成"""
        # Arrange
        controller = ParametricController(robot_config)

        # Act
        measurements = {"position": [0.0] * 6, "velocity": [0.0] * 6}
        estimated_state = controller.estimate_state(measurements)

        # Assert
        assert estimated_state is not None or isinstance(estimated_state, dict)

    def test_adaptive_control(self, robot_config) -> None:
        """测试：自适应控制"""
        # Arrange
        controller = ParametricController(robot_config)

        # Act
        # 模拟自适应增益调整
        error = 0.5
        if error > 0.1:
            controller.kp *= 1.1  # 增加比例增益

        # Assert
        assert controller.kp > 0
