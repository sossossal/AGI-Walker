import pytest
import time
import threading
import numpy as np
from python_api.godot_robot_env.gym_env import GodotRobotEnv, RandomizerConfig
from tests.mock_godot_server import MockGodotServer

@pytest.fixture
def mock_server():
    server = MockGodotServer(port=9992)
    thread = threading.Thread(target=server.start, daemon=True)
    thread.start()
    time.sleep(0.1)  # Wait for server to start
    yield server
    server.stop()

def test_domain_randomization(mock_server):
    """测试 Gym Env 的领域随机化功能与通信"""
    config = RandomizerConfig(
        mass_range=(0.5, 2.0),
        friction_range=(0.5, 2.0),
        motor_strength_range=(0.5, 2.0),
        motor_lag_range=(0.0, 0.0),  # 禁用延迟以加快测试
        sensor_noise_std=0.05,
        enable_randomization=True
    )
    
    env = GodotRobotEnv(
        port=9992,
        randomizer_config=config,
        timeout=2.0
    )
    
    obs, info = env.reset()
    
    # 验证物理参数是否被随机生成并写入 info 中
    assert "dynamics" in info
    dyn = info["dynamics"]
    assert 0.5 <= dyn["mass_scale"] <= 2.0
    assert 0.5 <= dyn["friction_scale"] <= 2.0
    assert 0.5 <= dyn["motor_strength"] <= 2.0
    
    # 验证通信是否畅通
    action = np.array([0.0, 10.0, -10.0, 0.0], dtype=np.float32)
    next_obs, reward, terminated, truncated, step_info = env.step(action)
    
    assert next_obs is not None
    # 验证加入 sensor_noise 之后，全零输入是否也带有噪声
    has_noise = np.any(next_obs["joint_angles"] != 0)
    assert has_noise == True
    
    env.close()
