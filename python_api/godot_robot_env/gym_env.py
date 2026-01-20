"""
Godot Robot Environment - OpenAI Gym/Gymnasium Compatible Interface
支持动力学随机化，实施Sim2Real
"""
import gymnasium as gym
import numpy as np
from typing import Dict, Any, Tuple, Optional
import socket
import json
import time
import random
from dataclasses import dataclass

@dataclass
class RandomizerConfig:
    """随机化配置"""
    mass_range: Tuple[float, float] = (0.9, 1.1)        # 质量缩放范围
    friction_range: Tuple[float, float] = (0.5, 1.5)    # 摩擦力缩放范围
    motor_strength_range: Tuple[float, float] = (0.8, 1.05) # 电机强度范围
    motor_lag_range: Tuple[float, float] = (0.0, 0.04)  # 延迟范围 (秒)
    sensor_noise_std: float = 0.01                      # 传感器噪声标准差
    enable_randomization: bool = True                   # 开关

class GodotRobotEnv(gym.Env):
    """
    OpenAI Gym 兼容的 Godot 机器人仿真环境
    
    支持强化学习训练，与 Godot 仿真器通过 TCP 通信。
    集成动力学随机化，增强 Sim2Real 鲁棒性。
    """
    
    metadata = {
        'render_modes': ['human', 'rgb_array'],
        'render_fps': 60
    }
    
    def __init__(
        self,
        robot_config: Optional[Dict] = None,
        physics_config: Optional[Dict] = None,
        randomizer_config: Optional[RandomizerConfig] = None,
        host: str = "127.0.0.1",
        port: int = 9999,
        timeout: float = 10.0
    ):
        """
        初始化环境
        
        Args:
            robot_config: 机器人配置（零件列表等）
            physics_config: 物理环境配置（重力、摩擦等）
            randomizer_config: 随机化配置
            host: Godot 服务器地址
            port: Godot 服务器端口
            timeout: 连接超时时间（秒）
        """
        super().__init__()
        
        self.host = host
        self.port = port
        self.timeout = timeout
        self.socket = None
        self.connected = False
        
        self.robot_config = robot_config or self._default_robot_config()
        self.physics_config = physics_config or {}
        self.randomizer_config = randomizer_config or RandomizerConfig()
        
        # 当前随机化参数
        self.current_dynamics = {
            "mass_scale": 1.0,
            "friction_scale": 1.0,
            "motor_strength": 1.0,
            "motor_lag": 0.0
        }
        
        # 定义观察空间（传感器数据）
        self.observation_space = gym.spaces.Dict({
            # IMU 数据
            'imu_orient': gym.spaces.Box(
                low=-180, high=180, shape=(3,), dtype=np.float32
            ),  # roll, pitch, yaw (degrees)
            'imu_angular_vel': gym.spaces.Box(
                low=-500, high=500, shape=(3,), dtype=np.float32
            ),  # rad/s
            'imu_linear_acc': gym.spaces.Box(
                low=-50, high=50, shape=(3,), dtype=np.float32
            ),  # m/s²
            
            # 关节数据
            'joint_angles': gym.spaces.Box(
                low=-180, high=180, shape=(4,), dtype=np.float32
            ),  # degrees
            'joint_velocities': gym.spaces.Box(
                low=-100, high=100, shape=(4,), dtype=np.float32
            ),  # deg/s
            'joint_torques': gym.spaces.Box(
                low=-10, high=10, shape=(4,), dtype=np.float32
            ),  # N·m
            
            # 接触传感器
            'foot_contacts': gym.spaces.MultiBinary(2),  # 左右脚
            
            # 位置信息（如果有）
            'torso_height': gym.spaces.Box(
                low=0, high=3, shape=(1,), dtype=np.float32
            ),  # meters

            # ================= 多模态扩展 =================
            # RGB 摄像头 (模拟分辨率 64x64)
            'rgb_camera': gym.spaces.Box(
                low=0, high=255, shape=(64, 64, 3), dtype=np.uint8
            ),
            
            # 深度/高程图 (模拟局部地图 32x32)
            'elevation_map': gym.spaces.Box(
                low=-2.0, high=2.0, shape=(32, 32), dtype=np.float32
            ),
            # ============================================
        })
        
        # 定义动作空间（电机目标角度）
        # 4-DOF 步行机器人：hip_left, hip_right, knee_left, knee_right
        self.action_space = gym.spaces.Box(
            low=np.array([-45, -45, -120, -120], dtype=np.float32),
            high=np.array([90, 90, 0, 0], dtype=np.float32),
            dtype=np.float32
        )
        
        # 统计信息
        self.episode_count = 0
        self.total_steps = 0
        self.current_step = 0
        self.episode_reward = 0.0
        
        # 性能统计
        self.communication_time = 0.0
        self.computation_time = 0.0
        
    def _default_robot_config(self) -> Dict:
        """默认机器人配置"""
        return {
            "name": "Walker4DOF",
            "parts": [
                {"type": "motor", "id": "dynamixel_xl430_w250", "joint": "hip_left"},
                {"type": "motor", "id": "dynamixel_xl430_w250", "joint": "hip_right"},
                {"type": "motor", "id": "dynamixel_xl430_w250", "joint": "knee_left"},
                {"type": "motor", "id": "dynamixel_xl430_w250", "joint": "knee_right"},
                {"type": "sensor", "id": "bosch_bno055", "location": "torso"}
            ]
        }
    
    def connect(self) -> bool:
        """连接到 Godot 仿真器"""
        if self.connected:
            return True
        
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            self.socket.connect((self.host, self.port))
            self.connected = True
            print(f"✅ Connected to Godot simulator at {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"❌ Failed to connect to Godot: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
            self.connected = False
    
    def _randomize_dynamics(self):
        """生成新的随机动力学参数"""
        if not self.randomizer_config.enable_randomization:
            return
        
        cfg = self.randomizer_config
        self.current_dynamics["mass_scale"] = random.uniform(*cfg.mass_range)
        self.current_dynamics["friction_scale"] = random.uniform(*cfg.friction_range)
        self.current_dynamics["motor_strength"] = random.uniform(*cfg.motor_strength_range)
        self.current_dynamics["motor_lag"] = random.uniform(*cfg.motor_lag_range)
        
        # 更新 physics_config 以发送给 Godot
        self.physics_config["mass_scale"] = self.current_dynamics["mass_scale"]
        self.physics_config["friction_scale"] = self.current_dynamics["friction_scale"]
        self.physics_config["motor_strength"] = self.current_dynamics["motor_strength"]

    def _apply_sensor_noise(self, obs: Dict) -> Dict:
        """应用传感器噪声"""
        if not self.randomizer_config.enable_randomization:
            return obs
            
        noise_std = self.randomizer_config.sensor_noise_std
        
        # 为连续值添加高斯噪声
        for key in ['imu_orient', 'imu_angular_vel', 'imu_linear_acc', 
                   'joint_angles', 'joint_velocities', 'joint_torques']:
            noise = np.random.normal(0, noise_std, obs[key].shape)
            obs[key] = (obs[key] + noise).astype(np.float32)
            
        return obs

    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[Dict] = None
    ) -> Tuple[Dict, Dict]:
        """
        重置环境
        """
        super().reset(seed=seed)
        
        # 1. 生成随机参数
        self._randomize_dynamics()
        
        # 确保连接
        if not self.connected:
            self.connect()
        
        # 发送重置指令（包含新的随机化物理参数）
        reset_command = {
            "type": "reset",
            "robot_config": self.robot_config,
            "physics_config": self.physics_config,
            "sim_params": self.current_dynamics,
            "terrain_seed": random.randint(0, 999999) # 新增: 地形随机种子
        }
        
        self._send_command(reset_command)
        
        # 接收初始观察
        obs = self._receive_observation()
        
        # 应用噪声
        obs = self._apply_sensor_noise(obs)
        
        # 重置统计
        self.current_step = 0
        self.episode_reward = 0.0
        self.episode_count += 1
        
        info = {
            "episode": self.episode_count,
            "dynamics": self.current_dynamics # 记录当前episode的物理参数
        }
        
        return obs, info
    
    def step(
        self,
        action: np.ndarray
    ) -> Tuple[Dict, float, bool, bool, Dict]:
        """
        执行一步动作
        """
        start_time = time.time()
        
        # 模拟通信/执行延迟
        if self.randomizer_config.enable_randomization:
            time.sleep(self.current_dynamics["motor_lag"])
            
        # 发送动作指令
        action_command = {
            "type": "action",
            "motors": {
                "hip_left": float(action[0]),
                "hip_right": float(action[1]),
                "knee_left": float(action[2]),
                "knee_right": float(action[3])
            }
        }
        
        self._send_command(action_command)
        
        # 接收新观察
        obs = self._receive_observation()
        
        # 应用噪声
        obs = self._apply_sensor_noise(obs)
        
        comm_time = time.time()
        
        # 计算奖励
        reward = self._calculate_reward(obs, action)
        
        # 检查终止条件
        terminated = self._is_terminated(obs)
        truncated = self._is_truncated()
        
        # 统计信息
        self.current_step += 1
        self.total_steps += 1
        self.episode_reward += reward
        
        comp_time = time.time()
        self.communication_time += (comm_time - start_time)
        self.computation_time += (comp_time - comm_time)
        
        info = self._get_info(obs)
        
        return obs, reward, terminated, truncated, info
    
    def _send_command(self, command: Dict):
        """发送指令到 Godot"""
        if not self.connected:
            raise RuntimeError("Not connected to Godot simulator")
        
        try:
            message = json.dumps(command) + "\n"
            self.socket.sendall(message.encode('utf-8'))
        except Exception as e:
            print(f"❌ Failed to send command: {e}")
            self.connected = False
            raise
    
    def _receive_observation(self) -> Dict:
        """从 Godot 接收观察数据"""
        if not self.connected:
            raise RuntimeError("Not connected to Godot simulator")
        
        try:
            # 接收数据直到换行符
            buffer = b""
            while b"\n" not in buffer:
                chunk = self.socket.recv(4096)
                if not chunk:
                    raise ConnectionError("Connection closed by Godot")
                buffer += chunk
            
            # 解析 JSON
            message = buffer.decode('utf-8').strip()
            data = json.loads(message)
            
            # 转换为 observation 格式
            return self._parse_sensor_data(data)
            
        except Exception as e:
            print(f"❌ Failed to receive observation: {e}")
            self.connected = False
            raise
    
    def _parse_sensor_data(self, data: Dict) -> Dict:
        """解析传感器数据为观察空间格式"""
        sensors = data.get('sensors', {})
        imu = sensors.get('imu', {})
        joints = sensors.get('joints', {})
        contacts = sensors.get('contacts', {})
        
        return {
            'imu_orient': np.array(imu.get('orient', [0, 0, 0]), dtype=np.float32),
            'imu_angular_vel': np.array(imu.get('angular_vel', [0, 0, 0]), dtype=np.float32),
            'imu_linear_acc': np.array(imu.get('linear_acc', [0, 0, 9.81]), dtype=np.float32),
            'joint_angles': np.array(joints.get('angles', [0, 0, 0, 0]), dtype=np.float32),
            'joint_velocities': np.array(joints.get('velocities', [0, 0, 0, 0]), dtype=np.float32),
            'joint_torques': np.array(joints.get('torques', [0, 0, 0, 0]), dtype=np.float32),
            'foot_contacts': np.array([
                contacts.get('foot_left', False),
                contacts.get('foot_right', False)
            ], dtype=np.int8),
            'torso_height': np.array([data.get('torso_height', 1.0)], dtype=np.float32),
            
            # 多模态填充 (如果Godot未发送真实图像，使用全0/噪声填充)
            'rgb_camera': np.zeros((64, 64, 3), dtype=np.uint8), # Placeholder
            'elevation_map': np.zeros((32, 32), dtype=np.float32) # Placeholder
        }
    
    def _calculate_reward(self, obs: Dict, action: np.ndarray) -> float:
        """
        计算奖励函数
        """
        reward = 0.0
        
        # 1. 生存奖励
        reward += 0.1
        
        # 2. 平衡奖励（姿态接近直立）
        roll, pitch, yaw = obs['imu_orient']
        balance_penalty = -(abs(roll) + abs(pitch)) * 0.01
        reward += balance_penalty
        
        # 3. 能量惩罚（减少不必要的动作）
        action_magnitude = np.sum(np.abs(action))
        energy_penalty = -action_magnitude * 0.001
        reward += energy_penalty
        
        # 4. 高度保持（保持在合理高度）
        height = obs['torso_height'][0]
        target_height = 1.0  # 期望高度
        height_penalty = -abs(height - target_height) * 0.1
        reward += height_penalty
        
        # 5. 接触奖励（双脚接地更稳定）
        if obs['foot_contacts'][0] and obs['foot_contacts'][1]:
            reward += 0.05  # 双脚接地奖励
        
        return reward
    
    def _is_terminated(self, obs: Dict) -> bool:
        """检查是否终止（摔倒等失败条件）"""
        # 1. 姿态检查
        roll, pitch, yaw = obs['imu_orient']
        if abs(roll) > 60 or abs(pitch) > 60:
            return True  # 倾斜过大
        
        # 2. 高度检查
        height = obs['torso_height'][0]
        if height < 0.3:
            return True  # 摔倒
        
        return False
    
    def _is_truncated(self) -> bool:
        """检查是否截断（超时等）"""
        # 最大步数限制
        max_steps = 1000
        if self.current_step >= max_steps:
            return True
        
        return False
    
    def _get_info(self, obs: Dict) -> Dict:
        """获取额外信息"""
        return {
            "step": self.current_step,
            "episode_reward": self.episode_reward,
            "torso_height": obs['torso_height'][0],
            "orientation": obs['imu_orient'].tolist(),
            "communication_time_ms": self.communication_time * 1000,
            "computation_time_ms": self.computation_time * 1000,
        }
    
    def render(self):
        """渲染（Godot 自己渲染，这里不需要实现）"""
        pass
    
    def close(self):
        """关闭环境"""
        self.disconnect()
    
    def set_physics_params(self, params: Dict):
        """动态修改物理参数"""
        command = {
            "type": "set_physics",
            "params": params
        }
        self._send_command(command)
    
    def load_robot_from_parts(self, parts_list: list):
        """从零件列表构建机器人"""
        command = {
            "type": "build_robot",
            "parts": parts_list
        }
        self._send_command(command)
