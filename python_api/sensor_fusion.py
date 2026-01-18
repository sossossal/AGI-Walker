"""
传感器融合系统
Sensor Fusion System

功能:
- 多传感器数据整合
- 卡尔曼滤波
- 传感器噪声模拟
- 数据延迟处理
- 故障检测
"""

import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum


class SensorType(Enum):
    """传感器类型"""
    IMU = "IMU"
    ENCODER = "编码器"
    FORCE = "力传感器"
    LIDAR = "激光雷达"
    CAMERA = "相机"
    GPS = "GPS"


@dataclass
class SensorReading:
    """传感器读数"""
    sensor_type: SensorType
    timestamp: float
    value: np.ndarray
    noise_level: float = 0.01
    delay: float = 0.0


class KalmanFilter:
    """卡尔曼滤波器"""
    
    def __init__(self, state_dim: int, measurement_dim: int):
        """
        初始化卡尔曼滤波器
        
        参数:
            state_dim: 状态维度
            measurement_dim: 测量维度
        """
        self.state_dim = state_dim
        self.measurement_dim = measurement_dim
        
        # 状态向量
        self.x = np.zeros(state_dim)
        
        # 状态协方差矩阵
        self.P = np.eye(state_dim)
        
        # 状态转移矩阵 (将根据dt更新)
        self.F = np.eye(state_dim)
        
        # 测量矩阵
        self.H = np.zeros((measurement_dim, state_dim))
        self.H[:measurement_dim, :measurement_dim] = np.eye(measurement_dim)
        
        # 过程噪声协方差
        self.Q = np.eye(state_dim) * 0.01
        
        # 测量噪声协方差
        self.R = np.eye(measurement_dim) * 0.1
    
    def predict(self, dt: float):
        """预测步骤"""
        # 更新状态转移矩阵 (考虑时间间隔)
        # 对于位置-速度系统: x' = x + v*dt
        if self.state_dim == 6:  # [x, y, z, vx, vy, vz]
            self.F = np.eye(6)
            self.F[0, 3] = dt
            self.F[1, 4] = dt
            self.F[2, 5] = dt
        
        # 预测状态
        self.x = self.F @ self.x
        
        # 预测协方差
        self.P = self.F @ self.P @ self.F.T + self.Q
    
    def update(self, z: np.ndarray):
        """更新步骤"""
        # 创新 (innovation)
        y = z - self.H @ self.x
        
        # 创新协方差
        S = self.H @ self.P @ self.H.T + self.R
        
        # 卡尔曼增益
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # 更新状态
        self.x = self.x + K @ y
        
        # 更新协方差
        I = np.eye(self.state_dim)
        self.P = (I - K @ self.H) @ self.P
    
    def get_state(self) -> np.ndarray:
        """获取当前状态估计"""
        return self.x.copy()


class ComplementaryFilter:
    """互补滤波器 (用于IMU姿态估计)"""
    
    def __init__(self, alpha: float = 0.98):
        """
        初始化互补滤波器
        
        参数:
            alpha: 陀螺仪权重 (0-1, 通常0.95-0.99)
        """
        self.alpha = alpha
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
    
    def update(self, gyro: np.ndarray, accel: np.ndarray, dt: float):
        """
        更新姿态估计
        
        参数:
            gyro: 陀螺仪数据 [gx, gy, gz] (rad/s)
            accel: 加速度计数据 [ax, ay, az] (m/s²)
            dt: 时间间隔
        """
        # 从陀螺仪积分得到姿态变化
        gyro_pitch = self.pitch + gyro[1] * dt
        gyro_roll = self.roll + gyro[0] * dt
        
        # 从加速度计计算姿态
        accel_pitch = np.arctan2(accel[1], np.sqrt(accel[0]**2 + accel[2]**2))
        accel_roll = np.arctan2(-accel[0], accel[2])
        
        # 互补滤波
        self.pitch = self.alpha * gyro_pitch + (1 - self.alpha) * accel_pitch
        self.roll = self.alpha * gyro_roll + (1 - self.alpha) * accel_roll
        
        # 偏航只能从陀螺仪积分 (没有磁力计)
        self.yaw += gyro[2] * dt
    
    def get_orientation(self) -> Tuple[float, float, float]:
        """获取姿态 (roll, pitch, yaw)"""
        return (self.roll, self.pitch, self.yaw)


class SensorModel:
    """传感器模型 (包含噪声和延迟)"""
    
    def __init__(self, sensor_type: SensorType, noise_std: float = 0.01, delay_ms: float = 0):
        self.sensor_type = sensor_type
        self.noise_std = noise_std
        self.delay_s = delay_ms / 1000.0
        self.is_functional = True
        self.failure_rate = 0.0001  # 故障概率
    
    def read(self, true_value: np.ndarray, timestamp: float) -> SensorReading:
        """
        读取传感器值 (添加噪声和延迟)
        
        参数:
            true_value: 真实值
            timestamp: 时间戳
        
        返回:
            传感器读数
        """
        # 检查传感器是否故障
        if not self.is_functional or np.random.random() < self.failure_rate:
            return SensorReading(
                self.sensor_type,
                timestamp + self.delay_s,
                np.full_like(true_value, np.nan),
                self.noise_std,
                self.delay_s
            )
        
        # 添加高斯噪声
        noise = np.random.normal(0, self.noise_std, size=true_value.shape)
        noisy_value = true_value + noise
        
        return SensorReading(
            self.sensor_type,
            timestamp + self.delay_s,
            noisy_value,
            self.noise_std,
            self.delay_s
        )
    
    def set_failure(self, failed: bool):
        """设置传感器故障状态"""
        self.is_functional = not failed


class SensorFusion:
    """传感器融合系统"""
    
    def __init__(self):
        # 传感器模型
        self.sensors = {}
        
        # 滤波器
        self.kalman_filter = KalmanFilter(state_dim=6, measurement_dim=3)  # 位置+速度
        self.complementary_filter = ComplementaryFilter(alpha=0.98)
        
        # 融合状态
        self.position = np.zeros(3)  # [x, y, z]
        self.velocity = np.zeros(3)  # [vx, vy, vz]
        self.orientation = np.zeros(3)  # [roll, pitch, yaw]
        
        # 数据缓冲 (用于处理延迟)
        self.data_buffer = []
        self.max_buffer_size = 100
        
        # 统计
        self.total_readings = 0
        self.failed_readings = 0
    
    def add_sensor(self, name: str, sensor_type: SensorType, 
                   noise_std: float = 0.01, delay_ms: float = 0):
        """添加传感器"""
        self.sensors[name] = SensorModel(sensor_type, noise_std, delay_ms)
    
    def process_imu(self, gyro: np.ndarray, accel: np.ndarray, dt: float):
        """
        处理IMU数据
        
        参数:
            gyro: 陀螺仪 [gx, gy, gz] rad/s
            accel: 加速度 [ax, ay, az] m/s²
            dt: 时间间隔
        """
        # 读取传感器 (添加噪声)
        imu_sensor = self.sensors.get('IMU')
        if imu_sensor:
            gyro_reading = imu_sensor.read(gyro, 0)
            accel_reading = imu_sensor.read(accel, 0)
            
            # 检查有效性
            if not np.any(np.isnan(gyro_reading.value)) and not np.any(np.isnan(accel_reading.value)):
                # 互补滤波更新姿态
                self.complementary_filter.update(gyro_reading.value, accel_reading.value, dt)
                self.orientation = np.array(self.complementary_filter.get_orientation())
            else:
                self.failed_readings += 1
        
        self.total_readings += 1
    
    def process_encoder(self, velocities: np.ndarray, dt: float):
        """
        处理编码器数据
        
        参数:
            velocities: 速度 [vx, vy, vz] m/s
            dt: 时间间隔
        """
        encoder_sensor = self.sensors.get('ENCODER')
        if encoder_sensor:
            vel_reading = encoder_sensor.read(velocities, 0)
            
            if not np.any(np.isnan(vel_reading.value)):
                # 更新卡尔曼滤波器
                self.kalman_filter.predict(dt)
                self.kalman_filter.update(vel_reading.value)
                
                # 获取融合后的状态
                state = self.kalman_filter.get_state()
                self.position = state[:3]
                self.velocity = state[3:]
            else:
                self.failed_readings += 1
        
        self.total_readings += 1
    
    def get_fused_state(self) -> Dict:
        """获取融合后的状态"""
        return {
            'position': self.position.copy(),
            'velocity': self.velocity.copy(),
            'orientation': self.orientation.copy(),
            'orientation_deg': np.degrees(self.orientation),
            'speed': np.linalg.norm(self.velocity),
            'data_quality': 1.0 - (self.failed_readings / max(self.total_readings, 1))
        }
    
    def get_fusion_report(self) -> str:
        """生成融合报告"""
        report = []
        report.append("="*70)
        report.append("传感器融合报告")
        report.append("="*70)
        
        report.append(f"\n传感器状态:")
        for name, sensor in self.sensors.items():
            status = "正常" if sensor.is_functional else "故障"
            report.append(f"  {name:<15} {sensor.sensor_type.value:<10} "
                         f"噪声: {sensor.noise_std:.4f}, "
                         f"延迟: {sensor.delay_s*1000:.1f}ms, "
                         f"状态: {status}")
        
        report.append(f"\n融合状态:")
        report.append(f"  位置: [{self.position[0]:.3f}, {self.position[1]:.3f}, {self.position[2]:.3f}] m")
        report.append(f"  速度: [{self.velocity[0]:.3f}, {self.velocity[1]:.3f}, {self.velocity[2]:.3f}] m/s")
        report.append(f"  速率: {np.linalg.norm(self.velocity):.3f} m/s")
        report.append(f"  姿态: Roll={np.degrees(self.orientation[0]):.1f}°, "
                     f"Pitch={np.degrees(self.orientation[1]):.1f}°, "
                     f"Yaw={np.degrees(self.orientation[2]):.1f}°")
        
        report.append(f"\n数据质量:")
        report.append(f"  总读数: {self.total_readings}")
        report.append(f"  失败读数: {self.failed_readings}")
        quality = 1.0 - (self.failed_readings / max(self.total_readings, 1))
        report.append(f"  数据质量: {quality*100:.1f}%")
        
        return "\n".join(report)


if __name__ == "__main__":
    print("传感器融合系统加载完成")
    
    # 示例
    fusion = SensorFusion()
    fusion.add_sensor('IMU', SensorType.IMU, noise_std=0.01, delay_ms=5)
    fusion.add_sensor('ENCODER', SensorType.ENCODER, noise_std=0.001, delay_ms=2)
    
    # 模拟运行
    dt = 0.01
    for i in range(100):
        # 模拟IMU数据
        gyro = np.array([0.1, 0.05, 0.02])  # rad/s
        accel = np.array([0, 0, -9.81])     # m/s²
        fusion.process_imu(gyro, accel, dt)
        
        # 模拟编码器数据
        vel = np.array([0.5, 0, 0])  # m/s
        fusion.process_encoder(vel, dt)
        
        if i % 20 == 0:
            state = fusion.get_fused_state()
            print(f"\n时间 {i*dt:.2f}s:")
            print(f"  位置: {state['position']}")
            print(f"  速度: {state['velocity']}")
            print(f"  姿态: {state['orientation_deg']}°")
    
    print("\n" + fusion.get_fusion_report())
