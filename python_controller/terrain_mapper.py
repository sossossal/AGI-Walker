"""
地形测绘模块 (Terrain Mapping)
实现以机器人为中心的滚动网格高程图 (Robot-Centric Rolling Grid Elevation Map)
用于 '建议D: 局部地形图构建'
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, List, Optional

@dataclass
class MapConfig:
    """地图配置"""
    grid_size: int = 100        # 网格单元数量 (例如 100x100)
    resolution: float = 0.05    # 每个单元格的分辨率 (米) -> 覆盖 5m x 5m
    fill_value: float = -1.0    # 初始/未知区域的高度值
    height_clip: Tuple[float, float] = (-2.0, 2.0) # 高度范围


class ElevationMapBuilder:
    """
    高程图构建器
    
    维护一个局部的高程网格图。
    当机器人移动时，网格会"滚动"以保持机器人位于中心。
    """
    
    def __init__(self, config: MapConfig = MapConfig()):
        self.config = config
        self.map_data = np.full(
            (config.grid_size, config.grid_size), 
            config.fill_value, 
            dtype=np.float32
        )
        
        # 机器人当前在全局坐标系下的位置 (x, y)
        self.center_pos = np.array([0.0, 0.0])
        
        # 预计算网格偏移
        self.half_size = config.grid_size // 2
        
    def update_odometry(self, robot_x: float, robot_y: float):
        """
        根据里程计移动地图（滚动网格）
        
        Args:
            robot_x, robot_y: 机器人在全局坐标系下的新位置
        """
        # 计算位移 (以网格为单位)
        dx_meters = robot_x - self.center_pos[0]
        dy_meters = robot_y - self.center_pos[1]
        
        shift_x = int(round(dx_meters / self.config.resolution))
        shift_y = int(round(dy_meters / self.config.resolution))
        
        # 如果只有微小移动，不滚动
        if shift_x == 0 and shift_y == 0:
            return

        # 滚动地图 (使用 numpy.roll, 注意这对循环缓冲区有效，但我们需要填充新区域)
        # 这里我们使用一种简单的方法：平移并填充 fill_value
        
        new_map = np.full_like(self.map_data, self.config.fill_value)
        
        # 计算源区域和目标区域 overlap
        # 这是一个简化的 shifting 逻辑
        
        src_x_start = max(0, shift_x)
        src_x_end = min(self.config.grid_size, self.config.grid_size + shift_x)
        dst_x_start = max(0, -shift_x)
        dst_x_end = min(self.config.grid_size, self.config.grid_size - shift_x)
        
        src_y_start = max(0, shift_y)
        src_y_end = min(self.config.grid_size, self.config.grid_size + shift_y)
        dst_y_start = max(0, -shift_y)
        dst_y_end = min(self.config.grid_size, self.config.grid_size - shift_y)

        # 只有在有重叠时才复制
        if (src_x_end > src_x_start) and (src_y_end > src_y_start):
             # 注意：由于坐标系定义的差异，这里需要仔细处理索引
             # 假设 map[x, y] 对应世界坐标系的 axis
             overlap = self.map_data[src_x_start:src_x_end, src_y_start:src_y_end]
             
             # 确保 dimensions 匹配
             if overlap.shape == new_map[dst_x_start:dst_x_end, dst_y_start:dst_y_end].shape:
                 new_map[dst_x_start:dst_x_end, dst_y_start:dst_y_end] = overlap
        
        self.map_data = new_map
        
        # 更新中心位置
        self.center_pos[0] += shift_x * self.config.resolution
        self.center_pos[1] += shift_y * self.config.resolution
        
    def add_point_cloud(self, points: np.ndarray):
        """
        添加深度点云数据更新地图
        
        Args:
            points: Nx3 array of [x, y, z] in GLOBAL frame
        """
        if len(points) == 0:
            return
            
        # 转换到局部网格索引
        # local_x = global_x - center_x
        freq_x = (points[:, 0] - self.center_pos[0]) / self.config.resolution
        freq_y = (points[:, 1] - self.center_pos[1]) / self.config.resolution
        
        idx_x = np.round(freq_x).astype(int) + self.half_size
        idx_y = np.round(freq_y).astype(int) + self.half_size
        
        # 过滤出在网格范围内的点
        mask = (
            (idx_x >= 0) & (idx_x < self.config.grid_size) &
            (idx_y >= 0) & (idx_y < self.config.grid_size)
        )
        
        valid_x = idx_x[mask]
        valid_y = idx_y[mask]
        valid_z = points[mask, 2]
        
        # 更新网格高度
        # 如果一个格子有多个点，取最高点 (保守策略，避免撞击障碍物)
        # 实际实现可以使用 numba 加速，这里用简单的循环或 scatter_max
        
        # 简单的最大值更新 (假设 z 是高度)
        # 注意：这在 Python 循环中可能较慢，实际应使用 np.maximum.at
        np.maximum.at(self.map_data, (valid_x, valid_y), valid_z)
        
        # Clip
        np.clip(self.map_data, self.config.height_clip[0], self.config.height_clip[1], out=self.map_data)

    def get_local_map(self) -> np.ndarray:
        """获取当前高程图"""
        return self.map_data.copy()
        
    def analyze_terrain(self, radius: float = 1.0) -> dict:
        """
        分析前方地形特征
        
        Returns:
            slope: 平均坡度
            roughness: 粗糙度 (标准差)
        """
        # 提取前方区域 (假设X轴向前)
        # 简单取中心前方的一个矩形
        center_idx = self.half_size
        range_idx = int(radius / self.config.resolution)
        
        front_region = self.map_data[
            center_idx : center_idx + range_idx,
            center_idx - range_idx // 2 : center_idx + range_idx // 2
        ]
        
        valid_mask = front_region > self.config.fill_value + 0.1
        if not np.any(valid_mask):
            return {"slope": 0.0, "roughness": 0.0}
            
        valid_data = front_region[valid_mask]
        
        slope = np.mean(np.gradient(front_region)[0][valid_mask]) # X方向梯度
        roughness = np.std(valid_data)
        
        return {
            "slope": float(slope),
            "roughness": float(roughness),
            "max_step_height": float(np.max(valid_data) - np.min(valid_data))
        }

# 测试代码
if __name__ == "__main__":
    print("构建地形图生成器测试...")
    
    # 1. 初始化
    builder = ElevationMapBuilder(MapConfig(grid_size=20, resolution=1.0))
    print(f"网格大小: {builder.map_data.shape}")
    
    # 2. 添加一些点 (模拟一个斜坡)
    points = []
    for x in range(-5, 15):
        for y in range(-5, 5):
            z = x * 0.1 # 坡度
            points.append([x, y, z])
    points = np.array(points)
    
    builder.add_point_cloud(points)
    print("添加点云完成")
    
    # 3. 分析
    stats = builder.analyze_terrain(radius=5.0)
    print(f"地形分析: {stats}")
    
    # 4. 模拟移动 (向前走 2m)
    print("机器人向前移动 2m...")
    builder.update_odometry(2.0, 0.0)
    
    # 验证中心是否平移 (原点 (0,0) 的数据应该移到了 (-2, 0) 的位置，即索引减小)
    print(f"新的地图中心值: {builder.map_data[10, 10]}") # 应该是之前的 (2,0) 处的值 = 0.2
    
    print("✅ 地形模块测试完成")
