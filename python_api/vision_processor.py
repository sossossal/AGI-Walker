"""
视觉处理模块（Vision Processor）
轻量级视觉处理器，用于边缘检测、障碍物检测等CV任务
在小模型上运行，支持低延迟处理
"""

import numpy as np
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
import base64
import io


@dataclass
class VisionConfig:
    """视觉处理配置"""
    resolution: Tuple[int, int] = (320, 240)  # 宽, 高
    canny_low: int = 50                        # Canny边缘检测低阈值
    canny_high: int = 150                      # Canny边缘检测高阈值
    obstacle_min_area: int = 500               # 障碍物最小面积
    ground_plane_rows: int = 60                # 地面检测区域行数
    depth_scale: float = 0.1                   # 深度估算比例因子


class VisionProcessor:
    """
    轻量级视觉处理器
    
    功能：
    - 边缘检测（Canny）
    - 障碍物检测（轮廓分析）
    - 地面平面估计
    - 帧差法运动检测
    """
    
    def __init__(self, config: Optional[VisionConfig] = None):
        self.config = config or VisionConfig()
        
        # 延迟导入OpenCV（可能未安装）
        self._cv2 = None
        self._init_cv2()
        
        # 历史帧（用于运动检测）
        self.prev_frame: Optional[np.ndarray] = None
        self.prev_gray: Optional[np.ndarray] = None
        
        # 统计
        self.frames_processed = 0
        self.total_processing_time = 0.0
    
    def _init_cv2(self):
        """初始化OpenCV"""
        try:
            import cv2
            self._cv2 = cv2
            print("✅ OpenCV已加载")
        except ImportError:
            print("⚠️ OpenCV未安装，视觉处理功能将受限")
            print("请运行: pip install opencv-python")
    
    @property
    def cv2(self):
        """获取OpenCV模块"""
        if self._cv2 is None:
            raise ImportError("OpenCV未安装")
        return self._cv2
    
    def process_frame(self, frame: np.ndarray) -> dict:
        """
        处理视频帧
        
        Args:
            frame: BGR格式图像数组
        
        Returns:
            处理结果字典
        """
        import time
        start_time = time.time()
        
        result = {
            "frame_id": self.frames_processed,
            "resolution": frame.shape[:2][::-1],  # (宽, 高)
            "edges": None,
            "obstacles": [],
            "motion": None,
            "ground_plane": None
        }
        
        try:
            # 转换为灰度图
            gray = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2GRAY)
            
            # 边缘检测
            edges = self._detect_edges(gray)
            result["edges"] = edges
            
            # 障碍物检测
            obstacles = self._detect_obstacles(edges, frame)
            result["obstacles"] = obstacles
            
            # 运动检测
            motion = self._detect_motion(gray)
            result["motion"] = motion
            
            # 地面平面估计
            ground = self._estimate_ground_plane(frame)
            result["ground_plane"] = ground
            
            # 更新历史帧
            self.prev_frame = frame.copy()
            self.prev_gray = gray.copy()
            
        except Exception as e:
            result["error"] = str(e)
        
        # 统计
        self.frames_processed += 1
        self.total_processing_time += time.time() - start_time
        
        return result
    
    def _detect_edges(self, gray: np.ndarray) -> dict:
        """边缘检测"""
        edges = self.cv2.Canny(
            gray,
            self.config.canny_low,
            self.config.canny_high
        )
        
        # 统计边缘密度
        edge_density = np.sum(edges > 0) / edges.size
        
        return {
            "density": float(edge_density),
            "shape": edges.shape
        }
    
    def _detect_obstacles(
        self,
        edges: dict,
        frame: np.ndarray
    ) -> List[dict]:
        """障碍物检测"""
        # 重新计算边缘（用于轮廓检测）
        gray = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2GRAY)
        edge_img = self.cv2.Canny(
            gray,
            self.config.canny_low,
            self.config.canny_high
        )
        
        # 膨胀操作以连接边缘
        kernel = np.ones((3, 3), np.uint8)
        dilated = self.cv2.dilate(edge_img, kernel, iterations=2)
        
        # 查找轮廓
        contours, _ = self.cv2.findContours(
            dilated,
            self.cv2.RETR_EXTERNAL,
            self.cv2.CHAIN_APPROX_SIMPLE
        )
        
        obstacles = []
        h, w = frame.shape[:2]
        
        for contour in contours:
            area = self.cv2.contourArea(contour)
            
            if area < self.config.obstacle_min_area:
                continue
            
            # 计算边界框
            x, y, bw, bh = self.cv2.boundingRect(contour)
            
            # 计算中心点和相对位置
            cx = x + bw / 2
            cy = y + bh / 2
            
            # 估算距离（基于画面位置，简化模型）
            # 越靠近画面底部，距离越近
            distance_estimate = 1.0 - (cy / h)
            distance_estimate = max(0.1, min(5.0, distance_estimate * 5.0))
            
            # 估算方向（相对画面中心）
            angle = (cx - w / 2) / w * 60  # 假设60度视角
            
            obstacles.append({
                "bbox": [int(x), int(y), int(bw), int(bh)],
                "center": [float(cx), float(cy)],
                "area": float(area),
                "distance_estimate": float(distance_estimate),
                "angle_deg": float(angle),
                "relative_size": float(area / (h * w))
            })
        
        # 按距离排序
        obstacles.sort(key=lambda o: o["distance_estimate"])
        
        return obstacles[:10]  # 最多返回10个障碍物
    
    def _detect_motion(self, gray: np.ndarray) -> Optional[dict]:
        """运动检测（帧差法）"""
        if self.prev_gray is None:
            return None
        
        # 计算帧差
        diff = self.cv2.absdiff(gray, self.prev_gray)
        
        # 阈值处理
        _, thresh = self.cv2.threshold(diff, 25, 255, self.cv2.THRESH_BINARY)
        
        # 计算运动量
        motion_ratio = np.sum(thresh > 0) / thresh.size
        
        # 查找运动区域
        contours, _ = self.cv2.findContours(
            thresh,
            self.cv2.RETR_EXTERNAL,
            self.cv2.CHAIN_APPROX_SIMPLE
        )
        
        # 找到最大运动区域
        max_contour = None
        max_area = 0
        
        for contour in contours:
            area = self.cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                max_contour = contour
        
        result = {
            "motion_ratio": float(motion_ratio),
            "is_moving": motion_ratio > 0.01,
            "max_motion_area": float(max_area)
        }
        
        if max_contour is not None and max_area > 100:
            x, y, w, h = self.cv2.boundingRect(max_contour)
            result["motion_bbox"] = [int(x), int(y), int(w), int(h)]
        
        return result
    
    def _estimate_ground_plane(self, frame: np.ndarray) -> dict:
        """估计地面平面"""
        h, w = frame.shape[:2]
        
        # 取画面底部区域作为地面参考
        ground_region = frame[h - self.config.ground_plane_rows:, :]
        
        # 转换到HSV
        hsv = self.cv2.cvtColor(ground_region, self.cv2.COLOR_BGR2HSV)
        
        # 计算主色调（地面颜色）
        hist_h = self.cv2.calcHist([hsv], [0], None, [180], [0, 180])
        dominant_hue = int(np.argmax(hist_h))
        
        # 计算亮度（判断光照条件）
        mean_brightness = np.mean(hsv[:, :, 2])
        
        # 边缘检测（判断地面平整度）
        gray_ground = self.cv2.cvtColor(ground_region, self.cv2.COLOR_BGR2GRAY)
        edges = self.cv2.Canny(gray_ground, 50, 150)
        edge_density = np.sum(edges > 0) / edges.size
        
        return {
            "dominant_hue": dominant_hue,
            "brightness": float(mean_brightness),
            "roughness": float(edge_density),  # 边缘密度高=地面粗糙
            "is_flat": edge_density < 0.1,
            "region_height": self.config.ground_plane_rows
        }
    
    def detect_obstacles_simple(self, frame: np.ndarray) -> List[dict]:
        """简化版障碍物检测（更快）"""
        gray = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2GRAY)
        
        # 使用阈值分割
        _, binary = self.cv2.threshold(gray, 0, 255, self.cv2.THRESH_BINARY + self.cv2.THRESH_OTSU)
        
        # 形态学操作
        kernel = np.ones((5, 5), np.uint8)
        cleaned = self.cv2.morphologyEx(binary, self.cv2.MORPH_CLOSE, kernel)
        
        # 查找轮廓
        contours, _ = self.cv2.findContours(
            cleaned,
            self.cv2.RETR_EXTERNAL,
            self.cv2.CHAIN_APPROX_SIMPLE
        )
        
        obstacles = []
        h, w = frame.shape[:2]
        
        for contour in contours:
            area = self.cv2.contourArea(contour)
            if area < self.config.obstacle_min_area:
                continue
            
            x, y, bw, bh = self.cv2.boundingRect(contour)
            cx = x + bw / 2
            cy = y + bh / 2
            
            obstacles.append({
                "bbox": [int(x), int(y), int(bw), int(bh)],
                "center": [float(cx), float(cy)],
                "distance_estimate": max(0.1, (1.0 - cy / h) * 3.0)
            })
        
        return obstacles[:5]
    
    def decode_base64_image(self, base64_str: str) -> np.ndarray:
        """解码Base64图像"""
        try:
            from PIL import Image
            
            # 解码Base64
            img_data = base64.b64decode(base64_str)
            
            # 转换为PIL Image
            img = Image.open(io.BytesIO(img_data))
            
            # 转换为OpenCV格式（BGR）
            frame = np.array(img)
            if len(frame.shape) == 3 and frame.shape[2] == 3:
                frame = self.cv2.cvtColor(frame, self.cv2.COLOR_RGB2BGR)
            
            return frame
            
        except Exception as e:
            raise ValueError(f"图像解码失败: {e}")
    
    def get_stats(self) -> dict:
        """获取统计信息"""
        avg_time = (
            self.total_processing_time / self.frames_processed
            if self.frames_processed > 0 else 0
        )
        
        return {
            "frames_processed": self.frames_processed,
            "avg_processing_time_ms": avg_time * 1000,
            "fps_capacity": 1.0 / avg_time if avg_time > 0 else 0,
            "resolution": self.config.resolution
        }
    
    def reset(self):
        """重置状态"""
        self.prev_frame = None
        self.prev_gray = None
        self.frames_processed = 0
        self.total_processing_time = 0.0


class DummyVisionProcessor:
    """
    虚拟视觉处理器
    当OpenCV不可用时使用，返回模拟数据
    """
    
    def __init__(self, config: Optional[VisionConfig] = None):
        self.config = config or VisionConfig()
        self.frames_processed = 0
        print("⚠️ 使用虚拟视觉处理器（OpenCV不可用）")
    
    def process_frame(self, frame: np.ndarray) -> dict:
        """返回模拟处理结果"""
        self.frames_processed += 1
        
        return {
            "frame_id": self.frames_processed,
            "resolution": self.config.resolution,
            "edges": {"density": 0.05, "shape": self.config.resolution},
            "obstacles": [],
            "motion": {"motion_ratio": 0.0, "is_moving": False},
            "ground_plane": {
                "dominant_hue": 30,
                "brightness": 128.0,
                "roughness": 0.05,
                "is_flat": True
            },
            "dummy": True
        }
    
    def detect_obstacles_simple(self, frame: np.ndarray) -> List[dict]:
        return []
    
    def get_stats(self) -> dict:
        return {
            "frames_processed": self.frames_processed,
            "avg_processing_time_ms": 0.0,
            "fps_capacity": float('inf'),
            "dummy": True
        }


def create_vision_processor(config: Optional[VisionConfig] = None) -> VisionProcessor:
    """
    工厂函数：创建视觉处理器
    
    如果OpenCV不可用，返回虚拟处理器
    """
    try:
        import cv2
        return VisionProcessor(config)
    except ImportError:
        return DummyVisionProcessor(config)


# 测试代码
if __name__ == "__main__":
    import json
    
    print("视觉处理模块测试\n")
    
    # 创建处理器
    processor = create_vision_processor()
    
    # 检查OpenCV是否可用
    if isinstance(processor, DummyVisionProcessor):
        print("使用虚拟处理器进行测试\n")
        
        # 虚拟测试
        dummy_frame = np.zeros((240, 320, 3), dtype=np.uint8)
        result = processor.process_frame(dummy_frame)
        print(f"处理结果: {json.dumps(result, indent=2)}")
    else:
        print("使用真实处理器进行测试\n")
        
        # 创建测试图像
        test_frame = np.zeros((240, 320, 3), dtype=np.uint8)
        
        # 添加一些测试形状
        import cv2
        cv2.rectangle(test_frame, (100, 80), (150, 130), (255, 255, 255), -1)
        cv2.circle(test_frame, (250, 150), 30, (128, 128, 128), -1)
        
        # 处理帧
        result = processor.process_frame(test_frame)
        
        print(f"边缘密度: {result['edges']['density']:.4f}")
        print(f"检测到障碍物: {len(result['obstacles'])}个")
        print(f"地面平整: {result['ground_plane']['is_flat']}")
        
        # 处理多帧测试性能
        import time
        start = time.time()
        for _ in range(100):
            processor.process_frame(test_frame)
        elapsed = time.time() - start
        
        print(f"\n性能测试: {100/elapsed:.1f} FPS")
    
    # 统计
    print("\n=== 统计信息 ===")
    stats = processor.get_stats()
    print(json.dumps(stats, indent=2))
