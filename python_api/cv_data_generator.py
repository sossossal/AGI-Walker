"""
计算机视觉训练数据生成器
CV Data Generator

功能:
- 与Godot通信获取渲染图像
- 生成完整的CV训练数据集
- 支持COCO格式导出
- 自动标注生成
"""

import numpy as np
import cv2
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import json
from tqdm import tqdm
import time
from datetime import datetime

from python_api.godot_vision_client import GodotVisionClient


class CVDatasetAnnotation:
    """数据集标注管理"""
    
    def __init__(self):
        self.images = []
        self.annotations = []
        self.categories = [
            {'id': 1, 'name': 'robot', 'supercategory': 'object'},
            {'id': 2, 'name': 'ground', 'supercategory': 'surface'},
            {'id': 3, 'name': 'obstacle', 'supercategory': 'object'}
        ]
        self.annotation_id = 0
    
    def add_image(self, image_id: int, filename: str, width: int, height: int):
        """添加图像信息"""
        self.images.append({
            'id': image_id,
            'file_name': filename,
            'width': width,
            'height': height
        })
    
    def add_bbox_annotation(self, image_id: int, category_id: int, 
                           bbox: List[float], area: float):
        """添加边界框标注"""
        self.annotations.append({
            'id': self.annotation_id,
            'image_id': image_id,
            'category_id': category_id,
            'bbox': bbox,  # [x, y, width, height]
            'area': area,
            'iscrowd': 0
        })
        self.annotation_id += 1
    
    def add_keypoint_annotation(self, image_id: int, keypoints: List[float],
                               num_keypoints: int):
        """添加关键点标注"""
        self.annotations.append({
            'id': self.annotation_id,
            'image_id': image_id,
            'category_id': 1,  # robot
            'keypoints': keypoints,  # [x1, y1, v1, x2, y2, v2, ...]
            'num_keypoints': num_keypoints
        })
        self.annotation_id += 1
    
    def to_coco_format(self) -> Dict:
        """转换为COCO格式"""
        return {
            'images': self.images,
            'annotations': self.annotations,
            'categories': self.categories
        }


class CVDataGenerator:
    """CV训练数据生成器"""
    
    def __init__(self, output_dir: str = "data/cv_dataset",
                 godot_host: str = 'localhost',
                 godot_port: int = 9999):
        """
        初始化CV数据生成器
        
        参数:
            output_dir: 输出目录
            godot_host: Godot服务器地址
            godot_port: Godot服务器端口
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Godot客户端
        self.godot_client = GodotVisionClient(godot_host, godot_port)
        
        # COCO标注
        self.coco_annotation = CVDatasetAnnotation()
        
        # 统计
        self.num_frames = 0
        self.num_episodes = 0
    
    def connect_to_godot(self) -> bool:
        """连接到Godot"""
        return self.godot_client.connect()
    
    def disconnect(self):
        """断开连接"""
        self.godot_client.disconnect()
    
    def generate_episode_images(self, episode_id: int, 
                               trajectory: List[Dict],
                               save_interval: int = 10,
                               cameras: Optional[List[int]] = None):
        """
        为一个episode生成图像
        
        参数:
            episode_id: Episode ID
            trajectory: 轨迹数据 [{'position': [...], 'orientation': [...], ...}, ...]
            save_interval: 保存间隔（每N帧保存一次）
            cameras: 要使用的相机ID列表
        """
        episode_dir = self.output_dir / f"episode_{episode_id:06d}"
        episode_dir.mkdir(exist_ok=True)
        
        # 创建子目录
        (episode_dir / "rgb").mkdir(exist_ok=True)
        (episode_dir / "depth").mkdir(exist_ok=True)
        (episode_dir / "segmentation").mkdir(exist_ok=True)
        
        metadata = {
            'episode_id': episode_id,
            'num_frames': 0,
            'save_interval': save_interval,
            'start_time': datetime.now().isoformat()
        }
        
        print(f"\n生成Episode {episode_id} 的图像数据...")
        
        for step_id, state in enumerate(tqdm(trajectory, desc=f"Episode {episode_id}")):
            if step_id % save_interval != 0:
                continue
            
            try:
                # 更新Godot中的机器人状态
                success = self.godot_client.update_robot_state(
                    position=state.get('position', [0, 0, 0]),
                    orientation=state.get('orientation', [0, 0, 0]),
                    joint_angles=state.get('joint_angles', [0]*6)
                )
                
                if not success:
                    print(f"Warning: Failed to update robot state at step {step_id}")
                    continue
                
                # 短暂等待以确保Godot更新完成
                time.sleep(0.01)
                
                # 捕获图像
                images = self.godot_client.capture_images(cameras)
                
                if not images:
                    print(f"Warning: No images captured at step {step_id}")
                    continue
                
                # 保存图像
                for key, image in images.items():
                    view_id = key.split('_')[-1] if '_' in key else '0'
                    
                    if 'rgb' in key.lower():
                        filename = episode_dir / "rgb" / f"frame_{step_id:06d}_view_{view_id}.png"
                        cv2.imwrite(str(filename), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
                        
                        # 添加到COCO标注
                        image_id = self.num_frames
                        rel_filename = f"episode_{episode_id:06d}/rgb/{filename.name}"
                        self.coco_annotation.add_image(
                            image_id, rel_filename, 
                            image.shape[1], image.shape[0]
                        )
                    
                    elif 'depth' in key.lower():
                        filename = episode_dir / "depth" / f"frame_{step_id:06d}_view_{view_id}.png"
                        # 深度图保存为16位
                        if image.max() <= 1.0:
                            depth_16bit = (image * 65535).astype(np.uint16)
                        else:
                            depth_16bit = image.astype(np.uint16)
                        cv2.imwrite(str(filename), depth_16bit)
                    
                    elif 'seg' in key.lower():
                        filename = episode_dir / "segmentation" / f"frame_{step_id:06d}_view_{view_id}.png"
                        cv2.imwrite(str(filename), image)
                
                # 获取关键点和边界框
                keypoints = self.godot_client.get_robot_keypoints_2d()
                bboxes = self.godot_client.get_bounding_boxes()
                
                # 添加标注
                image_id = self.num_frames
                
                for bbox in bboxes:
                    cat_id = 1 if bbox['class'] == 'robot' else 2
                    area = bbox['bbox'][2] * bbox['bbox'][3]
                    self.coco_annotation.add_bbox_annotation(
                        image_id, cat_id, bbox['bbox'], area
                    )
                
                if keypoints:
                    kp_flat = []
                    for kp in keypoints:
                        kp_flat.extend(kp['position'])
                        kp_flat.append(1 if kp.get('visible', True) else 0)
                    
                    self.coco_annotation.add_keypoint_annotation(
                        image_id, kp_flat, len(keypoints)
                    )
                
                self.num_frames += 1
                metadata['num_frames'] += 1
            
            except Exception as e:
                print(f"Error processing step {step_id}: {e}")
                continue
        
        # 保存元数据
        metadata['end_time'] = datetime.now().isoformat()
        with open(episode_dir / "metadata.json", 'w') as f:
            json.dump(metadata, f, indent=2)
        
        self.num_episodes += 1
    
    def save_coco_annotations(self, filename: str = "annotations.json"):
        """保存COCO格式标注"""
        output_file = self.output_dir / filename
        
        coco_data = self.coco_annotation.to_coco_format()
        
        with open(output_file, 'w') as f:
            json.dump(coco_data, f, indent=2)
        
        print(f"\nCOCO annotations saved to: {output_file}")
        print(f"  Images: {len(coco_data['images'])}")
        print(f"  Annotations: {len(coco_data['annotations'])}")
    
    def generate_summary(self) -> str:
        """生成数据集摘要"""
        summary = []
        summary.append("="*70)
        summary.append("CV数据集生成摘要")
        summary.append("="*70)
        
        summary.append(f"\n输出目录: {self.output_dir}")
        summary.append(f"Episodes: {self.num_episodes}")
        summary.append(f"总帧数: {self.num_frames}")
        
        # 计算存储大小
        total_size = sum(f.stat().st_size for f in self.output_dir.rglob('*') if f.is_file())
        summary.append(f"存储大小: {total_size / (1024**2):.1f} MB")
        
        return "\n".join(summary)


# 简化的测试用轨迹生成
def generate_test_trajectory(length: int = 100) -> List[Dict]:
    """生成测试轨迹"""
    trajectory = []
    
    for i in range(length):
        state = {
            'position': [i * 0.01, 0, 0],  # 缓慢前进
            'orientation': [0, 0, 0],
            'joint_angles': [np.sin(i * 0.1) * 0.2] * 6  # 轻微摆动
        }
        trajectory.append(state)
    
    return trajectory


if __name__ == "__main__":
    print("CV数据生成器测试")
    print("="*70)
    
    # 创建生成器
    generator = CVDataGenerator(
        output_dir="data/test_cv_dataset",
        godot_host='localhost',
        godot_port=9999
    )
    
    # 连接到Godot
    print("\n连接到Godot...")
    if generator.connect_to_godot():
        print("✓ 连接成功")
        
        # 生成测试数据
        print("\n生成测试数据 (1 episode, 100 frames)...")
        trajectory = generate_test_trajectory(100)
        
        generator.generate_episode_images(
            episode_id=0,
            trajectory=trajectory,
            save_interval=10  # 每10帧保存一次
        )
        
        # 保存COCO标注
        generator.save_coco_annotations()
        
        # 显示摘要
        print("\n" + generator.generate_summary())
        
        # 断开连接
        generator.disconnect()
    
    else:
        print("✗ 连接失败")
        print("\n提示: 确保Godot项目正在运行并已实现TCP服务器")
