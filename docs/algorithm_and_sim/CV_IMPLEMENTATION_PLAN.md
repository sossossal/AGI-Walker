# AGI-Walker 计算机视�?(CV) 数据生成实现方案

**日期**: 2026-01-18  
**版本**: 1.0  
**状�?*: 实施方案

---

## 📋 执行摘要

### 当前状�?
- �?数值数据生�? 92% (完成)
- ⚠️ 视觉数据生成: 60% (需要实�?

### 目标
实现完整的CV训练数据生成能力，包括：
- RGB图像
- 深度�?
- 分割掩码
- 关键点标�?
- 边界框标�?

### 预期完成�?
60% �?**95%**

---

## 🎯 方案概览

### 三种实现方案

| 方案 | 复杂�?| 效果 | 时间 | 推荐�?|
|------|-------|------|------|--------|
| 方案1: Godot集成 | �?| 优秀 | 5-7�?| ⭐⭐⭐⭐�?|
| 方案2: PyBullet集成 | �?| 良好 | 3-4�?| ⭐⭐⭐⭐ |
| 方案3: 简化渲�?| �?| 基础 | 1-2�?| ⭐⭐�?|

**推荐**: 方案1 (Godot集成) - 最完整的解决方�?

---

## 方案1: Godot集成 (推荐)

### 概述
利用现有的Godot项目，通过Python-Godot通信生成视觉数据

### 架构

```
Python控制�?                Godot渲染引擎
    �?                          �?
    ├─> 发送机器人状�?────────> �?
    �?                          ├─> 更新机器人姿�?
    �?                          ├─> 渲染场景
    �?  <──── 返回图像数据 <──── ├─> 捕获相机视图
    �?                          ├─> 生成深度�?
    �?                          └─> 语义分割
    �?
    └─> 保存图像 + 标注
```

### 实现步骤

#### 步骤1: Godot端实�?(2-3�?

**1.1 创建相机系统**

```gdscript
# godot_project/scripts/VisionDataGenerator.gd
extends Node

var cameras = []
var capture_resolution = Vector2(640, 480)

func _ready():
    setup_cameras()

func setup_cameras():
    # 第三人称相机
    var third_person_cam = create_camera(
        Vector3(2, 1.5, 2),  # 位置
        Vector3(-30, -45, 0)  # 旋转
    )
    cameras.append(third_person_cam)
    
    # 第一人称相机 (机器人视�?
    var first_person_cam = create_camera(
        Vector3(0, 0.3, 0.2),  # 相对机器�?
        Vector3(0, 0, 0)
    )
    cameras.append(first_person_cam)
    
    # 俯视相机
    var top_down_cam = create_camera(
        Vector3(0, 5, 0),
        Vector3(-90, 0, 0)
    )
    cameras.append(top_down_cam)

func create_camera(pos: Vector3, rot: Vector3) -> Camera:
    var camera = Camera.new()
    camera.transform.origin = pos
    camera.rotation_degrees = rot
    return camera

func capture_all_views() -> Dictionary:
    var images = {}
    
    for i in range(cameras.size()):
        var cam = cameras[i]
        
        # RGB图像
        images["rgb_" + str(i)] = capture_rgb(cam)
        
        # 深度�?
        images["depth_" + str(i)] = capture_depth(cam)
        
        # 分割�?
        images["segmentation_" + str(i)] = capture_segmentation(cam)
    
    return images

func capture_rgb(camera: Camera) -> Image:
    var viewport = get_viewport()
    viewport.set_clear_mode(Viewport.CLEAR_MODE_ONLY_NEXT_FRAME)
    
    # 渲染一�?
    yield(get_tree(), "idle_frame")
    
    # 捕获图像
    var image = viewport.get_texture().get_data()
    image.flip_y()
    
    return image

func capture_depth(camera: Camera) -> Image:
    # 切换到深度渲染模�?
    var shader = preload("res://shaders/depth_shader.shader")
    # ... 实现深度渲染
    pass

func capture_segmentation(camera: Camera) -> Image:
    # 语义分割渲染
    # 每个对象类别用不同颜�?
    pass
```

**1.2 创建深度着色器**

```glsl
// godot_project/shaders/depth_shader.shader
shader_type spatial;

varying float depth;

void vertex() {
    vec4 world_pos = WORLD_MATRIX * vec4(VERTEX, 1.0);
    vec4 view_pos = VIEW_MATRIX * world_pos;
    depth = -view_pos.z;
}

void fragment() {
    # 深度归一化到0-1
    float normalized_depth = depth / 10.0; # 10m最大深�?
    ALBEDO = vec3(normalized_depth);
}
```

**1.3 TCP通信服务�?*

```gdscript
# godot_project/scripts/TCPVisionServer.gd
extends Node

var server = TCP_Server.new()
var connection = null
var port = 9999

func _ready():
    server.listen(port)
    print("Vision server listening on port ", port)

func _process(delta):
    # 接受连接
    if server.is_connection_available():
        connection = server.take_connection()
        print("Client connected")
    
    # 处理请求
    if connection and connection.get_available_bytes() > 0:
        var request = connection.get_utf8_string(connection.get_available_bytes())
        handle_request(request)

func handle_request(request: String):
    var data = JSON.parse(request).result
    
    match data.command:
        "capture":
            var images = $VisionDataGenerator.capture_all_views()
            send_images(images)
        
        "update_robot":
            update_robot_state(data.state)
        
        "set_camera":
            set_camera_params(data.camera_id, data.params)

func send_images(images: Dictionary):
    var response = {
        "status": "success",
        "images": {}
    }
    
    for key in images:
        var image = images[key]
        # 转换为base64或保存到临时文件
        response.images[key] = image_to_base64(image)
    
    connection.put_data(JSON.print(response).to_utf8())

func image_to_base64(image: Image) -> String:
    var buffer = image.save_png_to_buffer()
    return Marshalls.raw_to_base64(buffer)
```

#### 步骤2: Python端实�?(2-3�?

**2.1 Godot通信客户�?*

```python
# python_api/godot_vision_client.py
"""
Godot视觉数据采集客户�?
"""

import socket
import json
import base64
import numpy as np
from PIL import Image
import io
from typing import Dict, List, Optional


class GodotVisionClient:
    """Godot视觉数据客户�?""
    
    def __init__(self, host: str = 'localhost', port: int = 9999):
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
    
    def connect(self):
        """连接到Godot服务�?""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            self.connected = True
            print(f"Connected to Godot server at {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.socket:
            self.socket.close()
            self.connected = False
    
    def send_command(self, command: Dict) -> Dict:
        """发送命令并接收响应"""
        if not self.connected:
            raise ConnectionError("Not connected to Godot server")
        
        # 发�?
        message = json.dumps(command).encode('utf-8')
        self.socket.sendall(message)
        
        # 接收
        response_data = b""
        while True:
            chunk = self.socket.recv(4096)
            if not chunk:
                break
            response_data += chunk
            
            # 尝试解析JSON（简化版�?
            try:
                response = json.loads(response_data.decode('utf-8'))
                return response
            except:
                continue
        
        return {}
    
    def update_robot_state(self, position: List[float], 
                          orientation: List[float],
                          joint_angles: List[float]):
        """更新机器人状�?""
        command = {
            'command': 'update_robot',
            'state': {
                'position': position,
                'orientation': orientation,
                'joint_angles': joint_angles
            }
        }
        
        return self.send_command(command)
    
    def capture_images(self) -> Dict[str, np.ndarray]:
        """捕获所有视角的图像"""
        command = {'command': 'capture'}
        
        response = self.send_command(command)
        
        if response.get('status') != 'success':
            raise RuntimeError("Image capture failed")
        
        # 解码图像
        images = {}
        for key, base64_data in response['images'].items():
            image_bytes = base64.b64decode(base64_data)
            image = Image.open(io.BytesIO(image_bytes))
            images[key] = np.array(image)
        
        return images
    
    def set_camera_params(self, camera_id: int, fov: float = 70.0,
                         position: Optional[List[float]] = None):
        """设置相机参数"""
        command = {
            'command': 'set_camera',
            'camera_id': camera_id,
            'params': {
                'fov': fov,
                'position': position or [0, 0, 0]
            }
        }
        
        return self.send_command(command)
```

**2.2 CV数据生成�?*

```python
# python_api/cv_data_generator.py
"""
计算机视觉训练数据生成器
"""

import numpy as np
import cv2
from pathlib import Path
from typing import Dict, List, Tuple
import json
from tqdm import tqdm

from python_api.godot_vision_client import GodotVisionClient
from python_api.data_recorder import DataRecorder


class CVDataGenerator:
    """CV训练数据生成�?""
    
    def __init__(self, output_dir: str = "data/cv_dataset"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Godot客户�?
        self.godot_client = GodotVisionClient()
        
        # 数据统计
        self.num_frames = 0
    
    def connect_to_godot(self) -> bool:
        """连接到Godot"""
        return self.godot_client.connect()
    
    def generate_episode_images(self, episode_id: int, 
                               trajectory: List[Dict],
                               save_interval: int = 10):
        """
        为一个episode生成图像
        
        参数:
            episode_id: Episode ID
            trajectory: 轨迹数据 (包含每步的状�?
            save_interval: 保存间隔
        """
        episode_dir = self.output_dir / f"episode_{episode_id:06d}"
        episode_dir.mkdir(exist_ok=True)
        
        # 创建子目�?
        (episode_dir / "rgb").mkdir(exist_ok=True)
        (episode_dir / "depth").mkdir(exist_ok=True)
        (episode_dir / "segmentation").mkdir(exist_ok=True)
        
        annotations = []
        
        for step_id, state in enumerate(tqdm(trajectory, desc=f"Episode {episode_id}")):
            if step_id % save_interval != 0:
                continue
            
            # 更新Godot中的机器人状�?
            self.godot_client.update_robot_state(
                position=state.get('position', [0, 0, 0]),
                orientation=state.get('orientation', [0, 0, 0]),
                joint_angles=state.get('joint_angles', [0]*6)
            )
            
            # 捕获图像
            images = self.godot_client.capture_images()
            
            # 保存图像
            for view_id, (key, image) in enumerate(images.items()):
                if 'rgb' in key:
                    filename = episode_dir / "rgb" / f"frame_{step_id:06d}_view_{view_id}.png"
                    cv2.imwrite(str(filename), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
                
                elif 'depth' in key:
                    filename = episode_dir / "depth" / f"frame_{step_id:06d}_view_{view_id}.png"
                    # 深度图保存为16�?
                    depth_16bit = (image * 65535).astype(np.uint16)
                    cv2.imwrite(str(filename), depth_16bit)
                
                elif 'segmentation' in key:
                    filename = episode_dir / "segmentation" / f"frame_{step_id:06d}_view_{view_id}.png"
                    cv2.imwrite(str(filename), image)
            
            # 生成标注
            annotation = self.generate_annotations(state, images)
            annotation['frame_id'] = step_id
            annotations.append(annotation)
            
            self.num_frames += 1
        
        # 保存标注文件
        with open(episode_dir / "annotations.json", 'w') as f:
            json.dump(annotations, f, indent=2)
    
    def generate_annotations(self, state: Dict, images: Dict) -> Dict:
        """
        生成标注信息
        
        包括:
        - 机器人姿�?
        - 关键点位�?
        - 边界�?
        - 语义标签
        """
        annotation = {
            'robot_state': state,
            'keypoints': self.detect_keypoints(images),
            'bounding_boxes': self.detect_bounding_boxes(images),
            'semantic_labels': self.extract_semantic_labels(images)
        }
        
        return annotation
    
    def detect_keypoints(self, images: Dict) -> List[Dict]:
        """检测关键点（关节位置）"""
        # 这里应该从Godot获取3D关节位置
        # 并投影到2D图像平面
        keypoints = [
            {'name': 'hip', 'position_2d': [320, 240, 1.0]},  # [x, y, visibility]
            {'name': 'knee', 'position_2d': [340, 300, 1.0]},
            # ... 更多关节
        ]
        return keypoints
    
    def detect_bounding_boxes(self, images: Dict) -> List[Dict]:
        """检测边界框"""
        # 机器人的边界�?
        boxes = [
            {
                'class': 'robot',
                'bbox': [100, 150, 500, 400],  # [x, y, w, h]
                'confidence': 1.0
            }
        ]
        return boxes
    
    def extract_semantic_labels(self, images: Dict) -> Dict:
        """提取语义标签"""
        # 从分割图中提�?
        labels = {
            'robot': 1,
            'ground': 2,
            'obstacle': 3,
            'background': 0
        }
        return labels
    
    def batch_generate(self, num_episodes: int, 
                      episode_length: int = 100,
                      save_interval: int = 5):
        """
        批量生成CV数据�?
        
        参数:
            num_episodes: Episode数量
            episode_length: 每个episode的长�?
            save_interval: 图像保存间隔
        """
        if not self.connect_to_godot():
            print("Failed to connect to Godot. Make sure Godot is running.")
            return
        
        print(f"Generating CV dataset: {num_episodes} episodes")
        
        for ep_id in range(num_episodes):
            # 这里应该运行仿真获取轨迹
            # 简化示例：使用随机轨迹
            trajectory = self.generate_random_trajectory(episode_length)
            
            # 生成图像
            self.generate_episode_images(ep_id, trajectory, save_interval)
        
        print(f"\nGeneration complete!")
        print(f"Total frames: {self.num_frames}")
        print(f"Output directory: {self.output_dir}")
    
    def generate_random_trajectory(self, length: int) -> List[Dict]:
        """生成随机轨迹（用于测试）"""
        trajectory = []
        
        for i in range(length):
            state = {
                'position': [i * 0.01, 0, 0],  # 前进
                'orientation': [0, 0, 0],
                'joint_angles': [0] * 6
            }
            trajectory.append(state)
        
        return trajectory
```

#### 步骤3: 数据集格�?(1�?

**COCO格式支持**

```python
# python_api/cv_dataset_converter.py
"""
转换CV数据集为标准格式
"""

import json
from pathlib import Path


class COCOConverter:
    """转换为COCO格式"""
    
    def __init__(self, dataset_dir: str):
        self.dataset_dir = Path(dataset_dir)
    
    def convert(self, output_file: str):
        """转换整个数据�?""
        coco_data = {
            'images': [],
            'annotations': [],
            'categories': [
                {'id': 1, 'name': 'robot'},
                {'id': 2, 'name': 'ground'},
                {'id': 3, 'name': 'obstacle'}
            ]
        }
        
        annotation_id = 0
        
        # 遍历所有episodes
        for episode_dir in self.dataset_dir.glob("episode_*"):
            annotations_file = episode_dir / "annotations.json"
            
            if not annotations_file.exists():
                continue
            
            with open(annotations_file, 'r') as f:
                annotations = json.load(f)
            
            for ann in annotations:
                frame_id = ann['frame_id']
                
                # 添加图像信息
                image_info = {
                    'id': len(coco_data['images']),
                    'file_name': f"{episode_dir.name}/rgb/frame_{frame_id:06d}_view_0.png",
                    'width': 640,
                    'height': 480
                }
                coco_data['images'].append(image_info)
                
                # 添加标注
                for bbox in ann['bounding_boxes']:
                    coco_ann = {
                        'id': annotation_id,
                        'image_id': image_info['id'],
                        'category_id': 1,  # robot
                        'bbox': bbox['bbox'],
                        'area': bbox['bbox'][2] * bbox['bbox'][3],
                        'iscrowd': 0
                    }
                    coco_data['annotations'].append(coco_ann)
                    annotation_id += 1
                
                # 添加关键�?
                for kp in ann['keypoints']:
                    coco_kp = {
                        'id': annotation_id,
                        'image_id': image_info['id'],
                        'category_id': 1,
                        'keypoints': kp['position_2d'] * 6,  # COCO格式
                        'num_keypoints': 6
                    }
                    coco_data['annotations'].append(coco_kp)
                    annotation_id += 1
        
        # 保存
        with open(output_file, 'w') as f:
            json.dump(coco_data, f, indent=2)
        
        print(f"COCO dataset saved to: {output_file}")
```

---

## 方案2: PyBullet集成 (备�?

### 概述
使用PyBullet的渲染功能生成图�?

### 优势
- 完全Python实现
- 更容易集�?
- 不需要Godot运行

---

## 方案3: 简化渲�?(快速方�?

### 概述
使用matplotlib或pygame进行2D渲染

### 适用场景
- 快速原�?
- 不需要真实感
- 2D任务

---

## 📊 对比总结

| 特�?| Godot | PyBullet | 简化渲�?|
|------|-------|----------|----------|
| 图像质量 | ⭐⭐⭐⭐�?| ⭐⭐⭐⭐ | ⭐⭐ |
| 实现难度 | �?| �?| �?|
| 开发时�?| 5-7�?| 3-4�?| 1-2�?|
| 推荐�?| ⭐⭐⭐⭐�?| ⭐⭐⭐⭐ | ⭐⭐�?|

---

## 🎯 推荐实施路线

### Phase 1: 基础实现 (3-4�?
1. Godot TCP服务�?
2. Python客户�?
3. 基础图像捕获

### Phase 2: 增强功能 (2-3�?
4. 深度图渲�?
5. 语义分割
6. 多相机视�?

### Phase 3: 集成优化 (1-2�?
7. 批量生成集成
8. 数据集格式转�?
9. 性能优化

**总时�?*: 6-9�?

---

## �?实施后效�?

**数据类型**:
- RGB图像 (640x480 或更�?
- 深度�?(16-bit)
- 语义分割�?
- 关键点标�?
- 边界框标�?
