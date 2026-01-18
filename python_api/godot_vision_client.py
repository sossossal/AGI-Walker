"""
Godot视觉数据采集客户端
Godot Vision Data Client

功能:
- TCP连接到Godot服务器
- 发送机器人状态更新
- 接收渲染的图像数据
- 支持RGB、深度图、分割图
"""

import socket
import json
import base64
import numpy as np
from PIL import Image
import io
from typing import Dict, List, Optional, Tuple
import time


class GodotVisionClient:
    """Godot视觉数据客户端"""
    
    def __init__(self, host: str = 'localhost', port: int = 9999, timeout: float = 30.0):
        """
        初始化客户端
        
        参数:
            host: Godot服务器地址
            port: 端口号
            timeout: 超时时间（秒）
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.socket = None
        self.connected = False
    
    def connect(self, retries: int = 3) -> bool:
        """
        连接到Godot服务器
        
        参数:
            retries: 重试次数
        
        返回:
            是否成功连接
        """
        for attempt in range(retries):
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(self.timeout)
                self.socket.connect((self.host, self.port))
                self.connected = True
                print(f"✓ Connected to Godot server at {self.host}:{self.port}")
                return True
            except Exception as e:
                print(f"Connection attempt {attempt + 1}/{retries} failed: {e}")
                if attempt < retries - 1:
                    time.sleep(2)
                else:
                    print(f"✗ Failed to connect after {retries} attempts")
        
        return False
    
    def disconnect(self):
        """断开连接"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.connected = False
            print("Disconnected from Godot server")
    
    def send_command(self, command: Dict) -> Optional[Dict]:
        """
        发送命令并接收响应
        
        参数:
            command: 命令字典
        
        返回:
            响应数据
        """
        if not self.connected:
            raise ConnectionError("Not connected to Godot server. Call connect() first.")
        
        try:
            # 发送命令
            message = json.dumps(command).encode('utf-8')
            # 添加消息结束标记
            message += b'\n'
            self.socket.sendall(message)
            
            # 接收响应
            response_data = b""
            buffer_size = 4096
            
            while True:
                try:
                    chunk = self.socket.recv(buffer_size)
                    if not chunk:
                        break
                    
                    response_data += chunk
                    
                    # 检查是否接收完整（简化版，实际可能需要更复杂的协议）
                    if b'\n' in chunk or len(chunk) < buffer_size:
                        break
                
                except socket.timeout:
                    print("Warning: Socket timeout while receiving data")
                    break
            
            # 解析响应
            if response_data:
                try:
                    response = json.loads(response_data.decode('utf-8').strip())
                    return response
                except json.JSONDecodeError as e:
                    print(f"JSON decode error: {e}")
                    print(f"Received data: {response_data[:200]}")  # 打印前200字节
                    return None
            
            return None
        
        except Exception as e:
            print(f"Error sending command: {e}")
            self.connected = False
            return None
    
    def update_robot_state(self, position: List[float], 
                          orientation: List[float],
                          joint_angles: List[float]) -> bool:
        """
        更新机器人状态
        
        参数:
            position: 位置 [x, y, z]
            orientation: 方向 [roll, pitch, yaw] (弧度)
            joint_angles: 关节角度列表
        
        返回:
            是否成功
        """
        command = {
            'command': 'update_robot',
            'state': {
                'position': position,
                'orientation': orientation,
                'joint_angles': joint_angles
            }
        }
        
        response = self.send_command(command)
        return response is not None and response.get('status') == 'success'
    
    def capture_images(self, cameras: Optional[List[int]] = None) -> Dict[str, np.ndarray]:
        """
        捕获所有视角的图像
        
        参数:
            cameras: 要捕获的相机ID列表，None表示全部
        
        返回:
            图像字典 {名称: numpy数组}
        """
        command = {
            'command': 'capture',
            'cameras': cameras
        }
        
        response = self.send_command(command)
        
        if not response or response.get('status') != 'success':
            print(f"Image capture failed: {response}")
            return {}
        
        # 解码图像
        images = {}
        image_data = response.get('images', {})
        
        for key, base64_data in image_data.items():
            try:
                # Base64解码
                image_bytes = base64.b64decode(base64_data)
                
                # 转换为PIL Image
                image = Image.open(io.BytesIO(image_bytes))
                
                # 转换为numpy数组
                images[key] = np.array(image)
            
            except Exception as e:
                print(f"Error decoding image {key}: {e}")
        
        return images
    
    def set_camera_params(self, camera_id: int, 
                         fov: float = 70.0,
                         position: Optional[List[float]] = None,
                         rotation: Optional[List[float]] = None) -> bool:
        """
        设置相机参数
        
        参数:
            camera_id: 相机ID
            fov: 视场角（度）
            position: 位置 [x, y, z]
            rotation: 旋转 [roll, pitch, yaw] (度)
        
        返回:
            是否成功
        """
        params = {'fov': fov}
        
        if position is not None:
            params['position'] = position
        
        if rotation is not None:
            params['rotation'] = rotation
        
        command = {
            'command': 'set_camera',
            'camera_id': camera_id,
            'params': params
        }
        
        response = self.send_command(command)
        return response is not None and response.get('status') == 'success'
    
    def set_environment(self, environment_type: str = 'flat',
                       lighting: str = 'day') -> bool:
        """
        设置环境
        
        参数:
            environment_type: 环境类型 ('flat', 'uphill', 'obstacles', etc.)
            lighting: 光照 ('day', 'night', 'indoor')
        
        返回:
            是否成功
        """
        command = {
            'command': 'set_environment',
            'environment_type': environment_type,
            'lighting': lighting
        }
        
        response = self.send_command(command)
        return response is not None and response.get('status') == 'success'
    
    def get_robot_keypoints_2d(self) -> List[Dict]:
        """
        获取机器人关键点的2D投影
        
        返回:
            关键点列表 [{'name': str, 'position': [x, y], 'visible': bool}, ...]
        """
        command = {'command': 'get_keypoints_2d'}
        
        response = self.send_command(command)
        
        if response and response.get('status') == 'success':
            return response.get('keypoints', [])
        
        return []
    
    def get_bounding_boxes(self) -> List[Dict]:
        """
        获取场景中对象的边界框
        
        返回:
            边界框列表 [{'class': str, 'bbox': [x, y, w, h], 'confidence': float}, ...]
        """
        command = {'command': 'get_bounding_boxes'}
        
        response = self.send_command(command)
        
        if response and response.get('status') == 'success':
            return response.get('bounding_boxes', [])
        
        return []
    
    def ping(self) -> bool:
        """
        Ping服务器检查连接
        
        返回:
            是否连通
        """
        command = {'command': 'ping'}
        
        response = self.send_command(command)
        return response is not None and response.get('status') == 'success'


if __name__ == "__main__":
    print("Godot视觉客户端测试")
    print("="*70)
    
    # 创建客户端
    client = GodotVisionClient(host='localhost', port=9999)
    
    # 尝试连接
    print("\n1. 连接测试...")
    if client.connect():
        print("✓ 连接成功")
        
        # Ping测试
        print("\n2. Ping测试...")
        if client.ping():
            print("✓ Ping成功")
        
        # 更新机器人状态
        print("\n3. 更新机器人状态...")
        if client.update_robot_state(
            position=[0, 0, 0],
            orientation=[0, 0, 0],
            joint_angles=[0] * 6
        ):
            print("✓ 状态更新成功")
        
        # 捕获图像
        print("\n4. 捕获图像...")
        images = client.capture_images()
        
        if images:
            print(f"✓ 捕获了 {len(images)} 张图像:")
            for name, img in images.items():
                print(f"  - {name}: {img.shape}")
        else:
            print("✗ 未捕获到图像（可能Godot服务器未完全实现）")
        
        # 断开连接
        client.disconnect()
    
    else:
        print("✗ 连接失败")
        print("\n提示:")
        print("  1. 确保Godot项目正在运行")
        print("  2. 确保TCP服务器已启动 (端口9999)")
        print("  3. 检查防火墙设置")
