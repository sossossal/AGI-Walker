"""
Godot仿真引擎通信客户端

连接到Godot TCP服务器，实现：
- 启动/停止仿真
- 发送机器人配置
- 动态调整参数
- 接收实时反馈数据
"""

import socket
import json
import threading
import time
from typing import Dict, List, Optional, Callable
import struct


class GodotSimulationClient:
    """Godot仿真客户端"""
    
    def __init__(self, host: str = "127.0.0.1", port: int = 9999):
        self.host = host
        self.port = port
        self.socket: Optional[socket.socket] = None
        self.connected = False
        self.running = False
        
        # 数据回调
        self.data_callback: Optional[Callable] = None
        
        # 接收线程
        self.receive_thread: Optional[threading.Thread] = None
        
    def connect(self, timeout: float = 5.0) -> bool:
        """
        连接到Godot服务器
        
        Args:
            timeout: 连接超时时间（秒）
            
        Returns:
            是否连接成功
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(timeout)
            self.socket.connect((self.host, self.port))
            self.socket.settimeout(None)  # 连接后取消超时
            
            self.connected = True
            print(f"✓ 已连接到Godot服务器 {self.host}:{self.port}")
            
            # 启动接收线程
            self.running = True
            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.receive_thread.start()
            
            return True
            
        except socket.timeout:
            print(f"✗ 连接超时: Godot服务器 {self.host}:{self.port} 未响应")
            return False
        except ConnectionRefusedError:
            print(f"✗ 连接被拒绝: Godot服务器未启动或端口 {self.port} 不可用")
            return False
        except Exception as e:
            print(f"✗ 连接失败: {e}")
            return False
            
    def disconnect(self):
        """断开连接"""
        self.running = False
        self.connected = False
        
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
            
        print("已断开Godot连接")
        
    def send_command(self, command: str, data: Optional[Dict] = None) -> bool:
        """
        发送命令到Godot
        
        Args:
            command: 命令类型 (start_sim, stop_sim, update_params, load_robot)
            data: 命令数据
            
        Returns:
            是否发送成功
        """
        if not self.connected or not self.socket:
            print("✗ 未连接到Godot服务器")
            return False
            
        try:
            message = {
                'command': command,
                'data': data or {},
                'timestamp': time.time()
            }
            
            # 转换为JSON
            json_str = json.dumps(message)
            json_bytes = json_str.encode('utf-8')
            
            # 发送长度前缀 + 数据
            length = len(json_bytes)
            self.socket.sendall(struct.pack('!I', length))
            self.socket.sendall(json_bytes)
            
            return True
            
        except Exception as e:
            print(f"✗ 发送命令失败: {e}")
            self.disconnect()
            return False
            
    def start_simulation(self, robot_config: Dict) -> bool:
        """
        启动仿真
        
        Args:
            robot_config: 机器人配置
            
        Returns:
            是否启动成功
        """
        return self.send_command('start_sim', {
            'robot': robot_config,
            'physics': {
                'gravity': 9.81,
                'timestep': 0.01
            }
        })
        
    def stop_simulation(self) -> bool:
        """停止仿真"""
        return self.send_command('stop_sim')
        
    def update_parameters(self, params: Dict) -> bool:
        """
        实时更新参数
        
        Args:
            params: 参数字典，例如 {'motor_power': 1.2, 'joint_stiffness': 2.0}
            
        Returns:
            是否更新成功
        """
        return self.send_command('update_params', params)
        
    def load_robot_config(self, parts: List[Dict], connections: List[Dict]) -> bool:
        """
        加载机器人配置
        
        Args:
            parts: 零件列表
            connections: 连接关系
            
        Returns:
            是否加载成功
        """
        return self.send_command('load_robot', {
            'parts': parts,
            'connections': connections
        })
        
    def set_data_callback(self, callback: Callable[[Dict], None]):
        """
        设置数据接收回调
        
        Args:
            callback: 回调函数，接收数据字典
        """
        self.data_callback = callback
        
    def _receive_loop(self):
        """接收数据循环（在后台线程运行）"""
        while self.running and self.connected:
            try:
                # 读取长度前缀
                length_bytes = self._recv_exactly(4)
                if not length_bytes:
                    break
                    
                length = struct.unpack('!I', length_bytes)[0]
                
                # 读取数据
                data_bytes = self._recv_exactly(length)
                if not data_bytes:
                    break
                    
                # 解析JSON
                json_str = data_bytes.decode('utf-8')
                data = json.loads(json_str)
                
                # 调用回调
                if self.data_callback:
                    self.data_callback(data)
                    
            except Exception as e:
                if self.running:
                    print(f"✗ 接收数据错误: {e}")
                break
                
        self.connected = False
        
    def _recv_exactly(self, n: int) -> Optional[bytes]:
        """
        接收指定字节数
        
        Args:
            n: 字节数
            
        Returns:
            接收到的数据，失败返回None
        """
        data = b''
        while len(data) < n:
            try:
                chunk = self.socket.recv(n - len(data))
                if not chunk:
                    return None
                data += chunk
            except:
                return None
        return data
        
    def is_connected(self) -> bool:
        """检查是否已连接"""
        return self.connected
        

# 模拟Godot服务器（用于测试）
class MockGodotServer:
    """模拟的Godot服务器，用于测试"""
    
    def __init__(self, port: int = 9999):
        self.port = port
        self.server_socket: Optional[socket.socket] = None
        self.running = False
        self.server_thread: Optional[threading.Thread] = None
        
    def start(self):
        """启动模拟服务器"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(('127.0.0.1', self.port))
            self.server_socket.listen(1)
            
            self.running = True
            self.server_thread = threading.Thread(target=self._server_loop, daemon=True)
            self.server_thread.start()
            
            print(f"✓ 模拟Godot服务器启动于端口 {self.port}")
            return True
            
        except Exception as e:
            print(f"✗ 启动模拟服务器失败: {e}")
            return False
            
    def stop(self):
        """停止服务器"""
        self.running = False
        if self.server_socket:
            self.server_socket.close()
            
    def _server_loop(self):
        """服务器循环"""
        while self.running:
            try:
                client, addr = self.server_socket.accept()
                print(f"✓ 客户端连接: {addr}")
                
                # 启动客户端处理线程
                thread = threading.Thread(
                    target=self._handle_client,
                    args=(client,),
                    daemon=True
                )
                thread.start()
                
            except:
                break
                
    def _handle_client(self, client: socket.socket):
        """处理客户端连接"""
        try:
            while self.running:
                # 读取长度
                length_bytes = client.recv(4)
                if not length_bytes:
                    break
                    
                length = struct.unpack('!I', length_bytes)[0]
                
                # 读取数据
                data_bytes = client.recv(length)
                message = json.loads(data_bytes.decode('utf-8'))
                
                print(f"收到命令: {message['command']}")
                
                # 模拟发送反馈数据
                if message['command'] == 'start_sim':
                    # 模拟仿真数据
                    for i in range(10):
                        feedback = {
                            'type': 'simulation_data',
                            'position': i * 0.1,
                            'velocity': 0.1,
                            'battery': 100 - i,
                            'timestamp': time.time()
                        }
                        
                        json_str = json.dumps(feedback)
                        json_bytes = json_str.encode('utf-8')
                        client.sendall(struct.pack('!I', len(json_bytes)))
                        client.sendall(json_bytes)
                        
                        time.sleep(0.1)
                        
        except Exception as e:
            print(f"客户端处理错误: {e}")
        finally:
            client.close()


# 使用示例
if __name__ == "__main__":
    # 测试模式
    print("=== Godot通信客户端测试 ===\n")
    
    # 启动模拟服务器
    server = MockGodotServer()
    server.start()
    time.sleep(0.5)
    
    # 创建客户端
    client = GodotSimulationClient()
    
    # 设置数据回调
    def on_data(data):
        print(f"接收数据: {data}")
        
    client.set_data_callback(on_data)
    
    # 连接
    if client.connect():
        # 加载机器人配置
        robot_config = {
            'parts': [
                {'id': 'motor_1', 'type': 'motor', 'power': 500}
            ],
            'connections': []
        }
        
        client.load_robot_config(robot_config['parts'], robot_config['connections'])
        time.sleep(0.2)
        
        # 启动仿真
        client.start_simulation(robot_config)
        time.sleep(2)
        
        # 更新参数
        client.update_parameters({'motor_power': 1.5})
        time.sleep(1)
        
        # 停止
        client.stop_simulation()
        time.sleep(0.2)
        
        client.disconnect()
    
    server.stop()
    print("\n测试完成")
