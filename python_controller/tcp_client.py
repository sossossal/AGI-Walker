"""
TCP客户端 - 连接到Godot仿真器
负责接收传感器数据，发送电机指令
"""

import socket
import json
import threading
import queue
import time
from typing import Optional, Dict


class GodotClient:
    """Godot仿真器TCP客户端"""
    
    def __init__(self, host: str = '127.0.0.1', port: int = 9999):
        self.host = host
        self.port = port
        self.socket: Optional[socket.socket] = None
        self.buffer = ""
        
        # 传感器数据队列（最多保留10个）
        self.sensor_queue = queue.Queue(maxsize=10)
        
        # 线程控制
        self.running = False
        self.recv_thread: Optional[threading.Thread] = None
        
        # 统计信息
        self.packets_received = 0
        self.packets_sent = 0
        
    def connect(self, timeout: float = 5.0) -> bool:
        """连接到Godot服务器"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(timeout)
            self.socket.connect((self.host, self.port))
            self.socket.settimeout(None)  # 切换到阻塞模式
            
            print(f"✅ 已连接到Godot仿真器 {self.host}:{self.port}")
            
            # 启动接收线程
            self.running = True
            self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
            self.recv_thread.start()
            
            return True
            
        except socket.timeout:
            print(f"❌ 连接超时: {self.host}:{self.port}")
            return False
        except ConnectionRefusedError:
            print(f"❌ 连接被拒绝，请确保Godot仿真器正在运行")
            return False
        except Exception as e:
            print(f"❌ 连接错误: {e}")
            return False
    
    def _recv_loop(self):
        """接收线程 - 持续接收传感器数据"""
        while self.running:
            try:
                data = self.socket.recv(4096).decode('utf-8')
                if not data:
                    print("⚠️ 服务器关闭连接")
                    break
                
                self.buffer += data
                
                # 处理完整的JSON行
                while '\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\n', 1)
                    line = line.strip()
                    
                    if not line:
                        continue
                    
                    try:
                        sensor_data = json.loads(line)
                        self.packets_received += 1
                        
                        # 非阻塞放入队列
                        try:
                            self.sensor_queue.put_nowait(sensor_data)
                        except queue.Full:
                            # 队列满时，丢弃最旧的数据
                            self.sensor_queue.get()
                            self.sensor_queue.put(sensor_data)
                            
                    except json.JSONDecodeError as e:
                        print(f"⚠️ JSON解析错误: {e}")
                        print(f"   原始数据: {line}")
                        
            except Exception as e:
                if self.running:
                    print(f"❌ 接收错误: {e}")
                break
        
        self.running = False
    
    def get_latest_sensors(self) -> Optional[Dict]:
        """获取最新的传感器数据（非阻塞）"""
        try:
            return self.sensor_queue.get_nowait()
        except queue.Empty:
            return None
    
    def wait_for_sensors(self, timeout: float = 1.0) -> Optional[Dict]:
        """等待传感器数据（阻塞，带超时）"""
        try:
            return self.sensor_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def send_motor_commands(self, commands: Dict) -> bool:
        """发送电机控制指令"""
        if not self.socket or not self.running:
            return False
        
        try:
            msg = json.dumps(commands) + '\n'
            self.socket.sendall(msg.encode('utf-8'))
            self.packets_sent += 1
            return True
            
        except Exception as e:
            print(f"❌ 发送错误: {e}")
            return False
    
    def get_stats(self) -> Dict:
        """获取统计信息"""
        return {
            "connected": self.running,
            "packets_received": self.packets_received,
            "packets_sent": self.packets_sent,
            "queue_size": self.sensor_queue.qsize()
        }
    
    def close(self):
        """关闭连接"""
        self.running = False
        
        if self.recv_thread and self.recv_thread.is_alive():
            self.recv_thread.join(timeout=1.0)
        
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        
        print("🔌 已断开连接")


# 测试代码
if __name__ == "__main__":
    client = GodotClient()
    
    if not client.connect():
        print("连接失败，退出")
        exit(1)
    
    print("\n📡 开始接收传感器数据...")
    print("按 Ctrl+C 退出\n")
    
    try:
        last_print = time.time()
        
        while True:
            # 获取最新传感器数据
            sensor_data = client.get_latest_sensors()
            
            if sensor_data:
                # 每秒打印一次
                now = time.time()
                if now - last_print >= 1.0:
                    print(f"[{sensor_data['timestamp']:.2f}s] "
                          f"躯干高度: {sensor_data['torso_height']:.3f}m | "
                          f"姿态: Roll={sensor_data['sensors']['imu']['orient'][0]:.1f}° "
                          f"Pitch={sensor_data['sensors']['imu']['orient'][1]:.1f}°")
                    
                    stats = client.get_stats()
                    print(f"   统计: 收到{stats['packets_received']}包 | "
                          f"发送{stats['packets_sent']}包 | "
                          f"队列{stats['queue_size']}/10")
                    
                    last_print = now
                
                # 发送简单的测试指令（让腿摆动）
                t = sensor_data['timestamp']
                angle = 30 * (1 if int(t) % 2 == 0 else -1)  # 每秒切换方向
                
                client.send_motor_commands({
                    "motors": {
                        "hip_left": angle,
                        "hip_right": -angle
                    }
                })
            
            time.sleep(0.01)  # 100Hz轮询
            
    except KeyboardInterrupt:
        print("\n\n⏹️ 用户中断")
    finally:
        client.close()
