"""
TCP ↔ Zenoh 双向桥接器
保持 AGI-Walker 现有 TCP 接口向后兼容，同时引入 Zenoh 通信能力
"""

import socket
import threading
import json
import time
from typing import Optional
from python_api.zenoh_interface import ZenohInterface, ZenohConfig


class TcpZenohBridge:
    """
    TCP-Zenoh 桥接器
    
    架构:
        Godot (TCP Client) ←→ TcpZenohBridge ←→ Zenoh Network
        
    功能:
        1. 接收 Godot 的 TCP 连接 (端口 9090)
        2. 将 TCP 数据转发到 Zenoh ("rt/godot/state")
        3. 将 Zenoh 数据转发到 TCP ("rt/python/cmd")
    """
    
    def __init__(
        self, 
        tcp_host: str = '127.0.0.1',
        tcp_port: int = 9090,
        zenoh_config: Optional[ZenohConfig] = None
    ):
        self.tcp_host = tcp_host
        self.tcp_port = tcp_port
        
        # TCP 服务器
        self.tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.tcp_client: Optional[socket.socket] = None
        
        # Zenoh 接口
        self.zenoh = ZenohInterface(zenoh_config or ZenohConfig())
        
        # 线程控制
        self.running = False
        self.tcp_thread: Optional[threading.Thread] = None
        self.zenoh_thread: Optional[threading.Thread] = None
        
        print(f"🌉 TCP-Zenoh 桥接器初始化完成")
    
    def start(self):
        """启动桥接器"""
        self.running = True
        
        # 启动 TCP 服务器
        self.tcp_server.bind((self.tcp_host, self.tcp_port))
        self.tcp_server.listen(1)
        print(f"🔌 TCP 服务器监听: {self.tcp_host}:{self.tcp_port}")
        
        # 订阅 Zenoh 命令 (Python → Godot)
        self.zenoh.declare_subscriber("rt/python/cmd", self._on_zenoh_cmd)
        
        # 启动 TCP 接收线程
        self.tcp_thread = threading.Thread(target=self._tcp_loop, daemon=True)
        self.tcp_thread.start()
        
        print("✅ 桥接器已启动")
    
    def _tcp_loop(self):
        """TCP 接收循环 (Godot → Zenoh)"""
        while self.running:
            try:
                # 等待 Godot 连接
                print("⏳ 等待 Godot 连接...")
                self.tcp_client, addr = self.tcp_server.accept()
                print(f"✅ Godot 已连接: {addr}")
                
                # 接收数据
                while self.running:
                    data = self.tcp_client.recv(4096)
                    if not data:
                        print("❌ Godot 断开连接")
                        break
                    
                    # 解析 JSON (假设 Godot 发送 JSON 格式)
                    try:
                        msg = json.loads(data.decode())
                        # 转发到 Zenoh
                        self.zenoh.publish("rt/godot/state", msg)
                    except json.JSONDecodeError:
                        print(f"⚠️ 无效 JSON: {data[:50]}")
                
            except Exception as e:
                print(f"❌ TCP 错误: {e}")
                time.sleep(1)
    
    def _on_zenoh_cmd(self, data):
        """Zenoh 命令回调 (Python → Godot)"""
        if self.tcp_client:
            try:
                # 将 Zenoh 数据转发到 TCP
                payload = json.dumps(data).encode() + b'\n'
                self.tcp_client.send(payload)
            except Exception as e:
                print(f"❌ 发送到 Godot 失败: {e}")
    
    def stop(self):
        """停止桥接器"""
        self.running = False
        
        if self.tcp_client:
            self.tcp_client.close()
        self.tcp_server.close()
        self.zenoh.close()
        
        print("🔌 桥接器已停止")


# ==================== 独立运行模式 ====================

if __name__ == "__main__":
    print("="*60)
    print("TCP-Zenoh 桥接器 (独立模式)")
    print("="*60)
    
    bridge = TcpZenohBridge()
    bridge.start()
    
    try:
        print("\n按 Ctrl+C 停止桥接器...")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\n停止中...")
        bridge.stop()
