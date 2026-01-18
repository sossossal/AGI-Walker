"""
OpenNeuro Zenoh → Godot桥接器

功能: 接收Zenoh数据并转发到Godot TCP服务器
"""

import zenoh
import socket
import json
import time
import threading
from pathlib import Path
import sys

# 添加AGI-Walker路径
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))
from python_api.godot_client import GodotSimulationClient


class ZenohGodotBridge:
    """Zenoh到Godot的桥接器"""
    
    def __init__(self, 
                 zenoh_config=None,
                 godot_host='127.0.0.1',
                 godot_port=9999):
        """
        初始化桥接器
        
        Args:
            zenoh_config: Zenoh配置（默认自动发现）
            godot_host: Godot服务器地址
            godot_port: Godot服务器端口
        """
        self.godot_host = godot_host
        self.godot_port = godot_port
        
        # Godot客户端
        self.godot_client = GodotSimulationClient(godot_host, godot_port)
        
        # Zenoh会话
        conf = zenoh.Config()
        if zenoh_config:
            conf.insert_json5(zenoh.config.MODE_KEY, json.dumps(zenoh_config))
        
        print(f"[Zenoh] 正在打开会话...")
        self.z_session = zenoh.open(conf)
        print(f"[Zenoh] ✓ 会话已建立")
        
        # 订阅器
        self.subscribers = []
        self.running = False
        
        # 统计
        self.msg_count = 0
        self.last_print_time = time.time()
        
    def connect_godot(self):
        """连接到Godot"""
        print(f"[Godot] 连接到 {self.godot_host}:{self.godot_port}...")
        if self.godot_client.connect(timeout=5.0):
            print(f"[Godot] ✓ 已连接")
            return True
        else:
            print(f"[Godot] ✗ 连接失败")
            return False
    
    def subscribe_imu(self):
        """订阅IMU数据"""
        topic = "/robot/sensor/imu"
        
        def callback(sample):
            try:
                # 解析JSON数据
                data_str = sample.payload.decode('utf-8')
                data = json.loads(data_str)
                
                # 转发到Godot
                self.forward_to_godot(data)
                
                # 统计
                self.msg_count += 1
                if time.time() - self.last_print_time >= 1.0:
                    print(f"[Stats] 消息数: {self.msg_count}, 频率: {self.msg_count}Hz")
                    self.msg_count = 0
                    self.last_print_time = time.time()
                    
            except Exception as e:
                print(f"[Error] 处理消息失败: {e}")
        
        sub = self.z_session.declare_subscriber(topic, callback)
        self.subscribers.append(sub)
        print(f"[Zenoh] ✓ 已订阅: {topic}")
    
    def forward_to_godot(self, data):
        """
        将Zenoh数据转发到Godot
        
        Args:
            data: 传感器数据字典
        """
        if not self.godot_client.is_connected():
            return
        
        # 构造Godot命令
        params = {
            'imu_accel': data.get('accel', [0, 0, 0]),
            'imu_gyro': data.get('gyro', [0, 0, 0]),
            'timestamp': data.get('time', 0)
        }
        
        # 更新参数
        self.godot_client.update_parameters(params)
    
    def run(self):
        """启动桥接"""
        print("=" * 60)
        print("OpenNeuro Zenoh → Godot 桥接器")
        print("=" * 60)
        
        # 连接Godot
        if not self.connect_godot():
            print("[Warning] Godot未连接，仅监听Zenoh数据")
        
        # 订阅topics
        self.subscribe_imu()
        
        self.running = True
        print("\n[Bridge] ✓ 桥接器运行中，按Ctrl+C停止\n")
        
        try:
            # 保持运行
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n[Bridge] 正在停止...")
        finally:
            self.stop()
    
    def stop(self):
        """停止桥接"""
        self.running = False
        
        # 取消订阅
        for sub in self.subscribers:
            sub.undeclare()
        
        # 关闭会话
        self.z_session.close()
        
        # 断开Godot
        if self.godot_client:
            self.godot_client.disconnect()
        
        print("[Bridge] ✓ 已停止")


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Zenoh → Godot桥接器')
    parser.add_argument('--godot-host', default='127.0.0.1', help='Godot地址')
    parser.add_argument('--godot-port', type=int, default=9999, help='Godot端口')
    parser.add_argument('--zenoh-mode', default='client', help='Zenoh模式')
    parser.add_argument('--zenoh-router', help='Zenoh Router地址 (tcp/IP:PORT)')
    
    args = parser.parse_args()
    
    # Zenoh配置
    zenoh_config = None
    if args.zenoh_router:
        zenoh_config = {
            "mode": args.zenoh_mode,
            "connect": {
                "endpoints": [args.zenoh_router]
            }
        }
    
    # 创建桥接器
    bridge = ZenohGodotBridge(
        zenoh_config=zenoh_config,
        godot_host=args.godot_host,
        godot_port=args.godot_port
    )
    
    # 运行
    bridge.run()


if __name__ == "__main__":
    main()
