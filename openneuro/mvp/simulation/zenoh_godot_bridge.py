"""
OpenNeuro → Godot Bridge
将虚拟Zone Controller的数据转发到Godot仿真引擎

功能:
1. 订阅所有Zone的状态数据
2. 转换为Godot可理解的格式
3. 通过TCP发送到Godot
"""

import zenoh
import socket
import json
import time
import threading
import sys
from pathlib import Path

# 添加AGI-Walker路径
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))

class OpenNeuroGodotBridge:
    def __init__(self, godot_host='127.0.0.1', godot_port=9999):
        self.godot_host = godot_host
        self.godot_port = godot_port
        self.godot_socket = None
        self.running = False
        
        # Zone状态缓存
        self.zone_states = {}
        
        # 初始化Zenoh
        print("[Bridge] Initializing Zenoh session...")
        self.z_session = zenoh.open(zenoh.Config())
        print("[Bridge] ✓ Zenoh session opened")
        
        # 订阅所有Zone的状态
        self.subscribers = []
        for zone_id in range(7):  # Zone 0-6
            topic = f"zone/{zone_id}/state"
            sub = self.z_session.declare_subscriber(
                topic,
                lambda sample, zid=zone_id: self.on_zone_state(zid, sample)
            )
            self.subscribers.append(sub)
            print(f"[Bridge] ✓ Subscribed to {topic}")
    
    def on_zone_state(self, zone_id, sample):
        """处理Zone状态更新"""
        try:
            data = json.loads(sample.payload.decode('utf-8'))
            self.zone_states[zone_id] = data
            
            # 转发到Godot
            if self.godot_socket:
                self.forward_to_godot(zone_id, data)
        except Exception as e:
            print(f"[Bridge] Error processing zone {zone_id}: {e}")
    
    def forward_to_godot(self, zone_id, data):
        """转发数据到Godot"""
        try:
            # 构造Godot命令
            godot_cmd = {
                "command": "update_zone",
                "zone_id": zone_id,
                "joints": data.get("joints", []),
                "timestamp": data.get("timestamp", time.time())
            }
            
            # 发送JSON + 换行符
            msg = json.dumps(godot_cmd) + "\n"
            self.godot_socket.sendall(msg.encode('utf-8'))
        except Exception as e:
            print(f"[Bridge] Godot send error: {e}")
            self.godot_socket = None
    
    def connect_godot(self):
        """连接到Godot TCP服务器"""
        print(f"[Bridge] Connecting to Godot at {self.godot_host}:{self.godot_port}...")
        try:
            self.godot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.godot_socket.connect((self.godot_host, self.godot_port))
            self.godot_socket.settimeout(1.0)
            print("[Bridge] ✓ Connected to Godot")
            return True
        except Exception as e:
            print(f"[Bridge] ✗ Failed to connect to Godot: {e}")
            self.godot_socket = None
            return False
    
    def run(self):
        """主循环"""
        print("=" * 60)
        print("OpenNeuro → Godot Bridge")
        print("=" * 60)
        
        # 尝试连接Godot (可选)
        self.connect_godot()
        
        self.running = True
        print("\n[Bridge] Running. Press Ctrl+C to stop.\n")
        
        try:
            while self.running:
                # 定期打印状态
                if self.zone_states:
                    active_zones = list(self.zone_states.keys())
                    print(f"\r[Bridge] Active zones: {active_zones}", end="", flush=True)
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n[Bridge] Stopping...")
        finally:
            self.stop()
    
    def stop(self):
        """清理资源"""
        self.running = False
        for sub in self.subscribers:
            sub.undeclare()
        self.z_session.close()
        if self.godot_socket:
            self.godot_socket.close()
        print("[Bridge] ✓ Stopped")


def main():
    import argparse
    parser = argparse.ArgumentParser(description='OpenNeuro → Godot Bridge')
    parser.add_argument('--godot-host', default='127.0.0.1', help='Godot host')
    parser.add_argument('--godot-port', type=int, default=9999, help='Godot port')
    args = parser.parse_args()
    
    bridge = OpenNeuroGodotBridge(args.godot_host, args.godot_port)
    bridge.run()


if __name__ == "__main__":
    main()
