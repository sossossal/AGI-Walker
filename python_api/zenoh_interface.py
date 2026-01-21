"""
Zenoh 通信接口层
提供统一的 Pub/Sub API，用于 AGI-Walker 与 OpenNeuro 生态集成
"""

import json
import time
from typing import Callable, Optional, Dict, Any
from dataclasses import dataclass

try:
    import zenoh
    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False
    print("⚠️ Zenoh 未安装，请运行: pip install eclipse-zenoh")


@dataclass
class ZenohConfig:
    """Zenoh 配置"""
    mode: str = "peer"  # "peer" 或 "client"
    connect: Optional[str] = None  # 例如 "tcp/127.0.0.1:7447"
    listen: Optional[str] = None   # 例如 "tcp/0.0.0.0:7447"


class ZenohInterface:
    """
    Zenoh 通信接口
    
    用法示例:
        zenoh_if = ZenohInterface()
        zenoh_if.declare_publisher("rt/robot/cmd")
        zenoh_if.publish("rt/robot/cmd", {"joint_0": 1.5})
        
        def callback(data):
            print(f"Received: {data}")
        zenoh_if.declare_subscriber("rt/robot/state", callback)
    """
    
    def __init__(self, config: Optional[ZenohConfig] = None):
        if not ZENOH_AVAILABLE:
            raise ImportError("Zenoh 库未安装")
        
        self.config = config or ZenohConfig()
        self.session = None
        self.publishers: Dict[str, Any] = {}
        self.subscribers: Dict[str, Any] = {}
        self._connect()
    
    def _connect(self):
        """建立 Zenoh 会话"""
        zenoh_config = zenoh.Config()
        
        if self.config.mode == "client" and self.config.connect:
            zenoh_config.insert_json5("connect/endpoints", json.dumps([self.config.connect]))
        elif self.config.mode == "peer" and self.config.listen:
            zenoh_config.insert_json5("listen/endpoints", json.dumps([self.config.listen]))
        
        self.session = zenoh.open(zenoh_config)
        print(f"✅ Zenoh 会话已建立 (mode: {self.config.mode})")
    
    def declare_publisher(self, key: str) -> None:
        """
        声明发布者
        
        Args:
            key: Zenoh 资源键，例如 "rt/robot/cmd"
        """
        if key not in self.publishers:
            pub = self.session.declare_publisher(key)
            self.publishers[key] = pub
            print(f"📤 Publisher 已创建: {key}")
    
    def publish(self, key: str, data: Any, serialize: bool = True) -> None:
        """
        发布数据
        
        Args:
            key: 资源键
            data: 要发布的数据 (dict/list 会自动 JSON 序列化)
            serialize: 是否自动序列化为 JSON
        """
        if key not in self.publishers:
            self.declare_publisher(key)
        
        payload = json.dumps(data).encode() if serialize else data
        self.publishers[key].put(payload)
    
    def declare_subscriber(self, key: str, callback: Callable[[Any], None]) -> None:
        """
        声明订阅者
        
        Args:
            key: 资源键
            callback: 回调函数，接收解析后的数据
        """
        def zenoh_callback(sample):
            try:
                data = json.loads(sample.payload.decode())
                callback(data)
            except json.JSONDecodeError:
                # 如果不是 JSON，直接传递原始字节
                callback(sample.payload)
        
        sub = self.session.declare_subscriber(key, zenoh_callback)
        self.subscribers[key] = sub
        print(f"📥 Subscriber 已创建: {key}")
    
    def close(self):
        """关闭 Zenoh 会话"""
        for pub in self.publishers.values():
            pub.undeclare()
        for sub in self.subscribers.values():
            sub.undeclare()
        
        if self.session:
            self.session.close()
        print("🔌 Zenoh 会话已关闭")


# ==================== 示例代码 ====================

if __name__ == "__main__":
    print("Zenoh Interface Demo")
    
    # 创建接口
    zenoh_if = ZenohInterface()
    
    # 订阅状态
    def on_state(data):
        print(f"[State] {data}")
    
    zenoh_if.declare_subscriber("rt/robot/state", on_state)
    
    # 发布命令
    zenoh_if.declare_publisher("rt/robot/cmd")
    
    for i in range(5):
        cmd = {"joint_positions": [i * 0.1, i * 0.2, i * 0.3]}
        zenoh_if.publish("rt/robot/cmd", cmd)
        print(f"[Cmd] Sent: {cmd}")
        time.sleep(1)
    
    zenoh_if.close()
