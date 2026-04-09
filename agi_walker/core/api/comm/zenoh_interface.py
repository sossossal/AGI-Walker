"""
Zenoh 通信接口层
提供统一的 Pub/Sub API，用于 AGI-Walker 与 OpenNeuro 生态集成
"""

import json
import time
import logging
import sys
import types
from typing import Callable, Optional, Dict, Any, Type
from dataclasses import dataclass
from unittest.mock import Mock

try:
    import zenoh

    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False
    zenoh = None
    print("Zenoh not installed. Run: pip install eclipse-zenoh")

try:
    from google.protobuf.message import Message

    PROTOBUF_AVAILABLE = True
except ImportError:
    PROTOBUF_AVAILABLE = False

logger = logging.getLogger(__name__)


@dataclass
class ZenohConfig:
    """Zenoh 配置"""

    mode: str = "peer"  # "peer" 或 "client"
    connect: Optional[str] = None  # 例如 "tcp/127.0.0.1:7447"
    listen: Optional[str] = None  # 例如 "tcp/0.0.0.0:7447"


class ZenohInterface:
    """
    AGI-Walker V2.0 Enhanced Zenoh Interface.
    Supports both JSON and High-Performance Protobuf binary streams.
    """

    def __init__(self, config: Optional[ZenohConfig] = None):
        if not ZENOH_AVAILABLE and not isinstance(zenoh, Mock):
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
            zenoh_config.insert_json5(
                "connect/endpoints", json.dumps([self.config.connect])
            )
        elif self.config.mode == "peer" and self.config.listen:
            zenoh_config.insert_json5(
                "listen/endpoints", json.dumps([self.config.listen])
            )

        self.session = zenoh.open(zenoh_config)
        logger.info(f"Zenoh session established (mode: {self.config.mode})")

    def declare_publisher(self, key: str) -> None:
        if key not in self.publishers:
            pub = self.session.declare_publisher(key)
            self.publishers[key] = pub
            logger.debug(f"Publisher created: {key}")

    def publish(self, key: str, data: Any) -> None:
        """
        发布数据。自动处理 Protobuf 消息和普通 Python 对象。
        """
        if key not in self.publishers:
            self.declare_publisher(key)

        if PROTOBUF_AVAILABLE and isinstance(data, Message):
            # 自动进行 Protobuf 二进制序列化
            payload = data.SerializeToString()
        elif isinstance(data, (dict, list)):
            # 自动进行 JSON 序列化
            payload = json.dumps(data).encode()
        else:
            # 原始字节或其他
            payload = bytes(data)

        self.publishers[key].put(payload)

    def declare_subscriber(
        self,
        key: str,
        callback: Callable[[Any], None],
        pb_class: Optional[Type[Message]] = None,
    ) -> None:
        """
        声明订阅者。

        Args:
            key: 资源键
            callback: 回调函数
            pb_class: (可选) Protobuf 类。如果指定，将自动解析二进制数据。
        """

        def zenoh_callback(sample):
            try:
                payload_bytes = bytes(sample.payload)

                if pb_class:
                    # 尝试进行 Protobuf 解析
                    msg = pb_class()
                    msg.ParseFromString(payload_bytes)
                    callback(msg)
                else:
                    # 回退到 JSON 或 原始字节
                    try:
                        data = json.loads(payload_bytes.decode())
                        callback(data)
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        callback(payload_bytes)

            except Exception as e:
                logger.error(f"Subscriber callback error on key {key}: {e}")

        sub = self.session.declare_subscriber(key, zenoh_callback)
        self.subscribers[key] = sub
        logger.debug(f"Subscriber created: {key}")

    def close(self):
        """关闭 Zenoh 会话"""
        if self.session:
            self.session.close()
            self.session = None
        logger.info("Zenoh session closed")


def _install_legacy_module_alias() -> None:
    """
    Expose this module under the historical python_api namespace so legacy
    patches in tests and old callers still target the same module object.
    """

    module = sys.modules[__name__]
    python_api_pkg = sys.modules.setdefault(
        "python_api", types.ModuleType("python_api")
    )
    comm_pkg = sys.modules.setdefault(
        "python_api.comm", types.ModuleType("python_api.comm")
    )

    if not hasattr(python_api_pkg, "__path__"):
        python_api_pkg.__path__ = []
    if not hasattr(comm_pkg, "__path__"):
        comm_pkg.__path__ = []

    setattr(python_api_pkg, "comm", comm_pkg)
    setattr(comm_pkg, "zenoh_interface", module)
    sys.modules["python_api.comm.zenoh_interface"] = module


_install_legacy_module_alias()


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
