import time
import json
import logging
import random
from typing import Dict, Any, List, Optional
from ..api.comm.zenoh_interface import ZenohInterface, ZenohConfig

logger = logging.getLogger(__name__)

class SwarmAgent:
    """
    AGI-Walker V3.0 Swarm Coordination Agent.
    Handles decentralized peer discovery and task bidding (Auction Protocol).
    """
    def __init__(self, robot_id: str, capabilities: List[str], zenoh_router: str = None):
        self.id = robot_id
        self.capabilities = capabilities
        self.battery_level = 1.0 # 0.0 to 1.0
        self.current_load = 0.0
        
        # Zenoh Link
        conf = ZenohConfig(connect=zenoh_router, mode="client") if zenoh_router else ZenohConfig()
        self.zenoh = ZenohInterface(conf)
        
        # 资源键
        self.discovery_key = "ag/swarm/discovery"
        self.auction_key = "ag/swarm/tasks/auction"
        self.bid_key = "ag/swarm/tasks/bids"
        
        self.active_peers: Dict[str, Dict[str, Any]] = {}
        self.running = False

    def start(self):
        self.running = True
        logger.info(f"Swarm Agent {self.id} started. Capabilities: {self.capabilities}")
        
        # 1. 订阅发现信号
        self.zenoh.declare_subscriber(self.discovery_key, self._on_discovery)
        # 2. 订阅任务拍卖
        self.zenoh.declare_subscriber(self.auction_key, self._on_task_auction)
        
        # 3. 主循环：广播心跳与状态
        while self.running:
            self._broadcast_status()
            # 模拟负载波动
            self.battery_level -= 0.001
            self.current_load = random.random() * 0.5
            time.sleep(2.0)

    def _broadcast_status(self):
        status = {
            "id": self.id,
            "capabilities": self.capabilities,
            "battery": self.battery_level,
            "load": self.current_load,
            "ts": time.time()
        }
        self.zenoh.publish(self.discovery_key, status)

    def _on_discovery(self, data: Dict[str, Any]):
        peer_id = data.get("id")
        if peer_id and peer_id != self.id:
            self.active_peers[peer_id] = data
            # logger.debug(f"Discovered Peer: {peer_id}")

    def _on_task_auction(self, task: Dict[str, Any]):
        """处理任务拍卖指令"""
        required_cap = task.get("required_capability")
        if required_cap in self.capabilities:
            # 计算竞标价格: 价格越低越容易胜出
            # Cost = (1/Battery) * Load * Distance_to_Target
            bid_price = (1.0 / (self.battery_level + 0.1)) * (self.current_load + 0.5)
            
            bid = {
                "task_id": task.get("id"),
                "robot_id": self.id,
                "price": round(bid_price, 3),
                "ts": time.time()
            }
            
            logger.info(f"💰 Bidding for task {task.get('id')}: price={bid['price']}")
            self.zenoh.publish(self.bid_key, bid)

    def stop(self):
        self.running = False
        self.zenoh.close()
