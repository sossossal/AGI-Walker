from typing import Optional, Dict, Callable, Any
import sys
import os
import asyncio

# 添加项目根目录以导入 python_api
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from python_api.godot_client import GodotSimulationClient

class GodotController:
    _instance = None
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(GodotController, cls).__new__(cls)
            cls._instance.client = GodotSimulationClient()
            cls._instance.broadcast_callback = None
        return cls._instance
    
    def set_broadcast_callback(self, callback: Callable[[Dict], Any]):
        """设置用于WebSocket广播的回调函数"""
        self.broadcast_callback = callback
        
        # 设置底层客户端的数据回调
        def on_godot_data(data):
            if self.broadcast_callback:
                # 包装为统一的消息格式
                msg = {
                    "type": "godot_data",
                    "data": data
                }
                # 注意：这里可能是在子线程调用的，需要确保callback能处理线程安全
                # 或者由server.py端处理 event loop
                self.broadcast_callback(msg)
                
        self.client.set_data_callback(on_godot_data)

    def connect(self, host: str, port: int) -> bool:
        self.client.host = host
        self.client.port = port
        success = self.client.connect()
        if success and self.broadcast_callback:
             self.broadcast_callback({
                 "type": "godot_connection", 
                 "status": "connected",
                 "host": host,
                 "port": port
             })
        return success

    def disconnect(self):
        self.client.disconnect()
        if self.broadcast_callback:
             self.broadcast_callback({
                 "type": "godot_connection", 
                 "status": "disconnected"
             })

    def is_connected(self) -> bool:
        return self.client.is_connected()

    
    def start_simulation(self, physics_config: Optional[Dict] = None) -> bool:
        # Harmonized logic: use cached robot config if available
        # This supports the stateless API call form /api/godot/start which only sends physics
        
        # Determine robot config to use
        # In a real scenario, Godot might already have the robot loaded and just needs 'start_sim'
        # But our protocol expects 'robot' in start_sim command data sometimes.
        # Let's send what we have.
        robot_config = getattr(self, 'cached_robot_config', {})
        
        return self.client.start_simulation(robot_config)

    def stop_simulation(self) -> bool:
        return self.client.stop_simulation()

    def load_robot(self, parts: list, connections: list) -> bool:
        # Cache the config for later start_simulation calls
        self.cached_robot_config = {
            'parts': parts,
            'connections': connections
        }
        return self.client.load_robot_config(parts, connections)

    def update_params(self, params: Dict) -> bool:
        return self.client.update_parameters(params)

# 全局单例
godot_controller = GodotController()
