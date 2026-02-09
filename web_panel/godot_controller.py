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
        # TODO: python_api 需要更新 start_simulation 以支持独立通过 physics 参数启动
        # 目前 API 需要 robot_config。我们将传入一个空配置或上次的配置
        # 暂时构造一个 dummy config 或者要求前端传 robot
        pass 
        # 修改 plan: 让 server.py 处理逻辑，这里只透传
        return self.client.send_command('start_sim', {'physics': physics_config or {}})

    def stop_simulation(self) -> bool:
        return self.client.stop_simulation()

    def load_robot(self, parts: list, connections: list) -> bool:
        return self.client.load_robot_config(parts, connections)

    def update_params(self, params: Dict) -> bool:
        return self.client.update_parameters(params)

# 全局单例
godot_controller = GodotController()
