import logging
import inspect
from typing import Any, Callable, Dict, Optional

from agi_walker.core.api.comm.godot_client import GodotSimulationClient
from web_panel.core_api import DEFAULT_GODOT_SESSION_ID
from web_panel.ws_protocol import MessageType, WsMessage

logger = logging.getLogger(__name__)


class GodotController:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(GodotController, cls).__new__(cls)
            cls._instance.clients: Dict[str, GodotSimulationClient] = {}
            cls._instance.cached_configs: Dict[str, dict] = {}
            cls._instance.legacy_session_id = DEFAULT_GODOT_SESSION_ID
            cls._instance.broadcast_callback = None
            cls._instance.broadcast_callback_accepts_session = False
        return cls._instance

    def _callback_accepts_session(self, callback: Callable[..., Any]) -> bool:
        try:
            signature = inspect.signature(callback)
        except (TypeError, ValueError):
            return False

        for parameter in signature.parameters.values():
            if parameter.kind == inspect.Parameter.VAR_POSITIONAL:
                return True

        positional = [
            parameter
            for parameter in signature.parameters.values()
            if parameter.kind
            in (
                inspect.Parameter.POSITIONAL_ONLY,
                inspect.Parameter.POSITIONAL_OR_KEYWORD,
            )
        ]
        return len(positional) >= 2

    def _resolve_session_id(self, session_id: Optional[str] = None) -> str:
        return session_id or DEFAULT_GODOT_SESSION_ID

    def _bind_client_callbacks(
        self, client: GodotSimulationClient, session_id: str
    ) -> GodotSimulationClient:
        def on_data(data: Any):
            if self.broadcast_callback:
                msg = WsMessage(
                    type=MessageType.TELEMETRY_UPDATE.value,
                    payload={"data": data},
                    status="push",
                ).to_dict()
                self._broadcast(msg, session_id=session_id)

        if hasattr(client, "set_data_callback"):
            client.set_data_callback(on_data)
        try:
            client.data_callback = on_data
        except Exception:
            pass
        return client

    def get_client(self, session_id: Optional[str] = None) -> GodotSimulationClient:
        target = self._resolve_session_id(session_id)
        if target not in self.clients:
            self.clients[target] = self._bind_client_callbacks(
                GodotSimulationClient(),
                target,
            )
        return self.clients[target]

    @property
    def client(self) -> GodotSimulationClient:
        return self.get_client(self.legacy_session_id)

    @client.setter
    def client(self, value: GodotSimulationClient) -> None:
        self.clients[self.legacy_session_id] = self._bind_client_callbacks(
            value,
            self.legacy_session_id,
        )

    @property
    def cached_robot_config(self) -> Dict[str, Any]:
        return self.cached_configs.get(self.legacy_session_id, {})

    @cached_robot_config.setter
    def cached_robot_config(self, value: Dict[str, Any]) -> None:
        self.cached_configs[self.legacy_session_id] = value

    def release_session(self, session_id: str) -> None:
        if session_id in self.clients:
            try:
                self.clients[session_id].disconnect()
            except Exception:
                pass
            del self.clients[session_id]
        if session_id in self.cached_configs:
            del self.cached_configs[session_id]

    def _broadcast(
        self, message: Dict[str, Any], session_id: Optional[str] = None
    ) -> None:
        if not self.broadcast_callback:
            return

        target_session_id = self._resolve_session_id(session_id)
        if self.broadcast_callback_accepts_session:
            self.broadcast_callback(target_session_id, message)
        else:
            self.broadcast_callback(message)

    def set_broadcast_callback(self, callback: Callable[..., Any]) -> None:
        """设置用于WebSocket广播的回调函数"""
        self.broadcast_callback = callback
        self.broadcast_callback_accepts_session = self._callback_accepts_session(
            callback
        )

    def connect(
        self,
        host: str,
        port: int,
        session_id: Optional[str] = None,
    ) -> bool:
        target_session_id = self._resolve_session_id(session_id)
        client = self.get_client(target_session_id)
        client.host = host
        client.port = port
        success = client.connect()
        if success:
            self._broadcast(
                WsMessage(
                    type=MessageType.CONNECTION_STATUS.value,
                    payload={
                        "connected": True,
                        "details": {
                            "mode": "legacy_controller",
                            "host": host,
                            "port": port,
                        },
                    },
                    status="push",
                ).to_dict(),
                session_id=target_session_id,
            )
        return success

    def disconnect(self, session_id: Optional[str] = None) -> None:
        target_session_id = self._resolve_session_id(session_id)
        client = self.get_client(target_session_id)
        client.disconnect()
        self._broadcast(
            WsMessage(
                type=MessageType.CONNECTION_STATUS.value,
                payload={
                    "connected": False,
                    "details": {"mode": "legacy_controller"},
                },
                status="push",
            ).to_dict(),
            session_id=target_session_id,
        )
        self.release_session(target_session_id)

    def is_connected(self, session_id: Optional[str] = None) -> bool:
        target_session_id = self._resolve_session_id(session_id)
        if target_session_id not in self.clients:
            return False
        return self.clients[target_session_id].is_connected()

    def start_simulation(
        self,
        physics_config: Optional[Dict] = None,
        session_id: Optional[str] = None,
    ) -> bool:
        target_session_id = self._resolve_session_id(session_id)
        client = self.get_client(target_session_id)
        robot_config = self.cached_configs.get(target_session_id, {})
        if not robot_config and target_session_id == self.legacy_session_id:
            robot_config = self.cached_robot_config

        return client.start_simulation(
            robot_config,
            physics_config=physics_config,
        )

    def stop_simulation(self, session_id: Optional[str] = None) -> bool:
        target_session_id = self._resolve_session_id(session_id)
        return self.get_client(target_session_id).stop_simulation()

    def load_robot(
        self,
        parts: list,
        connections: list,
        session_id: Optional[str] = None,
    ) -> bool:
        target_session_id = self._resolve_session_id(session_id)
        client = self.get_client(target_session_id)
        # Cache the config for later start_simulation calls
        self.cached_configs[target_session_id] = {
            "parts": parts,
            "connections": connections,
        }
        return client.load_robot_config(parts, connections)

    def update_params(self, params: Dict, session_id: Optional[str] = None) -> bool:
        target_session_id = self._resolve_session_id(session_id)
        return self.get_client(target_session_id).update_parameters(params)

    def send_instruction_set(
        self, instruction_payload: Dict, session_id: Optional[str] = None
    ) -> bool:
        target_session_id = self._resolve_session_id(session_id)
        return self.get_client(target_session_id).send_instruction_set(
            instruction_payload
        )

    def configure_simulated_circuit(
        self, circuit_payload: Dict, session_id: Optional[str] = None
    ) -> bool:
        target_session_id = self._resolve_session_id(session_id)
        return self.get_client(target_session_id).configure_simulated_circuit(
            circuit_payload
        )


# 全局单例
godot_controller = GodotController()
