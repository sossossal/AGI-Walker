"""
Web-Godot WebSocket Protocol Handler

Implements the v1.0 protocol for real-time communication between web panel
and Godot simulation engine. Handles message routing, request-response matching,
telemetry streaming, and error handling.

Protocol Specification: See WEB_GODOT_PROTOCOL.md
"""
import logging
logger = logging.getLogger(__name__)

from dataclasses import dataclass, field
from typing import Dict, Any, Optional, Callable
from enum import Enum
import json
from datetime import datetime
import uuid


class MessageType(Enum):
    """Message type enumeration for protocol"""
    # Commands (web → godot)
    SIMULATION_START = "simulation.start"
    SIMULATION_STOP = "simulation.stop"
    CONFIG_LOAD_ROBOT = "config.load_robot"
    PARAMS_UPDATE = "params.update"
    PING = "ping"
    
    # Pushes (godot → web)
    TELEMETRY_UPDATE = "telemetry.update"
    SIMULATION_STATUS = "simulation.status"
    SIMULATION_ERROR = "simulation.error"
    CONNECTION_STATUS = "connection.status"
    PONG = "pong"


@dataclass
class WsMessage:
    """
    WebSocket message structure
    
    Fields:
        type: Message type from MessageType enum
        id: Unique message ID for request-response matching (UUID)
        timestamp: ISO format timestamp of message creation
        payload: Message-specific data (optional for backward compatibility)
        status: Optional status for response messages (success/error)
    """
    type: str
    payload: Dict[str, Any] = field(default_factory=dict)
    id: Optional[str] = None
    timestamp: Optional[str] = None
    status: Optional[str] = None

    def __post_init__(self) -> None:
        if self.id is None:
            self.id = str(uuid.uuid4())
        if self.timestamp is None:
            self.timestamp = datetime.now().isoformat()
        if self.payload is None:
            self.payload = {}
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization"""
        result = {
            'type': self.type,
            'id': self.id,
            'timestamp': self.timestamp,
            'payload': self.payload
        }
        if self.status:
            result['status'] = self.status
        return result
    
    def to_json(self) -> str:
        """Convert to JSON string"""
        return json.dumps(self.to_dict())
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'WsMessage':
        """Create from dictionary (for parsing received messages)"""
        return cls(
            type=data.get('type', ''),
            id=data.get('id', str(uuid.uuid4())),
            timestamp=data.get('timestamp', datetime.now().isoformat()),
            payload=data.get('payload', {}),
            status=data.get('status')
        )
    
    @classmethod
    def from_json(cls, json_str: str) -> 'WsMessage':
        """Create from JSON string"""
        return cls.from_dict(json.loads(json_str))


class WebSocketProtocolHandler:
    """
    Protocol handler for Web-Godot WebSocket communication
    
    Manages:
    - Message routing to appropriate handlers
    - Request-response ID matching for commands
    - Pending request tracking
    - Telemetry push streaming
    - Error handling
    """
    
    def __init__(self):
        # Pending requests awaiting responses (keyed by message ID)
        self.pending_requests: Dict[str, Dict[str, Any]] = {}
        
        # Registered message handlers
        self.handlers: Dict[str, Callable] = {
            MessageType.SIMULATION_START.value: self._handle_start_simulation,
            MessageType.SIMULATION_STOP.value: self._handle_stop_simulation,
            MessageType.CONFIG_LOAD_ROBOT.value: self._handle_load_robot,
            MessageType.PARAMS_UPDATE.value: self._handle_update_params,
            MessageType.PING.value: self._handle_ping,
        }
        
        # Callback functions for executing actions (to be set by server.py)
        self.on_start_simulation: Optional[Callable] = None
        self.on_stop_simulation: Optional[Callable] = None
        self.on_load_robot: Optional[Callable] = None
        self.on_update_params: Optional[Callable] = None
        self.on_telemetry: Optional[Callable] = None
    
    def route_message(self, message: WsMessage) -> Optional[WsMessage]:
        """
        Route incoming message to appropriate handler
        
        Returns response message if synchronous, None if async/pushed
        """
        handler = self.handlers.get(message.type)
        
        if handler:
            return handler(message)
        else:
            return WsMessage(
                type=message.type,
                id=message.id,
                payload={'error': f'Unknown message type: {message.type}'},
                status='error'
            )
    
    def _handle_start_simulation(self, message: WsMessage) -> WsMessage:
        """Handle simulation.start command"""
        try:
            physics_config = message.payload.get('physics', {})
            
            if self.on_start_simulation:
                result = self.on_start_simulation(physics_config)
                if result is False:
                    raise RuntimeError("Failed to start simulation")
            
            return WsMessage(
                type=message.type,
                id=message.id,
                payload={'status': 'simulation_started'},
                status='success'
            )
        except Exception as e:
            return WsMessage(
                type=message.type,
                id=message.id,
                payload={'error': str(e)},
                status='error'
            )
    
    def _handle_stop_simulation(self, message: WsMessage) -> WsMessage:
        """Handle simulation.stop command"""
        try:
            if self.on_stop_simulation:
                result = self.on_stop_simulation()
                if result is False:
                    raise RuntimeError("Failed to stop simulation")
            
            return WsMessage(
                type=message.type,
                id=message.id,
                payload={'status': 'simulation_stopped'},
                status='success'
            )
        except Exception as e:
            return WsMessage(
                type=message.type,
                id=message.id,
                payload={'error': str(e)},
                status='error'
            )
    
    def _handle_load_robot(self, message: WsMessage) -> WsMessage:
        """Handle config.load_robot command"""
        try:
            robot_config = message.payload.get('robot_config', {})
            
            if self.on_load_robot:
                result = self.on_load_robot(robot_config)
                if result is False:
                    raise RuntimeError("Failed to load robot configuration")
            
            return WsMessage(
                type=message.type,
                id=message.id,
                payload={'status': 'robot_config_loaded'},
                status='success'
            )
        except Exception as e:
            return WsMessage(
                type=message.type,
                id=message.id,
                payload={'error': str(e)},
                status='error'
            )
    
    def _handle_update_params(self, message: WsMessage) -> WsMessage:
        """Handle params.update command"""
        try:
            params = message.payload.get('params', {})
            
            if self.on_update_params:
                result = self.on_update_params(params)
                if result is False:
                    raise RuntimeError("Failed to update parameters")
            
            return WsMessage(
                type=message.type,
                id=message.id,
                payload={'status': 'parameters_updated', 'params': params},
                status='success'
            )
        except Exception as e:
            return WsMessage(
                type=message.type,
                id=message.id,
                payload={'error': str(e)},
                status='error'
            )
    
    def _handle_ping(self, message: WsMessage) -> WsMessage:
        """Handle ping command"""
        return WsMessage(
            type=MessageType.PONG.value,
            id=message.id,
            payload={'timestamp': datetime.now().isoformat()},
            status='success'
        )
    
    # Push methods (godot → web)
    def push_telemetry(self, telemetry_data: Dict[str, Any]) -> WsMessage:
        """
        Push telemetry data from Godot to web
        
        Telemetry typically includes:
        - Sensor readings (IMU, joints, etc.)
        - Current state (position, velocity, etc.)
        - Performance metrics (speed, power, etc.)
        """
        return WsMessage(
            type=MessageType.TELEMETRY_UPDATE.value,
            payload={'data': telemetry_data},
            status='push'
        )
    
    def push_simulation_status(self, status: str, details: Dict[str, Any]) -> WsMessage:
        """
        Push simulation status update
        
        Status values: running, paused, stopped, error
        """
        return WsMessage(
            type=MessageType.SIMULATION_STATUS.value,
            payload={
                'status': status,
                'details': details,
                'timestamp': datetime.now().isoformat()
            },
            status='push'
        )
    
    def push_error(self, error_message: str, error_type: str = 'runtime') -> WsMessage:
        """
        Push error notification
        
        Error types: runtime, connection, physics, config
        """
        return WsMessage(
            type=MessageType.SIMULATION_ERROR.value,
            payload={
                'error': error_message,
                'error_type': error_type,
                'timestamp': datetime.now().isoformat()
            },
            status='push'
        )
    
    def push_connection_status(self, connected: bool, details: Dict[str, Any]) -> WsMessage:
        """
        Push connection status update
        """
        return WsMessage(
            type=MessageType.CONNECTION_STATUS.value,
            payload={
                'connected': connected,
                'details': details,
                'timestamp': datetime.now().isoformat()
            },
            status='push'
        )


# Global protocol handler instance
_protocol_handler: Optional[WebSocketProtocolHandler] = None


def get_protocol_handler() -> WebSocketProtocolHandler:
    """Get or create global protocol handler"""
    global _protocol_handler
    if _protocol_handler is None:
        _protocol_handler = WebSocketProtocolHandler()
    return _protocol_handler
