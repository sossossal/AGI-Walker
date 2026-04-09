import json
import logging
from typing import Any, Dict, List

from fastapi import WebSocket, WebSocketDisconnect

from web_panel.ws_protocol import WsMessage, get_protocol_handler


async def broadcast_all(
    active_connections: Dict[str, List[WebSocket]], message: Dict[str, Any]
):
    for _, connections in active_connections.items():
        disconnected = []
        for connection in connections:
            try:
                await connection.send_json(message)
            except Exception:
                disconnected.append(connection)
        for connection in disconnected:
            if connection in connections:
                connections.remove(connection)


async def broadcast_session(
    active_connections: Dict[str, List[WebSocket]],
    session_id: str,
    message: Dict[str, Any],
):
    disconnected = []
    connections = active_connections.get(session_id, [])
    for connection in connections:
        try:
            await connection.send_json(message)
        except Exception:
            disconnected.append(connection)
    for connection in disconnected:
        if connection in connections:
            connections.remove(connection)


async def handle_websocket(
    websocket: WebSocket,
    session_id: str,
    active_connections: Dict[str, List[WebSocket]],
    godot_controller,
    logger: logging.Logger,
):
    await websocket.accept()
    active_connections.setdefault(session_id, []).append(websocket)

    protocol_handler = get_protocol_handler()

    def start_sim_callback(physics_config):
        if not godot_controller.is_connected():
            raise RuntimeError("Godot is not connected")
        return godot_controller.start_simulation(
            physics_config,
            session_id=session_id,
        )

    def stop_sim_callback():
        if not godot_controller.is_connected():
            raise RuntimeError("Godot is not connected")
        return godot_controller.stop_simulation(session_id=session_id)

    def load_robot_callback(robot_config):
        if not godot_controller.is_connected():
            raise RuntimeError("Godot is not connected")
        parts = robot_config.get("parts", [])
        connections = robot_config.get("connections", [])
        return godot_controller.load_robot(
            parts,
            connections,
            session_id=session_id,
        )

    def update_params_callback(params):
        if not godot_controller.is_connected():
            raise RuntimeError("Godot is not connected")
        return godot_controller.update_params(params, session_id=session_id)

    protocol_handler.on_start_simulation = start_sim_callback
    protocol_handler.on_stop_simulation = stop_sim_callback
    protocol_handler.on_load_robot = load_robot_callback
    protocol_handler.on_update_params = update_params_callback

    try:
        while True:
            data = await websocket.receive_text()
            try:
                received_msg = WsMessage.from_json(data)
            except (json.JSONDecodeError, ValueError):
                error_response = WsMessage(
                    type="error",
                    payload={"message": "Invalid message format"},
                    status="error",
                )
                await websocket.send_json(error_response.to_dict())
                continue

            try:
                response = protocol_handler.route_message(received_msg)
                if response:
                    await websocket.send_json(response.to_dict())
            except Exception as exc:
                error_response = WsMessage(
                    type=received_msg.type,
                    id=received_msg.id,
                    payload={"error": str(exc)},
                    status="error",
                )
                await websocket.send_json(error_response.to_dict())

    except WebSocketDisconnect:
        if websocket in active_connections.get(session_id, []):
            active_connections[session_id].remove(websocket)
        if not active_connections.get(session_id) and hasattr(
            godot_controller, "release_session"
        ):
            godot_controller.release_session(session_id)
        disconnect_msg = protocol_handler.push_connection_status(
            connected=False,
            details={"reason": "client_disconnected"},
        )
        await broadcast_session(
            active_connections, session_id, disconnect_msg.to_dict()
        )
    except Exception as exc:
        logger.info("WebSocket error for session %s: %s", session_id, exc)
        if websocket in active_connections.get(session_id, []):
            active_connections[session_id].remove(websocket)
