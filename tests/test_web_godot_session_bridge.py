import asyncio
import json
import struct
import uuid

import pytest

pytest.importorskip("fastapi")

from fastapi.testclient import TestClient

import web_panel.godot_session_bridge
import web_panel.server
from web_panel.godot_session_bridge import (
    DEFAULT_OPERATOR_HISTORY_PAGE_SIZE,
    GODOT_PROJECT_DIR,
    GODOT_SESSION_STATUS_SCHEMA_VERSION,
    GodotBridge,
)
from web_panel.ws_protocol import MessageType, WsMessage


@pytest.fixture()
def client() -> TestClient:
    return TestClient(web_panel.server.app)


def test_godot_capabilities_exposes_both_modes(client: TestClient) -> None:
    response = client.get("/api/godot/capabilities")

    assert response.status_code == 200
    payload = response.json()
    assert payload["default_session_id"] == "default"
    assert payload["preferred_mode"] == "session_bridge"
    assert "legacy_controller" in payload["modes"]
    assert "session_bridge" in payload["modes"]
    assert payload["modes"]["legacy_controller"]["status"] == "compatibility_only"
    assert payload["modes"]["legacy_controller"]["session_query_param"] == "session_id"
    assert payload["modes"]["legacy_controller"]["push_messages"] == [
        "telemetry.update",
        "simulation.status",
        "simulation.error",
        "connection.status",
    ]
    assert payload["modes"]["session_bridge"]["tcp_commands"] == [
        "reset",
        "step",
        "get_schema",
        "load_robot",
        "instruction_set",
        "configure_simulated_circuit",
    ]
    assert payload["modes"]["session_bridge"]["status"] == "preferred"
    assert payload["modes"]["session_bridge"]["status_schema_version"] == "1.0"
    assert payload["modes"]["session_bridge"]["session_states"] == [
        "disconnected",
        "launching",
        "connected",
        "schema_ready",
        "running",
        "failed",
    ]
    assert (
        payload["modes"]["workflow_bridge"]["preferred_transport_mode"]
        == "session_bridge"
    )
    assert payload["modes"]["workflow_bridge"]["artifact_contract_version"] == "1.0"
    assert "canonical websocket pushes" in payload["note"]


def test_session_bridge_status_defaults_to_disconnected(client: TestClient) -> None:
    session_id = f"status-{uuid.uuid4().hex}"

    response = client.get(f"/api/godot/{session_id}/status")

    assert response.status_code == 200
    payload = response.json()
    assert payload["schema_version"] == GODOT_SESSION_STATUS_SCHEMA_VERSION
    assert payload["session_id"] == session_id
    assert payload["session_state"] == "disconnected"
    assert payload["engine_running"] is False
    assert payload["tcp_connected"] is False
    assert payload["schema_available"] is False
    assert payload["last_sensor"] == {}


def test_godot_bridge_persists_operator_history() -> None:
    session_id = f"history-{uuid.uuid4().hex}"
    bridge = GodotBridge(session_id, 9001)
    asyncio.run(bridge.clear_history())
    asyncio.run(
        bridge._record_command_history(
            "instruction_set",
            {
                "instruction_set": {"sequence_name": "persist-demo"},
                "route_mode": "session_bridge",
                "audit_username": "persist-audit",
                "audit_source": "bearer",
            },
        )
    )
    asyncio.run(
        bridge._record_command_history(
            "simulated_circuit",
            {
                "simulated_circuit": {"transport": "imc22_can_fd"},
                "route_mode": "session_bridge",
            },
        )
    )

    history_payload = asyncio.run(bridge.get_history_payload(limit=1, offset=0))
    reloaded_bridge = GodotBridge(session_id, 9002)
    reloaded_history_payload = asyncio.run(
        reloaded_bridge.get_history_payload(limit=10, offset=0)
    )

    assert history_payload["history_count"] == 2
    assert history_payload["limit"] == 1
    assert history_payload["offset"] == 0
    assert history_payload["has_more"] is True
    assert history_payload["history_storage"] == "database"
    assert reloaded_history_payload["history_count"] == 2
    assert reloaded_history_payload["history"][0]["kind"] == "simulated_circuit"
    assert reloaded_history_payload["history"][1]["kind"] == "instruction_set"
    assert reloaded_history_payload["history"][0]["operator"] is None
    assert reloaded_history_payload["history"][0]["tag"] is None
    assert reloaded_history_payload["history"][0]["note"] is None
    assert reloaded_history_payload["history"][1]["audit_username"] == "persist-audit"
    assert reloaded_history_payload["history"][1]["audit_source"] == "bearer"

    clear_payload = asyncio.run(reloaded_bridge.clear_history())
    assert clear_payload["history_count"] == 0


def test_session_bridge_launch_without_godot_exe_returns_real_error(
    client: TestClient,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    session_id = f"launch-{uuid.uuid4().hex}"
    monkeypatch.setattr(GodotBridge, "_find_godot_exe", lambda self: "")

    response = client.post(f"/api/godot/{session_id}/launch", json={})

    assert response.status_code == 200
    payload = response.json()
    assert payload["status"] == "error"
    assert payload["session_state"] == "failed"
    assert payload["session"]["failure_stage"] == "launch"
    assert "Godot" in payload["message"]


def test_session_bridge_control_without_session_returns_error(
    client: TestClient,
) -> None:
    session_id = f"control-{uuid.uuid4().hex}"

    response = client.post(
        f"/api/godot/{session_id}/control",
        json={"action": [0.1, -0.1]},
    )

    assert response.status_code == 200
    payload = response.json()
    assert payload["status"] == "error"
    assert payload["session_state"] == "disconnected"
    assert payload["session"]["session_state"] == "disconnected"
    assert "Session" in payload["message"]


def test_godot_bridge_launch_uses_scene_flag(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    bridge = GodotBridge("launch-scene-flag", 9010)
    observed: dict[str, object] = {}

    def fake_create_task(coro):
        observed["task_created"] = True
        coro.close()
        return None

    def fake_launch_windows_headless(cmd):
        observed["cmd"] = cmd
        bridge._detached_pid = 4242
        return {
            "status": "launched",
            "pid": 4242,
            "scene": "run_rl_server.tscn",
            "exe": r"C:\Godot\Godot.exe",
        }

    monkeypatch.setattr(bridge, "_find_godot_exe", lambda: r"C:\Godot\Godot.exe")
    monkeypatch.setattr(
        bridge, "_launch_windows_headless", fake_launch_windows_headless
    )
    monkeypatch.setattr("asyncio.create_task", fake_create_task)

    result = bridge.launch(scene="run_rl_server.tscn", headless=True)

    assert result["status"] == "launched"
    assert observed["cmd"] == [
        r"C:\Godot\Godot.exe",
        "--headless",
        "--path",
        GODOT_PROJECT_DIR,
        "--log-file",
        bridge._log_file_path,
        "--scene",
        "run_rl_server.tscn",
        "--",
        "--tcp-port=9010",
    ]
    assert observed["task_created"] is True
    assert bridge.get_pid() == 4242
    assert bridge._log_file_path.endswith("launch-scene-flag_run_rl_server.log")
    assert bridge.get_status_payload()["session_state"] == "launching"


def test_websocket_ping_round_trip(client: TestClient) -> None:
    session_id = f"ws-{uuid.uuid4().hex}"

    with client.websocket_connect(f"/ws/{session_id}") as websocket:
        websocket.send_text(WsMessage(type=MessageType.PING.value).to_json())
        response = websocket.receive_json()

    assert response["type"] == MessageType.PONG.value
    assert response["status"] == "success"
    assert "timestamp" in response["payload"]


def test_websocket_commands_return_error_when_godot_not_connected(
    client: TestClient,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        web_panel.server.godot_controller, "is_connected", lambda: False
    )
    session_id = f"ws-{uuid.uuid4().hex}"

    with client.websocket_connect(f"/ws/{session_id}") as websocket:
        websocket.send_text(
            WsMessage(
                type=MessageType.SIMULATION_START.value,
                payload={"physics": {"gravity": 9.81}},
            ).to_json()
        )
        start_response = websocket.receive_json()

        websocket.send_text(
            WsMessage(
                type=MessageType.CONFIG_LOAD_ROBOT.value,
                payload={"robot_config": {"parts": [], "connections": []}},
            ).to_json()
        )
        load_response = websocket.receive_json()

    assert start_response["status"] == "error"
    assert start_response["payload"]["error"] == "Godot is not connected"
    assert load_response["status"] == "error"
    assert load_response["payload"]["error"] == "Godot is not connected"


def test_websocket_instruction_and_circuit_commands_bind_controller_session(
    client: TestClient,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    session_id = f"ws-instruction-{uuid.uuid4().hex}"
    recorded = {}

    monkeypatch.setattr(web_panel.server.godot_controller, "is_connected", lambda: True)
    monkeypatch.setattr(
        web_panel.server.godot_controller,
        "send_instruction_set",
        lambda payload, session_id=None: (
            recorded.setdefault(
                "instruction", {"payload": payload, "session_id": session_id}
            )
            or True
        ),
    )
    monkeypatch.setattr(
        web_panel.server.godot_controller,
        "configure_simulated_circuit",
        lambda payload, session_id=None: (
            recorded.setdefault(
                "circuit", {"payload": payload, "session_id": session_id}
            )
            or True
        ),
    )

    with client.websocket_connect(f"/ws/{session_id}") as websocket:
        websocket.send_text(
            WsMessage(
                type=MessageType.INSTRUCTION_SET_APPLY.value,
                payload={
                    "instruction_set": {
                        "schema_version": "1.0",
                        "sequence_name": "demo",
                        "steps": [
                            {
                                "kind": "set_velocity",
                                "linear_x": 0.1,
                                "linear_y": 0.0,
                                "angular_z": 0.05,
                            }
                        ],
                    }
                },
            ).to_json()
        )
        instruction_response = websocket.receive_json()

        websocket.send_text(
            WsMessage(
                type=MessageType.SIMULATED_CIRCUIT_CONFIGURE.value,
                payload={
                    "simulated_circuit": {
                        "transport": "imc22_can_fd",
                        "bitrate": 1_000_000,
                    }
                },
            ).to_json()
        )
        circuit_response = websocket.receive_json()

    assert instruction_response["status"] == "success"
    assert instruction_response["payload"]["status"] == "instruction_set_applied"
    assert circuit_response["status"] == "success"
    assert circuit_response["payload"]["status"] == "simulated_circuit_configured"
    assert recorded["instruction"]["session_id"] == session_id
    assert recorded["instruction"]["payload"]["sequence_name"] == "demo"
    assert recorded["circuit"]["session_id"] == session_id
    assert recorded["circuit"]["payload"]["transport"] == "imc22_can_fd"


def test_websocket_legacy_commands_bind_controller_session(
    client: TestClient,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    session_id = f"legacy-bind-{uuid.uuid4().hex}"
    recorded = {}

    monkeypatch.setattr(web_panel.server.godot_controller, "is_connected", lambda: True)
    monkeypatch.setattr(
        web_panel.server.godot_controller,
        "start_simulation",
        lambda physics, session_id=None: (
            recorded.update({"physics": physics, "session_id": session_id}) or True
        ),
    )

    with client.websocket_connect(f"/ws/{session_id}") as websocket:
        websocket.send_text(
            WsMessage(
                type=MessageType.SIMULATION_START.value,
                payload={"physics": {"gravity": 1.62}},
            ).to_json()
        )
        response = websocket.receive_json()

    assert response["status"] == "success"
    assert recorded == {
        "physics": {"gravity": 1.62},
        "session_id": session_id,
    }


def test_session_bridge_instruction_and_circuit_routes(
    client: TestClient, monkeypatch: pytest.MonkeyPatch
) -> None:
    session_id = f"bridge-routes-{uuid.uuid4().hex}"
    observed: dict[str, object] = {}

    class FakeBridge:
        session_state = "running"
        simulated_circuit_config = {"transport": "imc22_can_fd"}
        command_history = [
            {
                "entry_id": "entry-1",
                "schema_version": "1.0",
                "kind": "instruction_set",
                "created_at": "2026-04-26T00:00:00",
                "session_id": session_id,
                "payload": {
                    "instruction_set": {"sequence_name": "route-demo"},
                    "route_mode": "session_bridge",
                },
            }
        ]

        def is_connected(self) -> bool:
            return True

        async def configure_simulated_circuit(
            self,
            simulated_circuit: dict[str, object],
            *,
            operator: str | None = None,
            tag: str | None = None,
            note: str | None = None,
            audit_user_id: int | None = None,
            audit_username: str | None = None,
            audit_source: str | None = None,
        ) -> dict[str, object]:
            observed["simulated_circuit"] = simulated_circuit
            observed["simulated_circuit_metadata"] = {
                "operator": operator,
                "tag": tag,
                "note": note,
                "audit_user_id": audit_user_id,
                "audit_username": audit_username,
                "audit_source": audit_source,
            }
            self.simulated_circuit_config = dict(simulated_circuit)
            return {
                "status": "success",
                "simulated_circuit": simulated_circuit,
            }

        async def apply_instruction_set(
            self,
            instruction_set: dict[str, object],
            *,
            compatibility_params: dict[str, object] | None = None,
            simulated_circuit_command_batch: list[dict[str, object]] | None = None,
            operator: str | None = None,
            tag: str | None = None,
            note: str | None = None,
            audit_user_id: int | None = None,
            audit_username: str | None = None,
            audit_source: str | None = None,
        ) -> dict[str, object]:
            observed["instruction_set"] = instruction_set
            observed["compatibility_params"] = compatibility_params or {}
            observed["command_batch"] = simulated_circuit_command_batch or []
            observed["instruction_metadata"] = {
                "operator": operator,
                "tag": tag,
                "note": note,
                "audit_user_id": audit_user_id,
                "audit_username": audit_username,
                "audit_source": audit_source,
            }
            return {
                "status": "success",
                "instruction_step_count": len(instruction_set.get("steps", [])),
            }

        def get_status_payload(self) -> dict[str, object]:
            return {
                "schema_version": "1.0",
                "session_state": self.session_state,
                "engine_running": True,
                "tcp_connected": True,
                "simulated_circuit_config": self.simulated_circuit_config,
                "history_count": len(self.command_history),
            }

        async def get_history_payload(
            self,
            limit: int = 10,
            offset: int = 0,
            *,
            session_query: str | None = None,
            operator: str | None = None,
            tag: str | None = None,
            note: str | None = None,
            note_exact: bool = False,
            kind: str | None = None,
            route_mode: str | None = None,
            created_after=None,
            created_before=None,
            sort_by: str = "created_at",
            sort_order: str = "desc",
        ) -> dict[str, object]:
            observed["history_filters"] = {
                "session_query": session_query,
                "operator": operator,
                "tag": tag,
                "note": note,
                "note_exact": note_exact,
                "kind": kind,
                "route_mode": route_mode,
                "created_after": created_after.isoformat() if created_after else None,
                "created_before": created_before.isoformat() if created_before else None,
                "sort_by": sort_by,
                "sort_order": sort_order,
            }
            return {
                "session_id": session_id,
                "history": self.command_history[offset : offset + limit],
                "history_count": len(self.command_history),
                "offset": offset,
                "limit": limit,
                "has_more": offset + limit < len(self.command_history),
                "history_storage": "database",
                "filters": observed["history_filters"],
            }

        async def clear_history(self) -> dict[str, object]:
            self.command_history = []
            return {
                "session_id": session_id,
                "history": [],
                "history_count": 0,
                "offset": 0,
                "limit": DEFAULT_OPERATOR_HISTORY_PAGE_SIZE,
                "has_more": False,
                "history_storage": "database",
            }

        async def replay_history_entry(self, entry_id: str | None = None) -> dict[str, object]:
            observed["replayed_entry_id"] = entry_id
            return {
                "status": "success",
                "entry": {
                    "entry_id": entry_id or "entry-1",
                    "kind": "instruction_set",
                },
                "dispatch_result": {"status": "success"},
            }

        def build_hardware_recovery_plan(self) -> dict[str, object]:
            observed["recovery_plan_called"] = True
            return {
                "status": "success",
                "recovery_plan": {"status": "ready", "actions": [{"node_id": 1}]},
                "hardware_fault_summary": {"fault_counts": {"overload": 1}},
            }

        def recover_hardware_faults(self) -> dict[str, object]:
            observed["recover_faults_called"] = True
            return {
                "status": "success",
                "recovery_result": {"status": "applied"},
                "hardware_fault_summary": {"fault_counts": {"overload": 1}},
            }

        def clear_hardware_faults(self) -> dict[str, object]:
            observed["clear_faults_called"] = True
            return {
                "status": "success",
                "clear_result": {"state": "ready"},
                "hardware_fault_summary": {"fault_counts": {}},
            }

    fake_bridge = FakeBridge()
    monkeypatch.setattr(web_panel.server._session_manager, "get_session", lambda _: fake_bridge)
    monkeypatch.setattr(
        web_panel.server._session_manager, "get_or_create", lambda _: fake_bridge
    )
    async def _fake_audit_identity(_authorization):
        return {
            "audit_user_id": 7,
            "audit_username": "audit-user",
            "audit_source": "bearer",
        }
    monkeypatch.setattr(
        web_panel.godot_session_bridge,
        "_resolve_optional_audit_identity",
        _fake_audit_identity,
    )

    instruction_response = client.post(
        f"/api/godot/{session_id}/instruction-set",
        json={
            "instruction_set": {
                "schema_version": "1.0",
                "sequence_name": "route-demo",
                "steps": [{"kind": "set_velocity", "linear_x": 0.1}],
            },
            "compatibility_params": {"velocity_scale": 0.1},
            "simulated_circuit_command_batch": [{"frame_id": 0x200}],
            "operator": "tester-a",
            "tag": "route",
            "note": "route coverage",
        },
        headers={"Authorization": "Bearer test-token"},
    )
    circuit_response = client.post(
        f"/api/godot/{session_id}/simulated-circuit",
        json={
            "simulated_circuit": {
                "transport": "imc22_can_fd",
                "bitrate": 1_000_000,
            },
            "operator": "tester-b",
            "tag": "circuit",
            "note": "circuit coverage",
        },
        headers={"Authorization": "Bearer test-token"},
    )
    history_response = client.get(
        f"/api/godot/{session_id}/history?limit=1&offset=0&session_query=route&operator=tester-a&tag=route&note=coverage&note_exact=true&kind=instruction_set&route_mode=session_bridge&created_after=2026-04-25T00:00:00&created_before=2026-04-27T00:00:00&sort_by=session_id&sort_order=asc"
    )
    replay_response = client.post(
        f"/api/godot/{session_id}/history/replay",
        json={"entry_id": "entry-1"},
    )
    clear_response = client.post(f"/api/godot/{session_id}/history/clear")
    recovery_plan_response = client.get(
        f"/api/godot/{session_id}/hardware/recovery-plan"
    )
    recover_response = client.post(
        f"/api/godot/{session_id}/hardware/recover",
    )
    clear_faults_response = client.post(
        f"/api/godot/{session_id}/hardware/clear-faults",
    )

    assert instruction_response.status_code == 200
    assert instruction_response.json()["status"] == "success"
    assert circuit_response.status_code == 200
    assert circuit_response.json()["status"] == "success"
    assert history_response.status_code == 200
    assert history_response.json()["status"] == "success"
    assert history_response.json()["history_count"] == 1
    assert history_response.json()["limit"] == 1
    assert history_response.json()["offset"] == 0
    assert history_response.json()["has_more"] is False
    assert history_response.json()["history_storage"] == "database"
    assert history_response.json()["filters"]["session_query"] == "route"
    assert history_response.json()["filters"]["operator"] == "tester-a"
    assert history_response.json()["filters"]["tag"] == "route"
    assert history_response.json()["filters"]["note"] == "coverage"
    assert history_response.json()["filters"]["note_exact"] is True
    assert history_response.json()["filters"]["kind"] == "instruction_set"
    assert history_response.json()["filters"]["route_mode"] == "session_bridge"
    assert history_response.json()["filters"]["created_after"] == "2026-04-25T00:00:00"
    assert history_response.json()["filters"]["created_before"] == "2026-04-27T00:00:00"
    assert history_response.json()["filters"]["sort_by"] == "session_id"
    assert history_response.json()["filters"]["sort_order"] == "asc"
    assert observed["history_filters"]["session_query"] == "route"
    assert observed["history_filters"]["operator"] == "tester-a"
    assert observed["history_filters"]["tag"] == "route"
    assert observed["history_filters"]["note"] == "coverage"
    assert observed["history_filters"]["note_exact"] is True
    assert observed["history_filters"]["kind"] == "instruction_set"
    assert observed["history_filters"]["route_mode"] == "session_bridge"
    assert observed["history_filters"]["sort_by"] == "session_id"
    assert observed["history_filters"]["sort_order"] == "asc"
    assert replay_response.status_code == 200
    assert replay_response.json()["status"] == "success"
    assert clear_response.status_code == 200
    assert clear_response.json()["history_count"] == 0
    assert recovery_plan_response.status_code == 200
    assert recovery_plan_response.json()["status"] == "success"
    assert recover_response.status_code == 200
    assert recover_response.json()["status"] == "success"
    assert clear_faults_response.status_code == 200
    assert clear_faults_response.json()["status"] == "success"
    assert observed["instruction_set"]["sequence_name"] == "route-demo"
    assert observed["compatibility_params"]["velocity_scale"] == 0.1
    assert observed["command_batch"][0]["frame_id"] == 0x200
    assert observed["instruction_metadata"]["operator"] == "tester-a"
    assert observed["instruction_metadata"]["tag"] == "route"
    assert observed["instruction_metadata"]["note"] == "route coverage"
    assert observed["instruction_metadata"]["audit_user_id"] == 7
    assert observed["instruction_metadata"]["audit_username"] == "audit-user"
    assert observed["instruction_metadata"]["audit_source"] == "bearer"
    assert observed["simulated_circuit"]["transport"] == "imc22_can_fd"
    assert observed["simulated_circuit_metadata"]["operator"] == "tester-b"
    assert observed["simulated_circuit_metadata"]["tag"] == "circuit"
    assert observed["simulated_circuit_metadata"]["note"] == "circuit coverage"
    assert observed["simulated_circuit_metadata"]["audit_user_id"] == 7
    assert observed["simulated_circuit_metadata"]["audit_username"] == "audit-user"
    assert observed["simulated_circuit_metadata"]["audit_source"] == "bearer"
    assert observed["replayed_entry_id"] == "entry-1"
    assert observed["recovery_plan_called"] is True
    assert observed["recover_faults_called"] is True
    assert observed["clear_faults_called"] is True


def test_session_bridge_history_rejects_invalid_datetime(
    client: TestClient,
) -> None:
    session_id = f"bridge-invalid-datetime-{uuid.uuid4().hex}"

    response = client.get(
        f"/api/godot/{session_id}/history?created_after=not-a-datetime"
    )

    assert response.status_code == 400
    assert "Invalid created_after" in response.json()["detail"]


def test_operator_history_rejects_invalid_sort(
    client: TestClient,
) -> None:
    response = client.get("/api/godot/history?sort_by=bad-field")

    assert response.status_code == 400
    assert "Invalid sort_by" in response.json()["detail"]


def test_operator_history_listing_across_sessions(client: TestClient) -> None:
    session_a = f"aggregate-a-{uuid.uuid4().hex}"
    session_b = f"aggregate-b-{uuid.uuid4().hex}"
    bridge_a = GodotBridge(session_a, 9101)
    bridge_b = GodotBridge(session_b, 9102)

    asyncio.run(bridge_a.clear_history())
    asyncio.run(bridge_b.clear_history())


def test_operator_history_summary_across_sessions(client: TestClient) -> None:
    session_a = f"summary-a-{uuid.uuid4().hex}"
    session_b = f"summary-b-{uuid.uuid4().hex}"
    bridge_a = GodotBridge(session_a, 9201)
    bridge_b = GodotBridge(session_b, 9202)

    asyncio.run(bridge_a.clear_history())
    asyncio.run(bridge_b.clear_history())
    asyncio.run(
        bridge_a._record_command_history(
            "instruction_set",
            {
                "instruction_set": {"sequence_name": "summary-a"},
                "route_mode": "session_bridge",
                "operator": "summary-operator-a",
                "tag": "summary-a",
                "note": "alpha note",
                "audit_username": "audit-a",
            },
        )
    )
    asyncio.run(
        bridge_b._record_command_history(
            "simulated_circuit",
            {
                "simulated_circuit": {"transport": "imc22_can_fd"},
                "route_mode": "legacy",
                "operator": "summary-operator-b",
                "tag": "summary-b",
                "note": "beta note",
                "audit_username": "audit-b",
            },
        )
    )

    response = client.get("/api/godot/history/summary?note=alpha%20note&note_exact=true")

    assert response.status_code == 200
    payload = response.json()
    assert payload["status"] == "success"
    assert payload["history_storage"] == "database"
    assert payload["total_entries"] >= 1
    assert payload["session_count"] >= 1
    assert payload["filters"]["note"] == "alpha note"
    assert payload["filters"]["note_exact"] is True
    assert payload["kind_counts"]["instruction_set"] >= 1
    assert payload["route_mode_counts"]["session_bridge"] >= 1
    assert any(item["session_id"] == session_a for item in payload["sessions"])

    asyncio.run(bridge_a.clear_history())
    asyncio.run(bridge_b.clear_history())
    asyncio.run(
        bridge_a._record_command_history(
            "instruction_set",
            {
                "instruction_set": {"sequence_name": "aggregate-a"},
                "route_mode": "session_bridge",
                "operator": "summary-operator-a",
                "tag": "summary-a",
                "note": "alpha note",
                "audit_username": "audit-a",
            },
        )
    )
    asyncio.run(
        bridge_b._record_command_history(
            "simulated_circuit",
            {
                "simulated_circuit": {"transport": "imc22_can_fd"},
                "route_mode": "legacy",
                "operator": "summary-operator-b",
                "tag": "summary-b",
                "note": "beta note",
                "audit_username": "audit-b",
            },
        )
    )

    response = client.get(
        "/api/godot/history?limit=10&offset=0&operator=summary-operator-b&tag=summary-b&note=beta&route_mode=legacy&sort_by=session_id&sort_order=asc"
    )

    assert response.status_code == 200
    payload = response.json()
    assert payload["status"] == "success"
    assert payload["session_id"] is None
    assert payload["history_count"] >= 1
    assert payload["filters"]["operator"] == "summary-operator-b"
    assert payload["filters"]["tag"] == "summary-b"
    assert payload["filters"]["note"] == "beta"
    assert payload["filters"]["route_mode"] == "legacy"
    assert payload["filters"]["sort_by"] == "session_id"
    assert payload["filters"]["sort_order"] == "asc"
    assert any(item["session_id"] == session_b for item in payload["history"])
    assert all(item["payload"].get("route_mode") == "legacy" for item in payload["history"])
    assert all(item["operator"] == "summary-operator-b" for item in payload["history"])
    assert all(item["tag"] == "summary-b" for item in payload["history"])
    assert all("beta" in (item["note"] or "") for item in payload["history"])
    assert all(item["audit_username"] == "audit-b" for item in payload["history"])

    asyncio.run(bridge_a.clear_history())
    asyncio.run(bridge_b.clear_history())


def test_operator_history_export_json_and_csv(client: TestClient) -> None:
    session_id = f"export-{uuid.uuid4().hex}"
    bridge = GodotBridge(session_id, 9301)

    asyncio.run(bridge.clear_history())
    asyncio.run(
        bridge._record_command_history(
            "instruction_set",
            {
                "instruction_set": {"sequence_name": "export-demo"},
                "route_mode": "session_bridge",
                "operator": "export-user",
                "tag": "acceptance",
                "note": "export note",
            },
        )
    )

    json_response = client.get(
        f"/api/godot/history/export?session_id={session_id}&format=json&sort_by=created_at&sort_order=desc"
    )
    csv_response = client.get(
        f"/api/godot/history/export?session_id={session_id}&format=csv&sort_by=created_at&sort_order=desc"
    )

    assert json_response.status_code == 200
    assert json_response.json()["status"] == "success"
    assert json_response.json()["history_storage"] == "database"
    assert "attachment; filename=\"operator_history_export.json\"" in json_response.headers["content-disposition"]

    assert csv_response.status_code == 200
    assert csv_response.headers["content-type"].startswith("text/csv")
    assert "entry_id,session_id,operator,tag,note,kind,route_mode,created_at,payload_json" in csv_response.text
    assert session_id in csv_response.text
    assert "export-user" in csv_response.text
    assert "acceptance" in csv_response.text
    assert "attachment; filename=\"operator_history_export.csv\"" in csv_response.headers["content-disposition"]

    asyncio.run(bridge.clear_history())


async def _run_session_bridge_get_sensors_polls_with_step(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    bridge = GodotBridge("session-test", 9000)
    observed: dict[str, object] = {}

    async def fake_send_recv(payload):
        observed["payload"] = payload
        return {"vector": [0.1, 0.2]}

    monkeypatch.setattr(bridge, "_send_recv", fake_send_recv)

    response = await bridge.get_sensors()

    assert observed["payload"] == {"type": "step", "action": []}
    assert response == {"vector": [0.1, 0.2]}
    assert bridge.last_sensor == {"vector": [0.1, 0.2]}
    assert bridge.get_status_payload()["session_state"] == "running"


async def _run_session_bridge_schema_state(monkeypatch: pytest.MonkeyPatch) -> None:
    bridge = GodotBridge("schema-test", 9001)

    async def fake_send_recv(payload):
        if payload["type"] == "get_schema":
            return {"sensors": {}, "actuators": {}}
        return {}

    monkeypatch.setattr(bridge, "_send_recv", fake_send_recv)

    schema = await bridge.wait_until_schema(timeout_seconds=0.1)

    assert schema == {"sensors": {}, "actuators": {}}
    status = bridge.get_status_payload()
    assert status["session_state"] == "schema_ready"
    assert status["schema_available"] is True
    assert sorted(status["schema"]) == ["actuators", "sensors"]


def test_session_bridge_get_sensors_polls_with_step(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    asyncio.run(_run_session_bridge_get_sensors_polls_with_step(monkeypatch))


def test_session_bridge_schema_state(monkeypatch: pytest.MonkeyPatch) -> None:
    asyncio.run(_run_session_bridge_schema_state(monkeypatch))


async def _run_session_bridge_instruction_and_circuit_state(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    bridge = GodotBridge(f"instruction-runtime-test-{uuid.uuid4().hex}", 9003)
    observed: list[dict[str, object]] = []

    async def fake_send_recv(payload):
        observed.append(payload)
        if payload["type"] == "configure_simulated_circuit":
            return {
                "status": "success",
                "simulated_circuit": payload["simulated_circuit"],
            }
        return {
            "status": "success",
            "sequence_name": payload["instruction_set"].get("sequence_name", ""),
            "instruction_step_count": len(payload["instruction_set"].get("steps", [])),
        }

    class FakeWriter:
        def is_closing(self) -> bool:
            return False

    bridge.writer = FakeWriter()
    monkeypatch.setattr(bridge, "_send_recv", fake_send_recv)

    circuit_response = await bridge.configure_simulated_circuit(
        {"transport": "imc22_can_fd", "bitrate": 1_000_000}
    )
    instruction_response = await bridge.apply_instruction_set(
        {
            "schema_version": "1.0",
            "sequence_name": "bridge-demo",
            "steps": [{"kind": "set_velocity", "linear_x": 0.1}],
        },
        compatibility_params={"velocity_scale": 0.1},
        simulated_circuit_command_batch=[{"frame_id": 0x200}],
    )

    assert observed[0]["type"] == "configure_simulated_circuit"
    assert observed[1]["type"] == "instruction_set"
    assert circuit_response["simulated_circuit"]["transport"] == "imc22_can_fd"
    assert instruction_response["sequence_name"] == "bridge-demo"
    status = bridge.get_status_payload()
    assert status["simulated_circuit_config"]["transport"] == "imc22_can_fd"
    assert (
        status["last_instruction_runtime"]["instruction_set"]["sequence_name"]
        == "bridge-demo"
    )
    assert "simulated_circuit_feedback" in status["last_instruction_runtime"]
    assert "hardware_fault_summary" in status["last_instruction_runtime"]
    assert status["history_count"] == 2
    assert status["session_state"] == "running"


def test_session_bridge_instruction_and_circuit_state(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    asyncio.run(_run_session_bridge_instruction_and_circuit_state(monkeypatch))


async def _run_session_bridge_send_recv_uses_little_endian() -> None:
    bridge = GodotBridge("framing-test", 9002)
    request = {"type": "get_schema"}
    response = {"sensors": {}, "actuators": {}}
    response_body = json.dumps(response).encode("utf-8")

    class FakeWriter:
        def __init__(self) -> None:
            self.buffer = b""

        def write(self, data: bytes) -> None:
            self.buffer += data

        async def drain(self) -> None:
            return None

        def is_closing(self) -> bool:
            return False

    class FakeReader:
        def __init__(self) -> None:
            self.parts = [
                struct.pack("<I", len(response_body)),
                response_body,
            ]

        async def readexactly(self, size: int) -> bytes:
            chunk = self.parts.pop(0)
            assert len(chunk) == size
            return chunk

    bridge.writer = FakeWriter()
    bridge.reader = FakeReader()

    payload = await bridge._send_recv(request)

    request_body = json.dumps(request).encode("utf-8")
    assert bridge.writer.buffer[:4] == struct.pack("<I", len(request_body))
    assert bridge.writer.buffer[4:] == request_body
    assert payload == response


def test_session_bridge_send_recv_uses_little_endian() -> None:
    asyncio.run(_run_session_bridge_send_recv_uses_little_endian())
