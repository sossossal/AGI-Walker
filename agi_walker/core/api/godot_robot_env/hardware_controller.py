"""
AGI-Walker 硬件控制器接口
用于与 IMC-22 Reflex 控制器通信
"""

import json
import logging
import struct
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

from agi_walker.core.drivers import real_robot_driver as real_driver
from agi_walker.core.drivers.real_robot_driver import RealRobotDriver

logger = logging.getLogger(__name__)

try:
    import can
except ImportError:
    can = None


IMC22_REPLAY_SCHEMA_VERSION = "1.0"
IMC22_TRANSPORT_PROFILE_SCHEMA_VERSION = "1.0"
IMC22_TRANSPORT_DIAGNOSTICS_SCHEMA_VERSION = "1.0"
IMC22_SAFETY_PROFILE_SCHEMA_VERSION = "1.0"
IMC22_FAULT_SUMMARY_SCHEMA_VERSION = "1.0"
IMC22_FAULT_TABLE_SCHEMA_VERSION = "1.0"
IMC22_FAULT_TELEMETRY_REPORT_SCHEMA_VERSION = "1.0"
IMC22_RECOVERY_POLICY_SCHEMA_VERSION = "1.0"
SUPPORTED_IMC22_TRANSPORTS = {
    "socketcan",
    "pcan",
    "replay",
    "serial_bridge",
}

IMC22_FAULT_CLASSES = {
    "ok",
    "watchdog_timeout",
    "sensor_fault",
    "overcurrent",
    "overload",
    "communication_fault",
    "unknown_fault",
}


def _build_transport_check(
    name: str,
    status: str,
    message: str,
    *,
    details: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    return {
        "name": name,
        "status": status,
        "message": message,
        "details": details or {},
    }


def default_imc22_safety_profile() -> Dict[str, Any]:
    return {
        "schema_version": IMC22_SAFETY_PROFILE_SCHEMA_VERSION,
        "max_abs_target_angle": 327.68,
        "min_compliance": 0.0,
        "max_compliance": 1.0,
        "watchdog_timeout_s": 1.0,
        "watchdog_hold_angle": 0.0,
    }


def normalize_imc22_safety_profile(
    profile: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    normalized = default_imc22_safety_profile()
    if profile:
        normalized.update(profile)
    return normalized


def validate_imc22_safety_profile(profile: Dict[str, Any]) -> List[str]:
    errors: List[str] = []
    if not isinstance(profile, dict):
        return ["safety profile must be a dict"]
    if profile.get("schema_version") != IMC22_SAFETY_PROFILE_SCHEMA_VERSION:
        errors.append(
            f"safety_profile.schema_version must be {IMC22_SAFETY_PROFILE_SCHEMA_VERSION!r}"
        )
    for key in ["max_abs_target_angle", "min_compliance", "max_compliance", "watchdog_timeout_s", "watchdog_hold_angle"]:
        if not isinstance(profile.get(key), (int, float)):
            errors.append(f"safety_profile.{key} must be numeric")
    min_compliance = profile.get("min_compliance")
    max_compliance = profile.get("max_compliance")
    if isinstance(min_compliance, (int, float)) and isinstance(max_compliance, (int, float)):
        if float(min_compliance) < 0.0 or float(max_compliance) > 1.0:
            errors.append("safety_profile compliance bounds must stay within [0.0, 1.0]")
        if float(min_compliance) > float(max_compliance):
            errors.append("safety_profile.min_compliance must be <= max_compliance")
    watchdog_timeout_s = profile.get("watchdog_timeout_s")
    if isinstance(watchdog_timeout_s, (int, float)) and float(watchdog_timeout_s) <= 0.0:
        errors.append("safety_profile.watchdog_timeout_s must be positive")
    max_abs_target_angle = profile.get("max_abs_target_angle")
    if isinstance(max_abs_target_angle, (int, float)) and float(max_abs_target_angle) <= 0.0:
        errors.append("safety_profile.max_abs_target_angle must be positive")
    return errors


def load_imc22_fault_table(source: str | Path | Dict[str, Any]) -> Dict[str, Any]:
    if isinstance(source, dict):
        payload = dict(source)
    else:
        payload = json.loads(Path(source).read_text(encoding="utf-8"))
    exact_codes = payload.get("exact_codes")
    if isinstance(exact_codes, dict):
        payload["exact_codes"] = {int(code): value for code, value in exact_codes.items()}
    errors = validate_imc22_fault_table(payload)
    if errors:
        raise ValueError("; ".join(errors))
    return payload


def default_imc22_recovery_policy(vendor: str = "imc22_reflex") -> Dict[str, Any]:
    if vendor != "imc22_reflex":
        raise ValueError(
            f"Unsupported IMC-22 recovery vendor {vendor!r}; expected 'imc22_reflex'"
        )
    return {
        "schema_version": IMC22_RECOVERY_POLICY_SCHEMA_VERSION,
        "vendor": vendor,
        "watchdog_action": {
            "action": "recover_hold_position",
            "target_angle_mode": "watchdog_hold_angle",
            "compliance_mode": "max_compliance",
        },
        "fault_actions": {
            "overload": {
                "action": "recover_hold_position",
                "target_angle_mode": "watchdog_hold_angle",
                "compliance_mode": "fixed",
                "fixed_compliance": 0.5,
            },
            "overcurrent": {
                "action": "recover_relaxed_hold",
                "target_angle_mode": "watchdog_hold_angle",
                "compliance_mode": "min_compliance",
            },
            "sensor_fault": {"action": "clear_only"},
            "communication_fault": {"action": "rediscover_node"},
            "unknown_fault": {"action": "clear_only"},
        },
    }


def normalize_imc22_recovery_policy(
    recovery_policy: Optional[Dict[str, Any]] = None,
    *,
    vendor: str = "imc22_reflex",
) -> Dict[str, Any]:
    normalized = default_imc22_recovery_policy(vendor)
    if recovery_policy:
        normalized.update(recovery_policy)
    return normalized


def validate_imc22_recovery_policy(recovery_policy: Dict[str, Any]) -> List[str]:
    errors: List[str] = []
    if not isinstance(recovery_policy, dict):
        return ["recovery policy must be a dict"]
    if recovery_policy.get("schema_version") != IMC22_RECOVERY_POLICY_SCHEMA_VERSION:
        errors.append(
            "recovery_policy.schema_version must be "
            f"{IMC22_RECOVERY_POLICY_SCHEMA_VERSION!r}"
        )
    vendor = recovery_policy.get("vendor")
    if not isinstance(vendor, str) or not vendor:
        errors.append("recovery_policy.vendor must be a non-empty string")
    watchdog_action = recovery_policy.get("watchdog_action")
    if not isinstance(watchdog_action, dict):
        errors.append("recovery_policy.watchdog_action must be a dict")
    fault_actions = recovery_policy.get("fault_actions")
    if not isinstance(fault_actions, dict):
        errors.append("recovery_policy.fault_actions must be a dict")
    else:
        for fault_class, action_spec in fault_actions.items():
            if fault_class not in IMC22_FAULT_CLASSES:
                errors.append(
                    f"recovery_policy.fault_actions key {fault_class!r} must be a valid fault class"
                )
            if not isinstance(action_spec, dict):
                errors.append(
                    f"recovery_policy.fault_actions[{fault_class!r}] must be a dict"
                )
                continue
            action = action_spec.get("action")
            if action not in {
                "recover_hold_position",
                "recover_relaxed_hold",
                "rediscover_node",
                "clear_only",
            }:
                errors.append(
                    f"recovery_policy.fault_actions[{fault_class!r}].action is invalid"
                )
            compliance_mode = action_spec.get("compliance_mode")
            if compliance_mode is not None and compliance_mode not in {
                "min_compliance",
                "max_compliance",
                "fixed",
            }:
                errors.append(
                    f"recovery_policy.fault_actions[{fault_class!r}].compliance_mode is invalid"
                )
    return errors


def load_imc22_recovery_policy(source: str | Path | Dict[str, Any]) -> Dict[str, Any]:
    if isinstance(source, dict):
        payload = dict(source)
    else:
        payload = json.loads(Path(source).read_text(encoding="utf-8"))
    errors = validate_imc22_recovery_policy(payload)
    if errors:
        raise ValueError("; ".join(errors))
    return payload


def default_imc22_fault_table(vendor: str = "imc22_reflex") -> Dict[str, Any]:
    if vendor != "imc22_reflex":
        raise ValueError(
            f"Unsupported IMC-22 fault vendor {vendor!r}; expected 'imc22_reflex'"
        )
    return {
        "schema_version": IMC22_FAULT_TABLE_SCHEMA_VERSION,
        "vendor": vendor,
        "exact_codes": {
            0: "ok",
            11: "overload",
            12: "overload",
            41: "overcurrent",
            45: "overcurrent",
            71: "sensor_fault",
            75: "sensor_fault",
            91: "communication_fault",
            95: "communication_fault",
        },
        "ranges": [
            {"min": 90.0, "max": None, "fault_class": "communication_fault"},
            {"min": 70.0, "max": 89.999, "fault_class": "sensor_fault"},
            {"min": 40.0, "max": 69.999, "fault_class": "overcurrent"},
            {"min": 10.0, "max": 39.999, "fault_class": "overload"},
        ],
        "fallback_fault_class": "unknown_fault",
    }


def normalize_imc22_fault_table(
    fault_table: Optional[Dict[str, Any]] = None,
    *,
    vendor: str = "imc22_reflex",
) -> Dict[str, Any]:
    normalized = default_imc22_fault_table(vendor)
    if fault_table:
        normalized.update(fault_table)
    return normalized


def validate_imc22_fault_table(fault_table: Dict[str, Any]) -> List[str]:
    errors: List[str] = []
    if not isinstance(fault_table, dict):
        return ["fault table must be a dict"]
    if fault_table.get("schema_version") != IMC22_FAULT_TABLE_SCHEMA_VERSION:
        errors.append(
            "fault_table.schema_version must be "
            f"{IMC22_FAULT_TABLE_SCHEMA_VERSION!r}"
        )
    vendor = fault_table.get("vendor")
    if not isinstance(vendor, str) or not vendor:
        errors.append("fault_table.vendor must be a non-empty string")
    exact_codes = fault_table.get("exact_codes")
    if not isinstance(exact_codes, dict):
        errors.append("fault_table.exact_codes must be a dict")
    else:
        for code, fault_class in exact_codes.items():
            if not isinstance(code, int):
                errors.append("fault_table.exact_codes keys must be ints")
            if fault_class not in IMC22_FAULT_CLASSES:
                errors.append(
                    "fault_table.exact_codes values must be valid fault classes"
                )
    ranges = fault_table.get("ranges")
    if not isinstance(ranges, list):
        errors.append("fault_table.ranges must be a list")
    else:
        for index, entry in enumerate(ranges):
            if not isinstance(entry, dict):
                errors.append(f"fault_table.ranges[{index}] must be a dict")
                continue
            fault_class = entry.get("fault_class")
            if fault_class not in IMC22_FAULT_CLASSES:
                errors.append(
                    f"fault_table.ranges[{index}].fault_class must be a valid fault class"
                )
            min_value = entry.get("min")
            max_value = entry.get("max")
            if min_value is not None and not isinstance(min_value, (int, float)):
                errors.append(f"fault_table.ranges[{index}].min must be numeric or null")
            if max_value is not None and not isinstance(max_value, (int, float)):
                errors.append(f"fault_table.ranges[{index}].max must be numeric or null")
            if isinstance(min_value, (int, float)) and isinstance(max_value, (int, float)):
                if float(min_value) > float(max_value):
                    errors.append(
                        f"fault_table.ranges[{index}].min must be <= max"
                    )
    fallback_fault_class = fault_table.get("fallback_fault_class")
    if fallback_fault_class not in IMC22_FAULT_CLASSES:
        errors.append("fault_table.fallback_fault_class must be a valid fault class")
    return errors


def classify_imc22_fault(
    error_value: float,
    *,
    vendor: str = "imc22_reflex",
    fault_table: Optional[Dict[str, Any]] = None,
) -> str:
    if error_value <= 0.0:
        return "ok"
    normalized = normalize_imc22_fault_table(fault_table, vendor=vendor)
    errors = validate_imc22_fault_table(normalized)
    if errors:
        raise ValueError("; ".join(errors))
    rounded_code = round(float(error_value))
    if abs(float(error_value) - rounded_code) < 1e-6:
        exact_match = normalized["exact_codes"].get(int(rounded_code))
        if exact_match:
            return exact_match
    for entry in normalized["ranges"]:
        min_value = entry.get("min")
        max_value = entry.get("max")
        if min_value is not None and float(error_value) < float(min_value):
            continue
        if max_value is not None and float(error_value) > float(max_value):
            continue
        return entry["fault_class"]
    return normalized["fallback_fault_class"]


class ReplayCANMessage:
    def __init__(self, arbitration_id: int, data: bytes, is_extended_id: bool) -> None:
        self.arbitration_id = arbitration_id
        self.data = data
        self.is_extended_id = is_extended_id


def encode_command_payload(target_angle: float, compliance: float = 0.5) -> bytes:
    angle_int16 = max(-32768, min(32767, int(target_angle * 100)))
    compliance_u8 = max(0, min(255, int(compliance * 255)))
    return struct.pack("<hB", angle_int16, compliance_u8)


def encode_status_payload(angle: float, current: float, error: float) -> bytes:
    angle_raw = max(-32768, min(32767, int(angle * 100)))
    current_raw = max(-32768, min(32767, int(current * 1000)))
    error_raw = max(0, min(65535, int(error * 100)))
    return struct.pack("<hhH", angle_raw, current_raw, error_raw)


def decode_status_payload(data: bytes) -> Dict[str, float]:
    angle_raw, current_raw, error_raw = struct.unpack("<hhH", data[:6])
    return {
        "angle": angle_raw * 0.01,
        "current": current_raw * 0.001,
        "error": error_raw * 0.01,
    }


def validate_imc22_replay_payload(payload: Dict[str, Any]) -> List[str]:
    errors: List[str] = []
    if not isinstance(payload, dict):
        return ["replay payload must be a dict"]
    if payload.get("schema_version") != IMC22_REPLAY_SCHEMA_VERSION:
        errors.append(f"schema_version must be {IMC22_REPLAY_SCHEMA_VERSION!r}")
    frames = payload.get("frames")
    if not isinstance(frames, list) or not frames:
        errors.append("frames must be a non-empty list")
        return errors
    for index, frame in enumerate(frames):
        if not isinstance(frame, dict):
            errors.append(f"frames[{index}] must be a dict")
            continue
        node_id = frame.get("node_id")
        if not isinstance(node_id, int) or node_id <= 0:
            errors.append(f"frames[{index}].node_id must be a positive int")
        for key in ["angle", "current", "error"]:
            value = frame.get(key)
            if not isinstance(value, (int, float)):
                errors.append(f"frames[{index}].{key} must be numeric")
    return errors


def load_imc22_replay_payload(source: str | Path | Dict[str, Any]) -> Dict[str, Any]:
    if isinstance(source, dict):
        payload = source
    else:
        payload = json.loads(Path(source).read_text(encoding="utf-8"))
    errors = validate_imc22_replay_payload(payload)
    if errors:
        raise ValueError("; ".join(errors))
    return payload


def default_imc22_transport_profile(transport: str = "socketcan") -> Dict[str, Any]:
    if transport not in SUPPORTED_IMC22_TRANSPORTS:
        raise ValueError(
            f"Unsupported IMC-22 transport {transport!r}; "
            f"expected one of {sorted(SUPPORTED_IMC22_TRANSPORTS)!r}"
        )

    profile: Dict[str, Any] = {
        "schema_version": IMC22_TRANSPORT_PROFILE_SCHEMA_VERSION,
        "transport": transport,
        "channel": "can0",
        "bustype": "socketcan",
        "bitrate": 1_000_000,
        "fault_vendor": "imc22_reflex",
        "fault_table_source": None,
        "recovery_policy_source": None,
    }
    if transport == "pcan":
        profile.update(
            {
                "channel": "PCAN_USBBUS1",
                "bustype": "pcan",
            }
        )
    elif transport == "replay":
        profile.update(
            {
                "channel": "replay",
                "bustype": "replay",
                "replay_source": None,
            }
        )
    elif transport == "serial_bridge":
        profile.update(
            {
                "channel": "serial-bridge",
                "bustype": "serial_bridge",
                "serial_port": "COM3",
                "baudrate": 115_200,
                "replay_source": None,
            }
        )
    return profile


def normalize_imc22_transport_profile(
    profile: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    requested_transport = "socketcan"
    if profile and isinstance(profile.get("transport"), str) and profile["transport"]:
        requested_transport = profile["transport"]
    normalized = default_imc22_transport_profile(requested_transport)
    if profile:
        normalized.update(profile)
    return normalized


def validate_imc22_transport_profile(profile: Dict[str, Any]) -> List[str]:
    errors: List[str] = []
    if not isinstance(profile, dict):
        return ["transport profile must be a dict"]
    if profile.get("schema_version") != IMC22_TRANSPORT_PROFILE_SCHEMA_VERSION:
        errors.append(
            "transport_profile.schema_version must be "
            f"{IMC22_TRANSPORT_PROFILE_SCHEMA_VERSION!r}"
        )

    transport = profile.get("transport")
    if transport not in SUPPORTED_IMC22_TRANSPORTS:
        errors.append(
            "transport_profile.transport must be one of "
            f"{sorted(SUPPORTED_IMC22_TRANSPORTS)!r}"
        )
        return errors

    if not isinstance(profile.get("channel"), str) or not profile["channel"]:
        errors.append("transport_profile.channel must be a non-empty string")
    if not isinstance(profile.get("fault_vendor"), str) or not profile["fault_vendor"]:
        errors.append("transport_profile.fault_vendor must be a non-empty string")
    if (
        profile.get("fault_table_source") is not None
        and not isinstance(profile.get("fault_table_source"), (str, Path))
    ):
        errors.append("transport_profile.fault_table_source must be a string path when set")
    if (
        profile.get("recovery_policy_source") is not None
        and not isinstance(profile.get("recovery_policy_source"), (str, Path))
    ):
        errors.append(
            "transport_profile.recovery_policy_source must be a string path when set"
        )
    if not isinstance(profile.get("bustype"), str) or not profile["bustype"]:
        errors.append("transport_profile.bustype must be a non-empty string")

    bitrate = profile.get("bitrate")
    if not isinstance(bitrate, int) or bitrate <= 0:
        errors.append("transport_profile.bitrate must be a positive int")

    expected_bustype = {
        "socketcan": "socketcan",
        "pcan": "pcan",
        "replay": "replay",
        "serial_bridge": "serial_bridge",
    }[transport]
    if profile.get("bustype") != expected_bustype:
        errors.append(
            "transport_profile.bustype must match transport "
            f"{transport!r} as {expected_bustype!r}"
        )

    if transport == "replay":
        replay_source = profile.get("replay_source")
        if replay_source is None:
            errors.append("transport_profile.replay_source is required for replay")
    elif transport == "serial_bridge":
        if (
            not isinstance(profile.get("serial_port"), str)
            or not profile["serial_port"]
        ):
            errors.append(
                "transport_profile.serial_port must be a non-empty string "
                "for serial_bridge"
            )
        baudrate = profile.get("baudrate")
        if not isinstance(baudrate, int) or baudrate <= 0:
            errors.append(
                "transport_profile.baudrate must be a positive int for serial_bridge"
            )
    return errors


def command_batch_to_imc22_replay_payload(
    command_batch: List[Dict[str, Any]],
    *,
    current_scale: float = 0.02,
    current_offset: float = 0.1,
    error_value: float = 0.0,
) -> Dict[str, Any]:
    """Project a simulated circuit command batch into canonical IMC-22 replay frames."""
    frames: List[Dict[str, Any]] = []
    for entry in command_batch:
        node_id = int(entry["node_id"])
        target_angle = float(entry["target_angle"])
        frames.append(
            {
                "node_id": node_id,
                "angle": target_angle,
                "current": round(abs(target_angle) * current_scale + current_offset, 6),
                "error": float(error_value),
            }
        )
    return {
        "schema_version": IMC22_REPLAY_SCHEMA_VERSION,
        "frames": frames,
    }


def simulate_imc22_command_batch_feedback(
    command_batch: List[Dict[str, Any]],
) -> Dict[str, Any]:
    """Replay a command batch through the IMC-22 replay path and return structured states."""
    replay_payload = command_batch_to_imc22_replay_payload(command_batch)
    if not replay_payload["frames"]:
        return {
            "schema_version": IMC22_REPLAY_SCHEMA_VERSION,
            "replay_payload": replay_payload,
            "states": {},
            "node_ids": [],
        }

    node_ids = sorted({int(frame["node_id"]) for frame in replay_payload["frames"]})
    controller = IMC22Controller.from_replay(replay_payload)
    try:
        states = controller.get_all_states(len(node_ids), timeout=0.05)
        return {
            "schema_version": IMC22_REPLAY_SCHEMA_VERSION,
            "replay_payload": replay_payload,
            "states": states,
            "node_ids": node_ids,
            "fault_telemetry_report": build_imc22_fault_telemetry_report(controller),
        }
    finally:
        controller.close()


def build_imc22_controller_from_feedback(
    feedback: Dict[str, Any],
    *,
    safety_profile: Optional[Dict[str, Any]] = None,
    fault_vendor: Optional[str] = None,
    fault_table: Optional[Dict[str, Any]] = None,
    recovery_policy: Optional[Dict[str, Any]] = None,
) -> "IMC22Controller":
    """Rehydrate one replay-backed controller from structured simulated feedback."""
    replay_bus = ReplayCANBus(
        [],
        status_id_base=IMC22Controller.ID_STATUS_BASE,
        message_factory=ReplayCANMessage,
    )
    telemetry_report = feedback.get("fault_telemetry_report") or {}
    summary = telemetry_report.get("fault_summary") or {}
    effective_fault_vendor = (
        fault_vendor
        or telemetry_report.get("fault_vendor")
        or summary.get("fault_vendor")
        or "imc22_reflex"
    )
    controller = IMC22Controller(
        channel="recovery-replay",
        bustype="replay",
        bus=replay_bus,
        message_factory=ReplayCANMessage,
        safety_profile=safety_profile,
        fault_vendor=effective_fault_vendor,
        fault_table=fault_table,
        recovery_policy=recovery_policy,
    )
    if telemetry_report.get("fault_table_source"):
        controller.fault_table_source = str(telemetry_report["fault_table_source"])
    states = feedback.get("states") or {}
    normalized_states: Dict[int, Dict[str, float]] = {}
    for raw_node_id, raw_state in states.items():
        if not isinstance(raw_state, dict):
            continue
        try:
            node_id = int(raw_node_id)
        except (TypeError, ValueError):
            continue
        normalized_states[node_id] = {
            "angle": float(raw_state.get("angle", 0.0)),
            "current": float(raw_state.get("current", 0.0)),
            "error": float(raw_state.get("error", 0.0)),
        }
    controller.node_states = normalized_states
    known_node_ids = feedback.get("node_ids") or normalized_states.keys()
    controller._known_node_ids = {
        int(node_id)
        for node_id in known_node_ids
        if isinstance(node_id, (int, float, str)) and str(node_id).strip()
    }
    if bool(summary.get("watchdog_tripped")):
        controller.watchdog_tripped_at = time.time()
        controller.safety_state = "watchdog_tripped"
    return controller


def create_imc22_controller_from_transport_profile(
    profile: Dict[str, Any],
    *,
    bus: Any = None,
    message_factory: Any = None,
) -> "IMC22Controller":
    normalized = normalize_imc22_transport_profile(profile)
    errors = validate_imc22_transport_profile(normalized)
    if errors:
        raise ValueError("; ".join(errors))
    fault_table = None
    if normalized.get("fault_table_source"):
        fault_table = load_imc22_fault_table(normalized["fault_table_source"])
    recovery_policy = None
    if normalized.get("recovery_policy_source"):
        recovery_policy = load_imc22_recovery_policy(
            normalized["recovery_policy_source"]
        )

    transport = normalized["transport"]
    if transport == "replay":
        controller = IMC22Controller.from_replay(
            normalized["replay_source"],
            message_factory=message_factory,
            fault_vendor=normalized["fault_vendor"],
            fault_table=fault_table,
            recovery_policy=recovery_policy,
        )
        controller.fault_table_source = (
            str(normalized["fault_table_source"])
            if normalized.get("fault_table_source")
            else None
        )
        controller.recovery_policy_source = (
            str(normalized["recovery_policy_source"])
            if normalized.get("recovery_policy_source")
            else None
        )
        return controller
    if transport == "serial_bridge":
        controller = IMC22Controller(
            channel=normalized["channel"],
            bustype=normalized["bustype"],
            bitrate=normalized["bitrate"],
            bus=SerialBridgeBus.from_transport_profile(
                normalized,
                status_id_base=IMC22Controller.ID_STATUS_BASE,
                command_id_base=IMC22Controller.ID_COMMAND_BASE,
                config_id_base=IMC22Controller.ID_CONFIG_BASE,
                message_factory=message_factory or ReplayCANMessage,
            ),
            message_factory=message_factory or ReplayCANMessage,
            fault_vendor=normalized["fault_vendor"],
            fault_table=fault_table,
            recovery_policy=recovery_policy,
        )
        controller.fault_table_source = (
            str(normalized["fault_table_source"])
            if normalized.get("fault_table_source")
            else None
        )
        controller.recovery_policy_source = (
            str(normalized["recovery_policy_source"])
            if normalized.get("recovery_policy_source")
            else None
        )
        return controller
    controller = IMC22Controller(
        channel=normalized["channel"],
        bustype=normalized["bustype"],
        bitrate=normalized["bitrate"],
        bus=bus,
        message_factory=message_factory,
        fault_vendor=normalized["fault_vendor"],
        fault_table=fault_table,
        recovery_policy=recovery_policy,
    )
    controller.fault_table_source = (
        str(normalized["fault_table_source"])
        if normalized.get("fault_table_source")
        else None
    )
    controller.recovery_policy_source = (
        str(normalized["recovery_policy_source"])
        if normalized.get("recovery_policy_source")
        else None
    )
    return controller


def build_imc22_fault_telemetry_report(
    controller: "IMC22Controller",
) -> Dict[str, Any]:
    entries = []
    for node_id, status in sorted(controller.node_states.items()):
        raw_error_value = float(status.get("error", 0.0))
        entries.append(
            {
                "node_id": int(node_id),
                "raw_error_value": raw_error_value,
                "fault_class": classify_imc22_fault(
                    raw_error_value,
                    vendor=controller.fault_vendor,
                    fault_table=controller.fault_table,
                ),
                "angle": float(status.get("angle", 0.0)),
                "current": float(status.get("current", 0.0)),
            }
        )
    return {
        "schema_version": IMC22_FAULT_TELEMETRY_REPORT_SCHEMA_VERSION,
        "fault_vendor": controller.fault_vendor,
        "fault_table_schema_version": controller.fault_table["schema_version"],
        "fault_table_source": controller.fault_table_source,
        "entries": entries,
        "fault_summary": controller.get_fault_summary(),
    }


def build_imc22_recovery_plan_summary(plan: Dict[str, Any]) -> Dict[str, Any]:
    """Build one compact machine-readable summary for a recovery plan."""
    actions = plan.get("actions") or []
    action_counts: Dict[str, int] = {}
    affected_node_ids: List[int] = []
    fault_classes: Dict[str, int] = {}
    for action in actions:
        action_name = str(action.get("action", "unknown"))
        action_counts[action_name] = action_counts.get(action_name, 0) + 1
        node_id = action.get("node_id")
        if isinstance(node_id, (int, float)):
            affected_node_ids.append(int(node_id))
        fault_class = str(action.get("fault_class", "unknown_fault"))
        fault_classes[fault_class] = fault_classes.get(fault_class, 0) + 1
    return {
        "status": plan.get("status", "unknown"),
        "action_count": len(actions),
        "action_counts": action_counts,
        "fault_class_counts": fault_classes,
        "affected_node_ids": sorted(set(affected_node_ids)),
    }


def build_imc22_recovery_result_summary(result: Dict[str, Any]) -> Dict[str, Any]:
    """Build one compact summary for one applied recovery result."""
    plan = result.get("recovery_plan") or {}
    safety_status = result.get("safety_status") or {}
    fault_summary = safety_status.get("fault_summary") or {}
    return {
        "status": result.get("status", "unknown"),
        "recovery_plan": build_imc22_recovery_plan_summary(plan),
        "post_recovery_fault_counts": fault_summary.get("fault_counts", {}),
        "post_recovery_watchdog_tripped": bool(
            safety_status.get("watchdog_tripped", False)
        ),
        "post_recovery_state": safety_status.get("state", "unknown"),
    }


def run_imc22_transport_diagnostics(
    profile: Dict[str, Any],
    *,
    attempt_connect: bool = False,
) -> Dict[str, Any]:
    normalized = normalize_imc22_transport_profile(profile)
    checks: List[Dict[str, Any]] = []
    telemetry_report: Optional[Dict[str, Any]] = None
    errors = validate_imc22_transport_profile(normalized)
    if errors:
        checks.append(
            _build_transport_check(
                "profile_validation",
                "blocked",
                "transport profile validation failed",
                details={"errors": errors},
            )
        )
        return {
            "schema_version": IMC22_TRANSPORT_DIAGNOSTICS_SCHEMA_VERSION,
            "status": "blocked",
            "attempt_connect": attempt_connect,
            "transport_profile": normalized,
            "checks": checks,
        }

    checks.append(
        _build_transport_check(
            "profile_validation",
            "passed",
            "transport profile is structurally valid",
        )
    )

    transport = normalized["transport"]
    if transport in {"socketcan", "pcan"}:
        if can is None:
            checks.append(
                _build_transport_check(
                    "python_can_runtime",
                    "blocked",
                    "python-can is not installed",
                )
            )
        else:
            checks.append(
                _build_transport_check(
                    "python_can_runtime",
                    "passed",
                    "python-can runtime is available",
                )
            )
        if attempt_connect and can is not None:
            try:
                controller = IMC22Controller.from_transport_profile(normalized)
                node_ids = controller.discover_nodes(timeout=0.05)
                controller.get_all_states(len(node_ids), timeout=0.05)
                telemetry_report = build_imc22_fault_telemetry_report(controller)
                controller.close()
                checks.append(
                    _build_transport_check(
                        "transport_connect",
                        "passed",
                        "transport connection opened and closed successfully",
                        details={
                            "node_ids": node_ids,
                            "fault_entry_count": len(telemetry_report["entries"]),
                        },
                    )
                )
            except Exception as exc:
                checks.append(
                    _build_transport_check(
                        "transport_connect",
                        "blocked",
                        "transport connection failed",
                        details={"error": str(exc)},
                    )
                )
    elif transport == "replay":
        try:
            payload = load_imc22_replay_payload(normalized["replay_source"])
            checks.append(
                _build_transport_check(
                    "replay_source",
                    "passed",
                    "replay payload loaded successfully",
                    details={"frame_count": len(payload["frames"])},
                )
            )
        except Exception as exc:
            checks.append(
                _build_transport_check(
                    "replay_source",
                    "blocked",
                    "replay payload failed to load",
                    details={"error": str(exc)},
                )
            )
        if attempt_connect:
            try:
                controller = IMC22Controller.from_transport_profile(normalized)
                node_ids = controller.discover_nodes(timeout=0.05)
                controller.get_all_states(len(node_ids), timeout=0.05)
                telemetry_report = build_imc22_fault_telemetry_report(controller)
                controller.close()
                checks.append(
                    _build_transport_check(
                        "transport_connect",
                        "passed",
                        "replay controller opened successfully",
                        details={
                            "node_ids": node_ids,
                            "fault_entry_count": len(telemetry_report["entries"]),
                        },
                    )
                )
            except Exception as exc:
                checks.append(
                    _build_transport_check(
                        "transport_connect",
                        "blocked",
                        "replay controller failed to initialize",
                        details={"error": str(exc)},
                    )
                )
    elif transport == "serial_bridge":
        if normalized.get("replay_source") is not None:
            try:
                payload = real_driver.load_real_robot_replay_payload(
                    normalized["replay_source"]
                )
                checks.append(
                    _build_transport_check(
                        "serial_replay_source",
                        "passed",
                        "serial replay payload loaded successfully",
                        details={"frame_count": len(payload["frames"])},
                    )
                )
            except Exception as exc:
                checks.append(
                    _build_transport_check(
                        "serial_replay_source",
                        "blocked",
                        "serial replay payload failed to load",
                        details={"error": str(exc)},
                    )
                )
        elif real_driver.serial is None:
            checks.append(
                _build_transport_check(
                    "pyserial_runtime",
                    "blocked",
                    "pyserial is not installed",
                )
            )
        else:
            checks.append(
                _build_transport_check(
                    "pyserial_runtime",
                    "passed",
                    "pyserial runtime is available",
                )
            )

        if attempt_connect:
            try:
                controller = IMC22Controller.from_transport_profile(normalized)
                node_ids = controller.discover_nodes(timeout=0.05)
                controller.get_all_states(len(node_ids), timeout=0.05)
                telemetry_report = build_imc22_fault_telemetry_report(controller)
                controller.close()
                checks.append(
                    _build_transport_check(
                        "transport_connect",
                        "passed",
                        "serial bridge transport opened successfully",
                        details={
                            "node_ids": node_ids,
                            "fault_entry_count": len(telemetry_report["entries"]),
                        },
                    )
                )
            except Exception as exc:
                checks.append(
                    _build_transport_check(
                        "transport_connect",
                        "blocked",
                        "serial bridge transport failed to initialize",
                        details={"error": str(exc)},
                    )
                )

    status = (
        "blocked"
        if any(check["status"] == "blocked" for check in checks)
        else "ready"
    )
    report = {
        "schema_version": IMC22_TRANSPORT_DIAGNOSTICS_SCHEMA_VERSION,
        "status": status,
        "attempt_connect": attempt_connect,
        "transport_profile": normalized,
        "checks": checks,
    }
    if telemetry_report is not None:
        report["fault_telemetry_report"] = telemetry_report
    return report


class ReplayCANBus:
    is_replay = True

    def __init__(
        self,
        frames: List[Dict[str, Any]],
        *,
        status_id_base: int,
        message_factory=ReplayCANMessage,
    ) -> None:
        self._frames = list(frames)
        self._cursor = 0
        self._status_id_base = status_id_base
        self._message_factory = message_factory
        self.sent_messages: List[Any] = []
        self.closed = False

    @classmethod
    def from_payload(
        cls,
        payload: Dict[str, Any],
        *,
        status_id_base: int,
        message_factory=ReplayCANMessage,
    ) -> "ReplayCANBus":
        return cls(
            payload["frames"],
            status_id_base=status_id_base,
            message_factory=message_factory,
        )

    def has_pending_frames(self) -> bool:
        return self._cursor < len(self._frames)

    def send(self, message: Any) -> None:
        self.sent_messages.append(message)

    def recv(self, timeout: float = 0.1) -> Optional[Any]:
        if not self.has_pending_frames():
            return None
        frame = self._frames[self._cursor]
        self._cursor += 1
        return self._message_factory(
            arbitration_id=self._status_id_base + frame["node_id"],
            data=encode_status_payload(
                angle=frame["angle"],
                current=frame["current"],
                error=frame["error"],
            ),
            is_extended_id=False,
        )

    def shutdown(self) -> None:
        self.closed = True


class SerialBridgeBus:
    is_serial_bridge = True

    def __init__(
        self,
        driver: RealRobotDriver,
        *,
        status_id_base: int,
        command_id_base: int,
        config_id_base: int,
        message_factory=ReplayCANMessage,
    ) -> None:
        self.driver = driver
        self._status_id_base = status_id_base
        self._command_id_base = command_id_base
        self._config_id_base = config_id_base
        self._message_factory = message_factory or ReplayCANMessage
        self._pending_node_ids: List[int] = []
        self.sent_messages: List[Any] = []
        self.config_messages: List[Dict[str, Any]] = []
        self.closed = False

    @classmethod
    def from_transport_profile(
        cls,
        profile: Dict[str, Any],
        *,
        status_id_base: int,
        command_id_base: int,
        config_id_base: int,
        message_factory=ReplayCANMessage,
    ) -> "SerialBridgeBus":
        replay_source = profile.get("replay_source")
        if replay_source is not None:
            driver = RealRobotDriver.from_replay(replay_source)
        else:
            driver = RealRobotDriver(
                port=profile["serial_port"],
                baudrate=profile["baudrate"],
            )
        if not driver.connect():
            raise RuntimeError("failed to connect serial bridge driver")
        return cls(
            driver,
            status_id_base=status_id_base,
            command_id_base=command_id_base,
            config_id_base=config_id_base,
            message_factory=message_factory,
        )

    def _refresh_pending_node_ids(self) -> None:
        motors = self.driver.get_state().get("motors", {})
        self._pending_node_ids = sorted(
            int(name.split("_")[-1])
            for name in motors
            if "_" in name and name.split("_")[-1].isdigit()
        )

    def _build_status_message(self, node_id: int) -> Optional[Any]:
        motor_state = self.driver.get_state().get("motors", {}).get(f"motor_{node_id}")
        if not motor_state:
            return None
        return self._message_factory(
            arbitration_id=self._status_id_base + node_id,
            data=encode_status_payload(
                angle=float(motor_state.get("pos", 0.0)),
                current=float(motor_state.get("torque", 0.0)),
                error=0.0,
            ),
            is_extended_id=False,
        )

    def send(self, message: Any) -> None:
        self.sent_messages.append(message)
        arbitration_id = getattr(message, "arbitration_id", 0)
        if (
            arbitration_id >= self._command_id_base
            and arbitration_id < self._config_id_base
        ):
            node_id = arbitration_id - self._command_id_base
            target_raw, compliance_raw = struct.unpack("<hB", message.data[:3])
            target_angle = target_raw * 0.01
            _compliance = compliance_raw / 255.0
            self.driver.send_motor_commands({f"motor_{node_id}": target_angle})
            self._pending_node_ids = [node_id]
            return
        if arbitration_id >= self._config_id_base:
            node_id = arbitration_id - self._config_id_base
            max_torque, kp, ki = struct.unpack("<fff", message.data[:12])
            config_message = {
                "node_id": node_id,
                "max_torque": max_torque,
                "kp": kp,
                "ki": ki,
            }
            self.config_messages.append(config_message)
            self.driver.send_motor_config(
                {
                    f"motor_{node_id}": {
                        "max_torque": max_torque,
                        "kp": kp,
                        "ki": ki,
                    }
                }
            )

    def recv(self, timeout: float = 0.1) -> Optional[Any]:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if getattr(self.driver.ser, "is_replay", False):
                self.driver.poll_once()
            if not self._pending_node_ids:
                self._refresh_pending_node_ids()
            if self._pending_node_ids:
                node_id = self._pending_node_ids.pop(0)
                message = self._build_status_message(node_id)
                if message is not None:
                    return message
            if (
                getattr(self.driver.ser, "is_replay", False)
                and not self.driver.ser.has_pending_packets()
            ):
                break
            time.sleep(0.001)
        return None

    def shutdown(self) -> None:
        self.driver.disconnect()
        self.closed = True


class IMC22Controller:
    """IMC-22 硬件控制器接口"""

    # CAN ID 定义
    ID_SYNC = 0x000
    ID_STATUS_BASE = 0x100
    ID_COMMAND_BASE = 0x200
    ID_CONFIG_BASE = 0x300
    ID_HANDSHAKE = 0x7FF

    def __init__(
        self,
        channel="can0",
        bustype="socketcan",
        bitrate=1000000,
        *,
        bus=None,
        message_factory=None,
        safety_profile: Optional[Dict[str, Any]] = None,
        fault_vendor: str = "imc22_reflex",
        fault_table: Optional[Dict[str, Any]] = None,
        recovery_policy: Optional[Dict[str, Any]] = None,
    ):
        """
        初始化硬件控制器

        Args:
            channel: CAN 通道 (Linux: 'can0', Windows: 'PCAN_USBBUS1')
            bustype: 总线类型 (Linux: 'socketcan', Windows: 'pcan')
            bitrate: 波特率 (默认 1 Mbps)
        """
        self.message_factory = message_factory or getattr(
            can, "Message", ReplayCANMessage
        )
        if bus is not None:
            self.bus = bus
        else:
            if can is None:
                raise ImportError(
                    "python-can library is required for hardware_controller. "
                    "Install it with: pip install python-can"
                )
            try:
                self.bus = can.interface.Bus(
                    channel=channel, bustype=bustype, bitrate=bitrate
                )
                logger.info(f"CAN 总线已连接: {channel} @ {bitrate} bps")
            except Exception as e:
                logger.error(f"无法连接 CAN 总线: {e}")
                raise

        self.node_states = {}  # 存储各节点状态
        self.safety_profile = normalize_imc22_safety_profile(safety_profile)
        errors = validate_imc22_safety_profile(self.safety_profile)
        if errors:
            raise ValueError("; ".join(errors))
        self.fault_vendor = fault_vendor
        self.fault_table = normalize_imc22_fault_table(
            fault_table,
            vendor=fault_vendor,
        )
        self.fault_table_source: Optional[str] = None
        fault_table_errors = validate_imc22_fault_table(self.fault_table)
        if fault_table_errors:
            raise ValueError("; ".join(fault_table_errors))
        self.recovery_policy = normalize_imc22_recovery_policy(
            recovery_policy,
            vendor=fault_vendor,
        )
        self.recovery_policy_source: Optional[str] = None
        recovery_policy_errors = validate_imc22_recovery_policy(self.recovery_policy)
        if recovery_policy_errors:
            raise ValueError("; ".join(recovery_policy_errors))
        self.last_command_at: Optional[float] = None
        self.last_command_details: Optional[Dict[str, Any]] = None
        self.watchdog_tripped_at: Optional[float] = None
        self.safety_state = "ready"
        self._known_node_ids: set[int] = set()

    @classmethod
    def from_replay(
        cls,
        replay_source: str | Path | Dict[str, Any],
        *,
        message_factory=ReplayCANMessage,
        fault_vendor: str = "imc22_reflex",
        fault_table: Optional[Dict[str, Any]] = None,
        recovery_policy: Optional[Dict[str, Any]] = None,
    ) -> "IMC22Controller":
        message_factory = message_factory or ReplayCANMessage
        payload = load_imc22_replay_payload(replay_source)
        return cls(
            channel="replay",
            bustype="replay",
            bus=ReplayCANBus.from_payload(
                payload,
                status_id_base=cls.ID_STATUS_BASE,
                message_factory=message_factory,
            ),
            message_factory=message_factory,
            fault_vendor=fault_vendor,
            fault_table=fault_table,
            recovery_policy=recovery_policy,
        )

    @classmethod
    def from_transport_profile(
        cls,
        profile: Dict[str, Any],
        *,
        bus: Any = None,
        message_factory: Any = None,
    ) -> "IMC22Controller":
        return create_imc22_controller_from_transport_profile(
            profile,
            bus=bus,
            message_factory=message_factory,
        )

    def send_command(self, node_id: int, target_angle: float, compliance: float = 0.5):
        """
        发送控制命令到指定节点

        Args:
            node_id: 节点 ID (1-255)
            target_angle: 目标角度 (度)
            compliance: 柔顺系数 (0.0 = 刚性, 1.0 = 柔性)
        """
        bounded_angle = max(
            -float(self.safety_profile["max_abs_target_angle"]),
            min(float(self.safety_profile["max_abs_target_angle"]), float(target_angle)),
        )
        bounded_compliance = max(
            float(self.safety_profile["min_compliance"]),
            min(float(self.safety_profile["max_compliance"]), float(compliance)),
        )
        msg = self.message_factory(
            arbitration_id=self.ID_COMMAND_BASE + node_id,
            data=encode_command_payload(bounded_angle, bounded_compliance),
            is_extended_id=False,
        )

        try:
            self.bus.send(msg)
            self.last_command_at = time.time()
            self.last_command_details = {
                "node_id": node_id,
                "requested_target_angle": float(target_angle),
                "bounded_target_angle": bounded_angle,
                "requested_compliance": float(compliance),
                "bounded_compliance": bounded_compliance,
                "clamped": (
                    bounded_angle != float(target_angle)
                    or bounded_compliance != float(compliance)
                ),
            }
            self._known_node_ids.add(int(node_id))
            if self.watchdog_tripped_at is not None:
                self.safety_state = "recovered_pending_clear"
            else:
                self.safety_state = "active"
        except Exception as e:
            logger.error(f"发送命令失败 (节点 {node_id}): {e}")

    def read_status(self, timeout: float = 0.1) -> Optional[Dict]:
        """
        读取节点状态

        Args:
            timeout: 超时时间 (秒)

        Returns:
            状态字典 {'node_id': int, 'angle': float, 'current': float, 'error': float}
            或 None (如果超时)
        """
        msg = self.bus.recv(timeout=timeout)

        if not msg:
            return None

        # 检查是否为状态消息
        if (
            msg.arbitration_id >= self.ID_STATUS_BASE
            and msg.arbitration_id < self.ID_COMMAND_BASE
        ):
            node_id = msg.arbitration_id - self.ID_STATUS_BASE

            status = {"node_id": node_id, **decode_status_payload(msg.data)}

            # 缓存状态
            self.node_states[node_id] = status
            self._known_node_ids.add(int(node_id))

            return status

        return None

    def get_all_states(self, num_nodes: int, timeout: float = 0.5) -> Dict[int, Dict]:
        """
        获取所有节点的状态

        Args:
            num_nodes: 节点数量
            timeout: 总超时时间

        Returns:
            {node_id: {'angle': float, 'current': float, 'error': float}}
        """
        start_time = time.time()
        states = {}

        while len(states) < num_nodes and (time.time() - start_time) < timeout:
            status = self.read_status(timeout=0.01)
            if status:
                states[status["node_id"]] = status
                continue
            if (
                getattr(self.bus, "is_replay", False)
                and not self.bus.has_pending_frames()
            ):
                break

        return states

    def set_config(self, node_id: int, max_torque: float, kp: float, ki: float):
        """
        配置节点参数

        Args:
            node_id: 节点 ID
            max_torque: 最大力矩 (N·m)
            kp: PID 比例系数
            ki: PID 积分系数
        """
        data = struct.pack("<fff", max_torque, kp, ki)

        msg = self.message_factory(
            arbitration_id=self.ID_CONFIG_BASE + node_id,
            data=data,
            is_extended_id=False,
        )

        self.bus.send(msg)
        logger.info(f"节点 {node_id} 配置已更新")
        self._known_node_ids.add(int(node_id))

    def get_safety_status(self) -> Dict[str, Any]:
        return {
            "schema_version": IMC22_SAFETY_PROFILE_SCHEMA_VERSION,
            "state": self.safety_state,
            "watchdog_tripped": self.watchdog_tripped_at is not None,
            "watchdog_tripped_at": self.watchdog_tripped_at,
            "last_command_at": self.last_command_at,
            "last_command_details": self.last_command_details,
            "known_node_ids": sorted(self._known_node_ids),
            "safety_profile": dict(self.safety_profile),
            "fault_vendor": self.fault_vendor,
            "recovery_policy_schema_version": self.recovery_policy["schema_version"],
            "fault_summary": self.get_fault_summary(),
        }

    def _resolve_recovery_action(
        self,
        fault_class: str,
        *,
        node_id: int,
        action_spec: Dict[str, Any],
    ) -> Dict[str, Any]:
        action = {
            "node_id": node_id,
            "fault_class": fault_class,
            "action": action_spec["action"],
        }
        if action_spec["action"] not in {
            "recover_hold_position",
            "recover_relaxed_hold",
        }:
            return action
        target_angle_mode = action_spec.get(
            "target_angle_mode",
            "watchdog_hold_angle",
        )
        if target_angle_mode == "watchdog_hold_angle":
            action["target_angle"] = float(self.safety_profile["watchdog_hold_angle"])
        compliance_mode = action_spec.get("compliance_mode", "max_compliance")
        if compliance_mode == "min_compliance":
            action["compliance"] = float(self.safety_profile["min_compliance"])
        elif compliance_mode == "fixed":
            action["compliance"] = float(action_spec.get("fixed_compliance", 0.5))
        else:
            action["compliance"] = float(self.safety_profile["max_compliance"])
        return action

    def get_fault_summary(self) -> Dict[str, Any]:
        per_node: Dict[int, Dict[str, Any]] = {}
        fault_counts: Dict[str, int] = {}
        for node_id, status in sorted(self.node_states.items()):
            fault_class = classify_imc22_fault(
                float(status.get("error", 0.0)),
                vendor=self.fault_vendor,
                fault_table=self.fault_table,
            )
            per_node[node_id] = {
                "fault_class": fault_class,
                "error": float(status.get("error", 0.0)),
                "angle": float(status.get("angle", 0.0)),
                "current": float(status.get("current", 0.0)),
            }
            fault_counts[fault_class] = fault_counts.get(fault_class, 0) + 1
        if self.watchdog_tripped_at is not None:
            fault_counts["watchdog_timeout"] = fault_counts.get("watchdog_timeout", 0) + len(
                self._known_node_ids
            )
        return {
            "schema_version": IMC22_FAULT_SUMMARY_SCHEMA_VERSION,
            "fault_table_schema_version": self.fault_table["schema_version"],
            "fault_vendor": self.fault_vendor,
            "fault_counts": fault_counts,
            "per_node": per_node,
            "watchdog_tripped": self.watchdog_tripped_at is not None,
        }

    def build_recovery_plan(self) -> Dict[str, Any]:
        summary = self.get_fault_summary()
        actions: List[Dict[str, Any]] = []
        seen_node_ids = set()
        if summary["watchdog_tripped"]:
            watchdog_action = self.recovery_policy["watchdog_action"]
            for node_id in sorted(self._known_node_ids):
                actions.append(
                    self._resolve_recovery_action(
                        "watchdog_timeout",
                        node_id=node_id,
                        action_spec=watchdog_action,
                    )
                )
                seen_node_ids.add(node_id)
        for node_id, node_summary in summary["per_node"].items():
            fault_class = node_summary["fault_class"]
            if fault_class == "ok" or node_id in seen_node_ids:
                continue
            action_spec = self.recovery_policy["fault_actions"].get(
                fault_class,
                self.recovery_policy["fault_actions"].get(
                    "unknown_fault",
                    {"action": "clear_only"},
                ),
            )
            actions.append(
                self._resolve_recovery_action(
                    fault_class,
                    node_id=node_id,
                    action_spec=action_spec,
                )
            )
        return {
            "schema_version": IMC22_FAULT_SUMMARY_SCHEMA_VERSION,
            "recovery_policy_schema_version": self.recovery_policy["schema_version"],
            "recovery_policy_source": self.recovery_policy_source,
            "status": "ready" if actions else "noop",
            "actions": actions,
            "fault_summary": summary,
        }

    def clear_faults(self) -> Dict[str, Any]:
        self.watchdog_tripped_at = None
        self.safety_state = "ready"
        return self.get_safety_status()

    def recover(
        self,
        *,
        recovery_angle: Optional[float] = None,
        compliance: Optional[float] = None,
    ) -> Dict[str, Any]:
        target_angle = (
            float(recovery_angle)
            if recovery_angle is not None
            else float(self.safety_profile["watchdog_hold_angle"])
        )
        target_compliance = (
            float(compliance)
            if compliance is not None
            else float(self.safety_profile["max_compliance"])
        )
        for node_id in sorted(self._known_node_ids):
            self.send_command(
                node_id=node_id,
                target_angle=target_angle,
                compliance=target_compliance,
            )
        self.watchdog_tripped_at = None
        self.safety_state = "ready"
        return self.get_safety_status()

    def recover_by_fault_class(self) -> Dict[str, Any]:
        plan = self.build_recovery_plan()
        for action in plan["actions"]:
            node_id = action["node_id"]
            if action["action"] in {"recover_hold_position", "recover_relaxed_hold"}:
                self.send_command(
                    node_id=node_id,
                    target_angle=float(action["target_angle"]),
                    compliance=float(action["compliance"]),
                )
            elif action["action"] == "rediscover_node":
                self.discover_nodes(timeout=0.05)
        self.watchdog_tripped_at = None
        self.safety_state = "ready" if plan["actions"] else self.safety_state
        return {
            "status": "applied" if plan["actions"] else "noop",
            "recovery_plan": plan,
            "safety_status": self.get_safety_status(),
        }

    def poll_watchdog(self, *, now: Optional[float] = None) -> Dict[str, Any]:
        effective_now = now if now is not None else time.time()
        timeout_s = float(self.safety_profile["watchdog_timeout_s"])
        if self.last_command_at is None:
            return self.get_safety_status()
        if (effective_now - self.last_command_at) <= timeout_s:
            return self.get_safety_status()

        hold_angle = float(self.safety_profile["watchdog_hold_angle"])
        target_node_ids = sorted(self._known_node_ids)
        for node_id in target_node_ids:
            msg = self.message_factory(
                arbitration_id=self.ID_COMMAND_BASE + node_id,
                data=encode_command_payload(
                    hold_angle,
                    float(self.safety_profile["max_compliance"]),
                ),
                is_extended_id=False,
            )
            self.bus.send(msg)

        self.watchdog_tripped_at = effective_now
        self.safety_state = "watchdog_tripped"
        return self.get_safety_status()

    def discover_nodes(
        self, timeout: float = 2.0, expected_count: Optional[int] = None
    ) -> List[int]:
        """
        发现总线上的所有节点

        Args:
            timeout: 扫描超时时间

        Returns:
            节点 ID 列表
        """
        logger.info("扫描 CAN 总线上的节点...")

        discovered = set()
        start_time = time.time()

        while (time.time() - start_time) < timeout:
            status = self.read_status(timeout=0.1)
            if status:
                discovered.add(status["node_id"])
                if expected_count and len(discovered) >= expected_count:
                    break
                continue
            if (
                getattr(self.bus, "is_replay", False)
                and not self.bus.has_pending_frames()
            ):
                break

        nodes = sorted(list(discovered))
        logger.info(f"发现 {len(nodes)} 个节点: {nodes}")

        return nodes

    def close(self):
        """关闭 CAN 总线"""
        if self.bus:
            self.bus.shutdown()
            logger.info("CAN 总线已关闭")


class HardwareEnvironment:
    """
    硬件环境包装器，兼容 Gymnasium 接口
    用于在真实硬件上测试策略
    """

    def __init__(
        self,
        num_joints: int = 12,
        control_freq_hz: int = 100,
        *,
        controller: Optional[IMC22Controller] = None,
    ):
        """
        Args:
            num_joints: 关节数量
            control_freq_hz: 控制频率 (Hz)
        """
        self.controller = controller or IMC22Controller()
        self.num_joints = num_joints
        self.control_period = 1.0 / control_freq_hz

        # 发现节点
        self.node_ids = self.controller.discover_nodes(expected_count=num_joints)
        if len(self.node_ids) != num_joints:
            logger.warning(
                f"期望 {num_joints} 个节点，实际发现 {len(self.node_ids)} 个"
            )

    @classmethod
    def from_transport_profile(
        cls,
        profile: Dict[str, Any],
        *,
        num_joints: int = 12,
        control_freq_hz: int = 100,
        bus: Any = None,
        message_factory: Any = None,
    ) -> "HardwareEnvironment":
        return cls(
            num_joints=num_joints,
            control_freq_hz=control_freq_hz,
            controller=IMC22Controller.from_transport_profile(
                profile,
                bus=bus,
                message_factory=message_factory,
            ),
        )

    def reset(self):
        """重置到初始状态"""
        # 所有关节归零
        for node_id in self.node_ids:
            self.controller.send_command(node_id, target_angle=0.0, compliance=0.5)

        time.sleep(1.0)  # 等待稳定

        # 读取初始状态
        states = self.controller.get_all_states(self.num_joints)
        return self._build_observation(states)

    def step(self, action):
        """
        执行一步控制

        Args:
            action: 动作数组 (每个关节的目标角度)
        """
        # 发送命令
        for i, node_id in enumerate(self.node_ids):
            if i < len(action):
                self.controller.send_command(node_id, action[i], compliance=0.5)

        # 等待控制周期
        time.sleep(self.control_period)

        # 读取新状态
        states = self.controller.get_all_states(self.num_joints)
        obs = self._build_observation(states)

        # 简化的奖励（实际需要根据任务计算）
        reward = 0.0
        terminated = False
        truncated = False
        info = {"states": states}

        return obs, reward, terminated, truncated, info

    def _build_observation(self, states: Dict) -> List[float]:
        """从节点状态构建观察"""
        obs = []
        for node_id in sorted(self.node_ids):
            if node_id in states:
                obs.extend(
                    [
                        states[node_id]["angle"],
                        states[node_id]["current"],
                        states[node_id]["error"],
                    ]
                )
            else:
                obs.extend([0.0, 0.0, 0.0])
        return obs

    def close(self):
        """关闭环境"""
        self.controller.close()


if __name__ == "__main__":
    # 简单测试
    try:
        controller = IMC22Controller()

        # 发现节点
        nodes = controller.discover_nodes()

        # 发送测试命令
        if nodes:
            controller.send_command(nodes[0], target_angle=45.0, compliance=0.5)
            time.sleep(0.1)

            # 读取状态
            status = controller.read_status()
            if status:
                print(f"节点状态: {status}")

        controller.close()

    except Exception as e:
        print(f"测试失败: {e}")
        print("提示: 确保 CAN 适配器已连接并配置正确")
