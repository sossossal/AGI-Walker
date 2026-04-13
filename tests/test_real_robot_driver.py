from __future__ import annotations

import csv
from pathlib import Path

import pytest

from agi_walker.core.drivers.collect_sysid_data import collect_data
from agi_walker.core.drivers.real_robot_driver import (
    CMD_MOTOR_COMMAND,
    RealRobotDriver,
    decode_motor_command_payload,
    parse_packet,
    validate_real_robot_replay_payload,
)

REPLAY_FIXTURE = Path(__file__).with_name("fixtures") / "real_robot_driver_replay.json"


def test_real_robot_driver_mock_connect_disconnect() -> None:
    driver = RealRobotDriver(mock=True)

    assert driver.connect() is True
    assert driver.running is True

    driver.disconnect()
    assert driver.running is False


def test_real_robot_driver_mock_send_updates_state() -> None:
    driver = RealRobotDriver(mock=True)
    driver.connect()

    assert driver.send_motor_commands({"motor_1": 1.57, "motor_2": -0.5}) is True

    state = driver.get_state()
    assert state["motors"]["motor_1"]["pos"] == pytest.approx(1.57)
    assert state["motors"]["motor_2"]["pos"] == pytest.approx(-0.5)


def test_real_robot_driver_replay_payload_validation_rejects_invalid() -> None:
    errors = validate_real_robot_replay_payload(
        {"schema_version": "0.0", "frames": [{"cmd": "bad"}]}
    )

    assert "schema_version must be '1.0'" in errors
    assert "frames[0].cmd must be 'sensor_state'" in errors
    assert "frames[0].motors must be a non-empty dict" in errors


def test_real_robot_driver_from_replay_updates_state_and_records_command() -> None:
    driver = RealRobotDriver.from_replay(REPLAY_FIXTURE)

    assert driver.connect() is True
    assert driver.poll_once() is True

    state = driver.get_state()
    assert state["motors"]["motor_1"]["pos"] == pytest.approx(1.0)
    assert state["motors"]["motor_1"]["vel"] == pytest.approx(0.1)
    assert state["motors"]["motor_2"]["torque"] == pytest.approx(0.3)

    assert driver.send_motor_commands({"motor_1": 0.25, "motor_2": -0.75}) is True
    command_frame = driver.ser.writes[0]
    cmd, payload = parse_packet(command_frame)

    assert cmd == CMD_MOTOR_COMMAND
    assert decode_motor_command_payload(payload) == {
        "motor_1": pytest.approx(0.25),
        "motor_2": pytest.approx(-0.75),
    }

    driver.disconnect()
    assert driver.ser.closed is True


def test_collect_sysid_data_with_mock_writes_csv(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(
        "agi_walker.core.drivers.collect_sysid_data.time.sleep",
        lambda *_args, **_kwargs: None,
    )
    out_file = tmp_path / "sysid.csv"

    collect_data("COM3", duration=0.05, out_file=str(out_file), mock=True)

    rows = list(csv.DictReader(out_file.read_text(encoding="utf-8").splitlines()))
    assert len(rows) >= 3
    assert {"time", "target_pos", "actual_pos", "actual_vel", "actual_torque"} <= set(
        rows[0]
    )


@pytest.mark.hardware
def test_real_robot_driver_real_serial_connection_opt_in() -> None:
    pytest.skip("需要真实串口硬件")
