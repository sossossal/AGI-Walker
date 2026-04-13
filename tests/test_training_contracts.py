import json

import pytest

from agi_walker.core.api.training_contracts import (
    TRAINING_CONTRACT_VERSION,
    build_training_run_artifact,
    validate_training_run_artifact,
    write_training_run_artifact,
)


def _valid_training_artifact():
    return build_training_run_artifact(
        run_id="phase4:rl_training",
        run_type="mock_training",
        stage="rl_training",
        status="completed",
        algorithm="PPO",
        environment={"kind": "mock", "name": "DummyEnv"},
        inputs={"total_timesteps": 100},
        metrics={"mean_reward": 1.0},
        artifacts=[
            {
                "name": "rl_model",
                "path": "models/rl/ppo_final.zip",
                "artifact_type": "model",
            }
        ],
        hardware_required=False,
        hardware_enabled=False,
        started_at="2026-04-12T00:00:00+00:00",
        finished_at="2026-04-12T00:00:01+00:00",
        duration_seconds=1.25,
    )


def test_training_run_artifact_contract_accepts_canonical_payload(tmp_path):
    artifact = _valid_training_artifact()

    assert artifact["schema_version"] == TRAINING_CONTRACT_VERSION
    assert artifact["artifact_type"] == "training_run"
    assert validate_training_run_artifact(artifact) == []

    output_path = write_training_run_artifact(
        artifact, tmp_path / "training_run_manifest.json"
    )
    saved = json.loads(output_path.read_text(encoding="utf-8"))
    assert validate_training_run_artifact(saved) == []
    assert saved["run_id"] == "phase4:rl_training"


def test_training_run_artifact_contract_reports_invalid_payload():
    errors = validate_training_run_artifact(
        {
            "schema_version": "bad",
            "artifact_type": "training_run",
            "run_type": "hardware_in_the_loop",
            "status": "done",
            "hardware_required": False,
            "hardware_enabled": True,
            "artifacts": [{"name": "", "path": "", "artifact_type": ""}],
            "duration_seconds": -1,
        }
    )

    assert "schema_version must be '1.0', got 'bad'" in errors
    assert "hardware_in_the_loop runs must set hardware_required=true" in errors
    assert "hardware_enabled=true requires hardware_required=true" in errors
    assert "duration_seconds must be a non-negative number" in errors


def test_write_training_run_artifact_rejects_invalid_payload(tmp_path):
    with pytest.raises(ValueError, match="missing required fields"):
        write_training_run_artifact(
            {"artifact_type": "training_run"}, tmp_path / "bad.json"
        )
