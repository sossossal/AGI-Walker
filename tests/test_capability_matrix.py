from pathlib import Path

from agi_walker.core.api.capability_matrix import (
    CAPABILITY_MATRIX_ROUTE,
    CAPABILITY_MATRIX_VERSION,
    build_capability_matrix_artifact,
    build_capability_matrix_summary,
    validate_capability_matrix_artifact,
    write_capability_matrix_artifact,
)


def test_capability_matrix_artifact_has_release_surfaces() -> None:
    payload = build_capability_matrix_artifact(generated_at="2026-04-12T12:00:00")

    assert payload["schema_version"] == CAPABILITY_MATRIX_VERSION
    assert payload["artifact_type"] == "capability_matrix"
    assert payload["summary"] == {
        "total_domains": 5,
        "ready_domains": 4,
        "diagnostic_ready_domains": 1,
    }
    domain_ids = [domain["id"] for domain in payload["domains"]]
    assert domain_ids == [
        "cli",
        "web_panel",
        "mcp",
        "distributed_runtime",
        "godot_integration",
    ]
    distributed = next(
        domain for domain in payload["domains"] if domain["id"] == "distributed_runtime"
    )
    assert distributed["status"] == "diagnostic_ready"
    assert distributed["contracts"][0] == {
        "name": "distributed_smoke_report",
        "version": "1.0",
    }


def test_capability_matrix_summary_exposes_route() -> None:
    summary = build_capability_matrix_summary()

    assert summary["schema_version"] == CAPABILITY_MATRIX_VERSION
    assert summary["artifact_type"] == "capability_matrix"
    assert summary["route"] == CAPABILITY_MATRIX_ROUTE
    assert summary["summary"]["total_domains"] == 5


def test_capability_matrix_validation_and_write(tmp_path: Path) -> None:
    payload = build_capability_matrix_artifact(generated_at="2026-04-12T12:00:00")

    assert validate_capability_matrix_artifact(payload) == []

    output_path = write_capability_matrix_artifact(
        payload, tmp_path / "capability_matrix.json"
    )

    assert output_path.exists()
    assert '"artifact_type": "capability_matrix"' in output_path.read_text(
        encoding="utf-8"
    )


def test_capability_matrix_rejects_missing_domain() -> None:
    payload = build_capability_matrix_artifact(generated_at="2026-04-12T12:00:00")
    payload["domains"] = [
        domain for domain in payload["domains"] if domain["id"] != "mcp"
    ]
    payload["summary"]["total_domains"] = 4
    payload["summary"]["ready_domains"] = 3

    errors = validate_capability_matrix_artifact(payload)

    assert "domains must include all required ids: mcp" in errors
