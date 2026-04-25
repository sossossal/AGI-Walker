from __future__ import annotations

from pathlib import Path

import pytest

from agi_walker.core.api.release_ops_contracts import CustomerAcceptanceBundleRequest
from agi_walker.ops.acceptance import execute_customer_acceptance_bundle
from tests.test_customer_acceptance_bundle import _write_ready_manifest


def test_execute_customer_acceptance_bundle_writes_ready_bundle(tmp_path: Path) -> None:
    manifest_path = _write_ready_manifest(tmp_path)
    output_path = tmp_path / "test_env" / "release" / "customer_acceptance_bundle.json"

    result = execute_customer_acceptance_bundle(
        CustomerAcceptanceBundleRequest(
            manifest=str(manifest_path),
            project_root=str(tmp_path),
            output=str(output_path),
        )
    )

    assert result.output_path == output_path
    assert result.output_path.exists()
    assert result.manifest_path == manifest_path
    assert result.payload["artifact_type"] == "customer_acceptance_bundle"
    assert result.payload["bundle_status"] == "ready"
    assert result.payload["vulnerability_exception_review"]["status"] == "passed"
    assert result.payload["external_mainline_execution_plan"]["status"] == "ready"
    assert result.payload["release_ops_execution"]["status"] == "passed"


def test_execute_customer_acceptance_bundle_rejects_missing_manifest(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="--manifest does not exist"):
        execute_customer_acceptance_bundle(
            CustomerAcceptanceBundleRequest(
                manifest=str(tmp_path / "missing.json"),
                project_root=str(tmp_path),
            )
        )
