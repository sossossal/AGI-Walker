from pathlib import Path


DOCKERFILE = Path("Dockerfile")
CONTRIBUTING = Path("CONTRIBUTING.md")
URDF_BATCH_CONVERT = Path("agi_walker/skills/urdf-generator/scripts/batch_convert.py")
PARAMETER_BATCH_OPTIMIZE = Path(
    "agi_walker/skills/parameter-optimizer/scripts/batch_optimize.py"
)


def test_dockerfile_does_not_reference_removed_source_layout() -> None:
    content = DOCKERFILE.read_text(encoding="utf-8")

    assert "COPY requirements.txt ." not in content
    assert "COPY python_api/ ./python_api/" not in content
    assert "COPY python_controller/ ./python_controller/" not in content
    assert "COPY pyproject.toml ." in content
    assert "COPY agi_walker/ ./agi_walker/" in content
    assert "COPY web_panel/ ./web_panel/" in content
    assert "RUN pip install --no-cache-dir ." in content


def test_contributing_uses_current_source_directories() -> None:
    content = CONTRIBUTING.read_text(encoding="utf-8")

    assert "flake8 python_api/ python_controller/" not in content
    assert "black --check python_api/ python_controller/" not in content
    assert "flake8 agi_walker/ web_panel/ tests/" in content
    assert "black --check agi_walker/ web_panel/ tests/" in content


def test_skill_batch_scripts_do_not_use_brittle_repo_root_traversal() -> None:
    urdf_content = URDF_BATCH_CONVERT.read_text(encoding="utf-8")
    optimizer_content = PARAMETER_BATCH_OPTIMIZE.read_text(encoding="utf-8")

    hardcoded_root = "sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))"

    assert hardcoded_root not in urdf_content
    assert hardcoded_root not in optimizer_content
    assert "Path(__file__).resolve().parents" in urdf_content
    assert "Path(__file__).resolve().parents" in optimizer_content
