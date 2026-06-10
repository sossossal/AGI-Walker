from pathlib import Path


WEB_PANEL_DOCKERFILE = Path("deployment/Dockerfile.web_panel")


def test_web_panel_final_image_removes_package_manager_and_bash() -> None:
    dockerfile = WEB_PANEL_DOCKERFILE.read_text(encoding="utf-8")

    assert "apt-get purge -y --auto-remove apt bash" in dockerfile
    assert "/var/cache/apt/*" in dockerfile
    assert dockerfile.rfind("apt-get purge -y --auto-remove apt bash") > dockerfile.rfind(
        "pip install --no-cache-dir"
    )
