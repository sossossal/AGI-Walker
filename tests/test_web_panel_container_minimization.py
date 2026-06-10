from pathlib import Path


WEB_PANEL_DOCKERFILE = Path("deployment/Dockerfile.web_panel")
APP_SOURCE_ROOTS = (
    Path("agi_walker"),
    Path("web_panel"),
    Path("tools"),
)


def test_web_panel_final_image_removes_runtime_vulnerability_candidates() -> None:
    dockerfile = WEB_PANEL_DOCKERFILE.read_text(encoding="utf-8")

    purge = "apt-get purge -y --auto-remove apt bash libbz2-1.0"
    assert purge in dockerfile
    assert "/var/cache/apt/*" in dockerfile
    assert dockerfile.rfind(purge) > dockerfile.rfind("pip install --no-cache-dir")


def test_web_panel_container_build_checks_application_imports_after_minimization() -> None:
    dockerfile = WEB_PANEL_DOCKERFILE.read_text(encoding="utf-8")

    assert 'python -c "import agi_walker; import web_panel.server; import web_panel.celery_app"' in dockerfile
    assert dockerfile.rfind("import web_panel.server") > dockerfile.rfind("COPY . /app")


def test_application_sources_do_not_depend_on_bz2_runtime() -> None:
    offenders = []
    for source_root in APP_SOURCE_ROOTS:
        for path in source_root.rglob("*.py"):
            text = path.read_text(encoding="utf-8", errors="ignore")
            if "import bz2" in text or "from bz2" in text:
                offenders.append(path.as_posix())

    assert offenders == []
