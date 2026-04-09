from agi_walker.core.controllers import load_monitor as load_monitor_module


def test_system_monitor_works_without_psutil(monkeypatch) -> None:
    monkeypatch.setattr(load_monitor_module, "psutil", None)

    monitor = load_monitor_module.SystemMonitor()
    stats = monitor.get_hw_stats()

    assert stats["cpu_percent"] == 0.0
    assert stats["memory_percent"] == 0.0
    assert stats["temperature"] is None
    assert stats["psutil_available"] is False
