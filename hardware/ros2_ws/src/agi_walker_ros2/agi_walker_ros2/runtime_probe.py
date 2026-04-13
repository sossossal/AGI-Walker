from __future__ import annotations

import importlib
from typing import Any, Callable


ROS2_RUNTIME_MODULES = (
    "rclpy",
    "tf2_ros",
    "sensor_msgs.msg",
    "geometry_msgs.msg",
    "std_srvs.srv",
)


def probe_ros2_python_runtime(
    importer: Callable[[str], Any] | None = None,
) -> dict[str, Any]:
    import_module = importer or importlib.import_module
    modules: dict[str, dict[str, Any]] = {}
    missing_modules: list[str] = []

    for module_name in ROS2_RUNTIME_MODULES:
        try:
            import_module(module_name)
            modules[module_name] = {"available": True, "error": None}
        except Exception as exc:
            modules[module_name] = {"available": False, "error": str(exc)}
            missing_modules.append(module_name)

    return {
        "available": len(missing_modules) == 0,
        "modules": modules,
        "missing_modules": missing_modules,
    }
