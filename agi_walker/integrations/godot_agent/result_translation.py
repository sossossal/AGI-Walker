from typing import Any, Dict, Iterable, List

SUCCESS_STATUSES = {"success"}


def translate_task_result(result: Any) -> Dict[str, Any]:
    if isinstance(result, dict):
        payload = dict(result)
    elif hasattr(result, "to_dict"):
        payload = result.to_dict()
    else:
        raise TypeError(f"Unsupported Godot Agent result type: {type(result)!r}")

    status = str(payload.get("status", "")).lower()
    if "success" not in payload:
        payload["success"] = status in SUCCESS_STATUSES or bool(payload.get("success"))

    if "message" not in payload:
        payload["message"] = status or "unknown"

    return payload


def translate_pipeline_results(results: Iterable[Any]) -> List[Dict[str, Any]]:
    return [translate_task_result(result) for result in results]


def build_roles_info(router: Any) -> List[Dict[str, Any]]:
    roles = []
    for role_name in router.get_available_roles():
        info = router.get_role_info(role_name) or {}
        roles.append(
            {
                "name": info.get("name", role_name),
                "description": info.get("description", ""),
                "capabilities": info.get("capabilities", []),
            }
        )
    return roles
