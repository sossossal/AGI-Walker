import argparse
import json
from pathlib import Path

from agent_system import GodotStudioRouter


def build_router(config_path: str, project_path: str | None) -> GodotStudioRouter:
    return GodotStudioRouter(config_path=config_path, godot_project_path=project_path)


def run_interactive(router: GodotStudioRouter) -> int:
    print("Godot Studio Agent CLI")
    print("输入自然语言命令，输入 `roles` 查看角色，输入 `history` 查看历史，输入 `exit` 退出。")

    while True:
        try:
            prompt = input("\nagent> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n退出。")
            return 0

        if not prompt:
            continue
        if prompt.lower() in {"exit", "quit"}:
            print("退出。")
            return 0
        if prompt.lower() == "roles":
            print(json.dumps(router.get_roles_info(), ensure_ascii=False, indent=2))
            continue
        if prompt.lower() == "history":
            print(json.dumps(router.get_history(), ensure_ascii=False, indent=2))
            continue

        result = router.execute(prompt)
        print(json.dumps(result, ensure_ascii=False, indent=2))


def main() -> int:
    base_dir = Path(__file__).resolve().parent
    default_config = base_dir / "config.yaml"

    parser = argparse.ArgumentParser(description="Godot Studio Agent CLI")
    parser.add_argument("--config", default=str(default_config), help="配置文件路径")
    parser.add_argument("--project-path", help="Godot 项目路径")
    parser.add_argument("--prompt", help="执行单条命令后退出")
    parser.add_argument(
        "--list-roles", action="store_true", help="列出内置角色并退出"
    )
    args = parser.parse_args()

    router = build_router(args.config, args.project_path)

    if args.list_roles:
        print(json.dumps(router.get_roles_info(), ensure_ascii=False, indent=2))
        return 0

    if args.prompt:
        print(json.dumps(router.execute(args.prompt), ensure_ascii=False, indent=2))
        return 0

    return run_interactive(router)


if __name__ == "__main__":
    raise SystemExit(main())
