import yaml
from pathlib import Path


def self_test():
    workflows_dir = Path(".github/workflows")
    all_passed = True

    # 验证规则 (Native Node 24 Final Standard)：
    # 1. 必须使用官方已正式发布并进入稳定分发网络的 Native Node 24 标签。
    # 2. 严禁使用任何 env 补丁 (如 FORCE_... 或 ALLOW_...)，因为原生版本不需要这些。
    # 3. 版本号必须具体到补丁版，以确保分发可靠性。

    for wf_file in workflows_dir.glob("*.yml"):
        with open(wf_file, "r", encoding="utf-8") as f:
            content = f.read()
            data = yaml.safe_load(content)

            issues = []

            # Check for redundant patches
            env = data.get("env", {})
            if (
                "FORCE_JAVASCRIPT_ACTIONS_TO_NODE24" in env
                or "ACTIONS_ALLOW_USE_UNSECURE_NODE_VERSION" in env
            ):
                issues.append("Found obsolete environment patches")

            # Version validation (Locked to confirmed stable Native Node 24 distribution)
            if (
                "actions/checkout@" in content
                and "actions/checkout@v6.0.2" not in content
            ):
                issues.append("actions/checkout must use stable native @v6.0.2")
            if (
                "actions/setup-python@" in content
                and "actions/setup-python@v6.2.0" not in content
            ):
                issues.append("actions/setup-python must use stable native @v6.2.0")

            if issues:
                print(f"❌ {wf_file.name}: {', '.join(issues)}")
                all_passed = False
            else:
                print(f"✅ {wf_file.name}: Passed native validation.")

    return all_passed


if __name__ == "__main__":
    if self_test():
        print("\nSelf-test PASSED. Ready for underlying fix.")
        exit(0)
    else:
        print("\nSelf-test FAILED. Issues detected in current configuration.")
        exit(1)
