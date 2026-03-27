import yaml
from pathlib import Path
import re

def self_test():
    workflows_dir = Path(".github/workflows")
    all_passed = True
    
    # 验证规则 (2026 过渡期最佳实践)：
    # 1. 优先使用官方 Major Tags (@v4, @v5)，因为这些标签在 GitHub 分发网络中最稳定。
    # 2. 严禁使用未经验证或非公开的 Commit SHA，防止 "Action not found" 报错。
    # 3. 移除所有临时 env 补丁，回归原生声明。
    
    for wf_file in workflows_dir.glob("*.yml"):
        with open(wf_file, "r", encoding="utf-8") as f:
            content = f.read()
            data = yaml.safe_load(content)
            
            issues = []
            
            # Check env patches
            env = data.get("env", {})
            if "FORCE_JAVASCRIPT_ACTIONS_TO_NODE24" in env:
                issues.append("Found redundant FORCE_JAVASCRIPT_ACTIONS_TO_NODE24")
            if "ACTIONS_ALLOW_USE_UNSECURE_NODE_VERSION" in env:
                issues.append("Found redundant ACTIONS_ALLOW_USE_UNSECURE_NODE_VERSION")
                
            # Version path validation
            # Checkout should be @v4
            # Setup-python should be @v5
            if "actions/checkout@" in content and "actions/checkout@v4" not in content:
                issues.append("actions/checkout should use stable @v4")
            if "actions/setup-python@" in content and "actions/setup-python@v5" not in content:
                issues.append("actions/setup-python should use stable @v5")

            if issues:
                print(f"❌ {wf_file.name}: {', '.join(issues)}")
                all_passed = False
            else:
                print(f"✅ {wf_file.name}: Passed stability validation.")
                
    return all_passed

if __name__ == "__main__":
    if self_test():
        print("\nSelf-test PASSED. Ready for underlying fix.")
        exit(0)
    else:
        print("\nSelf-test FAILED. Issues detected in current configuration.")
        exit(1)
