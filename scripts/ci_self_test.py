import yaml
from pathlib import Path
import re

def self_test():
    workflows_dir = Path(".github/workflows")
    all_passed = True
    
    # 验证规则：
    # 1. 不应包含 FORCE_JAVASCRIPT_ACTIONS_TO_NODE24 (因为我们现在原生支持)
    # 2. 不应包含 ACTIONS_ALLOW_USE_UNSECURE_NODE_VERSION
    # 3. 必须使用 Commit SHA (40位哈希) 锁定版本，以确保引用的是 node24 分支的代码
    
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
                
            # Check for SHA locking (Industrial Standard for underlying fix)
            # actions/checkout@SHA
            # actions/setup-python@SHA
            if "@v" in content:
                # We check if generic tags like @v4 or @v5 are still present
                tags = re.findall(r"actions/.*@(v\d+)", content)
                if tags:
                    issues.append(f"Using insecure tags {tags} instead of verified Commit SHA")

            if issues:
                print(f"❌ {wf_file.name}: {', '.join(issues)}")
                all_passed = False
            else:
                print(f"✅ {wf_file.name}: Passed underlying validation.")
                
    return all_passed

if __name__ == "__main__":
    if self_test():
        print("\nSelf-test PASSED. Ready for underlying fix.")
        exit(0)
    else:
        print("\nSelf-test FAILED. Issues detected in current configuration.")
        exit(1)
