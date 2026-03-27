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
                
    # SHA locking validation
    verified_shas = [
        "0c366fd6a839edf440554fa01a7085ccba70ac98", # checkout main (node24)
        "28f2168f4d98ee0445e3c6321f6e6616c83dd5ec"  # setup-python main (node24)
    ]
    
    for wf_file in workflows_dir.glob("*.yml"):
        with open(wf_file, "r", encoding="utf-8") as f:
            content = f.read()
            
            issues = []
            # Check for legacy tags
            if "@v" in content:
                issues.append("Found legacy @v tags")
            
            # Check if using unverified SHAs
            found_shas = re.findall(r"@([a-f0-9]{40})", content)
            for sha in found_shas:
                if sha not in verified_shas:
                    issues.append(f"Unverified SHA found: {sha}")
            
            if not found_shas:
                issues.append("No SHAs found (all actions must use verified SHAs)")

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
