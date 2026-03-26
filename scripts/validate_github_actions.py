import os
import yaml
from pathlib import Path

def validate_workflows():
    workflows_dir = Path(".github/workflows")
    if not workflows_dir.exists():
        print("No workflows directory found.")
        return True

    all_valid = True
    for wf_file in workflows_dir.glob("*.yml"):
        with open(wf_file, "r", encoding="utf-8") as f:
            try:
                data = yaml.safe_load(f)
                content = wf_file.read_text(encoding="utf-8")
                
                # Check 1: Environment Flag
                env = data.get("env", {})
                flag = env.get("FORCE_JAVASCRIPT_ACTIONS_TO_NODE24")
                has_flag = flag is not None and str(flag).lower() == 'true'
                
                # Check 2: Action versions
                checkout_v6 = "actions/checkout@v6" in content
                python_v6 = "actions/setup-python@v6" in content
                
                issues = []
                if not has_flag:
                    issues.append("Missing FORCE_JAVASCRIPT_ACTIONS_TO_NODE24: 'true'")
                if "actions/checkout@" in content and not checkout_v6:
                    issues.append("actions/checkout should be @v6")
                if "actions/setup-python@" in content and not python_v6:
                    issues.append("actions/setup-python should be @v6")
                
                if issues:
                    print(f"❌ {wf_file.name}: {', '.join(issues)}")
                    all_valid = False
                else:
                    print(f"✅ {wf_file.name}: Fully compliant (v6 + env flag).")
                    
            except Exception as e:
                print(f"❌ {wf_file.name}: Failed to parse YAML: {e}")
                all_valid = False
                
    return all_valid

if __name__ == "__main__":
    if validate_workflows():
        print("\nAll workflows are compliant with Node.js 24 migration requirements.")
        exit(0)
    else:
        print("\nFound non-compliant workflow files.")
        exit(1)
