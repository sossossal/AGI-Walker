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
                env = data.get("env", {})
                
                # Check for FORCE_JAVASCRIPT_ACTIONS_TO_NODE24
                # It should be present and equal to 'true' (string) or true (bool)
                node24_val = env.get("FORCE_JAVASCRIPT_ACTIONS_TO_NODE24")
                
                if node24_val is None:
                    print(f"❌ {wf_file.name}: Missing FORCE_JAVASCRIPT_ACTIONS_TO_NODE24 env variable.")
                    all_valid = False
                elif str(node24_val).lower() != 'true':
                    print(f"❌ {wf_file.name}: FORCE_JAVASCRIPT_ACTIONS_TO_NODE24 should be 'true', found '{node24_val}'.")
                    all_valid = False
                else:
                    print(f"✅ {wf_file.name}: Valid Node.js 24 configuration.")
                    
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
