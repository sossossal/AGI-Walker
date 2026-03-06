import os
import shutil
import subprocess
from pathlib import Path

repo_root = Path(r"d:\新建文件夹\AGI-Walker")
docs_dir = repo_root / "docs"

# Define the new structure
structure = {
    "getting_started": [
        "GETTING_STARTED.md",
        "QUICK_START.md",
        "BEGINNER_TUTORIAL.md",
        "HANDS_ON_GUIDE.md"
    ],
    "guides": [
        "ADVANCED_USAGE.md",
        "CLI_GUIDE.md",
        "GUI_GUIDE.md",
        "GUI_USER_GUIDE.md",
        "WEB_PANEL_GUIDE.md",
        "SKILLS_DEVELOPMENT.md",
        "TESTING_GUIDE.md",
        "OPTIMIZATION_GUIDE.md",
        "DISTRIBUTED_GUIDE.md",
        "COMPILE_OPTIMIZED.md",
        "GODOT_INTEGRATION_GUIDE.md",
        "GODOT_TESTING_GUIDE.md"
    ],
    "hardware": [
        "HARDWARE_DEPLOYMENT.md",
        "HARDWARE_INTEGRATION_GUIDE.md",
        "HARDWARE_SPEC.md",
        "MODULAR_ROBOT_BUILDER.md",
        "PARTS_LIBRARY_GUIDE.md",
        "BOM_biped.txt",
        "IMC22_WORKFLOW.md"
    ],
    "algorithm_and_sim": [
        "SIMULATION_GUIDE.md",
        "PHYSICS_ENVIRONMENT_GUIDE.md",
        "ENVIRONMENT_GENERATION.md",
        "PID_BALANCE_GUIDE.md",
        "PARAMETRIC_CONTROL.md",
        "PARAMETER_CONVERSION_GUIDE.md",
        "OFFLINE_RL.md",
        "IMITATION_LEARNING.md",
        "CV_IMPLEMENTATION_PLAN.md",
        "OPENNEURO_INTEGRATION.md"
    ],
    "ros2": [
        "ROS2_INTEGRATION_DESIGN.md",
        "ROS2_QUICK_START.md"
    ],
    "architecture": [
        "ADVANCED_FEATURES.md",
        "COMPLETE_FEATURES.md",
        "PROJECT_SUMMARY.md",
        "AI_SETUP_GUIDE.md",
        "MODEL_ZOO.md"
    ],
    "archive_and_reports": [
        "PROJECT_ARCHIVE.md",
        "FINAL_REPORT.md",
        "FINAL_STATUS.md",
        "TEST_REPORT.md",
        "OPENNEURO_TEST_REPORT.md",
        "RELEASE_NOTES.md",
        "GIT_RELEASE.md",
        "RELEASE_CHECKLIST.md",
        "SKILLS_CHANGELOG.md",
        "COMMUNITY_PLAN.md",
        "TECH_DEBT_PLAN.md",
        "LAUNCH_POST.md",
        "blog_hive_reflex.md",
        "demo_video_script.md",
        "GITHUB_UPDATE_GUIDE.md",
        "CPP_PLUGIN_BUILD.md",
        "3D_VISUALIZATION_PLAN.md"
    ]
}

# Build mapping: old filename -> new relative path within docs/
file_mapping = {}
for category, files in structure.items():
    cat_dir = docs_dir / category
    cat_dir.mkdir(exist_ok=True)
    for f in files:
        file_mapping[f] = f"{category}/{f}"

# Build a reverse lookup: old_basename -> new_path_relative_to_docs_root
# e.g. 'QUICK_START.md' -> 'getting_started/QUICK_START.md'

# 1. Move files
for filename, new_rel_path in file_mapping.items():
    old_path = docs_dir / filename
    new_path = docs_dir / new_rel_path
    if old_path.exists():
        new_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(old_path), str(new_path))
        print(f"Moved: {filename} -> docs/{new_rel_path}")
    else:
        print(f"Skipped (not found): {filename}")

# Stage all changes in docs/
subprocess.run("git add docs/", cwd=str(repo_root), shell=True, check=True)
print("Staged docs/ changes.")

# 2. Update links in markdown files across the whole repo
def fix_links_in_file(filepath: Path):
    try:
        content = filepath.read_text(encoding="utf-8")
    except Exception as e:
        print(f"  Could not read {filepath}: {e}")
        return

    original = content
    inside_docs = filepath.is_relative_to(docs_dir)

    for old_name, new_rel_path in file_mapping.items():
        # Category and file parts
        # new_rel_path is like "getting_started/QUICK_START.md"
        
        if not inside_docs:
            # File is outside docs. Links would be like `docs/OLD_NAME`
            content = content.replace(f"docs/{old_name}", f"docs/{new_rel_path}")
        else:
            # File is inside docs/. Old links pointed to sibling files.
            # Since the moved files are now in subdirs, and the file doing the linking
            # is already in a subdir itself, we need to recalculate relative depth.
            try:
                frel = filepath.relative_to(docs_dir)
                depth = len(frel.parts) - 1  # depth from docs/ root (0 if in root)
            except ValueError:
                depth = 0

            if depth == 0:
                # File is at docs/ root level, link was just `OLD_NAME`
                content = content.replace(f"]({old_name})", f"]({new_rel_path})")
                content = content.replace(f"]({old_name}#", f"]({new_rel_path}#")
            else:
                # File is in docs/category/, old link was OLD_NAME or ../OLD_NAME
                up = "../" * depth
                content = content.replace(f"]({old_name})", f"]({up}{new_rel_path})")
                content = content.replace(f"]({old_name}#", f"]({up}{new_rel_path}#")
                content = content.replace(f"](../{old_name})", f"]({up}{new_rel_path})")

            # Also fix absolute-style links docs/OLD_NAME inside docs files
            content = content.replace(f"](docs/{old_name})", f"]({new_rel_path})")

    if content != original:
        filepath.write_text(content, encoding="utf-8")
        print(f"  Updated links: {filepath.relative_to(repo_root)}")

# Iterate all markdown files across the repo (skip .git and test_env)
all_md_files = [
    p for p in repo_root.rglob("*.md")
    if ".git" not in p.parts and "test_env" not in p.parts and ".pytest_cache" not in p.parts
]
print(f"\nUpdating links in {len(all_md_files)} markdown files...")
for md in all_md_files:
    fix_links_in_file(md)

# Stage updated files
subprocess.run("git add -A", cwd=str(repo_root), shell=True, check=True)
print("\nAll changes staged successfully.")
