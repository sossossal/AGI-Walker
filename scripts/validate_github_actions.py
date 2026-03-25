#!/usr/bin/env python3
"""
GitHub Actions Node.js 24 兼容性检查脚本
验证工作流文件是否正确配置了 Node.js 24 支持
"""

import yaml
import sys
from pathlib import Path

def check_workflow_nodejs24_compatibility(workflow_file):
    """检查工作流文件的 Node.js 24 兼容性"""
    print(f"📋 检查工作流文件: {workflow_file}")
    print("-" * 60)
    
    try:
        with open(workflow_file, 'r', encoding='utf-8') as f:
            workflow = yaml.safe_load(f)
    except Exception as e:
        print(f"❌ 无法读取工作流文件: {e}")
        return False
    
    all_good = True
    
    # 检查 1: 环境变量
    print("\n✓ 检查 1: 环境变量配置")
    env = workflow.get('env', {})
    if 'FORCE_JAVASCRIPT_ACTIONS_TO_NODE24' in env:
        print(f"  ✅ 已配置: FORCE_JAVASCRIPT_ACTIONS_TO_NODE24 = {env['FORCE_JAVASCRIPT_ACTIONS_TO_NODE24']}")
    else:
        print("  ⚠️ 缺失: FORCE_JAVASCRIPT_ACTIONS_TO_NODE24")
        all_good = False
    
    # 检查 2: Actions 版本
    print("\n✓ 检查 2: GitHub Actions 版本")
    jobs = workflow.get('jobs', {})
    
    actions_to_check = {}
    
    for job_name, job_config in jobs.items():
        steps = job_config.get('steps', [])
        for step in steps:
            uses = step.get('uses', '').strip()
            if '@' in uses:
                action_name, version = uses.rsplit('@', 1)
                if action_name not in actions_to_check:
                    actions_to_check[action_name] = []
                actions_to_check[action_name].append(version)
    
    # 检查已知的 Node.js 20 兼容操作
    deprecated_actions = {
        'actions/checkout': 'v4',  # v4+ 支持 Node.js 24
        'actions/setup-python': 'v5',  # v5+ 支持 Node.js 24
    }
    
    for action, min_version in deprecated_actions.items():
        if action in actions_to_check:
            versions = set(actions_to_check[action])
            print(f"\n  📌 {action}")
            for version in versions:
                # 简单版本检查（假设 vX 格式）
                try:
                    major = int(version[1:].split('.')[0])
                    min_major = int(min_version[1:].split('.')[0])
                    if major >= min_major:
                        print(f"    ✅ {version} (支持 Node.js 24)")
                    else:
                        print(f"    ⚠️ {version} (需要更新)")
                        all_good = False
                except:
                    print(f"    ❓ {version} (无法解析版本)")
        else:
            print(f"\n  ℹ️ {action}：未使用")
    
    # 检查 3: Node.js 版本指定
    print("\n✓ 检查 3: Python 版本矩阵")
    test_job = jobs.get('test', {})
    if test_job:
        strategy = test_job.get('strategy', {})
        matrix = strategy.get('matrix', {})
        python_versions = matrix.get('python-version', [])
        print(f"  {python_versions}")
        if '3.10' in python_versions or '3.11' in python_versions:
            print("  ✅ Python 版本适配")
        else:
            print("  ⚠️ 建议包括 Python 3.10 和 3.11")
    
    return all_good

def check_all_workflows():
    """检查项目中的所有工作流文件"""
    print("\n" + "="*60)
    print("🔍 AGI-Walker GitHub Actions Node.js 24 兼容性检查")
    print("="*60)
    
    workflow_dir = Path('.github/workflows')
    if not workflow_dir.exists():
        print(f"❌ 工作流目录不存在: {workflow_dir}")
        return False
    
    yml_files = list(workflow_dir.glob('*.yml')) + list(workflow_dir.glob('*.yaml'))
    
    if not yml_files:
        print(f"❌ 未找到工作流文件")
        return False
    
    all_compatible = True
    
    for workflow_file in yml_files:
        if 'godot-cpp' not in str(workflow_file):  # 跳过子项目
            result = check_workflow_nodejs24_compatibility(str(workflow_file))
            all_compatible = all_compatible and result
            print()
    
    # 总结
    print("="*60)
    print("\n📊 检查总结\n")
    
    if all_compatible:
        print("✅ 所有工作流都已配置 Node.js 24 兼容性")
        print("\n建议下一步:")
        print("  1. git add .github/workflows/ci.yml")
        print("  2. git commit -m 'ci: enable Node.js 24 compatibility'")
        print("  3. git push origin develop")
        print("  4. 监控 GitHub Actions 运行结果")
    else:
        print("⚠️ 发现兼容性问题，需要修复:")
        print("\n修复方案:")
        print("  1. 将 FORCE_JAVASCRIPT_ACTIONS_TO_NODE24 添加到 env")
        print("  2. 更新已弃用的 actions 版本")
        print("  3. 重新测试工作流")
    
    return all_compatible

if __name__ == '__main__':
    try:
        success = check_all_workflows()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\n⏸️  检查被中断")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ 检查出错: {e}")
        sys.exit(1)
