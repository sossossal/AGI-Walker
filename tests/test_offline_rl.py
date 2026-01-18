"""
快速测试离线 RL 功能是否正常工作
"""

import sys
import os

# 添加项目路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

print("="*60)
print("离线 RL 功能测试")
print("="*60)

# 测试 1: 导入检查
print("\n[测试 1/4] 检查模块导入...")
try:
    from python_api.offline_rl import ExpertDataCollector, OfflineRLTrainer
    print("✓ offline_rl 模块导入成功")
except ImportError as e:
    print(f"✗ 导入失败: {e}")
    sys.exit(1)

# 测试 2: 检查 d3rlpy
print("\n[测试 2/4] 检查 d3rlpy 安装...")
try:
    import d3rlpy
    print(f"✓ d3rlpy 已安装 (版本 {d3rlpy.__version__})")
except ImportError:
    print("✗ d3rlpy 未安装")
    print("  请运行: pip install d3rlpy")
    sys.exit(1)

# 测试 3: 创建数据收集器
print("\n[测试 3/4] 测试数据收集器...")
try:
    collector = ExpertDataCollector("CartPole-v1")  # 使用简单环境测试
    print("✓ ExpertDataCollector 创建成功")
except Exception as e:
    print(f"✗ 创建失败: {e}")
    sys.exit(1)

# 测试 4: 创建训练器
print("\n[测试 4/4] 测试 CQL 训练器...")
try:
    trainer = OfflineRLTrainer("CartPole-v1", algorithm="cql")
    print("✓ OfflineRLTrainer 创建成功")
except Exception as e:
    print(f"✗ 创建失败: {e}")
    sys.exit(1)

print("\n" + "="*60)
print("✓ 所有测试通过！离线 RL 功能正常")
print("="*60)
print("\n下一步:")
print("1. 运行完整示例: python examples/offline_rl_demo.py")
print("2. 查看文档: docs/OFFLINE_RL.md")
