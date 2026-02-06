#!/usr/bin/env python
"""
AGI-Walker Skills 系统简化测试

使用动态导入测试所有功能。
"""

import sys
from pathlib import Path

# 添加项目根目录
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

print("\n" + "🧪" * 35)
print(" " * 20 + "AGI-Walker Skills 系统")
print(" " * 25 + "简化测试")
print("🧪" * 35 + "\n")

passed = 0
failed = 0


def test(name):
    """测试装饰器"""
    def decorator(func):
        def wrapper():
            global passed, failed
            try:
                func()
                print(f"  ✓ {name}")
                passed += 1
            except Exception as e:
                print(f"  ✗ {name}: {e}")
                failed += 1
        return wrapper
    return decorator


# ========== 测试1: Skills加载器 ==========
print("📦 测试 1: Skills 加载器")
print("-" * 70)

@test("导入skills_loader")
def test_import_loader():
    from agi_walker.skills_loader import get_skills_loader
    global loader
    loader = get_skills_loader()
    assert len(loader) >= 3

test_import_loader()

@test("列出所有skills")
def _test():
    skills = loader.get_skills_list()
    assert len(skills) == 3

_test()

@test("查找robot-modeling")
def _test():
    skill = loader.get_skill("robot-modeling")
    assert skill is not None
    assert skill.name == "robot-modeling"

_test()

@test("搜索功能")  
def _test():
    results = loader.search_skills("机器人")
    assert len(results) > 0

_test()

@test("分类功能")
def _test():
    categories = loader.get_categories()
    assert len(categories) >= 3

_test()

print()


# ========== 测试2: Robot Modeling ==========
print("🤖 测试 2: Robot Modeling Skill")
print("-" * 70)

@test("加载robot-modeling skill")
def _test():
    import importlib.util
    skill_file = project_root / "agi_walker" / "skills" / "robot-modeling" / "__init__.py"
    spec = importlib.util.spec_from_file_location("robot_modeling_skill", skill_file)
    global robot_modeling_module
    robot_modeling_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(robot_modeling_module)

_test()

@test("创建RobotBuilder")
def _test():
    robot = robot_modeling_module.RobotBuilder("test")\
        .add_torso(height=0.5, mass=5.0)\
        .build()
    assert robot.name == "test"
    assert len(robot.parts) > 0

_test()

@test("列出模板")
def _test():
    templates = robot_modeling_module.list_templates()
    assert len(templates) >= 2

_test()

@test("加载模板")
def _test():
    template = robot_modeling_module.load_template("biped_basic")
    assert template.name == "biped_basic"

_test()

print()


# ========== 测试3: Parameter Optimizer ==========
print("⚙️ 测试 3: Parameter Optimizer Skill")
print("-" * 70)

@test("加载parameter-optimizer skill")
def _test():
    import importlib.util
    skill_file = project_root / "agi_walker" / "skills" / "parameter-optimizer" / "__init__.py"
    spec = importlib.util.spec_from_file_location("parameter_optimizer_skill", skill_file)
    global param_opt_module
    param_opt_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(param_opt_module)

_test()

@test("质量分布优化")
def _test():
    robot = robot_modeling_module.load_template("biped_basic")
    result = param_opt_module.optimize_mass_distribution(
        robot.to_dict(),
        target_com_height=0.22,
        max_iterations=30
    )
    assert result.success
    assert result.iterations > 0

_test()

print()


# ========== 测试4: URDF Generator ==========
print("📄 测试 4: URDF Generator Skill")
print("-" * 70)

@test("加载urdf-generator skill")
def _test():
    import importlib.util
    skill_file = project_root / "agi_walker" / "skills" / "urdf-generator" / "__init__.py"
    spec = importlib.util.spec_from_file_location("urdf_generator_skill", skill_file)
    global urdf_gen_module
    urdf_gen_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(urdf_gen_module)

_test()

@test("生成URDF文件")
def _test():
    robot = robot_modeling_module.load_template("biped_basic")
    
    # 创建临时文件
    test_json = project_root / "configs" / "test_urdf_simple.json"
    test_urdf = project_root / "exports" / "test_simple.urdf"
    
    test_json.parent.mkdir(exist_ok=True)
    test_urdf.parent.mkdir(exist_ok=True)
    
    robot.save(str(test_json))
    
    urdf_gen_module.convert_to_urdf(
        str(test_json),
        str(test_urdf)
    )
    
    assert test_urdf.exists()
    
    # 清理
    test_json.unlink()
    test_urdf.unlink()

_test()

@test("验证URDF")
def _test():
    robot = robot_modeling_module.load_template("biped_basic")
    test_json = project_root / "configs" / "test_validate.json"
    test_urdf = project_root / "exports" / "test_validate.urdf"
    
    robot.save(str(test_json))
    urdf_gen_module.convert_to_urdf(str(test_json), str(test_urdf))
    
    is_valid = urdf_gen_module.validate_urdf(str(test_urdf))
    
    # 清理
    test_json.unlink()
    test_urdf.unlink()
    
    assert is_valid

_test()

print()


# ========== 测试5: 完整工作流 ==========
print("🔄 测试 5: 完整工作流")
print("-" * 70)

@test("步骤1-4: 建模→优化→保存→URDF")
def _test():
    # 步骤1: 建模
    robot = robot_modeling_module.RobotBuilder("workflow_test")\
        .add_torso(height=0.5, mass=5.0)\
        .add_leg_pair(thigh_length=0.3, shin_length=0.3)\
        .build()
    
    # 步骤2: 优化
    result = param_opt_module.optimize_mass_distribution(
        robot.to_dict(),
        target_com_height=0.22,
        max_iterations=30
    )
    assert result.success
    
    # 步骤3: 保存
    test_config = project_root / "configs" / "workflow_final.json"
    robot.save(str(test_config))
    assert test_config.exists()
    
    # 步骤4: 转换URDF
    test_urdf = project_root / "exports" / "workflow_final.urdf"
    urdf_gen_module.convert_to_urdf(str(test_config), str(test_urdf))
    assert test_urdf.exists()
    
    # 清理
    test_config.unlink()
    test_urdf.unlink()

_test()

print()


# ========== 总结 ==========
total = passed + failed
print("=" * 70)
print(f"测试总结: {passed}/{total} 通过")
print("=" * 70)

if passed == total:
    print("\n🎉 所有测试通过！系统运行正常。\n")
    sys.exit(0)
else:
    print(f"\n⚠️ {failed}个测试失败。\n")
    sys.exit(1)
