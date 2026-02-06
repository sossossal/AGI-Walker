#!/usr/bin/env python
"""
AGI-Walker Skills 系统综合测试

测试所有核心功能的完整性和正确性。
"""

import sys
from pathlib import Path
import traceback

# 添加项目路径
sys.path.insert(0, str(Path(__file__).parent.parent))

# 测试结果统计
class TestResults:
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.errors = []
    
    def record_pass(self, test_name):
        self.passed += 1
        print(f"  ✓ {test_name}")
    
    def record_fail(self, test_name, error):
        self.failed += 1
        self.errors.append((test_name, error))
        print(f"  ✗ {test_name}: {error}")
    
    def summary(self):
        total = self.passed + self.failed
        print("\n" + "=" * 70)
        print(f"测试总结: {self.passed}/{total} 通过")
        print("=" * 70)
        
        if self.failed > 0:
            print(f"\n失败的测试 ({self.failed}):")
            for name, error in self.errors:
                print(f"  - {name}: {error}")
        
        return self.failed == 0


results = TestResults()


def test_skills_loader():
    """测试Skills加载器"""
    print("\n📦 测试 1: Skills 加载器")
    print("-" * 70)
    
    try:
        from agi_walker.skills_loader import get_skills_loader
        
        loader = get_skills_loader()
        
        # 测试1.1: 加载Skills
        skills = loader.get_skills_list()
        if len(skills) >= 3:
            results.record_pass(f"加载Skills ({len(skills)}个)")
        else:
            results.record_fail("加载Skills", f"只找到{len(skills)}个skills")
        
        # 测试1.2: 验证必需的Skills存在
        skill_names = [s.name for s in skills]
        required = ["robot-modeling", "parameter-optimizer", "urdf-generator"]
        
        for req in required:
            if req in skill_names:
                results.record_pass(f"找到 {req}")
            else:
                results.record_fail(f"找到 {req}", "不存在")
        
        # 测试1.3: 搜索功能
        search_results = loader.search_skills("机器人")
        if len(search_results) > 0:
            results.record_pass(f"搜索功能 (找到{len(search_results)}个)")
        else:
            results.record_fail("搜索功能", "未找到结果")
        
        # 测试1.4: 分类功能
        categories = loader.get_categories()
        if len(categories) >= 3:
            results.record_pass(f"分类功能 ({len(categories)}个分类)")
        else:
            results.record_fail("分类功能", f"只有{len(categories)}个分类")
        
    except Exception as e:
        results.record_fail("Skills加载器", str(e))
        traceback.print_exc()


def test_robot_modeling():
    """测试Robot Modeling Skill"""
    print("\n🤖 测试 2: Robot Modeling Skill")
    print("-" * 70)
    
    try:
        # 动态导入skill模块
        import importlib.util
        skill_path = Path(__file__).parent.parent / "agi_walker" / "skills" / "robot-modeling" / "__init__.py"
        spec = importlib.util.spec_from_file_location("robot_modeling", skill_path)
        robot_modeling = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(robot_modeling)
        
        RobotBuilder = robot_modeling.RobotBuilder
        load_template = robot_modeling.load_template
        list_templates = robot_modeling.list_templates
        
        # 测试2.1: RobotBuilder基本功能
        robot = RobotBuilder("test_robot")
        robot.add_torso(height=0.5, mass=5.0)
        robot.add_leg_pair(thigh_length=0.3, shin_length=0.3)
        config = robot.build()
        
        if config.name == "test_robot":
            results.record_pass("RobotBuilder创建")
        else:
            results.record_fail("RobotBuilder创建", "名称不匹配")
        
        if len(config.parts) >= 3:  # torso + 2 legs
            results.record_pass(f"添加部件 ({len(config.parts)}个)")
        else:
            results.record_fail("添加部件", f"只有{len(config.parts)}个")
        
        # 测试2.2: 保存和加载
        test_file = Path("configs/test_robot.json")
        test_file.parent.mkdir(exist_ok=True)
        
        config.save(str(test_file))
        if test_file.exists():
            results.record_pass("保存配置")
            test_file.unlink()  # 清理
        else:
            results.record_fail("保存配置", "文件未创建")
        
        # 测试2.3: 模板系统
        templates = list_templates()
        if len(templates) >= 2:
            results.record_pass(f"列出模板 ({len(templates)}个)")
        else:
            results.record_fail("列出模板", f"只有{len(templates)}个")
        
        # 测试2.4: 加载模板
        template = load_template("biped_basic")
        if template.name == "biped_basic":
            results.record_pass("加载模板")
        else:
            results.record_fail("加载模板", "模板数据不正确")
        
    except Exception as e:
        results.record_fail("Robot Modeling", str(e))
        traceback.print_exc()


def test_parameter_optimizer():
    """测试Parameter Optimizer Skill"""
    print("\n⚙️ 测试 3: Parameter Optimizer Skill")
    print("-" * 70)
    
    try:
        from agi_walker.skills.parameter_optimizer import (
            optimize_mass_distribution,
            MassDistributionOptimizer
        )
        from agi_walker.skills.robot_modeling import load_template
        
        # 准备测试数据
        robot = load_template("biped_basic")
        
        # 测试3.1: 质量分布优化
        result = optimize_mass_distribution(
            robot.to_dict(),
            target_com_height=0.22,
            max_iterations=50,
            method="gradient"
        )
        
        if result.success:
            results.record_pass("质量分布优化")
        else:
            results.record_fail("质量分布优化", "优化失败")
        
        if result.iterations > 0:
            results.record_pass(f"迭代执行 ({result.iterations}次)")
        else:
            results.record_fail("迭代执行", "未执行迭代")
        
        if result.mass_distribution:
            results.record_pass(f"生成质量分布 ({len(result.mass_distribution)}个)")
        else:
            results.record_fail("生成质量分布", "结果为空")
        
        # 测试3.2: 优化器类
        optimizer = MassDistributionOptimizer(robot.to_dict())
        if optimizer.robot_config is not None:
            results.record_pass("创建优化器")
        else:
            results.record_fail("创建优化器", "配置为空")
        
    except Exception as e:
        results.record_fail("Parameter Optimizer", str(e))
        traceback.print_exc()


def test_urdf_generator():
    """测试URDF Generator Skill"""
    print("\n📄 测试 4: URDF Generator Skill")
    print("-" * 70)
    
    try:
        from agi_walker.skills.urdf_generator import (
            URDFGenerator, convert_to_urdf, validate_urdf
        )
        from agi_walker.skills.robot_modeling import load_template
        
        # 准备测试数据
        robot = load_template("biped_basic")
        test_json = Path("configs/test_urdf.json")
        test_urdf = Path("exports/test_robot.urdf")
        
        test_json.parent.mkdir(exist_ok=True)
        test_urdf.parent.mkdir(exist_ok=True)
        
        robot.save(str(test_json))
        
        # 测试4.1: URDF生成器
        generator = URDFGenerator()
        generator.load_config(str(test_json))
        
        if len(generator.links) > 0:
            results.record_pass(f"加载配置 ({len(generator.links)} links)")
        else:
            results.record_fail("加载配置", "未生成links")
        
        # 测试4.2: 转换为URDF
        convert_to_urdf(
            str(test_json),
            str(test_urdf),
            generate_meshes=False
        )
        
        if test_urdf.exists():
            results.record_pass("生成URDF文件")
        else:
            results.record_fail("生成URDF文件", "文件未创建")
        
        # 测试4.3: 验证URDF
        if test_urdf.exists():
            is_valid = validate_urdf(str(test_urdf))
            if is_valid:
                results.record_pass("验证URDF")
            else:
                results.record_fail("验证URDF", "验证失败")
        
        # 清理
        if test_json.exists():
            test_json.unlink()
        if test_urdf.exists():
            test_urdf.unlink()
        
    except Exception as e:
        results.record_fail("URDF Generator", str(e))
        traceback.print_exc()


def test_complete_workflow():
    """测试完整工作流"""
    print("\n🔄 测试 5: 完整工作流")
    print("-" * 70)
    
    try:
        from agi_walker.skills.robot_modeling import RobotBuilder
        from agi_walker.skills.parameter_optimizer import optimize_mass_distribution
        from agi_walker.skills.urdf_generator import convert_to_urdf
        
        # 步骤1: 建模
        robot = (
            RobotBuilder("workflow_test")
            .add_torso(height=0.5, mass=5.0)
            .add_leg_pair(thigh_length=0.3, shin_length=0.3)
            .build()
        )
        results.record_pass("步骤1: 建模")
        
        # 步骤2: 优化
        mass_result = optimize_mass_distribution(
            robot.to_dict(),
            target_com_height=0.22,
            max_iterations=30
        )
        
        if mass_result.success:
            results.record_pass("步骤2: 优化")
        else:
            results.record_fail("步骤2: 优化", "优化失败")
        
        # 应用优化结果
        for part in robot.parts:
            part_id = part["id"]
            if part_id in mass_result.mass_distribution:
                part["params"]["mass"] = mass_result.mass_distribution[part_id]
        
        # 步骤3: 保存
        test_config = Path("configs/workflow_test.json")
        robot.save(str(test_config))
        
        if test_config.exists():
            results.record_pass("步骤3: 保存配置")
        else:
            results.record_fail("步骤3: 保存配置", "文件未创建")
        
        # 步骤4: 转换URDF
        test_urdf = Path("exports/workflow_test.urdf")
        convert_to_urdf(str(test_config), str(test_urdf))
        
        if test_urdf.exists():
            results.record_pass("步骤4: 转换URDF")
        else:
            results.record_fail("步骤4: 转换URDF", "文件未创建")
        
        # 清理
        if test_config.exists():
            test_config.unlink()
        if test_urdf.exists():
            test_urdf.unlink()
        
    except Exception as e:
        results.record_fail("完整工作流", str(e))
        traceback.print_exc()


def test_cli_tools():
    """测试CLI工具"""
    print("\n💻 测试 6: CLI 工具")
    print("-" * 70)
    
    try:
        import subprocess
        
        # 测试6.1: skills list
        result = subprocess.run(
            ["python", "-m", "agi_walker.cli", "skills", "list"],
            capture_output=True,
            text=True,
            cwd=Path(__file__).parent.parent
        )
        
        if result.returncode == 0 and "Skills" in result.stdout:
            results.record_pass("CLI: skills list")
        else:
            results.record_fail("CLI: skills list", f"返回码{result.returncode}")
        
        # 测试6.2: skills info
        result = subprocess.run(
            ["python", "-m", "agi_walker.cli", "skills", "info", "robot-modeling"],
            capture_output=True,
            text=True,
            cwd=Path(__file__).parent.parent
        )
        
        if result.returncode == 0 and "robot-modeling" in result.stdout:
            results.record_pass("CLI: skills info")
        else:
            results.record_fail("CLI: skills info", f"返回码{result.returncode}")
        
        # 测试6.3: skills search
        result = subprocess.run(
            ["python", "-m", "agi_walker.cli", "skills", "search", "机器人"],
            capture_output=True,
            text=True,
            cwd=Path(__file__).parent.parent
        )
        
        if result.returncode == 0:
            results.record_pass("CLI: skills search")
        else:
            results.record_fail("CLI: skills search", f"返回码{result.returncode}")
        
    except Exception as e:
        results.record_fail("CLI工具", str(e))
        traceback.print_exc()


def main():
    """主测试函数"""
    print("\n" + "🧪" * 35)
    print(" " * 20 + "AGI-Walker Skills 系统")
    print(" " * 25 + "综合测试")
    print("🧪" * 35 + "\n")
    
    # 运行所有测试
    test_skills_loader()
    test_robot_modeling()
    test_parameter_optimizer()
    test_urdf_generator()
    test_complete_workflow()
    test_cli_tools()
    
    # 显示总结
    success = results.summary()
    
    if success:
        print("\n🎉 所有测试通过！系统运行正常。")
        return 0
    else:
        print("\n⚠️ 部分测试失败，请检查上述错误。")
        return 1


if __name__ == "__main__":
    sys.exit(main())
