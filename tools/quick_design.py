#!/usr/bin/env python
"""
AGI-Walker 快速设计向导 (Quick Design Wizard) - Real World Edition

交互式工具，用于快速生成基于真实零件的机器人设计配置和仿真参数。
功能：
1. 交互式收集设计需求
2. **自动选型**: 从 parts_library 中选择真实电机、电池、结构件
3. **精准建模**: 使用真实零件的物理参数 (质量、扭矩、转速)
4. **BOM 生成**: 输出物料清单和成本预估
5. 自动转换为 URDF 仿真模型
"""

import sys
import json
import importlib.util
from pathlib import Path

# ==========================================
# 1. 环境设置 & 依赖加载
# ==========================================


def _find_repo_root() -> Path:
    current = Path(__file__).resolve()
    for candidate in current.parents:
        if (candidate / "pyproject.toml").exists() and (candidate / "agi_walker").exists():
            return candidate
    return current.parent


def _configure_stdio() -> None:
    for stream_name in ("stdout", "stderr"):
        stream = getattr(sys, stream_name, None)
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")


_configure_stdio()
project_root = _find_repo_root()
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

BOLD = "\033[1m"
GREEN = "\033[92m"
BLUE = "\033[94m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
RESET = "\033[0m"


def print_header(text):
    print(f"\n{BOLD}{BLUE}{'=' * 60}{RESET}")
    print(f"{BOLD}{BLUE} {text}{RESET}")
    print(f"{BOLD}{BLUE}{'=' * 60}{RESET}\n")


# 加载 Skills
def load_skill_module(skill_name):
    skill_path = project_root / "agi_walker" / "skills" / skill_name / "__init__.py"
    if not skill_path.exists():
        return None

    module_name = f"agi_walker.skills.{skill_name.replace('-', '_')}"
    spec = importlib.util.spec_from_file_location(module_name, skill_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


print(f"{GREEN}正在初始化 Skills 系统...{RESET}")
robot_modeling = load_skill_module("robot-modeling")
urdf_generator = load_skill_module("urdf-generator")

if not robot_modeling:
    print(f"{BOLD}❌ 错误: 核心模块 robot-modeling 缺失。{RESET}")
    sys.exit(1)

# 加载零件库
try:
    PARTS_DB_PATH = project_root / "parts_library" / "complete_parts_database.json"
    with open(PARTS_DB_PATH, "r", encoding="utf-8") as f:
        PARTS_DB = json.load(f)
    print(f"{GREEN}✓ 零件库加载成功 ({PARTS_DB['metadata']['last_updated']}){RESET}")
except Exception as e:
    print(f"{YELLOW}⚠️ 警告: 无法加载零件库 ({e})，将回退到模拟模式。{RESET}")
    PARTS_DB = None

# ==========================================
# 2. 交互式设计流程
# ==========================================


def get_input(prompt, default=None, type_func=str):
    default_str = f" [{default}]" if default is not None else ""
    while True:
        try:
            full_prompt = f"{prompt}{default_str}: "
            user_input = input(full_prompt).strip()
            if not user_input and default is not None:
                return default
            if not user_input:
                continue
            return type_func(user_input)
        except ValueError:
            print(f"{YELLOW}输入无效，请重试。{RESET}")


def wizard_step_1_basics():
    print_header("步骤 1/5: 基础设计需求")
    print("请选择机器人类型:")
    print("  1. Biped (双足 - 适合行走/奔跑)")
    print("  2. Quadruped (四足 - 适合负载/越野)")

    type_map = {"1": "biped", "2": "quadruped"}
    choice = get_input("选择类型 (1-2)", "1")
    robot_type = type_map.get(choice, "biped")

    name = get_input("机器人名称", f"my_real_{robot_type}")

    print("\n应用场景 (决定零件选型):")
    print("  1. Eco (经济 - 低成本，适合教育/DIY)")
    print("  2. Performance (高性能 - 高扭矩，适合科研/竞速)")

    scenario_map = {"1": "eco", "2": "performance"}
    choice = get_input("选择场景 (1-2)", "1")
    scenario = scenario_map.get(choice, "eco")

    target_height = get_input(
        "目标高度 (m)", 0.4 if robot_type == "quadruped" else 0.6, float
    )

    return robot_type, name, scenario, target_height


# ==========================================
# 3. 真实零件选型逻辑 (核心算法)
# ==========================================


def parse_value(val_str):
    val_str = str(val_str).split()[0]
    if "-" in val_str:
        low, high = map(float, val_str.split("-"))
        return (low + high) / 2
    return float(val_str)


def select_parts(robot_type, scenario, target_height, overrides={}):
    """
    根据需求自动选择最佳真实零件 (支持自定义覆盖与高级参数)
    """
    print_header("步骤 2/5: 智能零件选型与校验")

    selected_parts = {}
    bom_list = []
    specs = {}
    warnings = []

    # --- 1. 电机选型 ---
    motors = PARTS_DB["parts"]["motors"] + PARTS_DB["parts"].get("motors_diy", [])

    if overrides.get("torque"):
        motor = {
            "name": f"Custom Motor ({overrides['torque']}Nm)",
            "type": "custom",
            "price": 0,
            "specs": {
                "weight": "0.5 kg",
                "max_torque": f"{overrides['torque']} Nm",
                "voltage": "24V",  # Default
            },
        }
        motor_torque = overrides["torque"]
        motor_weight = 0.5
    elif scenario == "performance":
        candidates = [
            m for m in motors if "servo" in m["type"] or "brushless" in m["type"]
        ]
        candidates.sort(key=lambda x: x["price"], reverse=True)
        motor = candidates[0]
        motor_torque = parse_value(
            motor["specs"].get("max_torque", motor["specs"].get("torque", "1.0"))
        )
        motor_weight = parse_value(motor["specs"]["weight"])
    else:
        candidates = [m for m in motors if "stepper" not in m["type"]]
        candidates.sort(key=lambda x: x["price"])
        motor = candidates[0]
        motor_torque = parse_value(
            motor["specs"].get("max_torque", motor["specs"].get("torque", "1.0"))
        )
        motor_weight = parse_value(motor["specs"]["weight"])

    selected_parts["motor"] = motor
    print(f"  ⚡ 电机: {CYAN}{motor['name']}{RESET} (Torque: {motor_torque} Nm)")

    # --- 2. 关节 ---
    joints = PARTS_DB["parts"]["joints"]
    if scenario == "performance":
        joint = next((j for j in joints if "harmonic" in j["type"]), joints[0])
    else:
        joint = next((j for j in joints if "planetary" in j["type"]), joints[-1])

    selected_parts["joint"] = joint
    ratio = parse_value(joint["specs"].get("reduction_ratio", 1))
    joint_weight = parse_value(joint["specs"]["weight"])
    joint_eff = (
        parse_value(joint["specs"].get("efficiency", "0.8").replace("%", "")) / 100.0
    )
    backlash = parse_value(joint["specs"].get("backlash", "0").replace("arcmin", ""))
    stiffness = parse_value(joint["specs"].get("stiffness", "10000"))

    print(
        f"  ⚙️ 关节: {CYAN}{joint['name']}{RESET} (Backlash: {backlash}', Stiffness: {stiffness})"
    )

    # --- 3. 结构 (Material) ---
    structs = PARTS_DB["parts"]["structure"]
    target_mat_type = overrides.get("material")

    s_mat = None
    if target_mat_type:
        if target_mat_type == "carbon_fiber":
            s_mat = next((s for s in structs if "carbon" in s["type"]), None)
        elif target_mat_type == "aluminum":
            s_mat = next((s for s in structs if "aluminum" in s["type"]), None)
        elif target_mat_type == "plastic":
            s_mat = next((s for s in structs if "3d_print" in s["type"]), None)

    if not s_mat:
        if scenario == "performance":
            s_mat = next(s for s in structs if "carbon" in s["type"])
        else:
            s_mat = next(
                s for s in structs if "aluminum" in s["type"] or "3d_print" in s["type"]
            )

    selected_parts["structure"] = s_mat
    print(f"  🦴 结构: {CYAN}{s_mat['name']}{RESET}")

    # --- 4. 电池 & 电气校验 ---
    powers = PARTS_DB["parts"]["power"]

    # Calculate motor voltage first
    motor_voltage_str = motor["specs"].get("voltage", "0V").replace("V", "")
    if "-" in motor_voltage_str:
        mv_min, mv_max = map(float, motor_voltage_str.split("-"))
        motor_voltage = (mv_min + mv_max) / 2
    else:
        motor_voltage = parse_value(motor_voltage_str)

    # Smart Select: Find battery with voltage closest to motor
    def get_voltage(p):
        v_str = p["specs"]["voltage"].replace("V", "")
        if "-" in v_str:
            low, high = map(float, v_str.split("-"))
            return (low + high) / 2
        return float(v_str)

    # Filter by lithium first
    candidates = [p for p in powers if "lipo" in p["type"] or "li-ion" in p["type"]]
    if not candidates:
        candidates = powers

    # Sort by voltage difference
    candidates.sort(key=lambda p: abs(get_voltage(p) - motor_voltage))
    battery = candidates[0]

    selected_parts["battery"] = battery
    batt_weight = parse_value(battery["specs"]["weight"])
    batt_voltage = get_voltage(battery)

    # Re-evaluate motor voltage for display/check (already calc'd above but let's be consistent)
    # motor_voltage is already float from step 1/4

    print(
        f"  🔋 电源: {CYAN}{battery['name']}{RESET} ({batt_voltage}V, {batt_weight} kg)"
    )

    # ⚡ Electrical Checks
    if abs(batt_voltage - motor_voltage) > 5.0:
        warn_msg = f"{YELLOW}⚠️  电气不匹配警告: 电池电压({batt_voltage}V) 与 电机额定电压({motor_voltage}V) 差距较大!{RESET}"
        print(warn_msg)
        warnings.append(warn_msg)

    # --- 5. 计算物理参数 ---

    target_torque = motor_torque * ratio * joint_eff
    joint_mass = motor_weight + joint_weight + 0.2

    # 尺寸推导
    if robot_type == "biped":
        leg_len = target_height * 0.55
        torso_sz = [target_height * 0.25, target_height * 0.2, target_height * 0.4]
        num_joints = 6
        num_motors = 6
    else:  # quadruped
        leg_len = target_height * 0.5
        torso_sz = [target_height * 0.8, target_height * 0.3, target_height * 0.2]
        num_joints = 12
        num_motors = 12

    # 估算总重
    electronics_mass = 0.5 + batt_weight
    base_mass = electronics_mass + (joint_mass * 1.2 * num_joints)

    if overrides.get("mass_override"):
        total_mass = overrides["mass_override"]
    else:
        total_mass = base_mass

    specs = {
        "total_mass": round(total_mass, 2),
        "leg_length": round(leg_len, 2),
        "torso_size": torso_sz,
        "joint_mass": round(joint_mass, 2),
        "motor_torque": round(target_torque, 1),
        "joint_damping": overrides.get("damping")
        or (1.0 if scenario == "performance" else 0.5),
        "max_velocity": overrides.get("speed")
        or (10.0 if scenario == "performance" else 5.0),
        "link_mass": round(joint_mass * 0.4, 2),
        "friction": overrides.get("friction") or 0.1,
        # Advanced Dynamics
        "stiffness": stiffness,
        "backlash": backlash,
        "thermal_resistance": parse_value(
            motor["specs"].get("thermal_resistance", "1.0")
        ),
        "continuous_torque": parse_value(
            motor["specs"]
            .get("continuous_torque", str(motor_torque * 0.6))
            .replace("Nm", "")
        ),
        "rotor_inertia": parse_value(motor["specs"].get("rotor_inertia", "0.001")),
    }

    # --- 6. 生成 BOM ---
    bom_list.append({"part": motor, "qty": num_motors})
    bom_list.append({"part": joint, "qty": num_joints})
    bom_list.append({"part": battery, "qty": 1})
    bom_list.append({"part": s_mat, "qty": 1})

    controllers = PARTS_DB["parts"]["controllers"]
    ctrl = next((c for c in controllers if "raspberry" in c["type"]), controllers[0])
    bom_list.append({"part": ctrl, "qty": 1})

    # Append warnings to Metadata via BOM or separate list? V4_build passes specs, V5_exports uses Robot object.
    # We'll attach warnings to specs for now and handle in export.
    specs["warnings"] = warnings

    return specs, bom_list


def wizard_step_3_bom(bom_list):
    """显示 BOM 和成本"""
    print_header("步骤 3/5: 物料清单与成本预估 (BOM)")

    total_cost = 0
    print(f"{'零件名称':<30} | {'单价':<10} | {'数量':<5} | {'小计'}")
    print("-" * 65)

    for item in bom_list:
        part = item["part"]
        qty = item["qty"]
        cost = part["price"] * qty
        total_cost += cost
        print(f"{part['name']:<30} | ${part['price']:<9.2f} | {qty:<5} | ${cost:.2f}")

    print("-" * 65)
    print(f"{BOLD}总预估成本: ${total_cost:.2f}{RESET}")
    return total_cost


def wizard_step_4_build(robot_type, name, specs):
    """基于真实参数构建机器人"""
    print_header("步骤 4/5: 构建真实映射模型")
    print("正在配置物理引擎参数...")
    print(f"  - 连杆质量: {specs['link_mass']} kg (基于关节与材料)")
    print(f"  - 关节扭矩: {specs['motor_torque']} Nm (基于电机曲线)")

    builder = robot_modeling.RobotBuilder(name)

    # 躯干 (包含电池和控制器)
    torso_mass = specs["total_mass"] - (
        specs["joint_mass"] * (6 if robot_type == "biped" else 12)
    )
    # 修正: 简单减法可能导致躯干过轻，这里给个最小值
    torso_mass = max(torso_mass, 2.0)

    if robot_type == "biped":
        builder.add_torso(mass=torso_mass, size=specs["torso_size"])
        builder.add_leg_pair(
            thigh_length=specs["leg_length"] / 2,
            shin_length=specs["leg_length"] / 2,
            mass_per_link=specs["link_mass"],
        )
    elif robot_type == "quadruped":
        builder.add_torso(mass=torso_mass, size=specs["torso_size"])
        builder.add_leg_pair(
            thigh_length=specs["leg_length"] / 2,
            shin_length=specs["leg_length"] / 2,
            offset=[specs["torso_size"][0] / 2, 0, 0],
        )
        builder.add_leg_pair(
            thigh_length=specs["leg_length"] / 2,
            shin_length=specs["leg_length"] / 2,
            offset=[-specs["torso_size"][0] / 2, 0, 0],
        )

    builder.customize(
        motor_torque=specs["motor_torque"],
        max_velocity=specs["max_velocity"],
        joint_damping=specs["joint_damping"],
    )

    return builder.build()


def wizard_step_5_export(robot, bom_list, cost, specs):
    """导出"""
    print_header("步骤 5/5: 导出")

    config_dir = project_root / "configs" / "generated"
    config_dir.mkdir(parents=True, exist_ok=True)
    json_path = config_dir / f"{robot.name}.json"

    # 将 BOM 信息注入到 metadata
    extra_metadata = {
        "is_real_world_config": True,
        "estimated_cost_usd": cost,
        "advanced_dynamics": {
            "stiffness": robot.physics_overrides.get("stiffness", 10000)
            if hasattr(robot, "physics_overrides")
            else specs.get("stiffness"),
            "backlash": specs.get("backlash", 0),
            "thermal_resistance": specs.get("thermal_resistance", 1.0),
            "continuous_torque": specs.get("continuous_torque"),
            "rotor_inertia": specs.get("rotor_inertia"),
        },
        "design_warnings": specs.get("warnings", []),
        "bom": [
            {"id": i["part"]["id"], "name": i["part"]["name"], "qty": i["qty"]}
            for i in bom_list
        ],
    }

    # 保存
    robot.save(str(json_path))

    # 读取并手动更新 metadata (RobotBuilder.save 可能会覆盖 metadata)
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    data["metadata"].update(extra_metadata)
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

    print(f"📂 配置文件: {BLUE}{json_path}{RESET}")
    print("   (包含 BOM 和成本信息)")

    # URDF
    if urdf_generator:
        urdf_dir = project_root / "exports"
        urdf_dir.mkdir(parents=True, exist_ok=True)
        urdf_path = urdf_dir / f"{robot.name}.urdf"
        urdf_generator.convert_to_urdf(str(json_path), str(urdf_path))
        print(f"🤖 仿真模型: {BLUE}{urdf_path}{RESET}")


def main():
    import argparse

    parser = argparse.ArgumentParser(description="AGI-Walker Quick Design Wizard")
    parser.add_argument(
        "--non-interactive", action="store_true", help="Run without interactive prompts"
    )

    # Basic Specs
    parser.add_argument("--type", choices=["biped", "quadruped"], help="Robot type")
    parser.add_argument("--name", help="Robot name")
    parser.add_argument(
        "--scenario", choices=["eco", "performance", "custom"], help="Usage scenario"
    )
    parser.add_argument("--height", type=float, help="Target height (m)")

    # Custom Overrides
    parser.add_argument("--mass", type=float, help="Target total mass [Override]")
    parser.add_argument(
        "--material",
        choices=["aluminum", "carbon_fiber", "plastic"],
        help="Structure material [Override]",
    )
    parser.add_argument("--torque", type=float, help="Motor Max Torque (Nm) [Override]")
    parser.add_argument("--speed", type=float, help="Motor Max Speed (RPM) [Override]")
    parser.add_argument("--damping", type=float, help="Joint Damping [Override]")
    parser.add_argument("--friction", type=float, help="Joint Friction [Override]")

    args = parser.parse_args()

    print(f"\n{BOLD}🚀 AGI-Walker 快速设计向导 (v2.0){RESET}")
    print("----------------------------------------")

    if not PARTS_DB:
        print(f"{YELLOW}错误: 无法连接零件库，请检查 parts_library 目录。{RESET}")
        return

    # 1. 收集需求 (CLI 或 交互式)
    if args.non_interactive:
        # P1: 检查核心参数
        if not all([args.type, args.name, args.scenario, args.height]):
            # 如果缺参数但没报错，可能是用户期望默认值？
            # 这里强校验
            sys.stderr.write(
                "Error: Missing required args for non-interactive mode: --type, --name, --scenario, --height\n"
            )
            sys.exit(1)

        r_type = args.type
        r_name = args.name
        scenario = args.scenario
        height = args.height

        print(f"模式: CLI (非交互) | 配置: {r_type}, {scenario}")
    else:
        # 交互式
        r_type, r_name, scenario, height = wizard_step_1_basics()

    # 2. 选型 & 参数生成
    # 传递 args 里的 override 信息
    overrides = {
        "material": args.material,
        "torque": args.torque,
        "speed": args.speed,
        "damping": args.damping,
        "friction": args.friction,
        "mass_override": args.mass,
    }

    specs, bom = select_parts(r_type, scenario, height, overrides)

    # 3. 成本
    cost = wizard_step_3_bom(bom)

    # 4. 建模
    robot = wizard_step_4_build(r_type, r_name, specs)

    # 5. 导出
    wizard_step_5_export(robot, bom, cost, specs)

    print("\n" + "=" * 60)
    print(f"{GREEN}🎉 设计完成！{RESET}")
    print("=" * 60)
    if args.non_interactive:
        print(
            f"OUTPUT_JSON_PATH:{project_root / 'configs' / 'generated' / f'{r_name}.json'}"
        )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n❌ 用户取消")
