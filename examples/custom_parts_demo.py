import sys
import os
import json

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from agi_walker.core.api.parts.parts_manager import PartsManager
from robot_models.base_robot import RobotConfig, LinkConfig, JointConfig, RobotType


def build_custom_quadruped():
    """
    使用零件库构建一个四足机器人
    """
    pm = PartsManager()

    print("=" * 60)
    print("模块化机器人构建器 (Modular Robot Builder)")
    print("=" * 60)

    # 1. 选择零件
    motor_id = "go_m8010"  # Unitree 风格电机
    battery_id = "lipo_4s_5000mah"
    imu_id = "imu_comsumer"

    # 验证零件是否存在
    motor = pm.get_part(motor_id)
    if not motor:
        print(f"Error: Motor {motor_id} not found!")
        return

    print(f"选择了核心执行器: {motor.name}")
    print(f"  - 峰值扭矩: {motor.specs['max_torque_nm']} Nm")
    print(f"  - 重量: {motor.weight_kg} kg")

    # 2. 定义结构 (这里仍然需要一些手动定义，但在高级版中可以全自动化)
    # 假设每条腿 3 个电机
    links = []
    joints = []

    # 机身
    body_mass = 2.0  # 铝合金框架估算
    # 加上电池重量
    battery = pm.get_part(battery_id)
    if battery:
        body_mass += battery.weight_kg
        print(f"加装电池: {battery.name} (+{battery.weight_kg}kg)")

    links.append(
        LinkConfig("trunk", "world", mass=body_mass, shape="box", size=[0.4, 0.2, 0.15])
    )

    # 生成四条腿
    legs = ["FL", "FR", "RL", "RR"]
    total_parts = [battery_id, imu_id]

    for leg in legs:
        # Hip
        joints.append(
            JointConfig(
                f"{leg}_hip",
                "hinge",
                max_torque=motor.specs["max_torque_nm"],
                # 自动映射电机参数
                damping=0.5,
            )
        )
        total_parts.append(motor_id)

        links.append(
            LinkConfig(f"{leg}_thigh", "trunk", mass=0.2, size=[0.2, 0.05, 0.05])
        )

        # Knee
        joints.append(
            JointConfig(f"{leg}_knee", "hinge", max_torque=motor.specs["max_torque_nm"])
        )
        total_parts.append(motor_id)

        links.append(
            LinkConfig(f"{leg}_calf", f"{leg}_thigh", mass=0.15, size=[0.2, 0.03, 0.03])
        )

    # 3. 计算 BOM
    bom = pm.calculate_bom(total_parts)

    # 4. 生成最终配置
    robot_config = RobotConfig(
        name="Custom-Quadruped-Go1-Mod",
        type=RobotType.QUADRUPED,
        description=f"基于 {motor.name} 构建的四足机器人",
        total_mass=bom["total_weight_kg"] + 2.0,  # 加上机身结构估算
        height=0.4,
        links=links,
        joints=joints,
        control_frequency=500.0,  # 高性能电机支持更高频控制
    )

    print("\n" + "-" * 60)
    print("构建报告 (Build Report)")
    print("-" * 60)
    print(f"机器人名称: {robot_config.name}")
    print(f"预估总重: {robot_config.total_mass:.2f} kg (含电池与电机)")
    print(f"关节数量: {len(joints)}")
    print(f"BOM 总成本: ${bom['total_cost_usd']:.2f}")

    print("\n详细零件清单:")
    part_counts = {}
    for p in bom["details"]:
        part_counts[p["name"]] = part_counts.get(p["name"], 0) + 1

    for name, count in part_counts.items():
        print(f"  - {name} x{count}")

    # 保存配置
    output_path = "custom_robot_config.json"
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(robot_config.to_dict(), f, indent=2, ensure_ascii=False)

    print(f"\n机器人配置已生成: {output_path}")
    print("现在可以将此配置加载到仿真环境中进行测试了！")


if __name__ == "__main__":
    try:
        build_custom_quadruped()
    except Exception as e:
        print(f"错误: {e}")
        import traceback

        traceback.print_exc()
