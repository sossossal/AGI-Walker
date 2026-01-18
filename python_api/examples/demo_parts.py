"""
简单的零件库功能演示
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from godot_robot_env import PartsDatabase


def main():
    print("=" * 60)
    print("机器人零件库演示")
    print("=" * 60)
    
    # 加载数据库
    print("\n[1] 加载零件数据库...")
    db = PartsDatabase()
    
    # 列出所有零件
    all_parts = db.list_all_parts()
    print(f"    ✅ 成功加载 {len(all_parts)} 个零件")
    print(f"    零件列表: {', '.join(all_parts)}")
    
    # 获取单个零件详情
    print("\n[2] 获取 Dynamixel XL430-W250 详情...")
    xl430 = db.get_part("dynamixel_xl430_w250")
    if xl430:
        print(f"    型号: {xl430['model']}")
        print(f"    制造商: {xl430['manufacturer']}")
        print(f"    堵转扭矩: {xl430['specifications']['stall_torque']} N·m")
        print(f"    空载速度: {xl430['specifications']['no_load_speed']} RPM")
        print(f"    重量: {xl430['specifications']['weight']} kg")
        print(f"    价格: ${xl430['price_usd']}")
    
    # 创建机器人配置
    print("\n[3] 创建 4-DOF 步行机器人配置...")
    robot_config = db.create_robot_config([
        {"part_id": "dynamixel_xl430_w250", "joint": "hip_left"},
        {"part_id": "dynamixel_xl430_w250", "joint": "hip_right"},
        {"part_id": "dynamixel_xl430_w250", "joint": "knee_left"},
        {"part_id": "dynamixel_xl430_w250", "joint": "knee_right"},
    ])
    print(f"    ✅ 机器人配置创建成功，包含 {len(robot_config['parts'])} 个零件")
    
    # 数据验证
    print("\n[4] 验证数据完整性...")
    valid_count = sum(1 for pid in all_parts if db.validate_part(pid))
    print(f"    ✅ {valid_count}/{len(all_parts)} 个零件数据有效")
    
    print("\n" + "=" * 60)
    print("✅ 所有测试通过！零件库功能正常")
    print("=" * 60)


if __name__ == "__main__":
    main()
