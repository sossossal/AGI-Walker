"""
测试零件库功能

展示如何从零件库加载数据并使用
"""
import sys
from pathlib import Path

# 添加父目录到路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from godot_robot_env import PartsDatabase


def test_parts_database():
    """测试零件数据库功能"""
    print("=" * 60)
    print("机器人零件库测试")
    print("=" * 60)
    
    #  加载数据库
    print("\n1. 加载零件数据库...")
    db = PartsDatabase()
    
    # 打印统计信息
    db.print_statistics()
    
    # 测试获取单个零件
    print("2. 获取单个零件...")
    xl430 = db.get_part("dynamixel_xl430_w250")
    if xl430:
        print(f"\n✅ 成功获取零件: {xl430['model']}")
        print(f"   制造商: {xl430['manufacturer']}")
        print(f"   类别: {xl430['category']}")
        print(f"   堵转扭矩: {xl430['specifications']['stall_torque']} N·m")
        print(f"   空载速度: {xl430['specifications']['no_load_speed']} RPM")
        print(f"   价格: ${xl430['price_usd']}")
    else:
        print("❌ 零件未找到")
    
    # 测试按类别查询
    print("\n3. 测试按类别查询...")
    servos = db.get_parts_by_category("actuator_servo")
    print(f"   找到 {len(servos)} 个舵机:")
    for servo in servos:
        print(f"     - {servo['part_id']}: {servo['model']} (${servo['price_usd']})")
    
    # 测试按制造商查询
    print("\n4. 测试按制造商查询...")
    robotis_parts = db.get_parts_by_manufacturer("ROBOTIS")
    print(f"   ROBOTIS 零件数量: {len(robotis_parts)}")
    
    # 测试获取电机规格
    print("\n5. 测试获取电机规格...")
    specs = db.get_motor_specs("dynamixel_xl430_w250")
    if specs:
        print("   电机规格:")
        print(f"     堵转扭矩: {specs['stall_torque']} N·m")
        print(f"     空载速度: {specs['no_load_speed']} RPM")
        print(f"     重量: {specs['weight']} kg")
        print(f"     最大电流: {specs['max_current']} A")
        print(f"     电压范围: {specs['voltage_range']} V")
    
    # 测试创建机器人配置
    print("\n6. 测试创建机器人配置...")
    robot_config = db.create_robot_config([
        {"part_id": "dynamixel_xl430_w250", "joint": "hip_left"},
        {"part_id": "dynamixel_xl430_w250", "joint": "hip_right"},
        {"part_id": "dynamixel_mx106", "joint": "knee_left"},
        {"part_id": "bosch_bno055", "location": "torso"}
    ])
    
    print(f"   机器人配置包含 {len(robot_config['parts'])} 个零件:")
    for part in robot_config['parts']:
        print(f"     - {part['part_id']} ({part['category']})")
    
    # 测试数据验证
    print("\n7. 测试数据验证...")
    valid_parts = [part_id for part_id in db.list_all_parts() if db.validate_part(part_id)]
    print(f"   有效零件数: {len(valid_parts)}/{len(db.list_all_parts())}")
    
    print("\n" + "=" * 60)
    print("✅ 所有测试完成!")
    print("=" * 60)


def compare_motors():
    """比较不同电机的性能"""
    print("\n" + "=" * 60)
    print("电机性能对比")
    print("=" * 60)
    
    db = PartsDatabase()
    
    motor_ids = ["dynamixel_xl430_w250", "dynamixel_mx106"]
    
    print("\n{:<25} {:<15} {:<12} {:<10}".format(
        "型号", "堵转扭矩", "空载速度", "价格"
    ))
    print("-" * 65)
    
    for motor_id in motor_ids:
        motor = db.get_part(motor_id)
        if motor:
            specs = motor['specifications']
            print("{:<25} {:>12.2f} N·m {:>10} RPM ${:>8.2f}".format(
                motor['model'],
                specs['stall_torque'],
                specs['no_load_speed'],
                motor['price_usd']
            ))
    
    print("\n性能/价格比:")
    for motor_id in motor_ids:
        motor = db.get_part(motor_id)
        if motor:
            specs = motor['specifications']
            torque_per_dollar = specs['stall_torque'] / motor['price_usd']
            print(f"  {motor['model']}: {torque_per_dollar:.4f} N·m/$")


if __name__ == "__main__":
    test_parts_database()
    compare_motors()
