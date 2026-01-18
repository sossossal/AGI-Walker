"""
Blender自动化脚本 - 创建简化零件3D模型

使用方法:
    blender --background --python create_simple_models.py

功能:
    - 为核心零件创建简化的3D几何体
    - 应用材质和颜色
    - 导出为GLTF格式
"""

import bpy
import math
from pathlib import Path


def clear_scene():
    """清空场景"""
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()
    
    # 清除孤立数据
    for block in bpy.data.meshes:
        if block.users == 0:
            bpy.data.meshes.remove(block)


def create_material(name, color, metallic=0.5, roughness=0.5):
    """
    创建PBR材质
    
    Args:
        name: 材质名称
        color: RGBA颜色 (r, g, b, a)
        metallic: 金属度 0-1
        roughness: 粗糙度 0-1
    """
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    
    # 清除默认节点
    nodes.clear()
    
    # 创建节点
    output = nodes.new('ShaderNodeOutputMaterial')
    bsdf = nodes.new('ShaderNodeBsdfPrincipled')
    
    # 设置属性
    bsdf.inputs['Base Color'].default_value = color
    bsdf.inputs['Metallic'].default_value = metallic
    bsdf.inputs['Roughness'].default_value = roughness
    
    # 连接节点
    mat.node_tree.links.new(bsdf.outputs['BSDF'], output.inputs['Surface'])
    
    return mat


def create_motor_xl430(output_path):
    """创建Dynamixel XL430电机模型（简化版）"""
    clear_scene()
    
    # 主体（圆柱体）
    bpy.ops.mesh.primitive_cylinder_add(
        radius=0.014,  # 28mm直径
        depth=0.0465,  # 46.5mm高度
        location=(0, 0, 0)
    )
    body = bpy.context.active_object
    body.name = "Motor_Body"
    
    # 输出轴（小圆柱）
    bpy.ops.mesh.primitive_cylinder_add(
        radius=0.006,
        depth=0.01,
        location=(0, 0, 0.028)
    )
    shaft = bpy.context.active_object
    shaft.name = "Motor_Shaft"
    
    # 安装孔（装饰）
    for angle in [0, 90, 180, 270]:
        rad = math.radians(angle)
        x = 0.015 * math.cos(rad)
        y = 0.015 * math.sin(rad)
        bpy.ops.mesh.primitive_cylinder_add(
            radius=0.0015,
            depth=0.05,
            location=(x, y, 0)
        )
    
    # 合并所有对象
    bpy.ops.object.select_all(action='SELECT')
    bpy.context.view_layer.objects.active = body
    bpy.ops.object.join()
    
    # 应用材质
    mat = create_material("Motor_Material", (0.2, 0.2, 0.2, 1.0), metallic=0.7, roughness=0.3)
    body.data.materials.append(mat)
    
    # 导出GLTF
    bpy.ops.export_scene.gltf(
        filepath=output_path,
        export_format='GLTF_SEPARATE',
        export_selected=True
    )
    
    print(f"✓ 已创建: {output_path}")


def create_imu_mpu6050(output_path):
    """创建MPU6050 IMU传感器模型"""
    clear_scene()
    
    # PCB板（矩形）
    bpy.ops.mesh.primitive_cube_add(
        size=1,
        location=(0, 0, 0)
    )
    pcb = bpy.context.active_object
    pcb.scale = (0.01, 0.008, 0.001)  # 20mm x 16mm x 2mm
    pcb.name = "IMU_PCB"
    
    # 芯片（小立方体）
    bpy.ops.mesh.primitive_cube_add(
        size=1,
        location=(0, 0, 0.002)
    )
    chip = bpy.context.active_object
    chip.scale = (0.004, 0.004, 0.001)
    chip.name = "IMU_Chip"
    
    # 合并
    bpy.ops.object.select_all(action='SELECT')
    bpy.context.view_layer.objects.active = pcb
    bpy.ops.object.join()
    
    # 应用材质（绿色PCB）
    mat = create_material("PCB_Material", (0.1, 0.5, 0.2, 1.0), metallic=0.2, roughness=0.7)
    pcb.data.materials.append(mat)
    
    # 导出
    bpy.ops.export_scene.gltf(filepath=output_path, export_format='GLTF_SEPARATE')
    print(f"✓ 已创建: {output_path}")


def create_raspberry_pi4(output_path):
    """创建Raspberry Pi 4模型"""
    clear_scene()
    
    # 主板
    bpy.ops.mesh.primitive_cube_add(size=1, location=(0, 0, 0))
    board = bpy.context.active_object
    board.scale = (0.043, 0.028, 0.002)  # 85mm x 56mm x 4mm
    board.name = "RPI_Board"
    
    # USB端口（4个）
    for i in range(2):
        bpy.ops.mesh.primitive_cube_add(
            size=1,
            location=(0.02, -0.015 + i*0.01, 0.003)
        )
        usb = bpy.context.active_object
        usb.scale = (0.006, 0.007, 0.003)
    
    # 以太网端口
    bpy.ops.mesh.primitive_cube_add(
        size=1,
        location=(0.02, 0.01, 0.003)
    )
    eth = bpy.context.active_object
    eth.scale = (0.008, 0.008, 0.003)
    
    # GPIO针脚（简化为一条）
    bpy.ops.mesh.primitive_cube_add(
        size=1,
        location=(-0.015, 0, 0.003)
    )
    gpio = bpy.context.active_object
    gpio.scale = (0.025, 0.003, 0.004)
    
    # 合并
    bpy.ops.object.select_all(action='SELECT')
    bpy.context.view_layer.objects.active = board
    bpy.ops.object.join()
    
    # 应用材质（绿色主板）
    mat = create_material("Board_Material", (0.15, 0.55, 0.25, 1.0), metallic=0.2, roughness=0.6)
    board.data.materials.append(mat)
    
    # 导出
    bpy.ops.export_scene.gltf(filepath=output_path, export_format='GLTF_SEPARATE')
    print(f"✓ 已创建: {output_path}")


def create_revolute_joint(output_path):
    """创建旋转关节模型"""
    clear_scene()
    
    # 基座（圆柱）
    bpy.ops.mesh.primitive_cylinder_add(
        radius=0.012,
        depth=0.03,
        location=(0, 0, 0)
    )
    base = bpy.context.active_object
    base.name = "Joint_Base"
    
    # 轴承（环形）
    bpy.ops.mesh.primitive_torus_add(
        major_radius=0.01,
        minor_radius=0.003,
        location=(0, 0, 0.015)
    )
    bearing = bpy.context.active_object
    
    # 输出臂
    bpy.ops.mesh.primitive_cube_add(
        size=1,
        location=(0.015, 0, 0.015)
    )
    arm = bpy.context.active_object
    arm.scale = (0.02, 0.008, 0.008)
    
    # 合并
    bpy.ops.object.select_all(action='SELECT')
    bpy.context.view_layer.objects.active = base
    bpy.ops.object.join()
    
    # 应用材质（金属灰）
    mat = create_material("Joint_Material", (0.3, 0.3, 0.3, 1.0), metallic=0.8, roughness=0.4)
    base.data.materials.append(mat)
    
    # 导出
    bpy.ops.export_scene.gltf(filepath=output_path, export_format='GLTF_SEPARATE')
    print(f"✓ 已创建: {output_path}")


def create_battery_pack(output_path):
    """创建电池包模型"""
    clear_scene()
    
    # 电池盒（长方体）
    bpy.ops.mesh.primitive_cube_add(size=1, location=(0, 0, 0))
    box = bpy.context.active_object
    box.scale = (0.025, 0.045, 0.015)  # 50mm x 90mm x 30mm
    box.name = "Battery_Box"
    
    # 正极端子
    bpy.ops.mesh.primitive_cylinder_add(
        radius=0.002,
        depth=0.003,
        location=(0.01, 0, 0.008)
    )
    pos_terminal = bpy.context.active_object
    
    # 负极端子
    bpy.ops.mesh.primitive_cylinder_add(
        radius=0.002,
        depth=0.003,
        location=(-0.01, 0, 0.008)
    )
    neg_terminal = bpy.context.active_object
    
    # 合并
    bpy.ops.object.select_all(action='SELECT')
    bpy.context.view_layer.objects.active = box
    bpy.ops.object.join()
    
    # 应用材质（蓝色电池）
    mat = create_material("Battery_Material", (0.1, 0.3, 0.7, 1.0), metallic=0.4, roughness=0.5)
    box.data.materials.append(mat)
    
    # 导出
    bpy.ops.export_scene.gltf(filepath=output_path, export_format='GLTF_SEPARATE')
    print(f"✓ 已创建: {output_path}")


def main():
    """主函数 - 创建所有核心零件模型"""
    
    # 输出目录
    output_dir = Path("../godot_project/assets/models")
    
    # 创建子目录
    (output_dir / "motors").mkdir(parents=True, exist_ok=True)
    (output_dir / "sensors").mkdir(parents=True, exist_ok=True)
    (output_dir / "controllers").mkdir(parents=True, exist_ok=True)
    (output_dir / "joints").mkdir(parents=True, exist_ok=True)
    (output_dir / "power").mkdir(parents=True, exist_ok=True)
    
    print("=" * 60)
    print("开始创建3D模型...")
    print("=" * 60)
    
    # 创建各个模型
    create_motor_xl430(str(output_dir / "motors/dynamixel_xl430.gltf"))
    create_imu_mpu6050(str(output_dir / "sensors/mpu6050.gltf"))
    create_raspberry_pi4(str(output_dir / "controllers/raspberry_pi4.gltf"))
    create_revolute_joint(str(output_dir / "joints/revolute_joint.gltf"))
    create_battery_pack(str(output_dir / "power/battery_pack.gltf"))
    
    print("=" * 60)
    print("✓ 所有模型创建完成！")
    print("=" * 60)


if __name__ == "__main__":
    main()
