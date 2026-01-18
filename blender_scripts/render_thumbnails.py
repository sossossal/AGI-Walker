"""
Blender缩略图渲染脚本

使用方法:
    blender --background --python render_thumbnails.py

功能:
    - 加载GLTF模型
    - 设置相机和光照
    - 渲染PNG缩略图
"""

import bpy
import math
from pathlib import Path


def setup_render_settings():
    """配置渲染设置"""
    scene = bpy.context.scene
    
    # 分辨率
    scene.render.resolution_x = 256
    scene.render.resolution_y = 256
    scene.render.resolution_percentage = 100
    
    # 输出格式
    scene.render.image_settings.file_format = 'PNG'
    scene.render.image_settings.color_mode = 'RGBA'  # 透明背景
    
    # 渲染引擎
    scene.render.engine = 'CYCLES'
    scene.cycles.samples = 64  # 采样数
    scene.cycles.use_denoising = True
    
    # 透明背景
    scene.render.film_transparent = True
    
    print("✓ 渲染设置已配置")


def clear_scene():
    """清空场景"""
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()
    
    # 清除灯光和相机
    for light in bpy.data.lights:
        bpy.data.lights.remove(light)
    for camera in bpy.data.cameras:
        bpy.data.cameras.remove(camera)


def setup_lighting():
    """设置3点光照"""
    # 主光（Key Light）
    bpy.ops.object.light_add(type='SUN', location=(5, -5, 10))
    key_light = bpy.context.active_object
    key_light.data.energy = 3.0
    key_light.rotation_euler = (math.radians(45), 0, math.radians(45))
    
    # 补光（Fill Light）
    bpy.ops.object.light_add(type='SUN', location=(-5, -5, 5))
    fill_light = bpy.context.active_object
    fill_light.data.energy = 1.5
    fill_light.rotation_euler = (math.radians(60), 0, math.radians(-45))
    
    # 轮廓光（Rim Light）
    bpy.ops.object.light_add(type='SUN', location=(0, 5, 3))
    rim_light = bpy.context.active_object
    rim_light.data.energy = 2.0
    rim_light.rotation_euler = (math.radians(120), 0, 0)
    
    print("✓ 光照已设置")


def setup_camera(distance=0.15):
    """
    设置相机（45度等距视图）
    
    Args:
        distance: 相机到原点的距离（米）
    """
    # 创建相机
    bpy.ops.object.camera_add(location=(distance, -distance, distance))
    camera = bpy.context.active_object
    
    # 指向原点
    direction = (-1, 1, -1)  # 45度角
    camera.rotation_euler = (
        math.radians(63.4),  # X旋转
        0,
        math.radians(45)     # Z旋转
    )
    
    # 设置为活动相机
    bpy.context.scene.camera = camera
    
    # 正交相机（可选，更适合技术图）
    # camera.data.type = 'ORTHO'
    # camera.data.ortho_scale = 0.1
    
    print("✓ 相机已设置")


def render_thumbnail(model_path, output_path):
    """
    渲染单个模型的缩略图
    
    Args:
        model_path: GLTF模型路径
        output_path: 输出PNG路径
    """
    print(f"\n处理: {model_path}")
    
    # 清空场景
    clear_scene()
    
    # 导入模型
    try:
        bpy.ops.import_scene.gltf(filepath=str(model_path))
        print(f"  ✓ 模型已导入")
    except Exception as e:
        print(f"  ✗ 导入失败: {e}")
        return False
    
    # 选择导入的对象并居中
    imported_objects = [obj for obj in bpy.context.scene.objects if obj.type == 'MESH']
    if not imported_objects:
        print(f"  ✗ 未找到网格对象")
        return False
    
    # 计算包围盒以确定相机距离
    all_coords = []
    for obj in imported_objects:
        for vertex in obj.data.vertices:
            all_coords.append(obj.matrix_world @ vertex.co)
    
    if all_coords:
        max_dimension = max([
            max(co[i] for co in all_coords) - min(co[i] for co in all_coords)
            for i in range(3)
        ])
        camera_distance = max_dimension * 2.5  # 适当的距离
    else:
        camera_distance = 0.15
    
    # 设置光照和相机
    setup_lighting()
    setup_camera(camera_distance)
    
    # 渲染
    bpy.context.scene.render.filepath = str(output_path)
    bpy.ops.render.render(write_still=True)
    
    print(f"  ✓ 已渲染: {output_path}")
    return True


def render_all_models():
    """渲染所有核心模型的缩略图"""
    
    # 模型和输出路径映射
    models = [
        ("../godot_project/assets/models/motors/dynamixel_xl430.gltf", 
         "../assets/thumbnails/motor_1.png"),
        ("../godot_project/assets/models/sensors/mpu6050.gltf",
         "../assets/thumbnails/imu_1.png"),
        ("../godot_project/assets/models/controllers/raspberry_pi4.gltf",
         "../assets/thumbnails/ctrl_1.png"),
        ("../godot_project/assets/models/joints/revolute_joint.gltf",
         "../assets/thumbnails/joint_1.png"),
        ("../godot_project/assets/models/power/battery_pack.gltf",
         "../assets/thumbnails/battery_1.png"),
    ]
    
    # 配置渲染
    setup_render_settings()
    
    # 创建输出目录
    Path("../assets/thumbnails").mkdir(parents=True, exist_ok=True)
    
    print("=" * 60)
    print("开始渲染缩略图...")
    print("=" * 60)
    
    success_count = 0
    for model_path, output_path in models:
        if Path(model_path).exists():
            if render_thumbnail(model_path, output_path):
                success_count += 1
        else:
            print(f"\n✗ 模型不存在: {model_path}")
    
    print("\n" + "=" * 60)
    print(f"✓ 完成! 成功渲染 {success_count}/{len(models)} 个缩略图")
    print("=" * 60)


def main():
    render_all_models()


if __name__ == "__main__":
    main()
