# 3D可视化快速启动指南

本指南帮助您快速生成5个核心零件的3D模型和缩略图。

---

## 📋 前提条件

### 安装Blender

**Windows**:
```bash
# 下载Blender 3.6+
# https://www.blender.org/download/

# 或使用Chocolatey
choco install blender
```

**Linux**:
```bash
sudo apt install blender
```

**验证安装**:
```bash
blender --version
```

---

## 🚀 快速开始

### 步骤1: 生成3D模型

在项目根目录运行:

```bash
cd d:\新建文件夹\AGI-Walker

# 运行Blender脚本创建模型
blender --background --python blender_scripts/create_simple_models.py
```

**预期输出**:
```
============================================================
开始创建3D模型...
============================================================
✓ 已创建: godot_project/assets/models/motors/dynamixel_xl430.gltf
✓ 已创建: godot_project/assets/models/sensors/mpu6050.gltf
✓ 已创建: godot_project/assets/models/controllers/raspberry_pi4.gltf
✓ 已创建: godot_project/assets/models/joints/revolute_joint.gltf
✓ 已创建: godot_project/assets/models/power/battery_pack.gltf
============================================================
✓ 所有模型创建完成！
============================================================
```

**生成的文件**:
```
godot_project/assets/models/
├── motors/
│   └── dynamixel_xl430.gltf
├── sensors/
│   └── mpu6050.gltf
├── controllers/
│   └── raspberry_pi4.gltf
├── joints/
│   └── revolute_joint.gltf
└── power/
    └── battery_pack.gltf
```

---

### 步骤2: 渲染缩略图

```bash
# 渲染PNG缩略图
blender --background --python blender_scripts/render_thumbnails.py
```

**预期输出**:
```
============================================================
开始渲染缩略图...
============================================================
处理: godot_project/assets/models/motors/dynamixel_xl430.gltf
  ✓ 模型已导入
  ✓ 已渲染: assets/thumbnails/motor_1.png

... (其他模型)

============================================================
✓ 完成! 成功渲染 5/5 个缩略图
============================================================
```

**生成的文件**:
```
assets/thumbnails/
├── motor_1.png       # 256x256, 透明背景
├── imu_1.png
├── ctrl_1.png
├── joint_1.png
└── battery_1.png
```

---

### 步骤3: 测试GUI显示

```bash
# 启动GUI配置器
python tools/robot_configurator_gui.py
```

**查看效果**:
- 在零件库中应该能看到3D缩略图
- 拖拽到画布时显示3D图像

---

### 步骤4: 在Godot中查看

1. 打开Godot项目:
```bash
cd godot_project
godot project.godot
```

2. 在文件系统中导航到 `assets/models/`

3. 双击任意`.gltf`文件预览3D模型

---

## 🔧 自定义和调整

### 修改模型尺寸

编辑 `blender_scripts/create_simple_models.py`:

```python
def create_motor_xl430(output_path):
    # 修改半径和深度
    bpy.ops.mesh.primitive_cylinder_add(
        radius=0.020,  # 从0.014改为0.020 (更大)
        depth=0.050,   # 从0.0465改为0.050
        location=(0, 0, 0)
    )
```

### 修改材质颜色

```python
# 改变电机颜色为红色
mat = create_material(
    "Motor_Material",
    (0.8, 0.2, 0.2, 1.0),  # RGB红色
    metallic=0.7,
    roughness=0.3
)
```

### 调整缩略图视角

编辑 `blender_scripts/render_thumbnails.py`:

```python
def setup_camera(distance=0.15):
    # 修改相机位置
    bpy.ops.object.camera_add(
        location=(distance*1.5, -distance, distance*2)  # 更高的视角
    )
```

---

## 🐛 故障排查

### 问题1: Blender命令未找到

**错误**: `'blender' is not recognized`

**解决**:
```bash
# Windows: 添加Blender到PATH
# 或使用完整路径
"C:\Program Files\Blender Foundation\Blender 3.6\blender.exe" --background --python ...
```

### 问题2: 模型未生成

**检查**:
1. 查看控制台错误信息
2. 确保输出目录存在
3. 检查Python脚本语法

### 问题3: 缩略图看起来太暗

**调整光照强度**:
```python
# 在render_thumbnails.py中
key_light.data.energy = 5.0  # 从3.0增加到5.0
```

### 问题4: GUI未显示3D图像

**原因**: GUI代码尚未更新

**下一步**: 等待GUI集成代码更新

---

## 📊 验证检查清单

完成后检查:

- [ ] 5个GLTF文件已生成
- [ ] 5个PNG缩略图已渲染
- [ ] 在Godot中可以预览模型
- [ ] 缩略图具有透明背景
- [ ] 文件大小合理 (<2MB每个模型)

---

## 🎯 下一步

1. **集成到GUI** - 更新GUI代码以显示缩略图
2. **Godot加载器** - 创建动态模型加载脚本
3. **扩展模型库** - 为更多零件创建模型

---

## 📚 参考

- [Blender Python API](https://docs.blender.org/api/current/)
- [GLTF格式规范](https://www.khronos.org/gltf/)
- [设计方案](3D_VISUALIZATION_PLAN.md)

---

**预估时间**: 初次运行约10-15分钟（取决于计算机性能）

**文件大小**: 
- 模型: ~500KB-2MB 总计
- 缩略图: ~100KB 总计
