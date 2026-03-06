# AGI-Walker 3D可视化增强方案

**版本**: 1.0  
**日期**: 2026-01-18  
**目标**: 为所有零件添加3D模型，统一GUI和Godot的视觉体验

---

## 📋 概述

### 当前状态

**GUI配置器**:
- ✅ 2D矩形表示零件
- ✅ 文本标签显示名称
- ⚠️ 缺少视觉细节

**Godot仿真**:
- ✅ 基础物理碰撞体
- ⚠️ 简单形状（立方体、球体）
- ⚠️ 缺少真实外观

### 目标

创建统一的3D可视化系统：
- 🎨 GUI中显示3D零件缩略图/预览
- 🎮 Godot中使用详细3D模型
- 🔄 保持视觉一致性
- 📦 35+零件的完整模型库

---

## 🎯 实施方案

### 方案A: 渐进式实现（推荐）⭐

#### 阶段1: 基础3D库（1-2周）
创建简化的3D模型用于快速实现

#### 阶段2: GUI 3D预览（1-2周）
在GUI中集成3D渲染

#### 阶段3: Godot详细模型（2-3周）
在Godot中使用高质量模型

#### 阶段4: 优化和统一（1周）
统一材质和风格

**总时间**: 5-8周

### 方案B: 快速原型（最小可行）

仅为关键零件（5-10个）创建模型
**总时间**: 1-2周

---

## 🛠️ 技术实现

### 1. 3D模型格式

#### GUI预览
**格式**: PNG/WebP缩略图（预渲染）
- 使用Blender预渲染各角度视图
- 轻量级，加载快速
- 适合Tkinter显示

**备选**: Matplotlib 3D（实时渲染）
```python
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
```

#### Godot仿真
**格式**: GLTF 2.0 / FBX
- 支持物理碰撞体
- 材质和纹理
- 优化的多边形数

### 2. 零件3D模型规范

#### 模型要求
```yaml
零件类别: [电机, 传感器, 控制器, 关节, 电池]

每个模型:
  - 格式: GLTF 2.0
  - 多边形数: <5000 (低模)
  - 尺寸: 实际物理尺寸（米）
  - 原点: 质心或安装点
  - 材质: PBR材质
  - LOD: 可选（高中低三个级别）
```

#### 示例：Dynamixel XL430电机
```
模型文件: models/motors/dynamixel_xl430.gltf
尺寸: 28.5mm × 46.5mm × 34mm
多边形: 2,847
材质: 金属+塑料
碰撞体: 简化立方体
```

---

## 📐 GUI 3D预览实现

### 选项1: 预渲染缩略图（推荐）⭐

**优势**:
- 快速加载
- 低CPU消耗
- 易于实现

**实现**:
```python
# tools/render_part_thumbnails.py

import bpy  # Blender Python API
from pathlib import Path

def render_part_thumbnail(model_path, output_path, angle=45):
    """
    使用Blender渲染零件缩略图
    
    Args:
        model_path: 3D模型路径
        output_path: 输出图片路径
        angle: 视角（度）
    """
    # 导入模型
    bpy.ops.import_scene.gltf(filepath=model_path)
    
    # 设置相机
    camera = bpy.data.objects['Camera']
    camera.location = (2, -2, 2)
    camera.rotation_euler = (math.radians(60), 0, math.radians(45))
    
    # 设置光照
    light = bpy.data.lights.new(name="Key Light", type='SUN')
    light.energy = 5.0
    
    # 渲染设置
    bpy.context.scene.render.resolution_x = 256
    bpy.context.scene.render.resolution_y = 256
    bpy.context.scene.render.image_settings.file_format = 'PNG'
    
    # 渲染
    bpy.context.scene.render.filepath = output_path
    bpy.ops.render.render(write_still=True)

# 批量渲染所有零件
for part in parts_library:
    render_part_thumbnail(
        f"models/{part.category}/{part.model}.gltf",
        f"assets/thumbnails/{part.id}.png"
    )
```

**在GUI中使用**:
```python
# tools/robot_configurator_gui.py

from PIL import Image, ImageTk

class PartNode:
    def __init__(self, canvas, part_id, part_data, x, y):
        # 加载3D缩略图
        thumbnail_path = f"assets/thumbnails/{part_id}.png"
        if Path(thumbnail_path).exists():
            img = Image.open(thumbnail_path)
            img = img.resize((60, 60), Image.LANCZOS)
            self.photo = ImageTk.PhotoImage(img)
            
            # 显示图片而非矩形
            self.image_obj = canvas.create_image(
                x + 30, y + 30,
                image=self.photo,
                tags=('part', part_id)
            )
        else:
            # 降级到2D矩形
            self.rect = canvas.create_rectangle(...)
```

### 选项2: Matplotlib 3D实时渲染

**优势**:
- 可交互旋转
- 动态更新

**劣势**:
- 较慢
- 占用CPU

**实现**:
```python
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def create_3d_preview_widget(parent, part_data):
    """创建3D预览窗口"""
    fig = plt.Figure(figsize=(3, 3))
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制简化模型（立方体示例）
    vertices = np.array([...])  # 从模型文件加载
    faces = [...]
    
    collection = Poly3DCollection(faces, alpha=0.8)
    ax.add_collection3d(collection)
    
    # 嵌入到Tkinter
    canvas = FigureCanvasTkAgg(fig, parent)
    canvas.get_tk_widget().pack()
```

---

## 🎮 Godot 3D模型集成

### 1. 模型导入

**目录结构**:
```
godot_project/
├── assets/
│   ├── models/
│   │   ├── motors/
│   │   │   ├── dynamixel_xl430.gltf
│   │   │   ├── dynamixel_ax12.gltf
│   │   │   └── ...
│   │   ├── sensors/
│   │   │   ├── mpu6050.gltf
│   │   │   └── ...
│   │   └── controllers/
│   │       └── raspberry_pi4.gltf
│   └── materials/
│       ├── metal_brushed.tres
│       └── plastic_black.tres
└── scripts/
    └── part_loader.gd
```

### 2. 动态加载脚本

**文件**: `godot_project/scripts/part_loader.gd`

```gdscript
extends Node3D
class_name PartLoader

# 零件模型缓存
var model_cache := {}

# 加载零件3D模型
func load_part(part_id: String, part_type: String) -> Node3D:
    var model_path = "res://assets/models/%s/%s.gltf" % [part_type, part_id]
    
    # 检查缓存
    if model_cache.has(model_path):
        return model_cache[model_path].duplicate()
    
    # 加载模型
    if ResourceLoader.exists(model_path):
        var scene = load(model_path)
        var instance = scene.instantiate()
        model_cache[model_path] = instance
        return instance.duplicate()
    else:
        # 降级到简单形状
        return create_simple_shape(part_type)

# 简单形状作为降级方案
func create_simple_shape(part_type: String) -> Node3D:
    var shape = MeshInstance3D.new()
    
    match part_type:
        "motor":
            shape.mesh = CylinderMesh.new()
            shape.mesh.height = 0.05
            shape.mesh.radius = 0.02
        "sensor":
            shape.mesh = BoxMesh.new()
            shape.mesh.size = Vector3(0.02, 0.02, 0.01)
        "controller":
            shape.mesh = BoxMesh.new()
            shape.mesh.size = Vector3(0.08, 0.06, 0.02)
        _:
            shape.mesh = SphereMesh.new()
    
    return shape

# 添加物理碰撞体
func add_collision_shape(part: Node3D, collision_data: Dictionary):
    var collision = CollisionShape3D.new()
    
    match collision_data.get("type", "box"):
        "box":
            var shape = BoxShape3D.new()
            shape.size = Vector3(
                collision_data.get("width", 0.05),
                collision_data.get("height", 0.05),
                collision_data.get("depth", 0.05)
            )
            collision.shape = shape
        "sphere":
            var shape = SphereShape3D.new()
            shape.radius = collision_data.get("radius", 0.025)
            collision.shape = shape
        "cylinder":
            var shape = CylinderShape3D.new()
            shape.height = collision_data.get("height", 0.05)
            shape.radius = collision_data.get("radius", 0.02)
            collision.shape = shape
    
    part.add_child(collision)
```

### 3. TCP服务器集成

更新`TCPSimulationServer.gd`以支持3D模型：

```gdscript
func handle_load_robot(data):
    var parts = data.get("parts", [])
    
    for part_info in parts:
        # 加载3D模型
        var part_node = part_loader.load_part(
            part_info.part_id,
            part_info.part_type
        )
        
        # 设置位置
        part_node.position = Vector3(
            part_info.position[0],
            part_info.position[1],
            part_info.position[2]
        )
        
        # 添加碰撞体
        if part_info.has("collision"):
            part_loader.add_collision_shape(
                part_node,
                part_info.collision
            )
        
        # 添加到场景
        robot_container.add_child(part_node)
```

---

## 🎨 视觉一致性策略

### 1. 统一材质系统

**Godot材质** (`.tres`文件):
```gdscript
# assets/materials/motor_body.tres
[resource]
type = "StandardMaterial3D"
albedo_color = Color(0.2, 0.2, 0.2, 1.0)  # 深灰
metallic = 0.8
roughness = 0.3
```

**Blender渲染设置** (缩略图):
- 相同的颜色值
- 相似的光照角度
- 一致的背景

### 2. 风格指南

所有模型遵循：
- **配色**: 工业灰、黑、金属色
- **细节**: 中等细节（非照片级）
- **比例**: 精确的物理尺寸
- **标准视角**: 45度等距视图

### 3. 图标规范

GUI缩略图：
```
尺寸: 256×256 (渲染) → 64×64 (显示)
背景: 透明或浅灰
光照: 3点光照（主光+补光+轮廓光）
视角: 45度俯视
```

---

## 📦 3D资产创建工作流

### 工具链

1. **Blender 3.6+** - 主要建模工具
2. **MeshLab** - 简化和优化
3. **gltf-pipeline** - 格式转换和压缩

### 创建流程

```bash
# 1. 在Blender中建模
blender --background --python create_motor_model.py

# 2. 导出到GLTF
# (在Blender中: File → Export → glTF 2.0)

# 3. 优化模型
gltf-pipeline -i input.gltf -o output.gltf -d

# 4. 渲染缩略图
blender --background --python render_thumbnail.py -- model.gltf

# 5. 复制到项目
cp output.gltf godot_project/assets/models/motors/
cp thumbnail.png assets/thumbnails/
```

### 批量处理脚本

**文件**: `tools/batch_create_models.sh`

```bash
#!/bin/bash
# 批量创建所有零件的3D模型

PARTS_CSV="parts_library/parts_specs.csv"

while IFS=, read -r id type model; do
    echo "Processing: $id ($model)"
    
    # 1. 创建模型（如果模板存在）
    if [ -f "blender_templates/${type}_template.blend" ]; then
        blender --background \
            blender_templates/${type}_template.blend \
            --python scripts/customize_model.py \
            -- --part-id "$id" --model "$model"
    fi
    
    # 2. 渲染缩略图
    blender --background \
        --python scripts/render_thumbnail.py \
        -- --input "output/${id}.gltf" \
           --output "assets/thumbnails/${id}.png"
    
done < "$PARTS_CSV"
```

---

## 🚀 实施步骤

### Phase 1: 准备工作（3-5天）

- [ ] 安装Blender和工具
- [ ] 创建模型规范文档
- [ ] 建立Blender模板
- [ ] 设置材质库

### Phase 2: 核心零件建模（1-2周）

优先级高的零件（10个）:
- [ ] Dynamixel XL430
- [ ] Dynamixel AX-12
- [ ] MPU6050
- [ ] Raspberry Pi 4
- [ ] 基础关节×2
- [ ] 电池×2
- [ ] 编码器×2

### Phase 3: GUI集成（1周）

- [ ] 实现缩略图加载
- [ ] 更新PartNode类
- [ ] 添加3D预览窗口（可选）
- [ ] 测试性能

### Phase 4: Godot集成（1-2周）

- [ ] 导入所有GLTF模型
- [ ] 实现part_loader.gd
- [ ] 更新TCP服务器
- [ ] 配置物理碰撞体
- [ ] 测试渲染性能

### Phase 5: 扩展剩余零件（1-2周）

- [ ] 完成所有35+零件
- [ ] 优化模型
- [ ] 统一视觉风格

### Phase 6: 优化和完善（1周）

- [ ] 性能优化
- [ ] LOD系统（可选）
- [ ] 文档更新
- [ ] 用户测试

**总时间**: 5-8周

---

## 📊 资源需求

### 存储空间

```
估算:
- 每个GLTF模型: 100KB - 2MB
- 每个PNG缩略图: 10-50KB
- 35个零件总计: ~50-100MB
```

### 性能影响

**GUI**:
- 缩略图方式: 几乎无影响
- 实时3D: +20-50% CPU

**Godot**:
- 低模版本: 可接受
- 高模版本: 需要LOD优化

---

## 💡 快速实现方案（推荐）

如果需要快速看到效果，建议：

### 最小可行产品（1周）

1. **选择5个代表性零件**:
   - Dynamixel XL430（电机）
   - MPU6050（传感器）
   - Raspberry Pi 4（控制器）
   - 旋转关节
   - 电池包

2. **使用简化模型**:
   - 基础几何体组合
   - 单一材质
   - 无复杂纹理

3. **GUI预渲染缩略图**:
   - 快速在Blender中渲染
   - 直接替换现有矩形

4. **Godot基础形状**:
   - 先用程序化几何体
   - 后续替换为详细模型

---

## 🎯 预期效果

### GUI效果
```
之前: 蓝色矩形 + 文本标签
之后: 真实3D零件缩略图 + 文本标签
```

### Godot效果
```
之前: 简单立方体/球体
之后: 详细3D模型 + 真实外观
```

### 用户体验
- ✅ 更直观的零件识别
- ✅ 更专业的视觉效果
- ✅ GUI和仿真的一致性
- ✅ 更容易理解机器人结构

---

## ⚠️ 注意事项

### 1. 版权问题
- 使用原创模型或开源资源
- 避免直接复制商业模型
- 注明模型来源

### 2. 性能考虑
- 保持低多边形数
- 使用纹理而非几何细节
- 实现LOD系统（可选）

### 3. 维护成本
- 新零件需要添加模型
- 更新模型需要同步GUI和Godot

---

## 📚 参考资源

### 学习资源
- [Blender官方文档](https://docs.blender.org/)
- [GLTF 2.0规范](https://www.khronos.org/gltf/)
- [Godot 3D教程](https://docs.godotengine.org/en/stable/tutorials/3d/)

### 模型资源
- [Sketchfab](https://sketchfab.com/) - 3D模型库
- [TurboSquid](https://www.turbosquid.com/) - 专业模型
- [OpenRoboticsAssets](https://fuel.gazebosim.org/) - 机器人零件

---

**建议**: 先用**快速实现方案**（1周）创建5个核心零件的3D模型，验证效果后再决定是否全面推进。

**下一步**: 
1. 确认是否要实施此方案
2. 决定采用完整版还是快速版
3. 我可以开始创建Blender脚本和模板
