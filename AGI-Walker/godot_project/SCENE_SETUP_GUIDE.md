# Godot场景搭建详细指南

本文档提供逐步指导，帮助您在Godot编辑器中创建盒子机器人仿真场景。

## 📋 前置准备

1. 已安装 Godot 4.2+
2. 已将项目文件夹 `AGI-Walker/godot_project` 导入Godot

---

## 🎬 步骤1: 打开项目

1. 启动Godot编辑器
2. 点击"导入"
3. 选择 `d:\新建文件夹\AGI-Walker\godot_project\project.godot`
4. 点击"导入并编辑"

---

## 🏗️ 步骤2: 创建主场景

### 2.1 创建场景根节点
1. 在场景面板点击"其他节点"（或按 Ctrl+A）
2. 搜索并选择 `Node3D`
3. 重命名为 `Main`
4. 保存场景为 `res://scenes/main.tscn`

---

## 🌍 步骤3: 添加地面

### 3.1 创建地面节点
1. 右键点击 `Main` → 添加子节点
2. 搜索 `StaticBody3D`
3. 重命名为 `Ground`

### 3.2 添加碰撞形状
1. 右键点击 `Ground` → 添加子节点
2. 选择 `CollisionShape3D`
3. 在检查器中，点击 Shape 的空白处
4. 选择 `新建 BoxShape3D`
5. 展开 Shape，设置 Size:
   - X: `20`
   - Y: `0.5`
   - Z: `20`

### 3.3 添加视觉网格
1. 右键点击 `Ground` → 添加子节点
2. 选择 `MeshInstance3D`
3. 在检查器中，点击 Mesh 的空白处
4. 选择 `新建 BoxMesh`
5. 展开 Mesh，设置 Size:
   - X: `20`
   - Y: `0.5`
   - Z: `20`

### 3.4 设置地面材质（可选）
1. 在 MeshInstance3D 的 Mesh 属性中
2. 展开 Material，点击"新建 StandardMaterial3D"
3. 设置颜色为灰色或棕色

---

## 🤖 步骤4: 创建盒子机器人

### 4.1 创建机器人根节点
1. 右键点击 `Main` → 添加子节点
2. 选择 `Node3D`
3. 重命名为 `Robot`
4. **附加脚本**: 在检查器中点击"附加脚本"
   - 路径: `res://scripts/box_robot.gd`
   - 点击"加载"
5. 设置 Transform → Position:
   - X: `0`
   - Y: `1.5`（离地面1.5米）
   - Z: `0`

---

### 4.2 创建躯干 (Torso)

1. 右键点击 `Robot` → 添加子节点
2. 选择 `RigidBody3D`
3. 重命名为 `Torso`
4. 设置属性:
   - Mass (质量): `10`
   - Gravity Scale: `1`

#### 4.2.1 躯干碰撞形状
1. 右键点击 `Torso` → 添加子节点
2. 选择 `CollisionShape3D`
3. Shape → 新建 `BoxShape3D`
4. Size:
   - X: `0.5`
   - Y: `1.0`
   - Z: `0.3`

#### 4.2.2 躯干网格
1. 右键点击 `Torso` → 添加子节点
2. 选择 `MeshInstance3D`
3. Mesh → 新建 `BoxMesh`
4. Size:
   - X: `0.5`
   - Y: `1.0`
   - Z: `0.3`
5. Material → 新建材质，设置颜色为蓝色

---

### 4.3 创建左腿 (LeftLeg)

1. 右键点击 `Robot` → 添加子节点
2. 选择 `RigidBody3D`
3. 重命名为 `LeftLeg`
4. 设置属性:
   - Mass: `2`
5. Transform → Position:
   - X: `-0.3`（躯干左侧）
   - Y: `-0.5`（躯干下方）
   - Z: `0`

#### 4.3.1 左腿碰撞和网格
重复4.2.1和4.2.2的步骤，但尺寸改为:
- X: `0.2`
- Y: `0.8`
- Z: `0.2`
- 材质颜色: 红色

---

### 4.4 创建右腿 (RightLeg)

1. 重复4.3的步骤
2. 重命名为 `RightLeg`
3. Position:
   - X: `0.3`（躯干右侧）
   - Y: `-0.5`
   - Z: `0`
4. 材质颜色: 绿色

---

### 4.5 创建左髋关节 (HipLeft)

1. 右键点击 `Robot` → 添加子节点
2. 选择 `HingeJoint3D`
3. 重命名为 `HipLeft`
4. Transform → Position:
   - X: `-0.25`
   - Y: `0`
   - Z: `0`

#### 4.5.1 配置关节连接
1. 在检查器中找到 `Node A` 和 `Node B`:
   - Node A: 选择 `../Torso`（点击文件夹图标）
   - Node B: 选择 `../LeftLeg`

#### 4.5.2 配置关节限位
1. 展开 `Angular Limit`
2. 勾选 `Enabled`
3. 设置:
   - Upper (上限): `90` 度
   - Lower (下限): `-45` 度

#### 4.5.3 启用电机（关键步骤！）
1. 展开 `Motor`
2. 勾选 `Enabled`
3. Target Velocity: `0`
4. Max Impulse: `500`

---

### 4.6 创建右髋关节 (HipRight)

1. 重复4.5的步骤
2. 重命名为 `HipRight`
3. Position:
   - X: `0.25`（右侧）
   - Y: `0`
   - Z: `0`
4. Node B 改为: `../RightLeg`

---

## 📷 步骤5: 添加相机和光源

### 5.1 添加相机
1. 右键点击 `Main` → 添加子节点
2. 选择 `Camera3D`
3. Transform → Position:
   - X: `5`
   - Y: `3`
   - Z: `5`
4. Transform → Rotation:
   - 在3D视图中调整相机朝向机器人
   - 或点击相机节点，按 Ctrl+Alt+F (对齐到视图)

### 5.2 添加光源
1. 右键点击 `Main` → 添加子节点
2. 选择 `DirectionalLight3D`
3. Transform → Rotation:
   - X: `-45` 度
   - Y: `30` 度
   - Z: `0`

---

## 🔌 步骤6: 添加TCP服务器

1. 右键点击 `Main` → 添加子节点
2. 选择 `Node`
3. 重命名为 `TCPServer`
4. 附加脚本: `res://scripts/tcp_server.gd`

---

## ✅ 步骤7: 检查场景结构

最终的场景树应该如下:

```
Main (Node3D)
├── Ground (StaticBody3D)
│   ├── CollisionShape3D
│   └── MeshInstance3D
│
├── Robot (Node3D) [脚本: box_robot.gd]
│   ├── Torso (RigidBody3D)
│   │   ├── CollisionShape3D
│   │   └── MeshInstance3D
│   ├── LeftLeg (RigidBody3D)
│   │   ├── CollisionShape3D
│   │   └── MeshInstance3D
│   ├── RightLeg (RigidBody3D)
│   │   ├── CollisionShape3D
│   │   └── MeshInstance3D
│   ├── HipLeft (HingeJoint3D)
│   └── HipRight (HingeJoint3D)
│
├── Camera3D
├── DirectionalLight3D
└── TCPServer (Node) [脚本: tcp_server.gd]
```

---

## 🚀 步骤8: 测试运行

1. 保存场景 (Ctrl+S)
2. 在菜单栏: `项目 → 项目设置 → Application → Run`
3. 设置 `Main Scene` 为 `res://scenes/main.tscn`
4. 关闭项目设置
5. 按 `F5` 运行项目

### 预期结果:
- 控制台显示: `✅ TCP服务器已启动: 127.0.0.1:9999`
- 看到盒子机器人站在地面上
- 机器人会因为重力落下（这是正常的）

---

## 🐛 常见问题

### 问题1: 机器人穿透地面
**解决**:
- 确保 Ground 是 `StaticBody3D`
- 确保 CollisionShape3D 的 Shape 已设置

### 问题2: 关节不动
**解决**:
- 确认 HingeJoint3D 的 Node A 和 Node B 已正确设置
- 确认电机已启用 (Motor → Enabled)

### 问题3: 机器人爆炸/抖动
**解决**:
- 降低电机的 Max Impulse (从500降到100)
- 增加物理帧率: `项目设置 → Physics → Common → Physics Ticks Per Second` 设为 120

### 问题4: TCP连接失败
**解决**:
- 确认 TCPServer 节点已添加脚本
- 查看控制台是否有错误信息
- 确认端口9999未被占用

---

## 📝 下一步

场景创建完成后:

1. 运行 Python 客户端测试:
   ```bash
   cd python_controller
   python tcp_client.py
   ```

2. 运行延迟测试:
   ```bash
   python test_latency.py
   ```

3. 如果一切正常，可以继续集成AI控制器!

---

## 💡 提示

- 使用 `F6` 运行当前场景（不需要设置为主场景）
- 使用 `F8` 暂停/继续物理模拟
- 在3D视图中按住鼠标中键旋转视角
- 按 `W` 启用移动模式，`E` 启用旋转模式

祝您搭建顺利！🎉
