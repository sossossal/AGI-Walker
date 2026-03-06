# 🎯 实践指导 - 第一步

让我们通过实际操作来体验系统的功能！

---

## 📝 实践清单

- [ ] **Step 1**: Python 零件库测试（5分钟）
- [ ] **Step 2**: Python Gym 环境测试（5分钟）
- [ ] **Step 3**: Godot 测试场景创建（15分钟）
- [ ] **Step 4**: 环境参数实验（10分钟）

---

## Step 1: Python 零件库测试 ✅

### 运行测试

打开 PowerShell，执行：

```powershell
cd d:\新建文件夹\AGI-Walker\python_api
python examples\demo_parts.py
```

### 预期结果

```
============================================================
机器人零件库演示
============================================================

[1] 加载零件数据库...
    ✅ 成功加载 3 个零件
    零件列表: dynamixel_xl430_w250, dynamixel_mx106, bosch_bno055

[2] 获取 Dynamixel XL430-W250 详情...
    型号: XL430-W250-T
    制造商: ROBOTIS
    堵转扭矩: 1.4 N·m
    空载速度: 50 RPM
    重量: 0.057 kg
    价格: $69.90

[3] 创建 4-DOF 步行机器人配置...
    ✅ 机器人配置创建成功，包含 4 个零件

[4] 验证数据完整性...
    ✅ 3/3 个零件数据有效

============================================================
✅ 所有测试通过！零件库功能正常
============================================================
```

**✅ 如果看到这个输出，Step 1 完成！**

---

## Step 2: Python Gym 环境测试 ✅

### 运行测试

```powershell
# 设置 Python 路径
$env:PYTHONPATH = "d:\新建文件夹\AGI-Walker\python_api"

# 运行测试
python examples\test_gym_env.py
```

### 预期结果

```
============================================================
Gymnasium 环境测试（离线模式）
============================================================

[1] 加载零件数据库...
    ✅ 成功加载 3 个零件

[2] 创建机器人配置...
    ✅ 配置包含 4 个零件

[3] 创建 Gym 环境...
    ✅ 环境创建成功

[4] 观察空间:
    类型: <class 'gymnasium.spaces.dict.Dict'>
    包含:
      - imu_orient: (3,) (float32)
      - imu_angular_vel: (3,) (float32)
      - imu_linear_acc: (3,) (float32)
      - joint_angles: (4,) (float32)
      - joint_velocities: (4,) (float32)
      - joint_torques: (4,) (float32)
      - foot_contacts: (2,) (int8)
      - torso_height: (1,) (float32)

[5] 动作空间:
    类型: <class 'gymnasium.spaces.box.Box'>
    形状: (4,)
    范围: [-45. -45. -120. -120.] 到 [90. 90. 0. 0.]

[6] 测试奖励函数...
    示例奖励: 0.150

[7] 测试终止条件...
    正常姿态终止: False ✅
    倾倒姿态终止: True ✅
    低高度终止: True ✅

============================================================
✅ 所有测试通过！
============================================================
```

**✅ 如果看到这个输出，Step 2 完成！**

---

## Step 3: Godot 测试场景创建

### 3.1 启动 Godot

1. 打开 Godot Engine 4.2+
2. 点击 "导入"
3. 浏览到 `d:\新建文件夹\AGI-Walker\godot_project\project.godot`
4. 点击 "导入并编辑"

### 3.2 启用插件

1. 菜单：`项目` → `项目设置`
2. 选择 `插件` 标签
3. 找到 **"Robot Simulation Toolkit"**
4. 勾选 "启用"
5. 关闭设置窗口

**控制台应该显示**:
```
Robot Simulation Toolkit plugin activated
```

### 3.3 创建测试场景

#### A. 新建场景

1. `场景` → `新建场景`
2. 选择 `3D Scene` （或点击 "3D 场景"）
3. 根节点重命名为 `TestEnvironment`

#### B. 添加环境控制器

1. 右键点击 `TestEnvironment`
2. 选择 `添加子节点`
3. 搜索 `Node`
4. 添加后重命名为 `EnvironmentController`
5. 选中 `EnvironmentController`
6. 在检查器中点击 `附加脚本`
7. 选择 `res://scripts/environment/environment_controller.gd`
8. 点击 "创建"

#### C. 添加材质库

1. 右键点击 `TestEnvironment`
2. `添加子节点` → `Node`
3. 重命名为 `GroundMaterialLibrary`
4. 附加脚本：`res://scripts/environment/ground_material_library.gd`

#### D. 创建地面

1. 右键点击 `TestEnvironment`
2. `添加子节点` → `StaticBody3D`
3. 重命名为 `Ground`
4. 选中 `Ground`，右键 → `添加子节点` → `CollisionShape3D`
5. 选中 `CollisionShape3D`
   - 在检查器的 `Shape` 属性，点击 `<空>` → `新建 BoxShape3D`
   - 点击 `BoxShape3D`，设置 `Size` 为 `(20, 1, 20)`
6. 再添加 `MeshInstance3D` 到 `Ground`
   - `Mesh` → `新建 BoxMesh`
   - 设置 `Size` 为 `(20, 1, 20)`

#### E. 添加测试物体（可选）

1. 右键点击 `TestEnvironment`
2. `添加子节点` → `RigidBody3D`
3. 重命名为 `TestCube`
4. 位置设置为 `(0, 5, 0)`
5. 添加 `CollisionShape3D` 和 `MeshInstance3D`
   - Shape: `BoxShape3D` (1, 1, 1)
   - Mesh: `BoxMesh` (1, 1, 1)

#### F. 添加相机和光源

1. 添加 `Camera3D`
   - 位置: `(0, 10, 15)`
   - 旋转: `(-30, 0, 0)`
2. 添加 `DirectionalLight3D`
   - 旋转: `(-45, -30, 0)`

#### G. 添加测试脚本

1. 选中根节点 `TestEnvironment`
2. 附加脚本：`res://scripts/test_environment.gd`

### 3.4 保存场景

1. `场景` → `保存场景`
2. 保存为 `res://scenes/test_environment.tscn`

### 3.5 运行测试

1. 按 **F5** 或点击 "播放场景"
2. 查看控制台输出

**预期输出**:
```
=== 环境系统测试 ===

🌍 EnvironmentController initialized
📚 Ground Material Library initialized with 8 materials

[1] 测试环境预设...
✅ Gravity set to: 9.81 m/s²
🌍 Loaded environment preset: 地球
✅ Gravity set to: 1.62 m/s²
🌍 Loaded environment preset: 月球
...

[2] 测试地面材质...
  可用材质: [concrete, wood, carpet, ice, metal, sand, grass, mud]
  - Concrete: 摩擦=0.9 弹性=0.1
  - Wood: 摩擦=0.6 弹性=0.2
  ...

[3] 测试动态参数...
  重力: 9.81 m/s²
  重力: 5.0 m/s²
  重力: 15.0 m/s²
  ...

=== 测试完成 ===
```

**✅ 如果看到这些输出，Step 3 完成！**

---

## Step 4: 环境参数实验

现在场景正在运行，尝试按键盘快捷键：

| 按键 | 功能 | 观察效果 |
|------|------|----------|
| **1** | 地球环境 | 正常重力，物体正常下落 |
| **2** | 月球环境 | 低重力，物体缓慢下落 |
| **3** | 火星环境 | 中等重力 |
| **C** | 混凝土地面 | 高摩擦 |
| **I** | 冰面 | 低摩擦，物体滑行 |
| **S** | 沙地 | 中摩擦 |

### 观察要点

1. **重力变化**: 
   - 地球 → 月球：物体下落明显变慢
   - 月球 → 木星：物体快速坠落

2. **材质变化**:
   - 混凝土 → 冰面：摩擦力显著降低
   - 材质颜色会改变

3. **控制台输出**:
   - 每次切换都有日志显示

---

## 🎉 完成实践！

如果您完成了所有 4 个步骤，恭喜！您已经：

- ✅ 验证了 Python 零件库功能
- ✅ 测试了 Gym 环境接口
- ✅ 在 Godot 中创建了测试场景
- ✅ 实验了环境参数系统

### 下一步可以做什么？

1. **修改测试场景**
   - 添加更多物体
   - 尝试不同的材质组合
   - 创建坡道测试倾斜

2. **实验零件库**
   - 查看不同电机的规格
   - 计算机器人成本
   - 比较性能指标

3. **学习进阶功能**
   - 阅读 [ADVANCED_USAGE.md](file:///d:/新建文件夹/AGI-Walker/ADVANCED_USAGE.md)
   - 创建自己的机器人
   - 尝试域随机化训练

---

## 🐛 遇到问题？

### 常见问题

**Q: Godot 找不到脚本**
- **A**: 确保脚本路径正确，检查 `res://scripts/environment/` 目录

**Q: Python 提示模块未找到**
- **A**: 设置 PYTHONPATH:
  ```powershell
  $env:PYTHONPATH = "d:\新建文件夹\AGI-Walker\python_api"
  ```

**Q: 场景运行但没有输出**
- **A**: 检查：
  1. 脚本是否正确附加
  2. Godot 输出面板（不是调试器）
  3. 节点名称是否匹配

---

**继续加油！** 🚀
