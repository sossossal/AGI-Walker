# 虚拟环境构建与生成指南 (Environment Generation Guide)

本文档回答关于"如何创建虚拟环境"以及"是否需要环境生成模型"的问题。

## 1. 虚拟环境创建的三种途径

针对 AGI-Walker 项目，我们推荐以下三种构建环境的路径，从易到难：

### 路径 A: 手动搭建 (Manual Setup)
**适合阶段**: 初期验证、特定场景测试 (如"台阶测试")。
*   **方法**: 使用 Godot 编辑器，拖拽 `CSGBox3D` 搭建地面、墙壁和障碍物。
*   **优点**: 精确控制，简单直观。
*   **指南**: 请参考 [SCENE_SETUP_GUIDE.md](../godot_project/SCENE_SETUP_GUIDE.md)。

### 路径 B: 程序化生成 (Procedural Generation - **推荐**)
**适合阶段**: 强化学习训练 (RL Training)。
*   **方法**: 编写 Godot 脚本 (`.gd`) 或 Python 脚本，利用算法 (如 Perlin Noise, Wave Function Collapse) 自动生成无限变化的 terrain。
*   **优点**: 能够生成近乎无限的训练场景，防止机器人"死记硬背"地图 (Overfitting)。

### 路径 C: AI 模型生成 (GenAI / Model-based)
**适合阶段**: 高级 Sim2Real 视觉训练。
*   **方法**: 接入 Stable Diffusion (生成纹理/背景) 或 Shap-E/Point-E (生成3D资产)。
*   **回答您的提问**: **目前阶段不需要**必接"环境生成模型"。对于双足机器人的运动控制训练，几何结构的程序化生成 (路径 B) 比视觉生成的 AI 模型更重要且更高效。

---

## 2. 方案详解：程序化生成 (PCG)

我们建议在 Godot 中使用 **HeightMapShape3D** 或 **GridMap** 来实现动态地形。

### 方案一：高程图地形 (HeightMap)
适用于模拟野外起伏地面。

**Godot 实现伪代码 (`Main.gd`):**

```gdscript
extends Node3D

func generate_terrain(size: int = 100):
    var height_map = HeightMapShape3D.new()
    height_map.map_width = size
    height_map.map_depth = size
    
    var data = PackedFloat32Array()
    var noise = FastNoiseLite.new() # 使用 Godot 内置噪声
    
    for y in range(size):
        for x in range(size):
            # 生成随机高度
            var h = noise.get_noise_2d(x, y) * 2.0 
            data.append(h)
    
    height_map.map_data = data
    
    # 创建碰撞体
    var collider = CollisionShape3D.new()
    collider.shape = height_map
    $Ground.add_child(collider)
```

### 方案二：网格地图 (GridMap)
适用于模拟城市环境、楼梯、走廊。

1.  **制作图块集 (MeshLibrary)**: 制作"平地"、"斜坡"、"台阶"、"墙壁"等预制件。
2.  **代码生成**:

```gdscript
extends GridMap

func generate_dungeon():
    clear()
    for x in range(20):
        for z in range(20):
            if randf() > 0.2:
                set_cell_item(Vector3i(x, 0, z), 0) # 0号是平地
            else:
                set_cell_item(Vector3i(x, 0, z), 1) # 1号是障碍物
```

---

## 3. Python 外部生成接口

如果您希望在 Python 端控制环境生成 (例如结合 `terrain_mapper.py`)，可以定义通信协议：

1.  **Python 端**: 生成一个 `N x N` 的高程矩阵 (numpy array)。
2.  **通信**: 通过 TCP 发送 `update_terrain` 指令及矩阵数据到 Godot。
3.  **Godot 端**: 接收数据并更新 `HeightMapShape3D`。

### 示例指令结构
```json
{
    "type": "update_terrain",
    "width": 32,
    "depth": 32,
    "data": [0.0, 0.1, 0.2, ...] // 展平的高度数组
}
```

---

## 4. 总结与建议

1.  **当前**: 继续使用手动搭建的简单场景 (`scenes/main.tscn`) 跑通基础 RL。
2.  **中期**: 实现一个简单的 Godot 脚本，每次 Reset 时随机化地面坡度或添加随机方块障碍物。
3.  **高级**: 只有在需要极高视觉逼真度时 (比如训练纯视觉导航)，才需要考虑接入 GenAI 模型生成环境资产。
