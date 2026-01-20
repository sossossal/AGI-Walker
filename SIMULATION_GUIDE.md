# AGI-Walker 仿真环境使用指南 (Simulation Guide)

本指南将帮助您快速上手 AGI-Walker 的两种仿真模式：**可视化 3D 仿真 (Godot)** 和 **快速数学仿真 (Python)**。

---

## 🛠️ 1. 环境准备

### 1.1 安装 Python 依赖
确保已安装项目所需的 Python 库：
```bash
pip install -r requirements.txt
```

### 1.2 安装 Godot 引擎 (用于 3D 仿真)
AGI-Walker 使用 **Godot Engine 4.x** 进行物理渲染和仿真。
1.  下载 Godot 4.x: [Godot 官网](https://godotengine.org/download)
2.  将 Godot 可执行文件路径添加到环境变量，或记下其路径。
3.  **导入项目**: 打开 Godot，点击 "Import"，选择 `AGI-Walker/godot_project/project.godot` 文件。

---

## 🖥️ 2. 模式一：可视化 3D 仿真 (Godot)
**适用场景**: 强化学习训练 (RL)、视觉数据采集、演示观察。

此模式下，Python 脚本充当"大脑"，Godot 充当"身体"和"世界"，两者通过 TCP 网络通信。

### 🛠️ 关键配置：启用地形生成 (首次运行必做)
为了使程序化地形生成 (PCG) 生效，您需要在 Godot 编辑器中**手动操作一次**：

> [!IMPORTANT]
> 此步骤对于在 RL 训练中获得**随机地形**至关重要！如果不做，机器人将只能在默认平地上行走。

1.  **打开主场景**: 在 Godot 文件系统中双击 `res://scenes/main.tscn`。
2.  **添加生成器节点**: 
    *   右键点击根节点 `Main` -> `Add Child Node`。
    *   搜索并选择 `Node3D`。
    *   **重命名**: 将新节点重命名为 `TerrainGenerator` (必须完全匹配，区分大小写)。
3.  **附加脚本**:
    *   将脚本 `scripts/environment/procedural_terrain.gd` 从文件面板拖拽到刚创建的 `TerrainGenerator` 节点上。
4.  **保存场景**: 按 `Ctrl+S` 保存。

---

### 启动步骤
1.  **启动 Godot**:
    *   在 Godot 编辑器中打开项目。
    *   点击右上角的 **Run (播放图标)** 启动仿真服务器。
    *   *此时 Godot 窗口应显示"Waiting for connection..."*

2.  **运行 Python 控制脚本**:
    *   打开终端，运行 RL 训练脚本：
        ```bash
        python examples/my_first_robot_train.py
        ```
    *   或者运行 Sim2Real 分析脚本：
        ```bash
        python python_controller/sim2real_analyzer.py
        ```

3.  **观察运行**:
    *   Python 端会显示训练日志/控制日志。
    *   Godot 端会显示机器人的实时动作。

---

## ⚡ 3. 模式二：快速数学仿真 (Python)
**适用场景**: 物理参数验证、TCO 成本分析、基本运动学演示。

此模式不需要 Godot，纯 Python 运行，速度极快，适合快速验证参数合理性。

### 运行示例
运行前进 1 米的参数调整演示：
```bash
python examples/walk_1m_demo.py
```
*   该脚本会对比"默认"、"高功率"、"过重"等不同配置下的机器人表现。
*   运行结束后会生成轨迹图 `robot_forward_1m_demo.png`。

---

## 🤖 4. 模拟机器人配置与切换

### 4.1 切换机器人类型
AGI-Walker 内置了三种标准构型：`Biped` (双足), `Quadruped` (四足), `Wheeled` (轮式)。

在代码中可以通过 `robot_models.base_robot` 创建不同实例：

```python
from robot_models.base_robot import create_robot

# 创建四足机器人
robot = create_robot("quadruped")
```

### 4.2 修改物理参数 (Parametric Tuning)
要修改机器人的物理属性（如质量、电机强度），可以直接编辑配置文件或在代码中动态调整。

**方法 A: 编辑 JSON 配置文件**
位于 `robot_models/biped/config.json` (运行一次示例脚本后会自动生成)。

**方法 B: 代码中动态调整**
在 RL 环境或 Sim2Real 配置中：
```python
from python_api.godot_robot_env.gym_env import RandomizerConfig

# 调整动力学参数
config = RandomizerConfig(
    mass_range=(1.0, 1.2),       # 质量随机化范围
    friction_range=(0.8, 1.0),   # 地面摩擦系数
    motor_strength_range=(0.9, 1.0) # 电机健康度
)
```

---

## ❓ 常见问题

**Q: 连接 Godot 失败 (Connection Refused)?**
*   检查 Godot 是否已点击"播放"并正在运行。
*   默认端口为 `9090`，请确保防火墙未拦截。

**Q: 机器人一动不动?**
*   检查 `walk_1m_demo.py` 输出，可能是功率重量比过低导致无法驱动。
*   在 Godot 中，检查是否因为 `motor_strength` 设置过低。

**Q: 如何加速训练?**
*   在 RL 训练中，可以使用 `CloudSimInterface` (详见 `python_controller/cloud_sim.py`) 来并行启动多个无头 (Headless) Godot 实例。

---

## 📺 5. 远程可视化 GUI (Remote Dashboard)

如果您希望在 Python 程序中直接看到 Godot 的渲染画面，可以启用视频流功能。

### 5.1 配置 Godot
1.  在场景中找到 `Camera3D` 节点。
2.  附加脚本 `scripts/camera_streamer.gd`。
3.  保存场景。

### 5.2 启动仪表盘
运行以下命令启动 Python 可视化界面：
```bash
python examples/dashboard_demo.py
```
*   该程序会自动尝试连接 Godot 的端口 `9998`。
*   它使用 TCP 传输 JPEG 压缩流，延迟极低。

---

## 🧩 6. 模块化机器人构建 (Modular Builder)

您可以像组装乐高一样，使用真实的电机零件（如 Unitree/Tesla 规格）来构建自定义机器人。

### 6.1 选择零件与生成配置
使用我们提供的构建脚本，自动计算 BOM 成本和总重量：

```bash
python examples/custom_parts_demo.py
```
*   此命令会从 `python_api/parts_library.json` 读取零件数据。
*   生成 `custom_robot_config.json` 配置文件。

### 6.2 加载自定义机器人
在您的代码中，使用该配置文件初始化仿真：

```python
from python_api.godot_robot_env.gym_env import GodotRobotEnv

# 加载刚刚生成的配置
env = GodotRobotEnv(
    robot_config_path="custom_robot_config.json"
)
```

更多详细信息，请参阅专门手册：[MODULAR_ROBOT_BUILDER.md](docs/MODULAR_ROBOT_BUILDER.md)。
