# Robot Modeling Skill - API 参考

## RobotBuilder 类

### 构造函数

```python
RobotBuilder(name: str)
```

创建新的机器人构建器。

**参数:**
- `name` (str): 机器人名称

---

### 添加部件方法

#### add_torso

```python
add_torso(height: float = 0.5, mass: float = 5.0, **kwargs) -> RobotBuilder
```

添加躯干部件。

**参数:**
- `height` (float): 躯干高度 (米), 默认 0.5
- `mass` (float): 质量 (千克), 默认 5.0
- `**kwargs`: 其他自定义参数

**返回:** RobotBuilder 实例 (支持链式调用)

---

#### add_leg_pair

```python
add_leg_pair(
    thigh_length: float = 0.3,
    shin_length: float = 0.3,
    hip_joint: str = "revolute",
    knee_joint: str = "revolute",
    **kwargs
) -> RobotBuilder
```

添加一对腿 (左右)。

**参数:**
- `thigh_length` (float): 大腿长度 (米)
- `shin_length` (float): 小腿长度 (米)
- `hip_joint` (str): 髋关节类型 ("revolute" | "prismatic" | "fixed")
- `knee_joint` (str): 膝关节类型
- `**kwargs`: 其他参数

**返回:** RobotBuilder 实例

**注意:** 如果已添加躯干,将自动创建连接关系。

---

#### add_arm_pair

```python
add_arm_pair(
    upper_arm_length: float = 0.3,
    forearm_length: float = 0.25,
    shoulder_joint: str = "revolute",
    **kwargs
) -> RobotBuilder
```

添加一对手臂 (左右)。

**参数:**
- `upper_arm_length` (float): 上臂长度 (米)
- `forearm_length` (float): 前臂长度 (米)
- `shoulder_joint` (str): 肩关节类型
- `**kwargs`: 其他参数

**返回:** RobotBuilder 实例

---

### 配置方法

#### set_joint_damping

```python
set_joint_damping(damping: float) -> RobotBuilder
```

设置全局关节阻尼系数。

**参数:**
- `damping` (float): 阻尼系数 (推荐范围: 0.1 - 1.0)

---

#### set_joint_limits

```python
set_joint_limits(joint_name: str, min_angle: float, max_angle: float) -> RobotBuilder
```

设置特定关节的角度限位。

**参数:**
- `joint_name` (str): 关节名称
- `min_angle` (float): 最小角度 (弧度)
- `max_angle` (float): 最大角度 (弧度)

---

#### customize

```python
customize(**params) -> RobotBuilder
```

添加自定义元数据参数。

**参数:**
- `**params`: 任意键值对参数

**示例:**
```python
robot.customize(
    center_of_mass_height=0.25,
    max_speed=2.0,
    stability_margin=0.05
)
```

---

#### build

```python
build() -> RobotConfig
```

完成构建,返回机器人配置对象。

**返回:** `RobotConfig` 实例

---

## RobotConfig 类

### 属性

- `name` (str): 机器人名称
- `parts` (List[Dict]): 部件列表
- `connections` (List[Dict]): 连接关系列表
- `metadata` (Dict): 元数据

### 方法

#### save

```python
save(filepath: str) -> None
```

保存配置到 JSON 文件。

**参数:**
- `filepath` (str): 输出文件路径

**示例:**
```python
robot.save("configs/my_robot.json")
```

---

## 工具函数

### load_template

```python
load_template(template_name: str) -> RobotConfig
```

加载预设模板。

**参数:**
- `template_name` (str): 模板名称

**可用模板:**
- `biped_basic`: 基础双足机器人
- `quadruped_dog`: 四足犬形机器人
- `wheeled_base`: 轮式底盘
- `humanoid_upper`: 类人上半身

**返回:** `RobotConfig` 实例

**示例:**
```python
from agi_walker.skills.robot_modeling import load_template

robot = load_template("biped_basic")
robot.customize(leg_length=0.35)
robot.save("configs/custom_biped.json")
```

---

### list_templates

```python
list_templates() -> List[str]
```

列出所有可用模板名称。

**返回:** 模板名称列表

---

## 完整示例

### 示例 1: 从零创建

```python
from agi_walker.skills.robot_modeling import RobotBuilder

robot = (
    RobotBuilder("fast_runner")
    .add_torso(height=0.4, mass=4.0)
    .add_leg_pair(
        thigh_length=0.35,
        shin_length=0.35,
        hip_joint="revolute"
    )
    .set_joint_damping(0.3)
    .set_joint_limits("hip_flex", -1.57, 1.57)
    .customize(
        max_speed=3.0,
        gait_pattern="trot"
    )
    .build()
)

robot.save("configs/fast_runner.json")
```

### 示例 2: 基于模板定制

```python
from agi_walker.skills.robot_modeling import load_template

# 加载模板
robot = load_template("quadruped_dog")

# 调整参数
robot.parts[0]["params"]["mass"] = 7.0  # 增加躯干质量
robot.metadata["max_speed"] = 4.0

# 保存
robot.save("configs/heavy_dog.json")
```

### 示例 3: 批量生成变体

```python
from agi_walker.skills.robot_modeling import RobotBuilder

# 生成不同腿长的机器人变体
for leg_length in [0.25, 0.30, 0.35, 0.40]:
    robot = (
        RobotBuilder(f"biped_leg{int(leg_length*100)}")
        .add_torso(height=0.5, mass=5.0)
        .add_leg_pair(
            thigh_length=leg_length / 2,
            shin_length=leg_length / 2
        )
        .build()
    )
    robot.save(f"configs/variants/biped_leg{int(leg_length*100)}.json")
```

---

## 数据格式

### JSON配置格式

```json
{
  "name": "机器人名称",
  "parts": [
    {
      "id": "part_1",
      "type": "torso|leg|arm|wheel",
      "side": "left|right|front_left|...",  // 可选
      "params": {
        // 部件特定参数
      }
    }
  ],
  "connections": [
    {
      "from": "parent_part_id",
      "to": "child_part_id",
      "joint_type": "revolute|prismatic|fixed",
      "offset": [x, y, z]  // 可选
    }
  ],
  "metadata": {
    // 自定义元数据
  }
}
```

---

## 最佳实践

1. **命名规范**: 使用描述性名称,如 `fast_biped` 而非 `robot1`
2. **质量分布**: 保持重心低且居中以提高稳定性
3. **关节限位**: 始终设置合理的关节限位避免非物理运动
4. **阻尼系数**: 从 0.5 开始,根据仿真效果调整
5. **模板优先**: 尽可能从模板开始,减少错误
