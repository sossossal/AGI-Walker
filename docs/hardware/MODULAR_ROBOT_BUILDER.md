# 🧩 模块化机器人构建指南 (Modular Robot Builder)

AGI-Walker 现已支持基于真实零部件数据（Data-Driven Parts）的机器人构建流程。您可以像组装电脑一样，选择电机、电池和传感器来"拼装"您的机器人�?

## 1. 零件�?(Parts Library)

核心数据库位�?`python_api/parts_library.json`。目前收录了以下主流硬件规格�?

### 🦾 执行�?(Actuators)
*   **Unitree Go-M8010 Style**: 高动态四足关节电�?(23.7Nm, 30rad/s)�?
*   **Tesla Optimus Style Hip**: 重型行星齿轮关节 (200Nm, 8rad/s)�?
*   **Tesla Optimus Style Knee**: 超重型膝关节 (300Nm)�?
*   **SG90 Servo**: 微型舵机�?

### 👁�?传感�?(Sensors)
*   **MPU-6050**: 消费�?IMU�?
*   **Xsens MTi**: 工业�?IMU�?
*   **VLP-16**: 16线激光雷达�?
*   **RealSense D435**: 深度相机�?

### 🔋 电池 (Batteries)
*   **LiPo 4S 5000mAh**: 航模电池 (0.5kg)�?
*   **Tesla Module 2kWh**: 动力电池�?(12kg)�?

---

## 2. 如何构建自定义机器人

我们提供了一�?Python API `PartsManager` 来辅助构建�?

### 步骤 1: 导入工具
```python
from python_api.parts_manager import PartsManager
from robot_models.base_robot import RobotConfig, LinkConfig, JointConfig

# 初始化管理器
pm = PartsManager()
```

### 步骤 2: 选择零件与计算BOM
```python
# 选择零件 ID
motor_id = "go_m8010"
battery_id = "lipo_4s_5000mah"

# 自动计算总重和总价
parts_list = [motor_id] * 12 + [battery_id]
bom = pm.calculate_bom(parts_list)

print(f"BOM Cost: ${bom['total_cost_usd']}")
print(f"Total Mass: {bom['total_weight_kg']} kg")
```

### 步骤 3: 生成机器人配�?
利用零件的参数（�?`max_torque_nm`）来填充 `JointConfig`，无需手动查表�?

```python
motor = pm.get_part(motor_id)

joint = JointConfig(
    name="hip_joint",
    type="hinge",
    max_torque=motor.specs["max_torque_nm"], # 自动引用
    max_speed=motor.specs["max_speed_rad_s"]
)
```

### 完整示例
请运行演示脚本查看完整流程：
```bash
python examples/custom_parts_demo.py
```

运行后会生成 `custom_robot_config.json`，您可以基于此文件加载仿真环境�?

---

## 3. 扩展零件�?

您只需编辑 `python_api/parts_library.json` 即可添加新零件�?
格式如下�?

```json
"my_new_motor": {
  "name": "Super Motor X",
  "weight_kg": 0.8,
  "max_torque_nm": 50.0,
  "cost_usd": 199
}
```
系统会自动读取新添加的零件�?
