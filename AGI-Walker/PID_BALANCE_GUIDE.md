# PID平衡控制使用指南

本文档说明如何使用和调优PID平衡控制系统。

---

## 📋 已创建的文件

### Godot端
1. **`pid_controller.gd`** - PID控制器类
2. **`balance_controller.gd`** - 平衡控制器节点

### Python端
1. **`pid_controller.py`** - Python版PID控制器
2. **`pid_tuner.py`** - 自动参数调优工具

---

## 🚀 快速开始

### 步骤1: 在Godot中集成

1. **添加BalanceController节点**到主场景:
   ```
   Main (Node3D)
   ├── ...
   └── BalanceController (Node) [脚本: balance_controller.gd]
   ```

2. **运行场景**，应该看到:
   ```
   ✅ 平衡控制器已初始化
   ```

3. **观察机器人**，它应该会自动保持直立姿态

### 步骤2: 启用调试模式

在Godot检查器中，将 `BalanceController` 的 `debug_mode` 设为 `true`，
查看实时平衡数据:

```
平衡控制 | Roll: 5.2°→-2.3° | Pitch: 3.1°→-1.5°
```

---

## 🎛️ PID参数说明

### 默认参数（已调优）

| 参数 | Roll | Pitch | 说明 |
|------|------|-------|------|
| **Kp** | 8.0 | 8.0 | 比例增益（响应速度） |
| **Ki** | 0.5 | 0.5 | 积分增益（消除稳态误差） |
| **Kd** | 3.0 | 3.0 | 微分增益（减少震荡） |

### 参数调整指南

#### 增加 Kp (比例增益)
- ✅ 效果: 响应更快
- ❌ 副作用: 可能震荡，超调

#### 增加 Ki (积分增益)
- ✅ 效果: 消除长期偏差
- ❌ 副作用: 可能积分饱和，响应变慢

#### 增加 Kd (微分增益)
- ✅ 效果: 减少震荡，平滑
- ❌ 副作用: 对噪声敏感

---

## 🧪 参数调优

### 方法1: 手动调优（Godot内）

在GDScript中调用:

```gdscript
# 获取平衡控制器
var balance = $BalanceController

# 调整Roll PID
balance.tune_pid("roll", 10.0, 0.3, 4.0)

# 调整Pitch PID
balance.tune_pid("pitch", 10.0, 0.3, 4.0)
```

**观察机器人行为**:
- 震荡太多 → 降低Kp，增加Kd
- 响应太慢 → 增加Kp
- 有稳态误差 → 增加Ki

### 方法2: Python自动调优

#### 网格搜索

```bash
cd python_controller
python pid_tuner.py
# 选择: 1 (网格搜索)
```

自动测试多组参数，输出最优配置。

**配置示例**:
```python
tuner.grid_search(
    kp_range=[4.0, 6.0, 8.0, 10.0],
    ki_range=[0.1, 0.3, 0.5],
    kd_range=[1.0, 2.0, 3.0, 4.0],
    test_duration=15.0
)
```

#### 自适应搜索

```python
tuner.adaptive_search(
    initial_params=(8.0, 0.5, 3.0),
    step_sizes=(1.0, 0.1, 0.5),
    iterations=10
)
```

从初始值开始，迭代优化。

---

## 📊 性能指标

### 评价标准

| 指标 | 目标值 | 说明 |
|------|--------|------|
| **平均偏差** | < 5° | Roll+Pitch总和 |
| **稳定时间** | > 25s/30s | 保持直立的时间比例 |
| **稳定性评分** | > 90/100 | 综合评价 |

### 查看实时性能

在Godot控制台中运行:

```gdscript
# 从远程调试器执行
get_node("/root/Main/BalanceController").print_debug_info()
```

输出:
```
==================================================
平衡控制器状态
==================================================
平衡时间: 25.3s
平衡质量: 92.5/100

Roll PID:
PID状态 [Kp=8.00, Ki=0.50, Kd=3.00]
  误差: 0.850°
  P项: 6.800
  I项: 0.425
  D项: -1.275
  ...
==================================================
```

---

## 🔧 故障排除

### 问题1: 机器人剧烈震荡

**原因**: Kp太高或Kd太低

**解决方案**:
```gdscript
balance.tune_pid("roll", 5.0, 0.5, 4.0)  # 降低Kp，增加Kd
balance.tune_pid("pitch", 5.0, 0.5, 4.0)
```

### 问题2: 机器人倾斜无法恢复

**原因**: Kp太低

**解决方案**:
```gdscript
balance.tune_pid("roll", 12.0, 0.5, 3.0)  # 增加Kp
```

### 问题3: 有持续小偏差

**原因**: Ki太小

**解决方案**:
```gdscript
balance.tune_pid("roll", 8.0, 1.0, 3.0)  # 增加Ki
```

### 问题4: 积分饱和

**症状**: 机器人反应迟钝

**解决方案**:
```gdscript
# 在balance_controller.gd中调整
roll_pid.set_integral_limits(-10, 10)  # 减小积分限制
```

---

## 🎓 高级技巧

### 动态参数调整

根据机器人状态动态调整PID:

```gdscript
func _physics_process(delta):
    # 获取倾斜角
    var tilt = abs(robot.get_sensor_data()['sensors']['imu']['orient'][0])
    
    # 倾斜严重时，增强响应
    if tilt > 15:
        roll_pid.set_tunings(12.0, 0.5, 4.0)  # 更激进的参数
    else:
        roll_pid.set_tunings(8.0, 0.5, 3.0)   # 正常参数
```

### 抗积分饱和

```gdscript
# 当输出饱和时，停止积分累积
if output >= output_max or output <= output_min:
    integral = 0.0  # 重置积分
```

### 级联PID

```gdscript
# 外环：位置PID
# 内环：姿态PID（已实现）

# 可用于路径跟随等高级功能
```

---

## 📈 Ziegler-Nichols调参法

经典PID调参方法:

### 步骤1: 找到临界参数

1. 设置 Ki=0, Kd=0
2. 逐渐增加 Kp 直到系统震荡
3. 记录临界 Kp (Ku) 和震荡周期 Pu

### 步骤2: 计算PID参数

| 控制类型 | Kp | Ki | Kd |
|---------|----|----|-----|
| **P** | 0.5*Ku | 0 | 0 |
| **PI** | 0.45*Ku | 0.54*Ku/Pu | 0 |
| **PID** | 0.6*Ku | 1.2*Ku/Pu | 0.075*Ku*Pu |

---

## 💡 最佳实践

1. **从保守参数开始**: Kp=5, Ki=0, Kd=2
2. **逐步调整**: 每次只改一个参数
3. **记录结果**: 使用`pid_tuner.py`自动记录
4. **测试鲁棒性**: 在不同初始姿态下测试
5. **持续监控**: 使用调试模式实时观察

---

## 📚 参考资源

- [PID控制理论](https://en.wikipedia.org/wiki/PID_controller)
- [Ziegler-Nichols方法](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method)
- 书籍: 《Feedback Control of Dynamic Systems》

---

> 💡 **提示**: PID参数调优是一个迭代过程。不要期望一次就找到完美参数，需要多次测试和调整。
