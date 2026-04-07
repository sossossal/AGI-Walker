# 硬件部署指南 (Hardware Deployment Guide)

## 概述

本指南将帮助您将 AGI-Walker 从仿真环境部署到真实硬件�?采用 **OpenNeuro Zone Architecture**�?

---

## 1. 硬件清单 (BOM)

### 1.1 核心计算单元

| 角色 | 硬件 | 数量 | 单价 (¥) | 说明 |
|------|------|------|---------|------|
| **Cortex** (大脑) | NVIDIA Jetson Nano / PC | 1 | 1000-3000 | 运行 RL 策略和视觉处�?|
| **Ganglion** (网关) | Raspberry Pi 4B (可�? | 0-4 | 300 | 每条腿一�?可�?|
| **Neuron** (执行�? | ESP32-DevKitC | 4-12 | 30 | 每个关节一�?MCU |

### 1.2 执行器与传感�?

| 组件 | 型号 | 数量 | 单价 (¥) |
|------|------|------|---------|
| 舵机 | MG996R (20kg.cm) | 8-12 | 25 |
| IMU | MPU6050 | 1 | 10 |
| 电源 | LiPo 3S 5000mAh | 1 | 150 |
| 稳压�?| LM2596 降压模块 | 2 | 5 |

**总成本估�?*: ¥2000-¥5000 (取决于配�?

---

## 2. 系统架构

### 2.1 拓扑�?

```
         ┌─────────────────�?
         �? Cortex (PC)    �?
         �? - ROS 2 节点    �?
         �? - Zenoh Router �?
         └────────┬────────�?
                  �?WiFi/Ethernet
         ┌────────▼────────�?
         �?Zenoh Network   �?
         └────────┬────────�?
                  �?
    ┌─────────────┼─────────────�?
    �?            �?            �?
┌───▼───�?    ┌───▼───�?    ┌───▼───�?
│ESP32  �?    │ESP32  �?    │ESP32  �?
│Neuron �?    │Neuron �?    │Neuron �?
�?FL    �?    �?FR    �?    �?RL    �?
└───┬───�?    └───┬───�?    └───┬───�?
    �?            �?            �?
  2x舵机        2x舵机        2x舵机
```

### 2.2 通信�?

**命令�?* (Cortex �?Neuron):
```
Python RL 策略
    �?
ROS 2 Topic (/robot/joint_commands)
    �?
Zenoh Router
    �?
WiFi �?ESP32 (Zenoh-Pico)
    �?
PWM �?舵机
```

**状态流** (Neuron �?Cortex):
```
传感�?(编码�?IMU)
    �?
ESP32 采集
    �?
Zenoh-Pico Publish
    �?
WiFi �?Zenoh Router
    �?
ROS 2 Topic (/robot/joint_states)
```

---

## 3. 接线�?

### 3.1 ESP32 引脚分配

```
ESP32 DevKit-C
┌─────────────────�?
�? 3V3  ────────  �?�?逻辑电平
�? GND  ────────  �?�?公共�?
�? GPIO12 ──────  �?�?舵机 0 (PWM)
�? GPIO13 ──────  �?�?舵机 1 (PWM)
�? GPIO14 ──────  �?�?舵机 2 (PWM)
�? GPIO15 ──────  �?�?舵机 3 (PWM)
�? GPIO21 ──────  �?�?IMU SDA
�? GPIO22 ──────  �?�?IMU SCL
└─────────────────�?
```

### 3.2 电源系统

```
LiPo 3S (11.1V)
    �?
    ├──�?LM2596 (降压�?5V) ──�?舵机电源
    �?
    └──�?LM2596 (降压�?5V) ──�?ESP32 VIN
```

> ⚠️ **警告**: 舵机�?ESP32 必须共地 (GND),但电源分开供电!

---

## 4. 固件烧录

### 4.1 安装 PlatformIO

```bash
# 方法 1: VSCode 扩展
# �?VSCode 中搜�?"PlatformIO IDE" 并安�?

# 方法 2: CLI
pip install platformio
```

### 4.2 配置 WiFi

编辑 `firmware/esp32_neuron/src/main.cpp`:

```cpp
const char* WIFI_SSID = "YourSSID";        // 改为你的 WiFi �?
const char* WIFI_PASSWORD = "YourPassword"; // 改为你的密码
const char* ZENOH_ROUTER = "tcp/192.168.1.100:7447"; // Cortex IP
```

### 4.3 烧录固件

```bash
cd firmware/esp32_neuron

# 编译
pio run

# 烧录 (连接 ESP32 �?USB)
pio run --target upload

# 查看串口输出
pio device monitor
```

**预期输出**:
```
🤖 AGI-Walker ESP32 Neuron 启动�?..
�?舵机初始化完�?
📶 连接 WiFi.....
�?WiFi 已连�? 192.168.1.105
�?Zenoh 会话已建�?
🚀 Neuron 就绪!
```

---

## 5. 系统启动流程

### 5.1 启动 Cortex (PC)

```bash
# 终端 1: 启动 Zenoh Router (可�?如果使用 peer 模式可跳�?
zenohd

# 终端 2: 启动 ROS 2 节点
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build
source install/setup.bash
ros2 launch agi_walker_ros2 robot.launch.py

# 终端 3: 发送测试命�?
ros2 topic pub /robot/joint_commands std_msgs/msg/Float64MultiArray \
  "data: [45, 90, 135, 90, 45, 90, 135, 90]"
```

### 5.2 验证通信

```bash
# 查看 Topic 列表
ros2 topic list

# 监听状�?
ros2 topic echo /robot/joint_states

# 查看 Zenoh 流量
zenoh scout
```

---

## 6. 调试技�?

### 6.1 ESP32 无法连接 WiFi
**检�?*:
- SSID 和密码是否正�?
- WiFi 是否�?2.4GHz (ESP32 不支�?5GHz)
- 路由器是否启用了 AP 隔离

### 6.2 Zenoh 连接失败
**检�?*:
- Cortex IP 地址是否正确
- 防火墙是否阻止端�?7447
- 使用 `ping` 测试网络连通�?

### 6.3 舵机不动
**检�?*:
- 电源是否充足 (舵机需�?5V 2A+)
- PWM 信号线是否连接正�?
- 串口输出是否显示收到命令

---

## 7. 性能优化

### 7.1 降低延迟
- 使用有线 Ethernet 替代 WiFi
- 启用 ESP32 �?WiFi 省电模式关闭
- 减少 Zenoh 消息大小 (仅发送变化的关节)

### 7.2 提高可靠�?
- 添加心跳检�?(watchdog)
- 实现故障转移 (备用 Neuron)
- 使用 PTP 时间同步 (高级)

---

## 8. 下一�?

1. **机械组装**: 使用 3D 打印或铝型材搭建机身
2. **PID 调优**: 调整关节控制参数
3. **RL 训练**: 在真实硬件上微调策略
4. **视觉集成**: 添加摄像头和深度传感�?

---

**更新日期**: 2026-01-21  
**版本**: v4.1.0-alpha
