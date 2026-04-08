# Hardware Integration Guide

更新日期：`2026-04-08`

本页说明 AGI-Walker 当前如何与真实硬件或硬件近似层集成。当前最现实的做法是先跑 mock / 驱动骨架，再逐步进入 CAN、IMC-22 和参考固件。

## 1. 当前几条硬件集成路径

### Mock 串口驱动

文件：

- `agi_walker/core/drivers/real_robot_driver.py`

特点：

- 不需要真实硬件
- 可验证命令格式和状态结构
- 适合先跑通数据采集和 driver 层接口

### CAN / IMC-22 控制器

文件：

- `agi_walker/core/api/godot_robot_env/hardware_controller.py`

特点：

- 依赖 `python-can`
- 提供节点发现
- 提供命令发送和状态读取
- 提供 `HardwareEnvironment`

### 参考硬件仓库层

目录：

- `hardware/hive-reflex/`
- `hardware/imc22-rtl/`
- `hardware/firmware/esp32_neuron/`

特点：

- 更偏参考实现和实验方向
- 不属于默认 Python 主线

## 2. 推荐的集成顺序

建议顺序：

1. 先用 `RealRobotDriver(mock=True)` 验证上层逻辑
2. 再跑 `collect_sysid_data.py --mock`
3. 再进入真实串口或 CAN
4. 最后才碰 IMC-22、Hive-Reflex、RTL 或 ESP32 firmware

## 3. Mock 路径

最便宜的硬件近似入口：

```python
from agi_walker.core.drivers.real_robot_driver import RealRobotDriver

driver = RealRobotDriver(mock=True)
driver.connect()
driver.send_motor_commands({"motor_1": 1.0})
print(driver.get_state())
driver.disconnect()
```

这条路径适合：

- 调试接口
- 验证控制命令结构
- 做上层数据采集与日志流程

## 4. SysID 数据采集

文件：

- `agi_walker/core/drivers/collect_sysid_data.py`

用途：

- 生成正弦扫频轨迹
- 采集目标位置、实际位置、速度和力矩
- 输出 CSV

最小命令：

```bash
python -m agi_walker.core.drivers.collect_sysid_data --mock --duration 1 --out sysid_data.csv
```

## 5. CAN / IMC-22 路径

`IMC22Controller` 当前要求：

- 安装 `python-can`
- 可用的 CAN 适配器
- 正确的 `channel`、`bustype`、`bitrate`

当前默认值偏 Linux / SocketCAN：

- `channel="can0"`
- `bustype="socketcan"`
- `bitrate=1000000`

Windows 场景下则需要你显式传入对应总线类型和设备名。

## 6. `HardwareEnvironment`

`HardwareEnvironment` 试图把真实硬件包成类似 Gym 的接口：

- `reset()`
- `step(action)`
- `close()`

它适合作为训练后验证或测试适配层，但不应被误写成“完全等价于仿真环境”。

## 7. 部署脚本现状

文件：

- `examples/deploy_to_hardware.py`

它试图串联：

- 仿真训练
- ONNX 导出
- INT8 量化
- 硬件测试

但要注意：

- 其中一部分路径仍带历史假设
- 依赖链很重
- 更适合作为参考脚本，而不是默认无修改可跑的正式流程

## 8. 当前更稳的验证方式

推荐优先验证：

- `agi_walker/core/drivers/test_driver.py`
- mock driver 行为
- `collect_sysid_data.py --mock`

而不是一上来就跑完整硬件部署。

## 9. 常见前提

### 串口路径

需要：

- `pyserial`
- 正确的串口号，例如 `COM3`

### CAN 路径

需要：

- `python-can`
- 对应驱动
- CAN 硬件已连接并正确配置

## 10. 常见误区

- 把 mock driver 当成真实硬件验证
- 把 `examples/deploy_to_hardware.py` 当成官方一键部署
- 把 `hive-reflex` 和 `imc22-rtl` 当成当前 Python 主线的一部分

## 结论

AGI-Walker 当前的硬件集成主线，最务实的是 mock driver、SysID 采集、CAN 控制骨架这三步。IMC-22、Hive-Reflex 和 firmware 目录属于后续扩展与参考层，不应写成默认前提。
