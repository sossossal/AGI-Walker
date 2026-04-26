# Hardware Integration Guide

更新日期：`2026-04-12`

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

真实串口驱动现在还支持 replay 路径：

```python
from agi_walker.core.drivers.real_robot_driver import RealRobotDriver

driver = RealRobotDriver.from_replay("tests/fixtures/real_robot_driver_replay.json")
driver.connect()
driver.poll_once()
print(driver.get_state())
driver.disconnect()
```

这条路径适合：

- 调试接口
- 验证控制命令结构
- 做上层数据采集与日志流程
- 在默认 pytest 中验证串口协议而不访问真实串口

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

同时它现在也支持两条非真实硬件路径：

- 通过 `bus=` 注入协议级 mock / fake CAN bus
- 通过 `IMC22Controller.from_replay(path)` 加载回放帧

当前默认值偏 Linux / SocketCAN：

- `channel="can0"`
- `bustype="socketcan"`
- `bitrate=1000000`

Windows 场景下则需要你显式传入对应总线类型和设备名。

仓库现在还补了一层 canonical transport profile，优先建议通过 profile 构造控制器，而不是在调用点散落 `channel / bustype / bitrate`：

- `default_imc22_transport_profile()`
- `validate_imc22_transport_profile()`
- `IMC22Controller.from_transport_profile(...)`
- `HardwareEnvironment.from_transport_profile(...)`

当前支持的 transport 值：

- `socketcan`
- `pcan`
- `replay`
- `serial_bridge`

其中：

- `socketcan` / `pcan` 已可直接构造真实 CAN controller
- `replay` 已可直接加载 replay fixture
- `serial_bridge` 已通过 `RealRobotDriver` 接到最小串口桥 adapter

`serial_bridge` 当前行为边界：

- 可直接走 `serial_port + baudrate` 真实串口路径
- 也支持 `replay_source`，用于默认 pytest 下复用串口 replay fixture
- 上层仍复用 `IMC22Controller` / `HardwareEnvironment` 的 `send_command / read_status / discover_nodes` 面
- `set_config()` 现在会落到 `RealRobotDriver.send_motor_config(...)`，通过独立串口 config packet 下发并同步到 driver state

最小示例：

```python
from agi_walker.core.api.godot_robot_env.hardware_controller import (
    HardwareEnvironment,
    IMC22Controller,
    default_imc22_transport_profile,
)

profile = {
    **default_imc22_transport_profile("pcan"),
    "channel": "PCAN_USBBUS1",
    "bitrate": 1_000_000,
}
controller = IMC22Controller.from_transport_profile(profile)
controller.close()

replay_env = HardwareEnvironment.from_transport_profile(
    {
        **default_imc22_transport_profile("replay"),
        "replay_source": "tests/fixtures/imc22_status_replay.json",
    },
    num_joints=2,
)
replay_env.close()
```

## 6. 协议级 mock / replay 路径

当前仓库已经补上协议级 replay 骨架，位置仍在：

- `agi_walker/core/api/godot_robot_env/hardware_controller.py`

当前可用能力：

- `encode_command_payload()` / `encode_status_payload()` / `decode_status_payload()`
- `validate_imc22_replay_payload()`
- `ReplayCANBus`
- `IMC22Controller.from_replay(...)`
- `HardwareEnvironment(controller=...)`
- `instruction_control_contracts.default_simulated_circuit_config()`
- `instruction_control_contracts.build_instruction_runtime_contract()`

测试回放 fixture 位于：

- `tests/fixtures/imc22_status_replay.json`

这条路径的作用是：

- 验证 IMC-22 协议编码/解码
- 验证节点发现
- 验证 `HardwareEnvironment.reset()` / `step()`
- 保证默认 pytest 不访问真实 CAN
- 把 Godot / ROS2 / 模拟电路控制面收口成一份结构化契约

当前 canonical 模拟电路参数固定为：

- `transport=imc22_can_fd`
- `bitrate=1000000`
- `control_freq_hz=100`
- `status_rate_hz=200`
- `command_base_id=0x200`
- `status_base_id=0x100`
- `config_base_id=0x300`

当前 controller 级安全执行面还新增了：

- 指令限幅：`max_abs_target_angle`
- compliance 边界：`min_compliance / max_compliance`
- stale-command watchdog：`watchdog_timeout_s`
- watchdog 保持位：`watchdog_hold_angle`
- 恢复接口：`clear_faults()` / `recover()`
- 错误分类：`ok / overload / overcurrent / sensor_fault / communication_fault`
- vendor-specific fault table：默认 `imc22_reflex`，优先按 exact code 映射，再按 range 兜底
- 分级恢复：`build_recovery_plan()` / `recover_by_fault_class()`

也就是说，在进入真实设备前，仓库现在至少能先验证：

- 越界目标角会被限制
- 过期命令会触发 hold-angle 安全回退
- fault 状态可显式清除或重新下发恢复命令
- 节点错误值会按 vendor-specific fault table 收敛成 machine-readable fault class
- 不同 fault class 会走不同恢复动作，而不是统一 recover

ROS2 一期指令集模拟控制面现已提供：

- topic `/instruction_set/json`
- topic `/simulated_circuit/json`
- publisher `/instruction_runtime/json`
- service `/instruction_set/replay_last`
- service `/simulated_circuit/apply_default`

这些入口会把结构化 payload 投影到：

- Godot `instruction_set`
- Godot `configure_simulated_circuit`
- legacy `update_params`
- `simulated_circuit_command_batch`

当前 replay 闭环还新增了一层：

- `simulated_circuit_command_batch -> IMC22 replay payload`
- `IMC22 replay payload -> status feedback`
- `status feedback -> /instruction_runtime/json`

也就是说，一期已经可以在非 live 模式下验证：

- 指令集投影出的 command batch
- 模拟电路回放后的反馈状态
- ROS2 运行态快照中的 feedback 透传

推荐命令：

```bash
python -m pytest tests/test_hardware_controller.py -q
```

如果要在进入真实设备前先做 transport 预检，仓库现在有一个最小诊断 runner：

```bash
python tools/run_hardware_transport_diagnostics.py --transport replay --replay-source tests/fixtures/imc22_status_replay.json --attempt-connect
```

对于串口桥 replay：

```bash
python tools/run_hardware_transport_diagnostics.py --transport serial_bridge --replay-source tests/fixtures/real_robot_driver_replay.json --attempt-connect
```

它会输出：

- 归一化后的 transport profile
- 结构化 checks 列表
- 最终 `ready / blocked` 结论
- 可选 `fault_telemetry_report`，用于导出原始 error 值与 fault class 对照

如果你要把 vendor-specific fault table 外置化：

```bash
python tools/run_hardware_transport_diagnostics.py --transport replay --replay-source tests/fixtures/imc22_status_replay.json --fault-table-file deployment/hardware/imc22_reflex_fault_table.json --attempt-connect --telemetry-output test_env/hardware_fault_telemetry_report.json
```

这条链现在支持：

- `fault_table_source`：从 JSON 文件加载 vendor-specific fault table
- `fault_telemetry_report.entries[].raw_error_value`：保留原始错误值
- `fault_telemetry_report.entries[].fault_class`：保留映射后的标准 fault class

## 7. `HardwareEnvironment`

`HardwareEnvironment` 试图把真实硬件包成类似 Gym 的接口：

- `reset()`
- `step(action)`
- `close()`

它适合作为训练后验证或测试适配层，但不应被误写成“完全等价于仿真环境”。

## 8. 部署脚本现状

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

## 9. 当前更稳的验证方式

推荐优先验证：

- `agi_walker/core/drivers/test_driver.py`
- mock driver 行为
- `collect_sysid_data.py --mock`
- `tests/test_real_robot_driver.py` 的 mock/replay 路径
- `tests/test_hardware_controller.py` 的 replay/mock 路径

而不是一上来就跑完整硬件部署。

## 10. 常见前提

### 串口路径

需要：

- `pyserial`
- 正确的串口号，例如 `COM3`

### CAN 路径

需要：

- `python-can`
- 对应驱动
- CAN 硬件已连接并正确配置

### ROS2 bridge live smoke

需要：

- ROS 2 Humble
- `rclpy`
- `sensor_msgs`
- `geometry_msgs`
- `std_srvs`
- `tf2_ros`

推荐命令：

```bash
export AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE=1
python -m pytest tests/test_ros2_bridge_smoke.py -q -m "integration and live"
```

这条路径会用仓库内的 mock Godot TCP server 验证：

- bridge 启动
- service 可用性
- `/joint_states` 发布
- `/cmd_vel` 到 Godot 参数转发

## 11. 常见误区

- 把 mock driver 当成真实硬件验证
- 把 replay fixture 当成真实硬件闭环
- 把 `examples/deploy_to_hardware.py` 当成官方一键部署
- 把 `hive-reflex` 和 `imc22-rtl` 当成当前 Python 主线的一部分

## 结论

AGI-Walker 当前的硬件集成主线，最务实的是 mock driver、SysID 采集、CAN 控制骨架，再加协议级 replay 验证这四步。IMC-22、Hive-Reflex 和 firmware 目录属于后续扩展与参考层，不应写成默认前提。
