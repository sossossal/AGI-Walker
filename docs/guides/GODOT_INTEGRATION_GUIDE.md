# AGI-Walker Godot集成指南

本指南说明如何将GUI配置器连接到Godot仿真引擎，实现真实的机器人仿真�?

---

## 🎯 架构概览

```
GUI配置�?(Python/Tkinter)
        �?
  godot_client.py (TCP Client)
        �?(TCP Socket)
Godot TCP Server (GDScript)
        �?
  Godot物理引擎/机器人场�?
```

---

##  已完成的组件

### 1. Python通信客户�?

**文件**: `python_api/godot_client.py`

**功能**:
- TCP连接管理
- 命令发送（启动/停止/参数更新�?
- 数据接收（异步后台线程）
- 回调机制

**使用示例**:
```python
from python_api.godot_client import GodotSimulationClient

# 创建客户�?
client = GodotSimulationClient(host="127.0.0.1", port=9999)

# 设置数据回调
client.set_data_callback(lambda data: print(data))

# 连接
if client.connect():
    # 启动仿真
    robot_config = {'parts': [...], 'connections': [...]}
    client.start_simulation(robot_config)
    
    # 实时更新参数
    client.update_parameters({'motor_power': 1.2})
    
    # 停止
    client.stop_simulation()
    client.disconnect()
```

### 2. GUI集成（部分）

**文件**: `tools/robot_configurator_gui.py`

**已添�?*:
- GodotSimulationClient导入
- 连接UI控件（地址/端口输入�?
- 连接状态显�?

**待完�?*: 将现有的FeedbackPanel完全改造为Godot集成版本

---

## 🔧 Godot端实现要�?

### TCP服务�?(GDScript)

**文件**: `godot_project/scripts/TCPSimulationServer.gd`

```gdscript
extends Node

var server = TCP_Server.new()
var clients = []
var port = 9999

func _ready():
    server.listen(port)
    print("仿真服务器启动于端口: ", port)

func _process(delta):
    # 接受连接
    if server.is_connection_available():
        var client = server.take_connection()
        clients.append(client)
        print("客户端已连接")
    
    # 处理消息
    for client in clients:
        if client.get_available_bytes() > 0:
            handle_message(client)

func handle_message(client):
    # 读取长度前缀
    var length_bytes = client.get_data(4)
    if length_bytes[0] != OK:
        return
    
    var length = bytes_to_var(length_bytes[1])
    
    # 读取JSON数据
    var data_bytes = client.get_data(length)
    if data_bytes[0] != OK:
        return
    
    var json_str = data_bytes[1].get_string_from_utf8()
    var data = JSON.parse(json_str).result
    
    match data.command:
        "start_sim":
            start_simulation(data.data)
        "stop_sim":
            stop_simulation()
        "update_params":
            update_parameters(data.data)
        "load_robot":
            load_robot_config(data.data)

func start_simulation(config):
    print("启动仿真: ", config)
    # TODO: 加载机器人配�?
    # TODO: 开始物理模�?
    pass

func send_feedback(client, data):
    var json_str = JSON.print(data)
    var json_bytes = json_str.to_utf8()
    
    var length = json_bytes.size()
    var length_bytes = var_to_bytes(length)
    
    client.put_data(length_bytes)
    client.put_data(json_bytes)
```

---

## 📋 集成步骤

### 步骤1: 完善Python GUI（已完成50%�?

- [x] 创建godot_client.py
- [x] 添加连接UI
- [ ] 完全重写FeedbackPanel
- [ ] 集成参数同步
- [ ] 添加错误处理

### 步骤2: 实现Godot服务�?

1. 创建 `godot_project/scripts/TCPSimulationServer.gd`
2. 将其添加到主场景作为自动加载节点
3. 实现消息处理逻辑
4. 添加仿真数据反馈

### 步骤3: 测试连接

1. 启动Godot项目（TCP服务器自动运行）
2. 启动GUI配置�?
3. 点击"连接Godot"
4. 验证连接状�?

### 步骤4: 实现仿真功能

1. 在Godot中实现机器人加载
2. 参数实时更新
3. 状态数据回�?
4. GUI显示实时数据

---

## 🔌 连接测试

### 方法1: 使用模拟服务�?

```bash
# 终端1: 启动模拟Godot服务�?
cd d:\新建文件夹\AGI-Walker
python -c "from python_api.godot_client import MockGodotServer; import time; s=MockGodotServer(); s.start(); time.sleep(999)"

# 终端2: 启动GUI
python tools\robot_configurator_gui.py
```

### 方法2: 使用真实Godot

1. 打开Godot项目: `godot_project/project.godot`
2. 添加TCPSimulationServer.gd到场�?
3. 运行Godot项目
4. 启动GUI并连�?

---

## 📊 数据协议

### Python �?Godot (命令)

```json
{
  "command": "start_sim",
  "data": {
    "robot": {
      "parts": [{"id": "motor_1", "type": "motor"}],
      "connections": [{"from": "motor_1", "to": "ctrl_1"}]
    },
    "physics": {
      "gravity": 9.81,
      "timestep": 0.01
    }
  },
  "timestamp": 1234567890.123
}
```

### Godot �?Python (反馈)

```json
{
  "type": "simulation_data",
  "position": 0.5,
  "velocity": 0.3,
  "battery": 85.0,
  "joint_angles": [0.1, 0.2, 0.3, 0.4],
  "timestamp": 1234567890.456
}
```

---

## 🧪 单元测试

### 测试godot_client.py

```bash
# 运行内置测试
cd d:\新建文件夹\AGI-Walker
python python_api\godot_client.py
```

### 测试GUI连接

1. 启动GUI
2. 在连接面板输�?`127.0.0.1:9999`
3. 点击"连接Godot"
4. 查看状态指示器

---

## ⚠️ 常见问题

### Q: 连接失败怎么办？
A: 检查：
1. Godot是否运行
2. TCP服务器是否启�?
3. 端口9999是否被占�?
4. 防火墙设�?

### Q: 数据不更新？
A: 检查：
1. 回调函数是否正确设置
2. Godot是否正在发送数�?
3. 网络延迟

### Q: GUI卡死�?
A: 所有网络操作都在后台线程，不应该卡死。检查：
1. 是否有异常未捕获
2. 数据回调是否使用了`after()`方法

---

## 🚀 下一�?

1. **完成FeedbackPanel改�?*
   - 将所有模拟数据替换为Godot数据
   - 添加连接状态监�?
   - 实现参数实时同步

2. **实现Godot服务�?*
   - 创建完整的TCP服务�?
   - 机器人配置加�?
   - 物理参数动态调�?

3. **添加测试**
   - 单元测试
   - 集成测试
   - 性能测试

---

**状�?*: 🟡 进行�?(60%)  
**下一目标**: 完成FeedbackPanel重写并测试基本连�?
