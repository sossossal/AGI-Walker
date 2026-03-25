# Web 控制面板指南

AGI-Walker 提供了一个基于 Web 的控制面板，用于管理任务、查看页面资源，并调试 Web-Godot 集成相关功能。

## 启动面板

在项目根目录下运行：
```bash
python web_panel/server.py
```
也可以使用模块方式：

```bash
python -m web_panel.server
```

启动后访问 [http://localhost:8000](http://localhost:8000)。

说明：

- `web_panel/` 内已经包含 FastAPI 服务、静态页面和 WebSocket 协议处理代码。
- 直接启动服务不等于完整 Godot 联动已验证；Godot 侧仍需要额外环境与联调。
- 如果在 Windows 终端中遇到编码问题，优先使用 UTF-8 终端。

## 当前可确认的内容

- Web 服务入口存在：`web_panel/server.py`
- 协议处理模块存在：`web_panel/ws_protocol.py`
- 静态页面资源存在：`web_panel/static/`
- 部分页面和接口面向 Web-Godot 联调

## 当前接口结构

当前仓库里的 Godot 集成分为两种模式，这一点需要明确：

### 1. Legacy Controller 模式

这条链路面向“连接 Godot、加载机器人、启动/停止仿真、更新参数”。

对应接口：
- `POST /api/godot/connect`
- `POST /api/godot/disconnect`
- `GET /api/godot/status`
- `POST /api/godot/load-robot`
- `POST /api/godot/start`
- `POST /api/godot/stop`
- `POST /api/godot/update-params`

WebSocket 协议中的这些命令也属于这一类：
- `simulation.start`
- `simulation.stop`
- `config.load_robot`
- `params.update`
- `ping`

### 2. Session Bridge 模式

这条链路面向“按会话启动 Godot 进程、通过 TCP 读取遥测/发送动作”，更接近 RL 或调试桥。

对应接口：
- `POST /api/godot/{session_id}/launch`
- `POST /api/godot/{session_id}/stop`
- `GET /api/godot/{session_id}/status`
- `POST /api/godot/{session_id}/control`
- `WS /ws/{session_id}`

当前 `godot_project/scripts/tcp_server.gd` 可确认支持的 TCP 命令是：
- `reset`
- `step`
- `get_schema`

这意味着 Session Bridge 目前不能直接替代 Legacy Controller。两套接口现在是并存关系，不应混为一套“已完全统一”的方案。

## 调试建议

1. 先确认基础服务可以启动并监听 `8000` 端口。
2. 再访问首页和静态页面，确认静态资源加载正常。
3. 最后再接入 Godot 仿真端，逐步调试 WebSocket 与控制接口。

## API 参考

### Godot 接口
- `POST /api/godot/connect`: 连接仿真器
- `POST /api/godot/start`: 启动仿真
- `POST /api/godot/stop`: 停止仿真
- `POST /api/godot/update-params`: 更新物理参数
- `GET /api/godot/capabilities`: 查看当前支持的 Godot 接入模式

### 任务接口
- `GET /api/tasks`: 获取任务列表
- `POST /api/tasks`: 创建任务
- `DELETE /api/tasks/{id}`: 删除任务
