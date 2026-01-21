# Web 控制面板使用指南

## 简介

AGI-Walker Web 控制面板提供了一个直观的 Web 界面,用于管理训练任务、监控系统状态和查看实时数据。

## 快速开始

### 1. 安装依赖
```bash
pip install fastapi uvicorn websockets
```

### 2. 启动服务器
```bash
python web_panel/server.py
```

### 3. 访问界面
打开浏览器访问: http://localhost:8000

## 功能特性

### 📊 系统状态监控
- 实时显示活跃任务数量
- WebSocket 连接状态
- 系统运行状态

### 📋 任务管理
- 创建新训练任务
- 查看任务列表
- 实时更新任务状态

### ⚡ 实时通信
- WebSocket 实时推送
- 自动重连机制
- 低延迟更新

## API 文档

### 获取所有任务
```http
GET /api/tasks
```

**响应**:
```json
{
  "tasks": [
    {
      "id": "task_1",
      "name": "楼梯攀爬训练",
      "status": "running",
      "created_at": "2026-01-21T12:00:00"
    }
  ]
}
```

### 创建任务
```http
POST /api/tasks
Content-Type: application/json

{
  "name": "新任务",
  "type": "training",
  "algorithm": "PPO"
}
```

### 更新任务
```http
PUT /api/tasks/{task_id}
Content-Type: application/json

{
  "status": "completed",
  "reward": 12.5
}
```

### 删除任务
```http
DELETE /api/tasks/{task_id}
```

### 系统状态
```http
GET /api/system/status
```

## WebSocket 协议

### 连接
```javascript
const ws = new WebSocket('ws://localhost:8000/ws');
```

### 消息格式
```json
{
  "type": "task_created",
  "task": {
    "id": "task_1",
    "name": "新任务"
  }
}
```

### 消息类型
- `task_created` - 任务创建
- `task_updated` - 任务更新
- `task_deleted` - 任务删除
- `ping/pong` - 心跳检测

## 集成示例

### Python 客户端
```python
import requests

# 创建任务
response = requests.post('http://localhost:8000/api/tasks', json={
    "name": "楼梯攀爬训练",
    "type": "training",
    "algorithm": "PPO",
    "timesteps": 1000000
})

task_id = response.json()['task_id']

# 更新任务状态
requests.put(f'http://localhost:8000/api/tasks/{task_id}', json={
    "status": "running",
    "progress": 0.5
})
```

## 自定义扩展

### 添加新的 API 端点
```python
# web_panel/server.py

@app.get("/api/custom/endpoint")
async def custom_endpoint():
    return {"data": "custom"}
```

### 修改前端样式
编辑 `web_panel/static/index.html` 中的 `<style>` 部分。

### 添加新功能
1. 在 `server.py` 添加 API 端点
2. 在 `index.html` 添加前端逻辑
3. 通过 WebSocket 实现实时更新

## 部署

### 生产环境
```bash
# 使用 Gunicorn
pip install gunicorn
gunicorn web_panel.server:app -w 4 -k uvicorn.workers.UvicornWorker
```

### Docker 部署
```dockerfile
# 已包含在主 Dockerfile 中
EXPOSE 8000
CMD ["python", "web_panel/server.py"]
```

## 故障排除

### 无法连接 WebSocket
- 检查防火墙设置
- 确保端口 8000 未被占用

### 任务列表不更新
- 刷新页面
- 检查浏览器控制台错误

### API 返回 404
- 确认服务器正在运行
- 检查 URL 拼写

## 未来计划

- [ ] 添加用户认证
- [ ] 训练曲线可视化
- [ ] 模型性能对比
- [ ] 日志查看器
- [ ] 资源使用监控
