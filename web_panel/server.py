"""
AGI-Walker Web 控制面板
基于 FastAPI 的 Web 服务器
"""

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
from typing import List, Dict, Any
import json
import asyncio
from datetime import datetime
import uvicorn

app = FastAPI(title="AGI-Walker Control Panel", version="1.0.0")

# 存储活跃的 WebSocket 连接
active_connections: List[WebSocket] = []

# 任务状态存储
tasks_db: Dict[str, Dict[str, Any]] = {}


@app.get("/")
async def root():
    """主页"""
    return HTMLResponse(content=open("web_panel/static/index.html").read())


@app.get("/api/tasks")
async def get_tasks():
    """获取所有任务"""
    return {"tasks": list(tasks_db.values())}


@app.post("/api/tasks")
async def create_task(task: Dict[str, Any]):
    """创建新任务"""
    task_id = f"task_{len(tasks_db) + 1}"
    task["id"] = task_id
    task["status"] = "pending"
    task["created_at"] = datetime.now().isoformat()
    tasks_db[task_id] = task
    
    # 广播更新
    await broadcast({"type": "task_created", "task": task})
    
    return {"task_id": task_id, "task": task}


@app.get("/api/tasks/{task_id}")
async def get_task(task_id: str):
    """获取单个任务"""
    if task_id not in tasks_db:
        return {"error": "Task not found"}, 404
    return {"task": tasks_db[task_id]}


@app.put("/api/tasks/{task_id}")
async def update_task(task_id: str, updates: Dict[str, Any]):
    """更新任务"""
    if task_id not in tasks_db:
        return {"error": "Task not found"}, 404
    
    tasks_db[task_id].update(updates)
    
    # 广播更新
    await broadcast({"type": "task_updated", "task": tasks_db[task_id]})
    
    return {"task": tasks_db[task_id]}


@app.delete("/api/tasks/{task_id}")
async def delete_task(task_id: str):
    """删除任务"""
    if task_id not in tasks_db:
        return {"error": "Task not found"}, 404
    
    del tasks_db[task_id]
    
    # 广播更新
    await broadcast({"type": "task_deleted", "task_id": task_id})
    
    return {"message": "Task deleted"}


@app.get("/api/system/status")
async def get_system_status():
    """获取系统状态"""
    return {
        "status": "running",
        "tasks_count": len(tasks_db),
        "active_connections": len(active_connections),
        "timestamp": datetime.now().isoformat()
    }


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket 连接 (实时更新)"""
    await websocket.accept()
    active_connections.append(websocket)
    
    try:
        while True:
            # 接收客户端消息
            data = await websocket.receive_text()
            message = json.loads(data)
            
            # 处理消息
            if message["type"] == "ping":
                await websocket.send_json({"type": "pong"})
            
    except WebSocketDisconnect:
        active_connections.remove(websocket)


async def broadcast(message: Dict[str, Any]):
    """广播消息到所有连接"""
    for connection in active_connections:
        try:
            await connection.send_json(message)
        except:
            pass


# 挂载静态文件
app.mount("/static", StaticFiles(directory="web_panel/static"), name="static")


if __name__ == "__main__":
    print("🌐 启动 AGI-Walker Web 控制面板")
    print("访问: http://localhost:8000")
    uvicorn.run(app, host="0.0.0.0", port=8000)
