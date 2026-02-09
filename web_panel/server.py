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
import subprocess
import os
import pydantic

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


@app.post("/api/generate_robot")
async def generate_robot(params: Dict[str, Any]):
    """生成机器人配置 (调用 quick_design.py)"""
    print(f"收到生成请求: {params}")
    
    # 构建命令
    cmd = [
        "python", "quick_design.py",
        "--non-interactive",
        "--name", params.get("name", "web_robot"),
        "--type", params.get("type", "quadruped"),
        "--scenario", params.get("scenario", "performance"),
        "--height", str(params.get("height", 0.4))
    ]
    
    # 处理覆盖参数
    if params.get("mass"):
        cmd.extend(["--mass", str(params["mass"])])
    if params.get("material"):
        cmd.extend(["--material", params["material"]])
        
    try:
        # 执行设计脚本 (假设 server.py 在 web_panel 下，所以 quick_design.py 在上一级)
        # 我们需要切换工作目录到 AGI-Walker 根目录
        root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        
        # 强制子进程使用 UTF-8 输出
        env = os.environ.copy()
        env["PYTHONIOENCODING"] = "utf-8"
        
        # 使用 Popen 以便更好地处理输出编码
        process = subprocess.Popen(
            cmd,
            cwd=root_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=env
        )
        stdout, stderr = process.communicate()
        
        # 子进程现在保证输出 UTF-8
        output_log = stdout.decode('utf-8', errors='replace')
                
        if process.returncode != 0:
             err_log = stderr.decode('utf-8', errors='replace')
             raise subprocess.CalledProcessError(process.returncode, cmd, output=output_log, stderr=err_log)

        # 安全打印到控制台 (避免 Windows GBK 编码错误)
        try:
            print(output_log)
        except UnicodeEncodeError:
            print(output_log.encode('gbk', 'replace').decode('gbk'))
        
        return {
            "status": "success",
            "message": "机器人生成成功",
            "config_path": f"configs/generated/{params.get('name', 'web_robot')}.json",
            "urdf_path": f"exports/{params.get('name', 'web_robot')}.urdf",
            "log": output_log
        }
        
    except subprocess.CalledProcessError as e:
        print(f"生成失败: {e.stderr}")
        return {
            "status": "error",
            "message": "生成脚本执行失败",
            "error": e.stderr
        }, 500
    except Exception as e:
        print(f"系统错误: {e}")
        return {
            "status": "error",
            "message": str(e)
        }, 500


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


from web_panel.godot_controller import godot_controller

# ... (Previous imports)

# 初始化 Godot 控制器回调
# 由于 broadcast 是 async 的，我们需要一个 sync wrapper 或者在线程中 run_coroutine
# 但 FastAPI/Uvicorn 是 async 的。
# 最佳实践：使用 asyncio.run_coroutine_threadsafe 将回调放入 event loop

loop = asyncio.get_event_loop()

def godot_broadcast_adaptor(message: Dict[str, Any]):
    """将同步回调转换为异步广播"""
    try:
        # 尝试获取当前 loop，如果被其他线程调用可能需要 asyncio.run_coroutine_threadsafe
        # 这里假设 server 是主线程，但 callbacks 来自 GodotClient 的接收子线程
        # 所以必须使用 call_soon_threadsafe 或 run_coroutine_threadsafe
        # 注意: loop 必须是 server 运行的那个 loop
        # 我们需要在 startup 事件中捕获 loop
        pass
    except Exception as e:
        print(f"广播适配器错误: {e}")

# 我们在 app startup 时设置这个
@app.on_event("startup")
async def startup_event():
    global loop
    loop = asyncio.get_running_loop()
    
    def async_broadcast(msg):
        asyncio.run_coroutine_threadsafe(broadcast(msg), loop)
        
    godot_controller.set_broadcast_callback(async_broadcast)


# --- Godot API Endpoints ---

class ConnectionRequest(pydantic.BaseModel):
    host: str = "127.0.0.1"
    port: int = 9999

@app.post("/api/godot/connect")
async def godot_connect(req: ConnectionRequest):
    """连接到 Godot"""
    if godot_controller.connect(req.host, req.port):
        return {"status": "connected", "host": req.host, "port": req.port}
    return {"status": "failed", "error": "Connection refused or timeout"}, 500

@app.post("/api/godot/disconnect")
async def godot_disconnect():
    """断开 Godot 连接"""
    godot_controller.disconnect()
    return {"status": "disconnected"}

@app.get("/api/godot/status")
async def godot_status():
    """获取连接状态"""
    return {
        "connected": godot_controller.is_connected(),
        "client_running": godot_controller.client.running
    }

class LoadRobotRequest(pydantic.BaseModel):
    parts: List[Dict[str, Any]]
    connections: List[Dict[str, Any]]

@app.post("/api/godot/load-robot")
async def godot_load_robot(req: LoadRobotRequest):
    """加载机器人配置到 Godot"""
    if not godot_controller.is_connected():
        return {"error": "Godot not connected"}, 400
        
    if godot_controller.load_robot(req.parts, req.connections):
        return {"status": "success", "message": "Robot config sent"}
    return {"status": "error", "message": "Failed to send command"}, 500

class StartSimRequest(pydantic.BaseModel):
    physics: Dict[str, Any] = {"gravity": 9.81, "timestep": 0.01}

@app.post("/api/godot/start")
async def godot_start(req: StartSimRequest):
    """启动仿真"""
    if not godot_controller.is_connected():
        return {"error": "Godot not connected"}, 400
        
    if godot_controller.start_simulation(req.physics):
        return {"status": "started"}
    return {"status": "error"}, 500

@app.post("/api/godot/stop")
async def godot_stop():
    """停止仿真"""
    if not godot_controller.is_connected():
        return {"error": "Godot not connected"}, 400
        
    if godot_controller.stop_simulation():
        return {"status": "stopped"}
    return {"status": "error"}, 500

@app.post("/api/godot/update-params")
async def godot_update_params(params: Dict[str, Any]):
    """实时更新参数"""
    if not godot_controller.is_connected():
        return {"error": "Godot not connected"}, 400
        
    if godot_controller.update_params(params):
        return {"status": "updated", "params": params}
    return {"status": "error"}, 500


# 挂载静态文件
app.mount("/static", StaticFiles(directory="web_panel/static"), name="static")


if __name__ == "__main__":
    print("🌐 启动 AGI-Walker Web 控制面板")
    print("访问: http://localhost:8000")
    # 确保在 Windows 上循环策略正确 (Python 3.8+)
    if os.name == 'nt':
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    uvicorn.run(app, host="0.0.0.0", port=8000)
