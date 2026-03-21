"""
AGI-Walker Web 控制面板
基于 FastAPI 的 Web 服务器
"""

from enum import Enum
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse, FileResponse
from typing import List, Dict, Any, Optional
import json
import asyncio
from datetime import datetime
import uvicorn
import subprocess
import os
import pydantic

app = FastAPI(title="AGI-Walker Control Panel", version="1.0.0")

# Mount Static Files
app.mount("/static", StaticFiles(directory="web_panel/static"), name="static")
os.makedirs("robots", exist_ok=True)
app.mount("/robots", StaticFiles(directory="robots"), name="robots")
app.mount("/docs", StaticFiles(directory="docs/build/html", html=True), name="docs")

# 存储活跃的 WebSocket 连接
active_connections: List[WebSocket] = []

# 任务状态存储
tasks_db: Dict[str, Dict[str, Any]] = {}


# --- 统一 WebSocket 消息类型枚举 ---
class WsMessageType(str, Enum):
    ping       = "ping"
    subscribe  = "subscribe"
    command    = "command"


@app.get("/")
async def root():
    """主页（使用 FileResponse 避免编码风险）"""
    html_path = os.path.join(os.path.dirname(__file__), "static", "index.html")
    return FileResponse(html_path, media_type="text/html")


# --- Pydantic 请求模型 ---
class TaskCreate(pydantic.BaseModel):
    name: str
    description: Optional[str] = ""
    priority: Optional[str] = "normal"
    extra: Optional[Dict[str, Any]] = None


@app.get("/api/tasks")
async def get_tasks():
    """获取所有任务"""
    return {"tasks": list(tasks_db.values())}


@app.post("/api/tasks")
async def create_task(task: TaskCreate):
    """创建新任务"""
    task_id = f"task_{len(tasks_db) + 1}"
    task_data = task.model_dump()
    task_data["id"] = task_id
    task_data["status"] = "pending"
    task_data["created_at"] = datetime.now().isoformat()
    tasks_db[task_id] = task_data

    # 广播更新
    await broadcast({"type": "task_created", "task": task_data})

    return {"task_id": task_id, "task": task_data}


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
        raise HTTPException(status_code=404, detail="Task not found")
    return {"task": tasks_db[task_id]}


@app.put("/api/tasks/{task_id}")
async def update_task(task_id: str, updates: Dict[str, Any]):
    """更新任务"""
    if task_id not in tasks_db:
        raise HTTPException(status_code=404, detail="Task not found")
    
    tasks_db[task_id].update(updates)
    
    # 广播更新
    await broadcast({"type": "task_updated", "task": tasks_db[task_id]})
    
    return {"task": tasks_db[task_id]}


@app.delete("/api/tasks/{task_id}")
async def delete_task(task_id: str):
    """删除任务"""
    if task_id not in tasks_db:
        raise HTTPException(status_code=404, detail="Task not found")
    
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
            data = await websocket.receive_text()
            try:
                message = json.loads(data)
            except json.JSONDecodeError:
                await websocket.send_json({"type": "error", "message": "invalid JSON"})
                continue

            msg_type = message.get("type", "")
            # 消息类型校验：只允许已知类型
            try:
                WsMessageType(msg_type)
            except ValueError:
                await websocket.send_json({
                    "type": "error",
                    "message": f"unknown message type: {msg_type!r}",
                    "allowed": [m.value for m in WsMessageType]
                })
                continue

            if msg_type == WsMessageType.ping:
                await websocket.send_json({"type": "pong"})

    except WebSocketDisconnect:
        active_connections.remove(websocket)


async def broadcast(message: Dict[str, Any]):
    """广播消息到所有连接"""
    disconnected = []
    for connection in active_connections:
        try:
            await connection.send_json(message)
        except Exception:
            disconnected.append(connection)
    for conn in disconnected:
        active_connections.remove(conn)


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

# --- Zenoh Monitor ---
zenoh_session = None
distributed_db: Dict[str, Dict[str, Any]] = {}

@app.on_event("startup")
async def startup_event():
    global loop
    loop = asyncio.get_running_loop()
    
    def async_broadcast(msg):
        asyncio.run_coroutine_threadsafe(broadcast(msg), loop)
        
    godot_controller.set_broadcast_callback(async_broadcast)

    # Initialize Zenoh
    print("🌐 [Zenoh] Initializing Monitor Node...")
    try:
        import zenoh
        z_conf = zenoh.Config()
        # Connect to the known Sidecar/Learner port for peeling
        z_conf.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
        
        global zenoh_session
        zenoh_session = zenoh.open(z_conf)
        
        def on_obs(sample):
            try:
                # Key: ag/<id>/obs
                key_parts = str(sample.key_expr).split('/')
                if len(key_parts) < 3: return
                actor_id = key_parts[-2]
                
                # Payload (Decompression)
                import zlib
                raw_bytes = sample.payload.to_bytes() if hasattr(sample.payload, 'to_bytes') else sample.payload
                
                if len(raw_bytes) > 0:
                    header = raw_bytes[0]
                    data_content = raw_bytes[1:]
                    
                    if header == 1: # Zlib
                        decompressed = zlib.decompress(data_content)
                        payload = json.loads(decompressed.decode('utf-8'))
                    elif header == 0: # Raw with header
                        payload = json.loads(data_content.decode('utf-8'))
                    else: # Legacy/Raw
                        payload = json.loads(raw_bytes.decode('utf-8'))
                else:
                    payload = {}
                
                # Update DB
                distributed_db[actor_id] = {
                    "id": actor_id,
                    "status": "active",
                    "last_seen": datetime.now().isoformat(),
                    "data": payload
                }
                
                # Broadcast Update (Throttled ideally, but raw for now)
                async_broadcast({
                    "type": "distributed_update",
                    "actor_id": actor_id,
                    "data": distributed_db[actor_id]
                })
            except Exception as e:
                print(f"❌ [Zenoh] Error processing obs: {e}")
                
        print("   ✅ Subscribing to ag/*/obs")
        zenoh_session.declare_subscriber("ag/*/obs", on_obs)
        
    except Exception as e:
        print(f"❌ [Zenoh] Failed to init: {e}")

@app.on_event("shutdown")
def shutdown_event():
    if zenoh_session:
        zenoh_session.close()

@app.get("/api/distributed/status")
async def get_distributed_status():
    """Get snapshot of distributed actors"""
    return {"actors": distributed_db}



# --- Godot API Endpoints ---

class ConnectionRequest(pydantic.BaseModel):
    host: str = "127.0.0.1"
    port: int = 9999

@app.post("/api/godot/connect")
async def godot_connect(req: ConnectionRequest):
    """连接到 Godot"""
    if godot_controller.connect(req.host, req.port):
        return {"status": "connected", "host": req.host, "port": req.port}
    raise HTTPException(status_code=500, detail="Connection refused or timeout")

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
        raise HTTPException(status_code=400, detail="Godot not connected")
        
    if godot_controller.load_robot(req.parts, req.connections):
        return {"status": "success", "message": "Robot config sent"}
    raise HTTPException(status_code=500, detail="Failed to send command")

class StartSimRequest(pydantic.BaseModel):
    physics: Dict[str, Any] = {"gravity": 9.81, "timestep": 0.01}

@app.post("/api/godot/start")
async def godot_start(req: StartSimRequest):
    """启动仿真"""
    if not godot_controller.is_connected():
        raise HTTPException(status_code=400, detail="Godot not connected")
        
    if godot_controller.start_simulation(req.physics):
        return {"status": "started"}
    raise HTTPException(status_code=500, detail="Failed to start simulation")

@app.post("/api/godot/stop")
async def godot_stop():
    """停止仿真"""
    if not godot_controller.is_connected():
        raise HTTPException(status_code=400, detail="Godot not connected")
        
    if godot_controller.stop_simulation():
        return {"status": "stopped"}
    raise HTTPException(status_code=500, detail="Failed to stop simulation")

class UpdateParamsRequest(pydantic.BaseModel):
    params: Dict[str, Any]


@app.post("/api/godot/update-params")
async def godot_update_params(req: UpdateParamsRequest):
    """实时更新参数"""
    if not godot_controller.is_connected():
        raise HTTPException(status_code=400, detail="Godot not connected")

    if godot_controller.update_params(req.params):
        return {"status": "updated", "params": req.params}
    raise HTTPException(status_code=500, detail="Failed to update parameters")


# --- Sim2Real API ---

class Sim2RealAnalyzeRequest(pydantic.BaseModel):
    mock: bool = True

@app.post("/api/sim2real/analyze")
async def analyze_sim2real_gap(req: Sim2RealAnalyzeRequest):
    """提取或计算最新的一段 Sim2Real 差距图谱"""
    import sys
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if root_dir not in sys.path:
        sys.path.insert(0, root_dir)
    
    try:
        from python_controller.sim2real_gap import Sim2RealGapEstimator
        estimator = Sim2RealGapEstimator(data_dir=os.path.join(root_dir, "offline_data", "sim2real"))
        
        # 制造一套有偏差的 mock_data 以供网页端演示
        sim_state = {
            "sensors": {
                "imu": {"orient": [5.0, -3.0, 0.0], "gyro": [0.1, 0.0, 0.0]},
                "joints": {
                    "hip_left": {"angle": 10.0, "velocity": 0.5},
                    "hip_right": {"angle": -8.0, "velocity": -0.3},
                },
            },
            "torso_height": 1.45,
        }
        real_state = {
            "sensors": {
                "imu": {"orient": [5.5, -2.8, 0.1], "gyro": [0.12, 0.02, 0.01]},
                "joints": {
                    "hip_left": {"angle": 10.5, "velocity": 0.55},
                    "hip_right": {"angle": -7.5, "velocity": -0.25},
                },
            },
            "torso_height": 1.43,
        }
        
        gap_report = estimator.estimate_gap(sim_state, real_state)
        return {"status": "success", "data": gap_report}
    except Exception as e:
        print(f"Sim2Real Error: {e}")
        return {"status": "error", "message": str(e)}

# --- Agent Command API ---


class CommandRequest(pydantic.BaseModel):
    command: str

@app.post("/api/agent/parse-command")
async def parse_command(req: CommandRequest):
    """解析自然语言指令并返回 Robot Config"""
    from web_panel.command_parser import CommandParser
    
    try:
        parser = CommandParser()
        config = parser.parse(req.command)
        return {"status": "success", "config": config}
    except Exception as e:
        print(f"Command parse error: {e}")
        return {"status": "error", "message": str(e)}


# ---------------------------------------------------------------------------
# Godot Studio Agent API (Milestone 2 integration)
# ---------------------------------------------------------------------------

class GodotAgentCommandRequest(pydantic.BaseModel):
    command: str
    context: Optional[Dict[str, Any]] = None
    godot_project_path: Optional[str] = None

class GodotAgentPipelineRequest(pydantic.BaseModel):
    commands: List[str]
    context: Optional[Dict[str, Any]] = None

def get_godot_agent_router():
    import sys
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    agent_dir = os.path.join(root_dir, "godot_studio_agent")
    if agent_dir not in sys.path:
        sys.path.insert(0, agent_dir)
        
    if not hasattr(app, "godot_agent_router"):
        from agent_system.router import GodotStudioRouter
        app.godot_agent_router = GodotStudioRouter()
    return app.godot_agent_router

@app.post("/execute")
async def execute_godot_agent_command(req: GodotAgentCommandRequest):
    """执行 Godot Studio Agent 命令 (供 Godot 插件调用)"""
    try:
        router = get_godot_agent_router()
        if req.godot_project_path:
            router.godot_cli.project_path = req.godot_project_path
        result = router.execute(req.command, req.context)
        result["timestamp"] = datetime.now().isoformat()
        return result
    except Exception as e:
        print(f"Agent Execute Error: {e}")
        return {"status": "error", "message": str(e), "data": {"code": ""}}

@app.post("/pipeline")
async def execute_godot_agent_pipeline(req: GodotAgentPipelineRequest):
    """执行多步骤命令流水线"""
    try:
        router = get_godot_agent_router()
        results = router.execute_pipeline(req.commands)
        return {
            "success": all(r.get("success") for r in results),
            "steps": len(results),
            "results": results,
        }
    except Exception as e:
        return {"success": False, "message": str(e)}

@app.get("/roles")
async def get_godot_agent_roles():
    """获取所有可用角色信息"""
    try:
        return {"roles": get_godot_agent_router().get_roles_info()}
    except Exception as e:
        return {"status": "error", "message": str(e)}

# ---------------------------------------------------------------------------
# Godot Agent Skills API
# ---------------------------------------------------------------------------
import glob

@app.get("/api/godot_skills/list")
async def list_godot_skills():
    """获取所有可用的 Godot 技能"""
    try:
        skills_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "godot_studio_agent", "agent_system", "godot_skills")
        skills = []
        for f in glob.glob(os.path.join(skills_dir, "*.json")):
            with open(f, "r", encoding="utf-8") as fp:
                data = json.load(fp)
                skills.append({
                    "id": data.get("id"),
                    "name": data.get("name"),
                    "description": data.get("description")
                })
        return {"status": "success", "skills": skills}
    except Exception as e:
        return {"status": "error", "message": str(e)}

class GodotSkillApplyRequest(pydantic.BaseModel):
    skill_id: str

@app.post("/api/godot_skills/apply")
async def apply_godot_skill(req: GodotSkillApplyRequest):
    """获取完整单个神盾局技能配置"""
    try:
        skills_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "godot_studio_agent", "agent_system", "godot_skills")
        skill_file = os.path.join(skills_dir, f"{req.skill_id}.json")
        if not os.path.exists(skill_file):
            return {"status": "error", "message": "Skill not found"}
        with open(skill_file, "r", encoding="utf-8") as fp:
            data = json.load(fp)
            return {"status": "success", "data": data}
    except Exception as e:
        return {"status": "error", "message": str(e)}

# --- Parts Store API ---

@app.get("/api/parts/market")
async def get_parts_market():
    """获取云端零件市场列表 (模拟)"""
    market_path = os.path.join("parts_library", "cloud_repo_mock", "manifest.json")
    try:
        if not os.path.exists(market_path):
            return {"status": "error", "message": "Market unavailable"}
            
        with open(market_path, 'r', encoding='utf-8') as f:
            manifest = json.load(f)
            
        # 遍历目录收集所有零件摘要
        parts_list = []
        base_dir = os.path.join("parts_library", "cloud_repo_mock", "parts")
        
        for root, dirs, files in os.walk(base_dir):
            for file in files:
                if file.endswith(".json"):
                    try:
                        with open(os.path.join(root, file), 'r', encoding='utf-8') as pf:
                            part_data = json.load(pf)
                            # 提取摘要信息
                            parts_list.append({
                                "id": part_data.get("id"),
                                "name": part_data.get("name"),
                                "type": part_data.get("type"),
                                "price": part_data.get("price"),
                                "supplier": part_data.get("supplier"),
                                "category": os.path.basename(root) # 使用文件夹名作为分类
                            })
                    except Exception as e:
                        print(f"Error reading part {file}: {e}")
                        
        return {"status": "success", "manifest": manifest, "parts": parts_list}
        
    except Exception as e:
        return {"status": "error", "message": str(e)}


class ImportPartRequest(pydantic.BaseModel):
    part_id: str
    category: str

@app.post("/api/parts/import")
async def import_part(req: ImportPartRequest):
    """导入零件到本地库"""
    # 1. 查找源文件
    source_path = os.path.join("parts_library", "cloud_repo_mock", "parts", req.category)
    # 模糊匹配文件名 (因为 id 是 MT-C01, 文件名可能是 mt_c01.json)
    target_file = None
    if os.path.exists(source_path):
        for file in os.listdir(source_path):
            if file.endswith(".json"):
                with open(os.path.join(source_path, file), 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    if data.get("id") == req.part_id:
                        target_file = os.path.join(source_path, file)
                        break
    
    if not target_file:
        raise HTTPException(status_code=404, detail="Part not found in market")
        
    try:
        # 2. 读取零件数据
        with open(target_file, 'r', encoding='utf-8') as f:
            new_part = json.load(f)
            
        # 3. 读取本地库
        local_db_path = os.path.join("parts_library", "complete_parts_database.json")
        with open(local_db_path, 'r', encoding='utf-8') as f:
            local_db = json.load(f)
            
        # 4. 检查是否已存在
        category_list = local_db["parts"].get(req.category, [])
        for part in category_list:
            if part["id"] == req.part_id:
                return {"status": "skipped", "message": "Part already exists"}
                
        # 5. 添加并保存
        if req.category not in local_db["parts"]:
            local_db["parts"][req.category] = []
            
        local_db["parts"][req.category].append(new_part)
        
        with open(local_db_path, 'w', encoding='utf-8') as f:
            json.dump(local_db, f, indent=4, ensure_ascii=False)
            
        return {"status": "success", "message": f"Imported {new_part['name']}"}
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))



# ---------------------------------------------------------------------------
# Skills System API
# ---------------------------------------------------------------------------

@app.get("/api/skills/list")
async def skills_list():
    """列出所有可用 Skills 及其元数据"""
    try:
        import sys
        root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        if root_dir not in sys.path:
            sys.path.insert(0, root_dir)
        from agi_walker.skills_loader import get_skills_loader
        loader = get_skills_loader()
        skills = loader.get_skills_list()
        result = []
        for skill in skills:
            result.append({
                "name": skill.name,
                "display_name": skill.display_name,
                "description": skill.description,
                "version": skill.version,
                "category": skill.category,
                "requires": skill.requires,
            })
        return {"status": "success", "skills": result, "count": len(result)}
    except Exception as e:
        return {"status": "error", "message": str(e)}


class ModelRequest(pydantic.BaseModel):
    name: str = "web_robot"
    robot_type: str = "biped"  # biped | quadruped
    torso_height: float = 0.5
    torso_mass: float = 5.0
    thigh_length: float = 0.3
    shin_length: float = 0.3

@app.post("/api/skills/model")
async def skills_model(req: ModelRequest):
    """调用 robot-modeling Skill 构建机器人模型"""
    try:
        import sys
        root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        if root_dir not in sys.path:
            sys.path.insert(0, root_dir)
        import importlib.util
        from pathlib import Path
        skill_file = Path(root_dir) / "agi_walker" / "skills" / "robot-modeling" / "__init__.py"
        spec = importlib.util.spec_from_file_location("robot_modeling_skill", skill_file)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        builder = module.RobotBuilder(req.name).add_torso(height=req.torso_height, mass=req.torso_mass)
        if req.robot_type == "biped":
            builder = builder.add_leg_pair(thigh_length=req.thigh_length, shin_length=req.shin_length)
        robot = builder.build()
        config_path = os.path.join(root_dir, "configs", f"{req.name}.json")
        robot.save(config_path)
        return {
            "status": "success",
            "robot_name": robot.name,
            "parts_count": len(robot.parts),
            "config_path": config_path,
            "robot_dict": robot.to_dict()
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}


class OptimizeRequest(pydantic.BaseModel):
    config_path: str
    target_com_height: float = 0.22
    max_iterations: int = 50

@app.post("/api/skills/optimize")
async def skills_optimize(req: OptimizeRequest):
    """调用 parameter-optimizer Skill 对机器人参数进行优化"""
    try:
        import sys
        root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        if root_dir not in sys.path:
            sys.path.insert(0, root_dir)
        import importlib.util, json
        from pathlib import Path
        skill_file = Path(root_dir) / "agi_walker" / "skills" / "parameter-optimizer" / "__init__.py"
        spec = importlib.util.spec_from_file_location("param_opt_skill", skill_file)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        with open(req.config_path, encoding="utf-8") as f:
            robot_dict = json.load(f)
        result = module.optimize_mass_distribution(
            robot_dict,
            target_com_height=req.target_com_height,
            max_iterations=req.max_iterations
        )
        return {
            "status": "success",
            "success": result.success,
            "iterations": result.iterations,
            "final_com_height": getattr(result, "final_com_height", None),
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}


class ExportURDFRequest(pydantic.BaseModel):
    config_path: str
    output_name: str = ""

@app.post("/api/skills/export-urdf")
async def skills_export_urdf(req: ExportURDFRequest):
    """调用 urdf-generator Skill 将机器人配置转换为 URDF"""
    try:
        import sys
        root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        if root_dir not in sys.path:
            sys.path.insert(0, root_dir)
        import importlib.util
        from pathlib import Path
        skill_file = Path(root_dir) / "agi_walker" / "skills" / "urdf-generator" / "__init__.py"
        spec = importlib.util.spec_from_file_location("urdf_gen_skill", skill_file)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        name = req.output_name or Path(req.config_path).stem
        exports_dir = Path(root_dir) / "exports"
        exports_dir.mkdir(exist_ok=True)
        output_path = str(exports_dir / f"{name}.urdf")
        module.convert_to_urdf(req.config_path, output_path)
        is_valid = module.validate_urdf(output_path)
        return {
            "status": "success",
            "urdf_path": output_path,
            "valid": is_valid,
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}


class PipelineRequest(pydantic.BaseModel):
    name: str = "pipeline_robot"
    robot_type: str = "biped"
    torso_height: float = 0.5
    torso_mass: float = 5.0
    thigh_length: float = 0.3
    shin_length: float = 0.3
    target_com_height: float = 0.22
    max_iterations: int = 30
    export_urdf: bool = True

@app.post("/api/skills/pipeline")
async def skills_pipeline(req: PipelineRequest):
    """一键完整流水线: 建模 → 参数优化 → URDF 导出"""
    log = []
    try:
        # Step 1: Model
        model_req = ModelRequest(
            name=req.name, robot_type=req.robot_type,
            torso_height=req.torso_height, torso_mass=req.torso_mass,
            thigh_length=req.thigh_length, shin_length=req.shin_length
        )
        model_res = await skills_model(model_req)
        if model_res["status"] != "success":
            return {"status": "error", "step": "model", "message": model_res["message"], "log": log}
        log.append(f"✅ 建模完成: {model_res['robot_name']} ({model_res['parts_count']} parts)")

        # Step 2: Optimize
        opt_req = OptimizeRequest(
            config_path=model_res["config_path"],
            target_com_height=req.target_com_height,
            max_iterations=req.max_iterations
        )
        opt_res = await skills_optimize(opt_req)
        if opt_res["status"] != "success":
            log.append(f"⚠️ 参数优化失败 (scipy 可能未安装): {opt_res['message']}")
        else:
            log.append(f"✅ 优化完成: {opt_res['iterations']} 次迭代, 成功={opt_res['success']}")

        # Step 3: Export URDF
        urdf_path = None
        if req.export_urdf:
            urdf_req = ExportURDFRequest(config_path=model_res["config_path"], output_name=req.name)
            urdf_res = await skills_export_urdf(urdf_req)
            if urdf_res["status"] != "success":
                log.append(f"⚠️ URDF 导出失败: {urdf_res['message']}")
            else:
                urdf_path = urdf_res["urdf_path"]
                log.append(f"✅ URDF 导出完成: {urdf_path} (valid={urdf_res['valid']})")

        return {
            "status": "success",
            "log": log,
            "config_path": model_res["config_path"],
            "urdf_path": urdf_path,
            "robot_dict": model_res["robot_dict"]
        }
    except Exception as e:
        return {"status": "error", "message": str(e), "log": log}


# ===========================================================================
# Godot 引擎远程控制桥接层 (Godot Bridge)
# ===========================================================================
import threading
import socket
import struct
import time

GODOT_PROJECT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "godot_project")

class GodotBridge:
    """管理 Godot 进程生命周期，并透过 TCP 与其通信。"""
    
    def __init__(self):
        self.process: Optional[subprocess.Popen] = None
        self.sock: Optional[socket.socket] = None
        self.last_sensor: Dict = {}
        self.tcp_lock = threading.Lock()
        self._tcp_host = "127.0.0.1"
        self._tcp_port = 9000  # 与 tcp_server.gd 一致

    def launch(self, scene: str = "demo_generated_biped.tscn", godot_exe: str = "") -> Dict:
        """启动 Godot 进程（Headless 或窗口模式）"""
        if self.process and self.process.poll() is None:
            return {"status": "already_running", "pid": self.process.pid}
        
        exe = godot_exe or self._find_godot_exe()
        if not exe:
            return {"status": "error", "message": "未找到 Godot 可执行文件，请在请求中传入 godot_exe 路径"}
        
        scene_path = os.path.join(GODOT_PROJECT_DIR, scene)
        if not os.path.exists(scene_path):
            return {"status": "error", "message": f"场景文件不存在: {scene_path}"}
        
        cmd = [exe, "--path", GODOT_PROJECT_DIR, scene_path]
        try:
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                creationflags=subprocess.CREATE_NEW_CONSOLE if os.name == "nt" else 0
            )
            # 等待引擎启动后自动连接 TCP
            threading.Thread(target=self._delayed_tcp_connect, daemon=True).start()
            return {"status": "launched", "pid": self.process.pid, "scene": scene, "exe": exe}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def _find_godot_exe(self) -> str:
        """智能搜索 Godot 可执行文件"""
        candidates = [
            "godot",
            r"C:\Program Files\Godot\Godot_v4.2.2-stable_win64.exe",
            r"C:\Godot\Godot_v4.2.2-stable_win64.exe",
            r"D:\Godot\Godot_v4.2.2-stable_win64.exe",
        ]
        for c in candidates:
            if os.path.isfile(c):
                return c
            # 也尝试 PATH 查找
            try:
                subprocess.run([c, "--version"], capture_output=True, timeout=2)
                return c
            except:
                continue
        return ""

    def _delayed_tcp_connect(self, delay: float = 3.0):
        """延迟连接 TCP（等 Godot 完全起来）"""
        time.sleep(delay)
        self._connect_tcp()

    def _connect_tcp(self) -> bool:
        with self.tcp_lock:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(3.0)
                s.connect((self._tcp_host, self._tcp_port))
                s.settimeout(None)
                self.sock = s
                return True
            except Exception as e:
                self.sock = None
                return False

    def stop(self) -> Dict:
        """终止 Godot 进程"""
        if self.sock:
            try: self.sock.close()
            except: pass
            self.sock = None
        if self.process:
            if self.process.poll() is None:
                self.process.terminate()
                self.process = None
                return {"status": "stopped"}
            else:
                self.process = None
                return {"status": "was_not_running"}
        return {"status": "no_process"}

    def is_running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def is_connected(self) -> bool:
        return self.sock is not None

    def _send_recv(self, payload: Dict) -> Optional[Dict]:
        """使用长度前缀协议与 Godot 通信"""
        if not self.sock:
            if not self._connect_tcp():
                return None
        with self.tcp_lock:
            try:
                data = json.dumps(payload).encode("utf-8")
                msg = struct.pack("<I", len(data)) + data
                self.sock.sendall(msg)
                
                # 读取响应长度
                raw_len = b""
                while len(raw_len) < 4:
                    chunk = self.sock.recv(4 - len(raw_len))
                    if not chunk: return None
                    raw_len += chunk
                resp_len = struct.unpack("<I", raw_len)[0]
                
                # 读取响应内容
                raw_body = b""
                while len(raw_body) < resp_len:
                    chunk = self.sock.recv(resp_len - len(raw_body))
                    if not chunk: return None
                    raw_body += chunk
                return json.loads(raw_body.decode("utf-8"))
            except Exception as e:
                self.sock = None
                return None

    def get_sensors(self) -> Dict:
        resp = self._send_recv({"type": "reset"}) or {}
        if resp:
            self.last_sensor = resp
        return resp

    def send_motor(self, hip_left: float, hip_right: float) -> Dict:
        resp = self._send_recv({"type": "step", "action": [hip_left, hip_right]}) or {}
        if resp:
            self.last_sensor = resp
        return resp


# 全局单例桥接对象
_godot_bridge = GodotBridge()


class GodotLaunchRequest(pydantic.BaseModel):
    scene: str = "demo_generated_biped.tscn"
    godot_exe: str = ""

class GodotMotorRequest(pydantic.BaseModel):
    hip_left: float = 0.0
    hip_right: float = 0.0


@app.post("/api/godot/launch")
async def godot_launch(req: GodotLaunchRequest):
    """启动 Godot 引擎并加载指定场景"""
    result = _godot_bridge.launch(scene=req.scene, godot_exe=req.godot_exe)
    return result

@app.post("/api/godot/stop")
async def godot_stop():
    """停止 Godot 引擎进程"""
    return _godot_bridge.stop()

@app.get("/api/godot/status")
async def godot_status():
    """获取 Godot 进程状态与最新传感器读数"""
    return {
        "engine_running": _godot_bridge.is_running(),
        "tcp_connected": _godot_bridge.is_connected(),
        "last_sensor": _godot_bridge.last_sensor,
        "pid": _godot_bridge.process.pid if _godot_bridge.process else None,
    }

@app.post("/api/godot/control")
async def godot_control(req: GodotMotorRequest):
    """向运行中的 Godot 机器人发送电机速度指令"""
    if not _godot_bridge.is_connected() and not _godot_bridge._connect_tcp():
        return {"status": "error", "message": "未连接到 Godot TCP 服务器，请先启动场景"}
    result = _godot_bridge.send_motor(req.hip_left, req.hip_right)
    return {"status": "ok", "response": result}


if __name__ == "__main__":
    print("🌐 启动 AGI-Walker Web 控制面板")
    print("访问: http://localhost:8000")
    # 确保在 Windows 上循环策略正确 (Python 3.8+)
    if os.name == 'nt':
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    uvicorn.run(app, host="0.0.0.0", port=8000)
