"""
AGI-Walker Web 控制面板
基于 FastAPI 的 Web 服务器
"""

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
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

# Mount Static Files
app.mount("/static", StaticFiles(directory="web_panel/static"), name="static")
os.makedirs("robots", exist_ok=True)
app.mount("/robots", StaticFiles(directory="robots"), name="robots")
app.mount("/docs", StaticFiles(directory="docs/build/html", html=True), name="docs")

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

@app.post("/api/godot/update-params")
async def godot_update_params(params: Dict[str, Any]):
    """实时更新参数"""
    if not godot_controller.is_connected():
        raise HTTPException(status_code=400, detail="Godot not connected")
        
    if godot_controller.update_params(params):
        return {"status": "updated", "params": params}
    raise HTTPException(status_code=500, detail="Failed to update parameters")


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


if __name__ == "__main__":
    print("🌐 启动 AGI-Walker Web 控制面板")
    print("访问: http://localhost:8000")
    # 确保在 Windows 上循环策略正确 (Python 3.8+)
    if os.name == 'nt':
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    uvicorn.run(app, host="0.0.0.0", port=8000)
