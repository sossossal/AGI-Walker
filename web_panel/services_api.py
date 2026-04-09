import importlib.util
import json
import logging
import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List

import pydantic
from fastapi import APIRouter
from fastapi.responses import JSONResponse

logger = logging.getLogger(__name__)


def _get_root_dir() -> str:
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _quick_design_script_path() -> Path:
    return Path(_get_root_dir()) / "tools" / "quick_design.py"


def _ensure_root_on_path() -> str:
    root_dir = _get_root_dir()
    if root_dir not in sys.path:
        sys.path.insert(0, root_dir)
    return root_dir


def _load_skill_module(skill_folder: str, module_name: str):
    root_dir = _ensure_root_on_path()
    skill_file = Path(root_dir) / "agi_walker" / "skills" / skill_folder / "__init__.py"
    spec = importlib.util.spec_from_file_location(module_name, skill_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ModelRequest(pydantic.BaseModel):
    name: str = "web_robot"
    robot_type: str = "biped"
    torso_height: float = 0.5
    torso_mass: float = 5.0
    thigh_length: float = 0.3
    shin_length: float = 0.3


class OptimizeRequest(pydantic.BaseModel):
    config_path: str
    target_com_height: float = 0.22
    max_iterations: int = 50


class ExportURDFRequest(pydantic.BaseModel):
    config_path: str
    output_name: str = ""


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


def generate_robot(params: Dict[str, Any]) -> tuple[Dict[str, Any], int]:
    logger.info(f"收到生成请求: {params}")

    script_path = _quick_design_script_path()
    if not script_path.exists():
        return {
            "status": "error",
            "message": "quick_design 脚本不存在",
            "error": str(script_path),
        }, 500

    cmd = [
        sys.executable,
        str(script_path),
        "--non-interactive",
        "--name",
        params.get("name", "web_robot"),
        "--type",
        params.get("type", "quadruped"),
        "--scenario",
        params.get("scenario", "performance"),
        "--height",
        str(params.get("height", 0.4)),
    ]

    if params.get("mass"):
        cmd.extend(["--mass", str(params["mass"])])
    if params.get("material"):
        cmd.extend(["--material", params["material"]])

    try:
        root_dir = _get_root_dir()
        env = os.environ.copy()
        env["PYTHONIOENCODING"] = "utf-8"
        env["PYTHONUTF8"] = "1"

        process = subprocess.Popen(
            cmd,
            cwd=root_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=env,
        )
        stdout, stderr = process.communicate()
        output_log = stdout.decode("utf-8", errors="replace")

        if process.returncode != 0:
            err_log = stderr.decode("utf-8", errors="replace")
            raise subprocess.CalledProcessError(
                process.returncode,
                cmd,
                output=output_log,
                stderr=err_log,
            )

        try:
            logger.info(output_log)
        except UnicodeEncodeError:
            logger.info(output_log.encode("gbk", "replace").decode("gbk"))

        robot_name = params.get("name", "web_robot")
        return {
            "status": "success",
            "message": "机器人生成成功",
            "config_path": f"configs/generated/{robot_name}.json",
            "urdf_path": f"exports/{robot_name}.urdf",
            "log": output_log,
        }, 200
    except subprocess.CalledProcessError as e:
        logger.info(f"生成失败: {e.stderr}")
        return {
            "status": "error",
            "message": "生成脚本执行失败",
            "error": e.stderr,
        }, 500
    except Exception as e:
        logger.info(f"系统错误: {e}")
        return {"status": "error", "message": str(e)}, 500


def list_skills() -> Dict[str, Any]:
    try:
        _ensure_root_on_path()
        from agi_walker.skills_loader import get_skills_loader

        loader = get_skills_loader()
        skills = loader.get_skills_list()
        result = []
        for skill in skills:
            result.append(
                {
                    "name": skill.name,
                    "display_name": getattr(skill, "display_name", skill.name),
                    "description": getattr(skill, "description", ""),
                    "version": getattr(skill, "version", ""),
                    "category": getattr(skill, "category", ""),
                    "requires": getattr(skill, "requires", []),
                }
            )
        return {"status": "success", "skills": result, "count": len(result)}
    except Exception as e:
        return {"status": "error", "message": str(e)}


async def skills_model(req: ModelRequest) -> Dict[str, Any]:
    try:
        root_dir = _ensure_root_on_path()
        module = _load_skill_module("robot-modeling", "robot_modeling_skill")

        builder = module.RobotBuilder(req.name).add_torso(
            height=req.torso_height,
            mass=req.torso_mass,
        )
        if req.robot_type == "biped":
            builder = builder.add_leg_pair(
                thigh_length=req.thigh_length,
                shin_length=req.shin_length,
            )
        robot = builder.build()
        config_path = os.path.join(root_dir, "configs", f"{req.name}.json")
        robot.save(config_path)
        return {
            "status": "success",
            "robot_name": robot.name,
            "parts_count": len(robot.parts),
            "config_path": config_path,
            "robot_dict": robot.to_dict(),
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}


async def skills_optimize(req: OptimizeRequest) -> Dict[str, Any]:
    try:
        module = _load_skill_module("parameter-optimizer", "param_opt_skill")
        with open(req.config_path, encoding="utf-8") as f:
            robot_dict = json.load(f)
        result = module.optimize_mass_distribution(
            robot_dict,
            target_com_height=req.target_com_height,
            max_iterations=req.max_iterations,
        )
        return {
            "status": "success",
            "success": result.success,
            "iterations": result.iterations,
            "final_com_height": getattr(result, "final_com_height", None),
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}


async def skills_export_urdf(req: ExportURDFRequest) -> Dict[str, Any]:
    try:
        root_dir = _ensure_root_on_path()
        module = _load_skill_module("urdf-generator", "urdf_gen_skill")

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


async def skills_pipeline(req: PipelineRequest) -> Dict[str, Any]:
    log: List[str] = []
    try:
        model_req = ModelRequest(
            name=req.name,
            robot_type=req.robot_type,
            torso_height=req.torso_height,
            torso_mass=req.torso_mass,
            thigh_length=req.thigh_length,
            shin_length=req.shin_length,
        )
        model_res = await skills_model(model_req)
        if model_res["status"] != "success":
            return {
                "status": "error",
                "step": "model",
                "message": model_res["message"],
                "log": log,
            }
        log.append(
            f"Model created: {model_res['robot_name']} ({model_res['parts_count']} parts)"
        )

        opt_req = OptimizeRequest(
            config_path=model_res["config_path"],
            target_com_height=req.target_com_height,
            max_iterations=req.max_iterations,
        )
        opt_res = await skills_optimize(opt_req)
        if opt_res["status"] != "success":
            log.append(f"Optimize skipped/failed: {opt_res['message']}")
        else:
            log.append(
                f"Optimize completed: {opt_res['iterations']} iterations, success={opt_res['success']}"
            )

        urdf_path = None
        if req.export_urdf:
            urdf_req = ExportURDFRequest(
                config_path=model_res["config_path"], output_name=req.name
            )
            urdf_res = await skills_export_urdf(urdf_req)
            if urdf_res["status"] != "success":
                log.append(f"URDF export failed: {urdf_res['message']}")
            else:
                urdf_path = urdf_res["urdf_path"]
                log.append(f"URDF exported: {urdf_path} (valid={urdf_res['valid']})")

        return {
            "status": "success",
            "log": log,
            "config_path": model_res["config_path"],
            "urdf_path": urdf_path,
            "robot_dict": model_res["robot_dict"],
        }
    except Exception as e:
        return {"status": "error", "message": str(e), "log": log}


def build_router() -> APIRouter:
    router = APIRouter()

    @router.post("/api/generate_robot")
    async def generate_robot_route(params: Dict[str, Any]):
        """生成机器人配置 (调用 quick_design.py)"""
        payload, status_code = generate_robot(params)
        return JSONResponse(status_code=status_code, content=payload)

    @router.get("/api/skills/list")
    async def skills_list_route():
        """列出所有可用 Skills 及其元数据"""
        return list_skills()

    @router.post("/api/skills/model")
    async def skills_model_route(req: ModelRequest):
        """调用 robot-modeling Skill 构建机器人模型"""
        return await skills_model(req)

    @router.post("/api/skills/optimize")
    async def skills_optimize_route(req: OptimizeRequest):
        """调用 parameter-optimizer Skill 对机器人参数进行优化"""
        return await skills_optimize(req)

    @router.post("/api/skills/export-urdf")
    async def skills_export_urdf_route(req: ExportURDFRequest):
        """调用 urdf-generator Skill 导出 URDF 文件"""
        return await skills_export_urdf(req)

    @router.post("/api/skills/pipeline")
    async def skills_pipeline_route(req: PipelineRequest):
        """一键完整流水线: 建模 → 参数优化 → URDF 导出"""
        return await skills_pipeline(req)

    return router
