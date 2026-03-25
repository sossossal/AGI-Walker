import json
import logging
import os
import sys
from typing import Any, Dict

import pydantic
from fastapi import APIRouter
from fastapi import HTTPException

logger = logging.getLogger(__name__)


class Sim2RealAnalyzeRequest(pydantic.BaseModel):
    mock: bool = True


class ImportPartRequest(pydantic.BaseModel):
    part_id: str
    category: str


def analyze_sim2real_gap(req: Sim2RealAnalyzeRequest) -> Dict[str, Any]:
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if root_dir not in sys.path:
        sys.path.insert(0, root_dir)

    try:
        from python_controller.sim2real_gap import Sim2RealGapEstimator

        estimator = Sim2RealGapEstimator(
            data_dir=os.path.join(root_dir, "offline_data", "sim2real")
        )

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
        logger.info(f"Sim2Real Error: {e}")
        return {"status": "error", "message": str(e)}


def get_parts_market() -> Dict[str, Any]:
    market_path = os.path.join("parts_library", "cloud_repo_mock", "manifest.json")
    try:
        if not os.path.exists(market_path):
            return {"status": "error", "message": "Market unavailable"}

        with open(market_path, "r", encoding="utf-8") as f:
            manifest = json.load(f)

        parts_list = []
        base_dir = os.path.join("parts_library", "cloud_repo_mock", "parts")

        for root, dirs, files in os.walk(base_dir):
            for file in files:
                if file.endswith(".json"):
                    try:
                        with open(os.path.join(root, file), "r", encoding="utf-8") as pf:
                            part_data = json.load(pf)
                            parts_list.append(
                                {
                                    "id": part_data.get("id"),
                                    "name": part_data.get("name"),
                                    "type": part_data.get("type"),
                                    "price": part_data.get("price"),
                                    "supplier": part_data.get("supplier"),
                                    "category": os.path.basename(root),
                                }
                            )
                    except Exception as e:
                        logger.info(f"Error reading part {file}: {e}")

        return {"status": "success", "manifest": manifest, "parts": parts_list}
    except Exception as e:
        return {"status": "error", "message": str(e)}


def import_part(req: ImportPartRequest) -> Dict[str, Any]:
    source_path = os.path.join("parts_library", "cloud_repo_mock", "parts", req.category)
    target_file = None
    if os.path.exists(source_path):
        for file in os.listdir(source_path):
            if file.endswith(".json"):
                with open(os.path.join(source_path, file), "r", encoding="utf-8") as f:
                    data = json.load(f)
                    if data.get("id") == req.part_id:
                        target_file = os.path.join(source_path, file)
                        break

    if not target_file:
        raise HTTPException(status_code=404, detail="Part not found in market")

    try:
        with open(target_file, "r", encoding="utf-8") as f:
            new_part = json.load(f)

        local_db_path = os.path.join("parts_library", "complete_parts_database.json")
        with open(local_db_path, "r", encoding="utf-8") as f:
            local_db = json.load(f)

        category_list = local_db["parts"].get(req.category, [])
        for part in category_list:
            if part["id"] == req.part_id:
                return {"status": "skipped", "message": "Part already exists"}

        if req.category not in local_db["parts"]:
            local_db["parts"][req.category] = []

        local_db["parts"][req.category].append(new_part)

        with open(local_db_path, "w", encoding="utf-8") as f:
            json.dump(local_db, f, indent=4, ensure_ascii=False)

        return {"status": "success", "message": f"Imported {new_part['name']}"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


def build_router() -> APIRouter:
    router = APIRouter()

    @router.post("/api/sim2real/analyze")
    async def analyze_sim2real_gap_route(req: Sim2RealAnalyzeRequest):
        """提取或计算最新的一段 Sim2Real 差距图谱"""
        return analyze_sim2real_gap(req)

    @router.get("/api/parts/market")
    async def get_parts_market_route():
        """获取云端零件市场列表 (模拟)"""
        return get_parts_market()

    @router.post("/api/parts/import")
    async def import_part_route(req: ImportPartRequest):
        """导入零件到本地库"""
        return import_part(req)

    return router
