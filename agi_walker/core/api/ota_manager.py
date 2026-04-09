import json
import logging
from pathlib import Path
from typing import Dict, Any, Optional
from datetime import datetime
from agi_walker.core.utils.paths import RuntimePaths

logger = logging.getLogger(__name__)


class OTAManager:
    """
    AGI-Walker V3.0 Over-The-Air (OTA) Evolution Manager (Isolated).
    Closes the loop: Trajectory -> Auto-Finetune -> Weight Deployment.
    """

    def __init__(
        self, model_dir: Optional[Path] = None, trajectory_dir: Optional[Path] = None
    ):
        self.model_dir = model_dir or RuntimePaths.MODELS
        self.trajectory_dir = trajectory_dir or RuntimePaths.TRAJECTORIES
        self.model_dir.mkdir(parents=True, exist_ok=True)

        # 模型版本库索引
        self.registry_file = self.model_dir / "registry.json"
        self.registry = self._load_registry()

    def _load_registry(self) -> Dict[str, Any]:
        if self.registry_file.exists():
            try:
                with open(self.registry_file, "r") as f:
                    return json.load(f)
            except Exception:
                pass
        return {"current_version": "v3.0.0", "history": []}

    def scan_and_trigger_evolution(self) -> bool:
        """扫描新轨迹并判定是否需要进化训练"""
        trajectories = list(self.trajectory_dir.glob("*.json"))
        success_count = 0

        for traj in trajectories:
            try:
                with open(traj, "r") as f:
                    data = json.load(f)
                if data and len(data) > 50:
                    success_count += 1
            except Exception:
                continue

        if success_count >= 5:
            logger.info(
                f"🚀 Evolution Triggered: {success_count} new trajectories ready."
            )
            return self._run_finetune_workflow()
        return False

    def _run_finetune_workflow(self) -> bool:
        logger.info("Triggering 'model_evolution_pipeline'...")
        new_version = self._increment_version(self.registry["current_version"])

        # 模拟产生新模型文件
        new_model_path = self.model_dir / f"walker_brain_{new_version}.onnx"
        try:
            with open(new_model_path, "w") as f:
                f.write("dummy_onnx_weights")
            self.registry["current_version"] = new_version
            self.registry["history"].append(
                {
                    "version": new_version,
                    "timestamp": datetime.now().isoformat(),
                    "path": str(new_model_path),
                }
            )
            self._save_registry()
            logger.info(f"✅ New Model Version {new_version} ready for OTA.")
            return True
        except Exception as e:
            logger.error(f"Finetune simulation failed: {e}")
            return False

    def _increment_version(self, version: str) -> str:
        parts = version.strip("v").split(".")
        parts[-1] = str(int(parts[-1]) + 1)
        return "v" + ".".join(parts)

    def _save_registry(self):
        with open(self.registry_file, "w") as f:
            json.dump(self.registry, f, indent=4)

    def get_latest_model_path(self) -> Optional[str]:
        if not self.registry["history"]:
            return None
        return self.registry["history"][-1]["path"]
