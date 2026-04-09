from pathlib import Path

# 获取项目根目录 (AGI-Walker/)
PROJECT_ROOT = Path(__file__).parent.parent.parent.parent

# 定义运行时数据根目录 (所有生成文件均在此目录下)
DATA_ROOT = PROJECT_ROOT / ".agi_data"


class RuntimePaths:
    """AGI-Walker 统一路径管理器 (V3.0 隔离版)"""

    # 定义顶级常量以兼容 Orchestrator 引用
    DATA_ROOT = DATA_ROOT

    # 1. 工作流相关
    WORKFLOWS = DATA_ROOT / "workflows"
    ARTIFACTS = WORKFLOWS / "artifacts"
    STATES = WORKFLOWS / "states"

    # 2. 仿真与轨迹
    TRAJECTORIES = DATA_ROOT / "trajectories"
    SESSIONS = DATA_ROOT / "sessions"

    # 3. 知识库与模型
    KNOWLEDGE = DATA_ROOT / "knowledge"
    MODELS = DATA_ROOT / "models"
    MODEL_REGISTRY = MODELS / "registry"

    @classmethod
    def ensure_directories(cls):
        """确保所有运行时目录都存在"""
        dirs = [
            cls.DATA_ROOT,
            cls.WORKFLOWS,
            cls.ARTIFACTS,
            cls.STATES,
            cls.TRAJECTORIES,
            cls.SESSIONS,
            cls.KNOWLEDGE,
            cls.MODELS,
            cls.MODEL_REGISTRY,
        ]
        for d in dirs:
            d.mkdir(parents=True, exist_ok=True)

    @classmethod
    def get_workflow_artifact_dir(cls, workflow_name: str) -> Path:
        path = cls.ARTIFACTS / workflow_name.replace("/", "_")
        path.mkdir(parents=True, exist_ok=True)
        return path


# 自动初始化
RuntimePaths.ensure_directories()
