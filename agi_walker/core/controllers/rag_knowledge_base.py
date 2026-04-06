"""
RAG物理知识库（Retrieval-Augmented Generation）
离线部署版本，用于增强大模型的物理知识。
V3.0: 切换为纯 JSON 存储以消除 Pickle 路径敏感导致的导入错误。
"""

import json
import logging
import numpy as np
from typing import List, Optional, Tuple, Dict, Any
from dataclasses import dataclass
from pathlib import Path

logger = logging.getLogger(__name__)

@dataclass
class KnowledgeEntry:
    """文本知识条目"""
    id: str
    category: str
    title: str
    content: str
    keywords: List[str]
    embedding: Optional[List[float]] = None

@dataclass
class ExperienceEntry:
    """具身智能经验条目 (轨迹数据)"""
    id: str
    scenario: str
    outcome: str
    state_pattern: List[float]
    action_ref: List[float]
    source_file: str

class PhysicsKnowledgeBase:
    """
    物理知识库 & 经验记忆 RAG 增强模块
    """

    BUILTIN_KNOWLEDGE = [
        {
            "category": "物理稳定性",
            "title": "ZMP 平衡判据",
            "content": "零力矩点 (ZMP) 必须保持在足端支撑多边形内，以确保机器人不发生倾倒。",
            "keywords": ["平衡", "ZMP", "稳定", "重心"],
        },
        {
            "category": "物理稳定性",
            "title": "摩擦力补偿",
            "content": "在光滑地面（摩擦系数 < 0.3）时，应减小关节峰值扭矩，避免足端打滑。",
            "keywords": ["摩擦", "打滑", "地面", "补偿"],
        },
    ]

    def __init__(self, index_path: str = "agi_walker/core/knowledge/physics_index", use_embeddings: bool = True):
        self.index_path = Path(index_path)
        self.use_embeddings = use_embeddings # Keep for interface compatibility
        self.entries: List[KnowledgeEntry] = []
        self.experiences: List[ExperienceEntry] = []
        self._load_or_create_index()

    def _load_or_create_index(self) -> None:
        index_file = self.index_path / "index.json"
        if index_file.exists():
            try:
                with open(index_file, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    self.entries = [KnowledgeEntry(**e) for e in data]
                logger.info(f"✅ 已加载 {len(self.entries)} 条知识条目")
            except Exception as e:
                logger.error(f"加载索引失败: {e}，正在重建...")
                self._rebuild_index()
        else:
            self._rebuild_index()

    def _rebuild_index(self) -> None:
        self.entries = []
        for i, item in enumerate(self.BUILTIN_KNOWLEDGE):
            self.entries.append(KnowledgeEntry(id=f"builtin_{i}", **item))
        self._save_index()

    def _save_index(self) -> None:
        self.index_path.mkdir(parents=True, exist_ok=True)
        data = [
            {"id": e.id, "category": e.category, "title": e.title, "content": e.content, "keywords": e.keywords, "embedding": e.embedding}
            for e in self.entries
        ]
        with open(self.index_path / "index.json", "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

    def index_historical_trajectories(self, trajectories_dir: str = ".output/trajectories"):
        path = Path(trajectories_dir)
        if not path.exists(): return
        
        logger.info(f"🧠 扫描历史轨迹: {trajectories_dir}")
        for traj_file in path.glob("*.json"):
            try:
                with open(traj_file, "r") as f: data = json.load(f)
                if not data: continue
                orientations = [d["state"]["sensors"]["imu"]["orient"] for d in data if "state" in d]
                avg_orient = np.mean(orientations, axis=0).tolist() if orientations else [0,0,0]
                self.experiences.append(ExperienceEntry(
                    id=traj_file.stem, scenario="recovery", outcome="success",
                    state_pattern=avg_orient, action_ref=[0.5]*12, source_file=str(traj_file)
                ))
            except Exception: continue
        logger.info(f"✅ 加载了 {len(self.experiences)} 条运行经验")

    def retrieve_experience(self, current_sensor: dict, top_k: int = 1) -> List[ExperienceEntry]:
        if not self.experiences: return []
        curr_orient = current_sensor.get("sensors", {}).get("imu", {}).get("orient", [0,0,0])
        def dist(exp): return np.linalg.norm(np.array(exp.state_pattern) - np.array(curr_orient))
        return sorted(self.experiences, key=dist)[:top_k]

    def augment_prompt(self, base_prompt: str, sensor_data: dict) -> str:
        # 简单关键词检索模拟
        context = " ".join([e.content for e in self.entries[:2]])
        exp_context = ""
        exps = self.retrieve_experience(sensor_data)
        if exps:
            exp_context = f"历史参考: 在姿态 {exps[0].state_pattern} 附近，执行动作 {exps[0].action_ref} 成功。"
        
        return f"{base_prompt}\n\n物理背景: {context}\n{exp_context}"

    def get_stats(self) -> dict:
        return {"knowledge_count": len(self.entries), "experience_count": len(self.experiences)}
