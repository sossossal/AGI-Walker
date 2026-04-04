"""
RAG物理知识库（Retrieval-Augmented Generation）
离线部署版本，用于增强大模型的物理知识
"""

import json
import pickle
from typing import List, Optional, Tuple
from dataclasses import dataclass
from pathlib import Path

import logging

logger = logging.getLogger(__name__)


@dataclass
class KnowledgeEntry:
    """知识条目"""

    id: str
    category: str
    title: str
    content: str
    keywords: List[str]
    embedding: Optional[List[float]] = None


class PhysicsKnowledgeBase:
    """
    物理知识库RAG增强模块

    支持：
    - 离线部署（无需网络）
    - 本地嵌入向量计算
    - 关键词匹配 + 语义检索
    """

    # 预置物理知识库
    BUILTIN_KNOWLEDGE = [
        # 重力与惯性
        {
            "id": "gravity_001",
            "category": "gravity",
            "title": "重力加速度",
            "content": "地球表面重力加速度g约为9.8m/s²。机器人受到向下的重力F=mg，其中m为质量。保持平衡需要地面提供等大反向的支撑力。",
            "keywords": ["重力", "加速度", "g", "9.8", "质量", "支撑力"],
        },
        {
            "id": "gravity_002",
            "category": "gravity",
            "title": "重心概念",
            "content": "重心(Center of Mass, CoM)是物体质量分布的几何中心。双足机器人需保持重心投影落在支撑多边形内才能稳定站立。重心高度越低，稳定性越好。",
            "keywords": ["重心", "CoM", "质量中心", "支撑多边形", "稳定性"],
        },
        {
            "id": "gravity_003",
            "category": "gravity",
            "title": "倾覆力矩",
            "content": "当机器人倾斜角度θ时，重力产生倾覆力矩M=mgh*sin(θ)，其中h为重心高度。需要关节力矩产生恢复力矩来抵消。",
            "keywords": ["倾覆", "力矩", "倾斜", "角度", "恢复力矩"],
        },
        # 摩擦力
        {
            "id": "friction_001",
            "category": "friction",
            "title": "静摩擦力",
            "content": "静摩擦力f≤μsN，其中μs为静摩擦系数，N为正压力。机器人脚底与地面的摩擦是防止滑动的关键。典型橡胶-混凝土静摩擦系数为0.6-0.8。",
            "keywords": ["静摩擦", "摩擦系数", "正压力", "滑动", "橡胶"],
        },
        {
            "id": "friction_002",
            "category": "friction",
            "title": "滑动摩擦",
            "content": "滑动摩擦力f=μkN，其中μk为动摩擦系数，通常小于静摩擦系数。一旦开始滑动，摩擦力降低，容易失控。",
            "keywords": ["滑动摩擦", "动摩擦", "失控", "滑动"],
        },
        # 平衡控制
        {
            "id": "balance_001",
            "category": "balance",
            "title": "零力矩点(ZMP)",
            "content": "零力矩点(Zero Moment Point)是动态平衡的关键指标。ZMP位于重力和惯性力合力作用线与地面的交点。ZMP必须保持在支撑多边形内以确保稳定。",
            "keywords": ["ZMP", "零力矩点", "动态平衡", "惯性力", "支撑"],
        },
        {
            "id": "balance_002",
            "category": "balance",
            "title": "倒立摆模型",
            "content": "双足机器人可简化为倒立摆模型。质量集中在重心，支点在脚踝。控制方程为θ''=(g/l)*sin(θ)-u/ml²，其中l为摆长，u为控制力矩。",
            "keywords": ["倒立摆", "摆长", "控制方程", "脚踝", "力矩"],
        },
        {
            "id": "balance_003",
            "category": "balance",
            "title": "PID平衡控制",
            "content": "PID控制器通过比例(P)、积分(I)、微分(D)三项控制误差。u=Kp*e+Ki*∫e+Kd*de/dt。对于平衡控制，典型参数范围Kp=2-10,Ki=0.1-1,Kd=0.5-5。",
            "keywords": ["PID", "比例", "积分", "微分", "参数", "控制器"],
        },
        # 步态规划
        {
            "id": "gait_001",
            "category": "gait",
            "title": "步态周期",
            "content": "步态周期包括支撑相和摆动相。支撑相占约60%，摆动相约40%。双支撑期是两脚同时着地的时间，约占20%，是最稳定的阶段。",
            "keywords": ["步态", "周期", "支撑相", "摆动相", "双支撑"],
        },
        {
            "id": "gait_002",
            "category": "gait",
            "title": "步幅与步频",
            "content": "步幅(stride length)是同一只脚两次着地间的距离。步频(cadence)是单位时间步数。行走速度=步幅×步频/2。正常人步幅约1.2-1.5m，步频约100-120步/分。",
            "keywords": ["步幅", "步频", "速度", "距离", "行走"],
        },
        # 关节控制
        {
            "id": "joint_001",
            "category": "joint",
            "title": "关节力矩",
            "content": "电机输出力矩T需克服负载力矩。T=J*α+b*ω+τ_load，其中J为转动惯量，α为角加速度，b为阻尼系数，τ_load为负载力矩。",
            "keywords": ["力矩", "电机", "转动惯量", "角加速度", "阻尼"],
        },
        {
            "id": "joint_002",
            "category": "joint",
            "title": "关节限位",
            "content": "人体髋关节活动范围：屈曲0-120°，伸展0-30°，外展0-45°。机器人关节应设置合适限位，避免过度运动造成损坏。",
            "keywords": ["限位", "髋关节", "活动范围", "屈曲", "伸展"],
        },
        # 惯性与动量
        {
            "id": "inertia_001",
            "category": "inertia",
            "title": "角动量守恒",
            "content": "无外力矩时角动量L=Iω守恒。机器人可通过改变姿态（改变转动惯量I）来调整角速度ω，类似滑冰运动员收紧手臂加速旋转。",
            "keywords": ["角动量", "守恒", "转动惯量", "角速度", "姿态"],
        },
        {
            "id": "inertia_002",
            "category": "inertia",
            "title": "冲量与动量",
            "content": "冲量J=F*Δt=Δp，等于动量变化。落地时通过弯曲膝盖延长接触时间，可减小地面冲击力，保护关节。",
            "keywords": ["冲量", "动量", "落地", "膝盖", "冲击力"],
        },
        # 能量
        {
            "id": "energy_001",
            "category": "energy",
            "title": "能量效率",
            "content": "行走的能量效率用CoT(Cost of Transport)衡量：CoT=P/(mg*v)，其中P为功率，v为速度。人类步行CoT约0.2，奔跑约0.9。",
            "keywords": ["能量", "效率", "CoT", "功率", "行走"],
        },
        {
            "id": "energy_002",
            "category": "energy",
            "title": "势能与动能转换",
            "content": "行走过程中势能(mgh)和动能(0.5mv²)相互转换。单腿支撑时重心先升高后降低，利用重力做功提高效率。",
            "keywords": ["势能", "动能", "转换", "重心", "效率"],
        },
    ]

    def __init__(
        self, index_path: str = "knowledge/physics_index", use_embeddings: bool = True
    ):
        """
        初始化知识库

        Args:
            index_path: 索引文件路径
            use_embeddings: 是否使用嵌入向量（需要sentence-transformers）
        """
        self.index_path = Path(index_path)
        self.use_embeddings = use_embeddings

        # 知识库
        self.entries: List[KnowledgeEntry] = []
        self.embedder = None

        # 加载或创建索引
        self._load_or_create_index()

    def _load_or_create_index(self) -> None:
        """加载或创建索引"""
        index_file = self.index_path / "index.pkl"

        if index_file.exists():
            logger.info(f"📚 加载知识库索引: {index_file}")
            with open(index_file, "rb") as f:
                self.entries = pickle.load(f)
        else:
            logger.info("📚 创建知识库索引...")
            self._create_index()
            self._save_index()

    def _create_index(self) -> None:
        """创建索引"""
        # 加载内置知识
        for item in self.BUILTIN_KNOWLEDGE:
            entry = KnowledgeEntry(
                id=item["id"],
                category=item["category"],
                title=item["title"],
                content=item["content"],
                keywords=item["keywords"],
            )
            self.entries.append(entry)

        # 如果启用嵌入，计算嵌入向量
        if self.use_embeddings:
            self._compute_embeddings()

        logger.info(f"✅ 知识库创建完成，共 {len(self.entries)} 条")

    def _compute_embeddings(self) -> List:
        """计算嵌入向量"""
        try:
            from sentence_transformers import SentenceTransformer

            logger.info("正在加载嵌入模型...")
            # 使用小型多语言模型
            self.embedder = SentenceTransformer("paraphrase-multilingual-MiniLM-L12-v2")

            logger.info("正在计算嵌入向量...")
            texts = [f"{e.title}: {e.content}" for e in self.entries]
            embeddings = self.embedder.encode(texts, show_progress_bar=True)

            for i, entry in enumerate(self.entries):
                entry.embedding = embeddings[i].tolist()

            logger.info("✅ 嵌入向量计算完成")

        except ImportError:
            logger.info("⚠️ sentence-transformers未安装，使用关键词匹配")
            self.use_embeddings = False

    def _save_index(self) -> None:
        """保存索引"""
        self.index_path.mkdir(parents=True, exist_ok=True)
        index_file = self.index_path / "index.pkl"

        with open(index_file, "wb") as f:
            pickle.dump(self.entries, f)

        logger.info(f"💾 索引已保存: {index_file}")

    def retrieve(
        self, query: str, top_k: int = 3, category: Optional[str] = None
    ) -> List[Tuple[KnowledgeEntry, float]]:
        """
        检索相关知识

        Args:
            query: 查询文本
            top_k: 返回结果数量
            category: 限定类别

        Returns:
            [(知识条目, 相关度分数), ...]
        """
        # 过滤类别
        candidates = self.entries
        if category:
            candidates = [e for e in candidates if e.category == category]

        if self.use_embeddings and self.embedder:
            return self._retrieve_by_embedding(query, candidates, top_k)
        else:
            return self._retrieve_by_keyword(query, candidates, top_k)

    def _retrieve_by_embedding(
        self, query: str, candidates: List[KnowledgeEntry], top_k: int
    ) -> List[Tuple[KnowledgeEntry, float]]:
        """使用嵌入向量检索"""
        import numpy as np

        # 计算查询嵌入
        query_emb = self.embedder.encode([query])[0]

        # 计算相似度
        scores = []
        for entry in candidates:
            if entry.embedding:
                entry_emb = np.array(entry.embedding)
                # 余弦相似度
                sim = np.dot(query_emb, entry_emb) / (
                    np.linalg.norm(query_emb) * np.linalg.norm(entry_emb)
                )
                scores.append((entry, float(sim)))

        # 排序返回
        scores.sort(key=lambda x: x[1], reverse=True)
        return scores[:top_k]

    def _retrieve_by_keyword(
        self, query: str, candidates: List[KnowledgeEntry], top_k: int
    ) -> List[Tuple[KnowledgeEntry, float]]:
        """使用关键词匹配检索"""
        query_lower = query.lower()
        query_words = set(query_lower.split())

        scores = []
        for entry in candidates:
            # 计算关键词匹配分数
            keyword_score = 0
            for kw in entry.keywords:
                if kw.lower() in query_lower:
                    keyword_score += 2
                elif any(kw.lower() in w for w in query_words):
                    keyword_score += 1

            # 标题匹配
            if any(w in entry.title.lower() for w in query_words):
                keyword_score += 1

            # 内容匹配
            content_matches = sum(1 for w in query_words if w in entry.content.lower())
            keyword_score += content_matches * 0.5

            if keyword_score > 0:
                # 归一化分数到0-1
                normalized = min(1.0, keyword_score / 10.0)
                scores.append((entry, normalized))

        scores.sort(key=lambda x: x[1], reverse=True)
        return scores[:top_k]

    def augment_prompt(
        self, base_prompt: str, sensor_data: dict, max_context_length: int = 500
    ) -> str:
        """
        用RAG检索结果增强Prompt

        Args:
            base_prompt: 基础Prompt
            sensor_data: 传感器数据（用于提取查询）
            max_context_length: 上下文最大长度

        Returns:
            增强后的Prompt
        """
        # 从传感器数据提取查询
        query = self._extract_query(sensor_data)

        # 检索相关知识
        results = self.retrieve(query, top_k=3)

        if not results:
            return base_prompt

        # 构建知识上下文
        context_parts = []
        total_length = 0

        for entry, score in results:
            if score < 0.3:  # 相关度阈值
                continue

            snippet = f"【{entry.title}】{entry.content}"

            if total_length + len(snippet) > max_context_length:
                break

            context_parts.append(snippet)
            total_length += len(snippet)

        if not context_parts:
            return base_prompt

        context = "\n".join(context_parts)

        return f"""{base_prompt}

## 相关物理知识
{context}

请结合以上物理知识进行分析和决策。"""

    def _extract_query(self, sensor_data: dict) -> str:
        """从传感器数据提取查询关键词"""
        queries = []

        orient = sensor_data.get("sensors", {}).get("imu", {}).get("orient", [0, 0, 0])
        height = sensor_data.get("torso_height", 1.0)

        # 根据状态添加查询关键词
        roll, pitch = orient[0], orient[1]

        if abs(roll) > 10 or abs(pitch) > 10:
            queries.append("平衡控制 倾斜 恢复")

        if height < 0.5:
            queries.append("重心 稳定性 跌倒")

        if abs(roll) > 30 or abs(pitch) > 30:
            queries.append("倾覆 力矩 紧急")

        # 默认查询
        if not queries:
            queries.append("平衡 站立 控制")

        return " ".join(queries)

    def add_knowledge(self, entry: dict) -> None:
        """添加新知识条目"""
        new_entry = KnowledgeEntry(
            id=entry.get("id", f"custom_{len(self.entries)}"),
            category=entry.get("category", "custom"),
            title=entry["title"],
            content=entry["content"],
            keywords=entry.get("keywords", []),
        )

        # 计算嵌入
        if self.use_embeddings and self.embedder:
            text = f"{new_entry.title}: {new_entry.content}"
            embedding = self.embedder.encode([text])[0]
            new_entry.embedding = embedding.tolist()

        self.entries.append(new_entry)
        self._save_index()

    def get_stats(self) -> dict:
        """获取统计信息"""
        categories = {}
        for entry in self.entries:
            categories[entry.category] = categories.get(entry.category, 0) + 1

        return {
            "total_entries": len(self.entries),
            "categories": categories,
            "embeddings_enabled": self.use_embeddings,
            "index_path": str(self.index_path),
        }


# 测试代码
if __name__ == "__main__":
    logger.info("RAG物理知识库测试\n")

    # 动态获取项目根目录
    project_root = Path(__file__).resolve().parent.parent

    # 创建知识库（离线模式，不使用嵌入）
    kb = PhysicsKnowledgeBase(
        index_path=str(project_root / "knowledge" / "physics_index"),
        use_embeddings=False,  # 离线模式
    )

    # 测试检索
    logger.info("\n=== 测试检索 ===")
    queries = ["机器人倾斜怎么办", "如何保持平衡", "PID参数调节", "步态规划"]

    for query in queries:
        logger.info(f"\n查询: {query}")
        results = kb.retrieve(query, top_k=2)
        for entry, score in results:
            logger.info(f"  - [{score:.2f}] {entry.title}")

    # 测试Prompt增强
    logger.info("\n=== 测试Prompt增强 ===")
    sensor_data = {
        "sensors": {
            "imu": {"orient": [15.0, -8.0, 0.0]},
            "joints": {"hip_left": {"angle": 5.0}, "hip_right": {"angle": -3.0}},
        },
        "torso_height": 1.2,
    }

    base_prompt = "请分析当前机器人状态并提供控制建议"
    enhanced = kb.augment_prompt(base_prompt, sensor_data)
    logger.info(enhanced[:500] + "...")

    # 统计
    logger.info("\n=== 统计信息 ===")
    stats = kb.get_stats()
    logger.info(json.dumps(stats, indent=2, ensure_ascii=False))
