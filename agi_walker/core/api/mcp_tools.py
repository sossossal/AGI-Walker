import logging
import json
import asyncio
from typing import Dict, Any, List, Optional
from agi_walker.workflow_orchestrator import get_workflow_orchestrator
from agi_walker.core.api.simple_planner import SimplePlanner
from agi_walker.core.controllers.load_monitor import SystemMonitor
from agi_walker.core.controllers.rag_knowledge_base import PhysicsKnowledgeBase

logger = logging.getLogger("AGI-Walker-MCP")

class MCPToolProvider:
    """
    AGI-Walker V3.0 MCP Tool Bridge.
    Exposes project capabilities as structured tools for Gemini CLI.
    """
    def __init__(self):
        self.orchestrator = get_workflow_orchestrator()
        self.planner = SimplePlanner()
        self.monitor = SystemMonitor()
        self.kb = PhysicsKnowledgeBase()

    async def execute_mission(self, instruction: str) -> str:
        """执行一个端到端的具身任务"""
        logger.info(f"MCP Request: Execute mission '{instruction}'")
        try:
            # 1. 规划
            graph = self.planner.plan(instruction)
            # 2. 执行
            result = self.orchestrator.execute_task_graph(graph)
            return json.dumps(result.to_dict(), indent=2, ensure_ascii=False)
        except Exception as e:
            return f"Mission failed: {str(e)}"

    def get_telemetry(self) -> str:
        """获取系统实时遥测数据"""
        stats = self.monitor.get_hw_stats()
        # 补充 RAG 统计
        stats["rag"] = self.kb.get_stats()
        return json.dumps(stats, indent=2)

    def query_rag(self, orient: List[float]) -> str:
        """根据当前姿态检索经验"""
        sensor_data = {"sensors": {"imu": {"orient": orient}}}
        exps = self.kb.retrieve_experience(sensor_data, top_k=1)
        if not exps:
            return "No matching experiences found."
        return f"Found match: {exps[0].id} (Scenario: {exps[0].scenario})"

    def list_available_workflows(self) -> List[str]:
        """列出所有已注册的工作流"""
        return self.orchestrator.list_workflows()
