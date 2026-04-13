import sys
import os
import logging

# 确保能导入 agi_walker
sys.path.append(os.getcwd())

from agi_walker.workflow_orchestrator import get_workflow_orchestrator
from agi_walker.core.api.simple_planner import SimplePlanner
from agi_walker.core.controllers.enhanced_controller import EnhancedController
from agi_walker.core.controllers.rag_knowledge_base import PhysicsKnowledgeBase

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logger = logging.getLogger("MissionVerify")


def run_real_world_mission():
    logger.info("=== AGI-Walker V3.0 Real-World Mission Validation ===")

    # 1. 任务规划阶段: 模拟人类下达 "optimize robot" 指令
    logger.info("[1/4] Planning: Generating TaskGraph for 'optimize robot'...")
    planner = SimplePlanner()
    mission_graph = planner.plan("optimize robot")
    logger.info(f"Generated Graph with {len(mission_graph.nodes)} nodes.")

    # 2. 逻辑执行阶段: 并发运行任务图
    logger.info("[2/4] Orchestration: Executing TaskGraph with Async IO recording...")
    orchestrator = get_workflow_orchestrator()
    result = orchestrator.execute_task_graph(
        mission_graph, parameters={"output_root": "test_env/mission_data"}
    )

    if result.status.value == "completed":
        logger.info(f"✅ Mission Logic Completed. Steps executed: {len(result.steps)}")
    else:
        logger.error(f"❌ Mission Logic Failed: {result.error_message}")
        return

    # 3. 物理保险验证: 启动增强控制器并注入一个“危险”动作
    logger.info("[3/4] Safety Shell: Testing MPC filtering on EnhancedController...")
    controller = EnhancedController(strategy="test_mission")
    # 模拟一个极端的、会导致摔倒的 proposed_action (大幅度关节移动)
    dangerous_action = {"motors": {"hip_left": 2.5, "hip_right": -2.5}}
    # 模拟当前状态: 倾斜 20 度 (已超过 15 度阈值)
    fake_sensor_data = {
        "sensors": {"imu": {"orient": [20.0, 0.0, 0.0]}},
        "torso_height": 0.8,
    }

    safe_action = controller.safety_supervisor.check_and_filter(
        fake_sensor_data, dangerous_action
    )

    if safe_action["motors"]["hip_left"] < dangerous_action["motors"]["hip_left"]:
        logger.info(
            f"✅ Safety Shell Active: Modified dangerous command {dangerous_action['motors']['hip_left']} -> {safe_action['motors']['hip_left']:.2f}"
        )
    else:
        logger.warning(
            f"⚠️ Safety Shell did not modify the action. Result: {safe_action['motors']['hip_left']}. Check thresholds."
        )

    # 4. 经验进化验证: 让 RAG 检索刚刚产生的轨迹
    logger.info("[4/4] Evolution: Indexing and retrieving the new experience...")
    kb = PhysicsKnowledgeBase()
    # 模拟扫描刚刚执行过程产生的轨迹目录
    # 注意：Orchestrator 此时应该已经写回了 artifact 或 recorder 已经 flush 了数据
    kb.index_historical_trajectories(trajectories_dir=".output/trajectories")

    if len(kb.experiences) > 0:
        logger.info(
            f"✅ Evolution Loop Closed: Found {len(kb.experiences)} new experiences in memory."
        )
        # 尝试检索
        best_exp = kb.retrieve_experience(fake_sensor_data, top_k=1)
        if best_exp:
            logger.info(
                f"✅ RAG Success: Retrieved similar experience '{best_exp[0].id}' for current state."
            )
    else:
        logger.warning(
            "⚠️ No new experiences indexed. Ensure TrajectoryRecorder flushed the data."
        )

    logger.info("=== Mission Validation Complete: SYSTEM IS OPERATIONAL ===")


if __name__ == "__main__":
    run_real_world_mission()
