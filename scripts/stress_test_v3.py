import time
import sys
import os
import threading
import numpy as np
import psutil
import json
import logging
from concurrent.futures import ThreadPoolExecutor

# 确保能导入 agi_walker
sys.path.append(os.getcwd())

from agi_walker.core.api.comm.proto import robot_protocol_pb2
from agi_walker.workflow_orchestrator import WorkflowOrchestrator, StepStatus
from agi_walker.core.api.task_planning import TaskGraph, TaskNode
from agi_walker.core.controllers.load_monitor import SystemMonitor

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("StressTest")

def stress_test_comm_throughput(duration=5):
    """
    维度 A: 测试二进制协议的饱和吞吐量
    """
    logger.info("--- Testing Comm Throughput (Protobuf) ---")
    msg = robot_protocol_pb2.RobotMessage()
    msg.type = robot_protocol_pb2.RobotMessage.COMMAND
    for i in range(12):
        cmd = msg.commands.add()
        cmd.motor_id = i
        cmd.target_pos = 0.0
    
    count = 0
    start = time.time()
    while time.time() - start < duration:
        _ = msg.SerializeToString()
        count += 1
    
    elapsed = time.time() - start
    logger.info(f"Result: {count / elapsed:.2f} serializations/sec")

def stress_test_orchestrator_scaling(node_count=500):
    """
    维度 B: 测试 TaskGraph 在大规模节点下的调度性能
    """
    logger.info(f"--- Testing Orchestrator Scaling ({node_count} nodes) ---")
    orchestrator = WorkflowOrchestrator(use_real_executors=False)
    graph = TaskGraph()
    
    # 建立一个复杂的链式+并行结构
    prev_id = None
    for i in range(node_count):
        node = TaskNode(name=f"task_{i}", skill="robot_modeling", action="ping")
        curr_id = graph.add_node(node)
        if prev_id:
            graph.add_edge(prev_id, curr_id, condition="always")
        prev_id = curr_id
        
    start = time.time()
    result = orchestrator.execute_task_graph(graph)
    elapsed = time.time() - start
    
    logger.info(f"Result: Executed {len(result.steps)} nodes in {elapsed:.2f}s")
    logger.info(f"Avg overhead per node: {elapsed/node_count*1000:.4f}ms")

def stress_test_adaptive_safety():
    """
    维度 C: 测试在高负载下的自适应降级响应
    """
    logger.info("--- Testing Adaptive Safety Response ---")
    monitor = SystemMonitor(cpu_threshold=50.0) # 设置较低阈值以易于触发
    
    def cpu_burner():
        # 制造 CPU 压力
        x = 0
        while getattr(threading.current_thread(), "run", True):
            x = x * x
            
    t = threading.Thread(target=cpu_burner)
    t.run = True
    
    logger.info("Monitoring system (Normal)...")
    logger.info(f"Initial Overload State: {monitor.is_overloaded()}")
    
    logger.info("Starting CPU Burner...")
    t.start()
    
    start = time.time()
    triggered = False
    for _ in range(10):
        if monitor.is_overloaded():
            logger.info(f"🔥 Safety Triggered in {time.time() - start:.4f}s!")
            triggered = True
            break
        time.sleep(0.5)
    
    t.run = False
    if not triggered:
        logger.warning("Safety not triggered within timeout.")

if __name__ == "__main__":
    logger.info("=== AGI-Walker V3.0 Industrial Stress Test Suite ===")
    
    # 1. 通讯压测
    stress_test_comm_throughput()
    
    # 2. 调度压测
    stress_test_orchestrator_scaling(300)
    
    # 3. 硬件感知压测 (仅在非核心生产环境建议运行)
    stress_test_adaptive_safety()
    
    logger.info("=== Stress Test Complete ===")
