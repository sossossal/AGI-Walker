"""
AI驱动的机器人控制器
使用3B小模型进行实时控制
"""

import time
import threading
import logging
import numpy as np
from typing import Optional, Dict, Any, List
from .tcp_client import GodotClient
from .onnx_inference import ONNXInferenceEngine
from .ai_model import create_ai_model, BaseAIModel
from .load_monitor import SystemMonitor

logger = logging.getLogger(__name__)

class SafetyChecker:
    """AGI-Walker V2.0 工业级安全检查器"""
    def __init__(self) -> None:
        # 关节限位 (弧度)
        self.joint_limits = {"hip_left": (-0.8, 1.5), "hip_right": (-0.8, 1.5)}
        # 速度限制 (rad/s)
        self.max_velocity = 5.0 
        self.last_pos = {}
        self.last_time = time.time()

    def check(self, action: Dict[str, Any]) -> Dict[str, Any]:
        """高频安全校验"""
        now = time.time()
        dt = now - self.last_time
        safe_action = {"motors": {}}

        for joint, pos in action.get("motors", {}).items():
            if joint in self.joint_limits:
                low, high = self.joint_limits[joint]
                pos = max(low, min(high, pos))

            if joint in self.last_pos and dt > 0:
                vel = (pos - self.last_pos[joint]) / dt
                if abs(vel) > self.max_velocity:
                    pos = self.last_pos[joint] + np.sign(vel) * self.max_velocity * dt

            safe_action["motors"][joint] = pos
            self.last_pos[joint] = pos

        self.last_time = now
        return safe_action

class AIController:
    """
    AGI-Walker V2.0 Decoupled AI Controller with Adaptive Degradation.
    Prioritizes hardware safety by falling back to pre-defined safe modes.
    """
    def __init__(
        self,
        model_path: Optional[str] = None,
        backend: str = "onnx",
        strategy: str = "balanced_standing"
    ):
        self.client = GodotClient()
        self.safety = SafetyChecker()
        self.monitor = SystemMonitor()
        self.strategy = strategy
        self.backend_type = backend
        
        # 推理后端初始化
        if backend == "onnx" and model_path:
            self.engine = ONNXInferenceEngine(model_path)
        else:
            self.engine = create_ai_model(engine="ollama")

        # 共享状态
        self.latest_action = {"motors": {}}
        self.sensor_buffer = {}
        self.running = False
        self.degraded_mode = False
        self.lock = threading.Lock()
        
        # 统计
        self.ctrl_count = 0
        self.inf_count = 0

    def _inference_worker(self, hz: float):
        """低频 AI 推理线程 (支持自适应挂起)"""
        interval = 1.0 / hz
        logger.info(f"Inference thread started at {hz} Hz")
        
        while self.running:
            if self.degraded_mode:
                # 降级模式下暂停 AI 推理，减少 CPU 压力
                time.sleep(0.5)
                continue

            start_time = time.time()
            with self.lock:
                sensors = self.sensor_buffer.copy()
            
            if sensors:
                action = self.engine.predict(sensors) if hasattr(self.engine, 'predict') else {}
                with self.lock:
                    self.latest_action = action
                    self.inf_count += 1
            
            elapsed = time.time() - start_time
            # 性能预警
            if elapsed > 0.1: # 超过 100ms 则认为推理太慢
                logger.warning(f"Extreme Inference Latency: {elapsed*1000:.1f}ms")

            time.sleep(max(0, interval - elapsed))

    def run(self, duration: float = 60.0, ctrl_hz: float = 500.0, inf_hz: float = 30.0):
        if not self.client.connect():
            logger.error("Failed to connect to simulation.")
            return

        self.running = True
        inf_thread = threading.Thread(target=self._inference_worker, args=(inf_hz,))
        inf_thread.daemon = True
        inf_thread.start()

        logger.info(f"V2.0 Adaptive Controller active: {ctrl_hz}Hz balance loop")
        start_time = time.time()
        ctrl_interval = 1.0 / ctrl_hz

        try:
            while time.time() - start_time < duration:
                loop_start = time.time()

                # 1. 硬件自适应监测 (每秒检查一次)
                if self.ctrl_count % int(ctrl_hz) == 0:
                    self._check_hardware_health()

                # 2. 获取传感器
                sensors = self.client.get_latest_sensors()
                if not sensors:
                    continue
                
                with self.lock:
                    self.sensor_buffer = sensors
                    if not self.degraded_mode:
                        action = self.latest_action.copy()
                    else:
                        # 降级模式: 使用预设的“安全站立”姿态数据
                        action = {"motors": {"hip_left": 0.0, "hip_right": 0.0}}

                # 3. 高频安全控制 (1kHz)
                safe_action = self.safety.check(action)

                # 4. 发送指令
                self.client.send_motor_commands(safe_action)
                self.ctrl_count += 1

                # 5. 控制频率同步
                elapsed = time.time() - loop_start
                if elapsed < ctrl_interval:
                    time.sleep(ctrl_interval - elapsed)

        except KeyboardInterrupt:
            logger.info("Interrupted by user.")
        finally:
            self.running = False
            inf_thread.join(timeout=1.0)
            self.client.close()
            logger.info("Controller shutdown complete.")

    def _check_hardware_health(self):
        """硬件健康检查逻辑"""
        is_overloaded = self.monitor.is_overloaded()
        
        if is_overloaded and not self.degraded_mode:
            self.degraded_mode = True
            logger.error("SYSTEM OVERLOAD: FALLING BACK TO SAFE MODE!")
        elif not is_overloaded and self.degraded_mode:
            # 自动恢复 (如果硬件负载回到正常范围)
            self.degraded_mode = False
            logger.info("Hardware recovered: Resuming AI Inference.")
        
        self.monitor.report()

if __name__ == "__main__":
    controller = AIController()
    controller.run()


# 使用示例
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="AI驱动的机器人控制器")
    parser.add_argument("--model", default="phi3:mini", help="Ollama模型名称")
    parser.add_argument("--duration", type=float, default=60.0, help="运行时长（秒）")
    parser.add_argument("--hz", type=float, default=30.0, help="目标控制频率")
    parser.add_argument("--strategy", default="保持躯干直立，站立稳定", help="控制策略")

    args = parser.parse_args()

    # 创建AI模型
    ai_model = create_ai_model(engine="ollama", model_name=args.model)

    # 创建控制器
    controller = AIController(ai_model=ai_model, strategy=args.strategy)

    # 运行
    controller.run(duration=args.duration, target_hz=args.hz)
