"""
增强控制器（Enhanced Controller）
集成三层模型架构、动态负载均衡和多模态输入的完整控制器
"""

import time
import json
import argparse
from typing import Optional, Dict

# 导入核心模块
from tcp_client import GodotClient
from model_orchestrator import ModelOrchestrator, create_orchestrator
from load_monitor import LoadMonitor, SimplePIDController, ControlMode
from rag_knowledge_base import PhysicsKnowledgeBase

# 导入视觉和融合模块
import sys
sys.path.insert(0, '../python_api')
try:
    from vision_processor import create_vision_processor
    from multimodal_fusion import create_multimodal_fusion
    VISION_AVAILABLE = True
except ImportError:
    VISION_AVAILABLE = False
    print("⚠️ 视觉模块不可用")


class EnhancedController:
    """
    增强控制器
    
    架构特性：
    - 三层模型协同（3B小模型 + 7B中模型 + 70B大模型）
    - 动态负载均衡（20ms延迟阈值，自动PID fallback）
    - RAG物理知识库增强
    - 多模态传感器融合（可选视觉输入）
    """
    
    def __init__(
        self,
        small_model: str = "phi3:mini",
        medium_model: str = "mistral:7b",
        strategy: str = "保持躯干直立，站立稳定",
        enable_vision: bool = False,
        enable_rag: bool = True
    ):
        """
        初始化增强控制器
        
        Args:
            small_model: 小模型名称（实时控制）
            medium_model: 中模型名称（半实时任务）
            strategy: 控制策略
            enable_vision: 是否启用视觉输入
            enable_rag: 是否启用RAG知识库
        """
        print("=" * 50)
        print("初始化增强控制器")
        print("=" * 50)
        
        self.strategy = strategy
        self.enable_vision = enable_vision and VISION_AVAILABLE
        
        # Godot客户端
        print("\n1. 初始化Godot客户端...")
        self.client = GodotClient()
        
        # PID控制器（fallback）
        print("2. 初始化PID控制器...")
        self.pid_controller = SimplePIDController(kp=2.5, ki=0.15, kd=0.8)
        
        # 负载监控器
        print("3. 初始化负载监控器...")
        self.load_monitor = LoadMonitor(self.pid_controller)
        self.load_monitor.on_mode_change = self._on_mode_change
        
        # 模型编排器
        print("4. 初始化模型编排器...")
        self.orchestrator = create_orchestrator(
            small_model=small_model,
            medium_model=medium_model
        )
        
        # RAG知识库
        self.knowledge_base = None
        if enable_rag:
            print("5. 初始化RAG知识库...")
            try:
                self.knowledge_base = PhysicsKnowledgeBase(
                    index_path="d:/新建文件夹/AGI-Walker/knowledge/physics_index",
                    use_embeddings=False  # 离线模式
                )
            except Exception as e:
                print(f"⚠️ RAG初始化失败: {e}")
        
        # 视觉处理（可选）
        self.vision_processor = None
        self.fusion_module = None
        if self.enable_vision:
            print("6. 初始化视觉处理模块...")
            self.vision_processor = create_vision_processor()
            self.fusion_module = create_multimodal_fusion(self.vision_processor)
        
        # 运行状态
        self.is_running = False
        self.loop_count = 0
        self.start_time = 0.0
        
        # 统计
        self.stats = {
            "total_loops": 0,
            "ai_loops": 0,
            "pid_loops": 0,
            "hybrid_loops": 0,
            "errors": 0,
            "avg_loop_time": 0.0
        }
        
        print("\n✅ 增强控制器初始化完成")
        print("=" * 50)
    
    def _on_mode_change(self, old_mode: ControlMode, new_mode: ControlMode):
        """控制模式切换回调"""
        print(f"🔄 控制模式切换: {old_mode.value} -> {new_mode.value}")
    
    def run(
        self,
        duration: float = 120.0,
        target_hz: float = 30.0,
        adjustment_interval: float = 2.0,
        verbose: bool = True
    ):
        """
        运行增强控制循环
        
        Args:
            duration: 运行时长（秒）
            target_hz: 目标控制频率
            adjustment_interval: 中模型调整间隔（秒）
            verbose: 是否打印详细信息
        """
        print("\n🚀 启动增强控制器")
        print(f"   目标频率: {target_hz}Hz")
        print(f"   持续时间: {duration}秒")
        print(f"   视觉模式: {'启用' if self.enable_vision else '禁用'}")
        print(f"   RAG增强: {'启用' if self.knowledge_base else '禁用'}")
        
        # 连接Godot
        if not self.client.connect():
            print("❌ 无法连接到Godot仿真")
            return
        
        self.is_running = True
        self.start_time = time.time()
        loop_interval = 1.0 / target_hz
        last_adjustment_time = 0.0
        last_print_time = 0.0
        
        loop_times = []
        
        try:
            while self.is_running and (time.time() - self.start_time) < duration:
                loop_start = time.time()
                
                # 1. 获取传感器数据
                sensor_data = self.client.get_latest_sensors()
                if sensor_data is None:
                    time.sleep(0.01)
                    continue
                
                # 2. 多模态融合（如果启用）
                if self.fusion_module:
                    # 这里可以传入视觉帧（如果有）
                    sensor_data = self.fusion_module.fuse_sensors(sensor_data)
                
                # 3. 获取控制动作
                action, ai_time = self._get_control_action(sensor_data)
                
                # 4. 记录延迟
                self.load_monitor.record_latency(ai_time * 1000)
                
                # 5. 发送控制指令
                if action and 'motors' in action:
                    self.client.send_motor_commands(action)
                
                # 6. 周期性环境调整
                current_time = time.time()
                if current_time - last_adjustment_time > adjustment_interval:
                    self._do_environment_adjustment(sensor_data)
                    last_adjustment_time = current_time
                
                # 7. 添加日志
                self._add_log_entry(sensor_data, action)
                
                # 8. 打印状态
                if verbose and current_time - last_print_time > 1.0:
                    self._print_status(sensor_data, ai_time, loop_times)
                    last_print_time = current_time
                
                # 统计
                loop_time = time.time() - loop_start
                loop_times.append(loop_time)
                if len(loop_times) > 100:
                    loop_times.pop(0)
                
                self.loop_count += 1
                self.stats["total_loops"] += 1
                
                # 控制循环频率
                sleep_time = loop_interval - loop_time
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
        except KeyboardInterrupt:
            print("\n⏹ 用户中断")
        except Exception as e:
            print(f"\n❌ 运行错误: {e}")
            self.stats["errors"] += 1
        finally:
            self._cleanup()
    
    def _get_control_action(self, sensor_data: dict) -> tuple:
        """获取控制动作"""
        start_time = time.time()
        
        # 检查是否需要fallback到PID
        if self.load_monitor.should_fallback():
            action = self.load_monitor.get_control_action(sensor_data)
            mode = self.load_monitor.current_mode
            
            if mode == ControlMode.PID:
                self.stats["pid_loops"] += 1
            elif mode == ControlMode.HYBRID:
                self.stats["hybrid_loops"] += 1
                # 混合模式：同时计算AI动作
                ai_action = self._get_ai_action(sensor_data)
                action = self.load_monitor.get_control_action(sensor_data, ai_action)
            
            return action, time.time() - start_time
        
        # 正常AI控制
        action = self._get_ai_action(sensor_data)
        self.stats["ai_loops"] += 1
        
        return action, time.time() - start_time
    
    def _get_ai_action(self, sensor_data: dict) -> dict:
        """获取AI模型动作"""
        # 增强Prompt（如果启用RAG）
        if self.knowledge_base:
            self.knowledge_base.augment_prompt(
                self.strategy,
                sensor_data,
                max_context_length=300
            )
        
        # 使用模型编排器处理
        result = self.orchestrator.process(sensor_data, context="realtime")
        
        return result
    
    def _do_environment_adjustment(self, sensor_data: dict):
        """执行环境感知调整"""
        try:
            result = self.orchestrator.process(sensor_data, context="adjustment")
            
            if result.get('skip'):
                return
            
            # 处理调整建议
            adjustment = result.get('adjustment', {})
            
            # 调整PID参数
            if 'pid_tuning' in adjustment:
                tuning = adjustment['pid_tuning']
                self.pid_controller.kp *= tuning.get('kp_factor', 1.0)
                self.pid_controller.ki *= tuning.get('ki_factor', 1.0)
                self.pid_controller.kd *= tuning.get('kd_factor', 1.0)
            
        except Exception as e:
            print(f"⚠️ 环境调整错误: {e}")
    
    def _add_log_entry(self, sensor_data: dict, action: dict):
        """添加日志条目"""
        orient = sensor_data.get('sensors', {}).get('imu', {}).get('orient', [0, 0, 0])
        height = sensor_data.get('torso_height', 1.0)
        
        log_entry = {
            "timestamp": time.time(),
            "level": "INFO",
            "roll": orient[0],
            "pitch": orient[1],
            "height": height,
            "control_mode": self.load_monitor.current_mode.value
        }
        
        # 检查异常状态
        if abs(orient[0]) > 30 or abs(orient[1]) > 30:
            log_entry["level"] = "WARNING"
            log_entry["message"] = "姿态倾斜过大"
        
        if height < 0.5:
            log_entry["level"] = "WARNING"
            log_entry["type"] = "LOW_HEIGHT"
            log_entry["message"] = "高度过低"
        
        self.orchestrator.add_log(log_entry)
    
    def _print_status(self, sensor_data: dict, ai_time: float, loop_times: list):
        """打印状态信息"""
        orient = sensor_data.get('sensors', {}).get('imu', {}).get('orient', [0, 0, 0])
        height = sensor_data.get('torso_height', 0)
        
        elapsed = time.time() - self.start_time
        avg_loop = sum(loop_times) / len(loop_times) if loop_times else 0
        fps = 1.0 / avg_loop if avg_loop > 0 else 0
        
        mode = self.load_monitor.current_mode.value
        mode_emoji = {"ai": "🤖", "pid": "🔧", "hybrid": "🔀"}.get(mode, "❓")
        
        print(f"\r[{elapsed:6.1f}s] {mode_emoji} {mode:6s} | "
              f"Roll: {orient[0]:+6.1f}° Pitch: {orient[1]:+6.1f}° | "
              f"高度: {height:.2f}m | "
              f"AI延迟: {ai_time*1000:5.1f}ms | "
              f"FPS: {fps:.0f}", end="")
    
    def _cleanup(self):
        """清理资源"""
        self.is_running = False
        
        print("\n\n" + "=" * 50)
        print("控制器统计")
        print("=" * 50)
        
        elapsed = time.time() - self.start_time
        
        print(f"运行时长: {elapsed:.1f}秒")
        print(f"总循环数: {self.stats['total_loops']}")
        print(f"AI控制: {self.stats['ai_loops']} ({100*self.stats['ai_loops']/max(1,self.stats['total_loops']):.1f}%)")
        print(f"PID控制: {self.stats['pid_loops']} ({100*self.stats['pid_loops']/max(1,self.stats['total_loops']):.1f}%)")
        print(f"混合模式: {self.stats['hybrid_loops']} ({100*self.stats['hybrid_loops']/max(1,self.stats['total_loops']):.1f}%)")
        print(f"错误数: {self.stats['errors']}")
        
        # 负载监控统计
        print("\n负载监控:")
        load_stats = self.load_monitor.get_stats()
        print(f"  EMA延迟: {load_stats['ema_latency_ms']:.1f}ms")
        print(f"  超标率: {load_stats['over_threshold_rate']*100:.1f}%")
        print(f"  模式切换: {load_stats['mode_switches']}次")
        
        # 模型编排统计
        print("\n模型调用:")
        orch_stats = self.orchestrator.get_stats()
        for tier, count in orch_stats.get('tier_usage', {}).items():
            print(f"  {tier}: {count}次")
        
        self.client.close()
        print("\n连接已关闭")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description="AGI-Walker增强控制器")
    
    parser.add_argument("--duration", type=float, default=120.0,
                        help="运行时长（秒）")
    parser.add_argument("--hz", type=float, default=30.0,
                        help="目标控制频率")
    parser.add_argument("--small-model", default="phi3:mini",
                        help="小模型名称")
    parser.add_argument("--medium-model", default="mistral:7b",
                        help="中模型名称")
    parser.add_argument("--strategy", default="保持躯干直立，站立稳定",
                        help="控制策略")
    parser.add_argument("--with-vision", action="store_true",
                        help="启用视觉输入")
    parser.add_argument("--no-rag", action="store_true",
                        help="禁用RAG知识库")
    parser.add_argument("--quiet", action="store_true",
                        help="安静模式（减少输出）")
    
    args = parser.parse_args()
    
    # 创建控制器
    controller = EnhancedController(
        small_model=args.small_model,
        medium_model=args.medium_model,
        strategy=args.strategy,
        enable_vision=args.with_vision,
        enable_rag=not args.no_rag
    )
    
    # 运行
    controller.run(
        duration=args.duration,
        target_hz=args.hz,
        verbose=not args.quiet
    )


if __name__ == "__main__":
    main()
