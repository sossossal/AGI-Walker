"""
异步并行控制器
使用asyncio + multiprocessing实现高性能并行控制
"""

import asyncio
import time
import json
import argparse
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from typing import Dict, Optional, List
from dataclasses import dataclass
from queue import Queue
import multiprocessing as mp

# 导入模块
from async_tcp_client import AsyncGodotClient, AsyncClientConfig
from load_monitor import LoadMonitor, SimplePIDController, ControlMode


@dataclass
class AsyncControllerConfig:
    """异步控制器配置"""
    target_hz: float = 50.0  # 目标控制频率
    inference_workers: int = 2  # 推理进程数
    enable_onnx: bool = True  # 启用ONNX推理
    enable_fault_injection: bool = False  # 启用故障注入
    log_interval: float = 1.0  # 日志间隔


class AsyncController:
    """
    异步并行控制器
    
    架构：
    - 主进程：asyncio事件循环
    - 传感器协程：接收传感器数据
    - 控制协程：发送控制指令
    - 日志协程：记录日志
    - 推理进程池：并行AI推理
    """
    
    def __init__(self, config: Optional[AsyncControllerConfig] = None):
        self.config = config or AsyncControllerConfig()
        
        # 客户端
        self.client = AsyncGodotClient()
        
        # PID控制器（fallback）
        self.pid_controller = SimplePIDController(kp=2.5, ki=0.15, kd=0.8)
        
        # 负载监控器
        self.load_monitor = LoadMonitor(self.pid_controller)
        
        # 进程池（推理）
        self.inference_pool: Optional[ProcessPoolExecutor] = None
        
        # 线程池（IO密集型任务）
        self.io_pool: Optional[ThreadPoolExecutor] = None
        
        # 异步队列
        self.sensor_queue: asyncio.Queue = asyncio.Queue(maxsize=100)
        self.action_queue: asyncio.Queue = asyncio.Queue(maxsize=100)
        self.log_queue: asyncio.Queue = asyncio.Queue(maxsize=1000)
        
        # 状态
        self.running = False
        self.start_time = 0.0
        
        # ONNX引擎（延迟加载）
        self.onnx_engine = None
        
        # 统计
        self.stats = {
            "total_frames": 0,
            "inference_frames": 0,
            "pid_frames": 0,
            "avg_loop_time": 0.0,
            "errors": 0
        }
        
        # 最新传感器数据
        self.latest_sensor_data: Optional[dict] = None
    
    def _init_pools(self):
        """初始化进程池和线程池"""
        self.inference_pool = ProcessPoolExecutor(
            max_workers=self.config.inference_workers
        )
        self.io_pool = ThreadPoolExecutor(max_workers=4)
        print(f"✅ 进程池初始化完成（{self.config.inference_workers}个推理进程）")
    
    def _init_onnx(self):
        """初始化ONNX引擎"""
        if not self.config.enable_onnx:
            return
        
        try:
            from onnx_inference import create_onnx_engine
            self.onnx_engine = create_onnx_engine(use_gpu=False)
            print("✅ ONNX引擎初始化完成")
        except Exception as e:
            print(f"⚠️ ONNX引擎初始化失败: {e}")
    
    async def run(self, duration: float = 120.0):
        """
        运行异步控制循环
        
        Args:
            duration: 运行时长（秒）
        """
        print("=" * 50)
        print("启动异步并行控制器")
        print("=" * 50)
        
        self._init_pools()
        self._init_onnx()
        
        # 连接到Godot
        if not await self.client.connect():
            print("❌ 无法连接到Godot仿真器")
            return
        
        self.running = True
        self.start_time = time.time()
        
        try:
            # 创建并发任务
            tasks = [
                asyncio.create_task(self._sensor_loop()),
                asyncio.create_task(self._inference_loop()),
                asyncio.create_task(self._control_loop()),
                asyncio.create_task(self._logging_loop()),
                asyncio.create_task(self._monitor_loop()),
            ]
            
            # 等待指定时间
            await asyncio.sleep(duration)
            
        except asyncio.CancelledError:
            print("\n⏹ 任务取消")
        except KeyboardInterrupt:
            print("\n⏹ 用户中断")
        except Exception as e:
            print(f"\n❌ 错误: {e}")
            self.stats["errors"] += 1
        finally:
            await self._cleanup()
    
    async def _sensor_loop(self):
        """传感器接收协程"""
        await self.client.receive_loop()
    
    async def _inference_loop(self):
        """推理协程"""
        loop = asyncio.get_event_loop()
        interval = 1.0 / self.config.target_hz
        
        while self.running:
            loop_start = time.time()
            
            # 获取最新传感器数据
            sensor_data = self.client.get_latest_sensors()
            
            if sensor_data:
                self.latest_sensor_data = sensor_data
                
                # 检查是否需要fallback
                if self.load_monitor.should_fallback():
                    # 使用PID控制
                    action = self.load_monitor.get_control_action(sensor_data)
                    self.stats["pid_frames"] += 1
                else:
                    # 异步推理（在进程池中执行）
                    try:
                        action = await loop.run_in_executor(
                            self.inference_pool,
                            self._run_inference,
                            sensor_data
                        )
                        self.stats["inference_frames"] += 1
                    except Exception as e:
                        print(f"⚠️ 推理错误: {e}")
                        action = self.load_monitor.get_control_action(sensor_data)
                        self.stats["pid_frames"] += 1
                
                # 记录延迟
                latency = (time.time() - loop_start) * 1000
                self.load_monitor.record_latency(latency)
                
                # 放入动作队列
                try:
                    self.action_queue.put_nowait((sensor_data, action))
                except asyncio.QueueFull:
                    self.action_queue.get_nowait()
                    self.action_queue.put_nowait((sensor_data, action))
            
            # 控制频率
            elapsed = time.time() - loop_start
            if elapsed < interval:
                await asyncio.sleep(interval - elapsed)
    
    def _run_inference(self, sensor_data: dict) -> dict:
        """
        推理函数（在进程池中执行）
        
        注意：此函数在独立进程中运行
        """
        # 简化的推理逻辑（实际应使用AI模型）
        orient = sensor_data.get('sensors', {}).get('imu', {}).get('orient', [0, 0, 0])
        roll, pitch = orient[0], orient[1]
        
        # 简单PID计算
        kp = 2.0
        hip_left = -kp * pitch - kp * roll * 0.5
        hip_right = -kp * pitch + kp * roll * 0.5
        
        # 限幅
        hip_left = max(-45, min(45, hip_left))
        hip_right = max(-45, min(45, hip_right))
        
        return {
            "motors": {
                "hip_left": hip_left,
                "hip_right": hip_right
            },
            "confidence": 0.8
        }
    
    async def _control_loop(self):
        """控制输出协程"""
        while self.running:
            try:
                # 从动作队列获取
                sensor_data, action = await asyncio.wait_for(
                    self.action_queue.get(),
                    timeout=0.1
                )
                
                # 发送控制指令
                if action and 'motors' in action:
                    await self.client.send_commands(action)
                    self.stats["total_frames"] += 1
                
                # 添加日志
                await self.log_queue.put({
                    "timestamp": time.time(),
                    "sensor": sensor_data,
                    "action": action,
                    "mode": self.load_monitor.current_mode.value
                })
                
            except asyncio.TimeoutError:
                continue
            except Exception as e:
                print(f"⚠️ 控制错误: {e}")
    
    async def _logging_loop(self):
        """日志协程"""
        log_buffer: List[dict] = []
        
        while self.running:
            try:
                # 收集日志
                while not self.log_queue.empty():
                    log_entry = self.log_queue.get_nowait()
                    log_buffer.append(log_entry)
                
                # 定期写入
                if len(log_buffer) >= 100:
                    # 异步写入文件
                    await self._write_logs(log_buffer)
                    log_buffer.clear()
                
                await asyncio.sleep(0.1)
                
            except Exception as e:
                print(f"⚠️ 日志错误: {e}")
    
    async def _write_logs(self, logs: List[dict]):
        """异步写入日志"""
        # 使用线程池执行IO操作
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(
            self.io_pool,
            self._write_logs_sync,
            logs
        )
    
    def _write_logs_sync(self, logs: List[dict]):
        """同步写入日志"""
        # 简化：仅保存到内存
        # 实际应用可以写入文件或数据库
        pass
    
    async def _monitor_loop(self):
        """监控协程"""
        last_print = 0.0
        
        while self.running:
            current_time = time.time()
            
            if current_time - last_print >= self.config.log_interval:
                elapsed = current_time - self.start_time
                fps = self.stats["total_frames"] / elapsed if elapsed > 0 else 0
                
                mode = self.load_monitor.current_mode.value
                mode_emoji = {"ai": "🤖", "pid": "🔧", "hybrid": "🔀"}.get(mode, "❓")
                
                # 获取传感器状态
                if self.latest_sensor_data:
                    orient = self.latest_sensor_data.get('sensors', {}).get('imu', {}).get('orient', [0,0,0])
                    height = self.latest_sensor_data.get('torso_height', 0)
                    print(f"\r[{elapsed:6.1f}s] {mode_emoji} {mode:6s} | "
                          f"Roll: {orient[0]:+6.1f}° Pitch: {orient[1]:+6.1f}° | "
                          f"高度: {height:.2f}m | FPS: {fps:.0f}", end="")
                
                last_print = current_time
            
            await asyncio.sleep(0.1)
    
    async def _cleanup(self):
        """清理资源"""
        self.running = False
        
        print("\n\n" + "=" * 50)
        print("控制器统计")
        print("=" * 50)
        
        elapsed = time.time() - self.start_time
        fps = self.stats["total_frames"] / elapsed if elapsed > 0 else 0
        
        print(f"运行时长: {elapsed:.1f}秒")
        print(f"总帧数: {self.stats['total_frames']}")
        print(f"平均FPS: {fps:.1f}")
        print(f"推理帧: {self.stats['inference_frames']}")
        print(f"PID帧: {self.stats['pid_frames']}")
        print(f"错误数: {self.stats['errors']}")
        
        # 负载监控统计
        load_stats = self.load_monitor.get_stats()
        print(f"\n负载监控:")
        print(f"  EMA延迟: {load_stats['ema_latency_ms']:.1f}ms")
        print(f"  超标率: {load_stats['over_threshold_rate']*100:.1f}%")
        print(f"  模式切换: {load_stats['mode_switches']}次")
        
        # 关闭资源
        await self.client.disconnect()
        
        if self.inference_pool:
            self.inference_pool.shutdown(wait=False)
        if self.io_pool:
            self.io_pool.shutdown(wait=False)
        
        print("\n资源已清理")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description="异步并行控制器")
    
    parser.add_argument("--duration", type=float, default=120.0,
                        help="运行时长（秒）")
    parser.add_argument("--hz", type=float, default=50.0,
                        help="目标控制频率")
    parser.add_argument("--workers", type=int, default=2,
                        help="推理进程数")
    parser.add_argument("--with-onnx", action="store_true",
                        help="启用ONNX推理")
    parser.add_argument("--fault-injection", action="store_true",
                        help="启用故障注入")
    
    args = parser.parse_args()
    
    config = AsyncControllerConfig(
        target_hz=args.hz,
        inference_workers=args.workers,
        enable_onnx=args.with_onnx,
        enable_fault_injection=args.fault_injection
    )
    
    controller = AsyncController(config)
    asyncio.run(controller.run(duration=args.duration))


if __name__ == "__main__":
    main()
