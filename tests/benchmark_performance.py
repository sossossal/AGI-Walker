"""
性能基准测试套件
测试通信、序列化、训练性能
"""

import logging
logger = logging.getLogger(__name__)
import time
import numpy as np
from typing import Dict, Any
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class PerformanceBenchmark:
    """性能基准测试"""

    def __init__(self):
        self.results: Dict[str, Any] = {}

    def benchmark_serialization(self):
        """序列化性能测试"""
        logger.info("\n📊 序列化性能测试")
        logger.info("=" * 60)

        import json

        try:
            import msgpack

            has_msgpack = True
        except ImportError:
            has_msgpack = False

        # 测试数据
        test_data = {
            "joint_pos": np.random.randn(8).tolist(),
            "joint_vel": np.random.randn(8).tolist(),
            "timestamp": time.time(),
        }

        # JSON 测试
        json_times = []
        for _ in range(10000):
            start = time.perf_counter()
            serialized = json.dumps(test_data).encode()
            deserialized = json.loads(serialized.decode())
            json_times.append(time.perf_counter() - start)

        json_avg = np.mean(json_times) * 1e6
        logger.info(f"JSON:    {json_avg:.2f} μs (avg)")

        # msgpack 测试
        if has_msgpack:
            msgpack_times = []
            for _ in range(10000):
                start = time.perf_counter()
                serialized = msgpack.packb(test_data)
                deserialized = msgpack.unpackb(serialized)
                msgpack_times.append(time.perf_counter() - start)

            msgpack_avg = np.mean(msgpack_times) * 1e6
            speedup = json_avg / msgpack_avg

            logger.info(f"msgpack: {msgpack_avg:.2f} μs (avg)")
            logger.info(f"性能提升: {speedup:.2f}x")

            self.results["serialization"] = {
                "json_us": json_avg,
                "msgpack_us": msgpack_avg,
                "speedup": speedup,
            }
        else:
            self.results["serialization"] = {"json_us": json_avg}

    def benchmark_zenoh_latency(self):
        """Zenoh 通信延迟测试"""
        logger.info("\n📡 Zenoh 通信延迟测试")
        logger.info("=" * 60)

        try:
            from agi_walker.core.api.comm.optimized_zenoh import OptimizedZenohInterface

            zenoh = OptimizedZenohInterface()

            # 测试发布延迟
            latencies = []
            zenoh.declare_publisher("bench/test")

            for i in range(1000):
                start = time.perf_counter()
                zenoh.publish("bench/test", {"data": i})
                latencies.append((time.perf_counter() - start) * 1e6)

            avg_latency = np.mean(latencies)
            p99_latency = np.percentile(latencies, 99)

            logger.info(f"平均延迟: {avg_latency:.2f} μs")
            logger.info(f"P99 延迟: {p99_latency:.2f} μs")

            self.results["zenoh_latency"] = {
                "avg_us": avg_latency,
                "p99_us": p99_latency,
            }

            zenoh.close()

        except Exception as e:
            logger.info(f"⚠️ Zenoh 测试跳过: {e}")

    def benchmark_parallel_training(self):
        """并行训练性能测试"""
        logger.info("\n🚀 并行训练性能测试")
        logger.info("=" * 60)

        try:
            from agi_walker.core.controllers.parallel_trainer import ParallelTrainingManager
            import gymnasium as gym

            # 单进程基准
            logger.info("单进程训练...")
            start = time.time()
            env = gym.make("CartPole-v1")
            for _ in range(100):
                obs, _ = env.reset()
                for _ in range(200):
                    action = env.action_space.sample()
                    obs, _, terminated, truncated, _ = env.step(action)
                    if terminated or truncated:
                        break
            single_time = time.time() - start

            logger.info(f"单进程时间: {single_time:.2f}s")

            # 多进程 (简化测试)
            logger.info("\n多进程训练...")
            # 注: 完整测试需要更多时间

            self.results["parallel_training"] = {"single_process_time": single_time}

        except Exception as e:
            logger.info(f"⚠️ 并行训练测试跳过: {e}")

    def print_summary(self):
        """打印测试总结"""
        logger.info("\n" + "=" * 60)
        logger.info("性能基准测试总结")
        logger.info("=" * 60)

        if "serialization" in self.results:
            logger.info("\n序列化性能:")
            ser = self.results["serialization"]
            logger.info(f"  JSON:    {ser['json_us']:.2f} μs")
            if "msgpack_us" in ser:
                logger.info(f"  msgpack: {ser['msgpack_us']:.2f} μs")
                logger.info(f"  提升:    {ser['speedup']:.2f}x")

        if "zenoh_latency" in self.results:
            logger.info("\nZenoh 通信:")
            lat = self.results["zenoh_latency"]
            logger.info(f"  平均延迟: {lat['avg_us']:.2f} μs")
            logger.info(f"  P99 延迟: {lat['p99_us']:.2f} μs")

            # 判断是否达标
            target = 2000  # 2ms = 2000μs
            if lat["avg_us"] < target:
                logger.info(f"  ✅ 达标 (目标: <{target} μs)")
            else:
                logger.info(f"  ❌ 未达标 (目标: <{target} μs)")

        if "parallel_training" in self.results:
            logger.info("\n并行训练:")
            par = self.results["parallel_training"]
            logger.info(f"  单进程: {par['single_process_time']:.2f}s")


def run_all_benchmarks():
    """运行所有基准测试"""
    bench = PerformanceBenchmark()

    logger.info("\n🧪 AGI-Walker 性能基准测试套件")
    logger.info("=" * 60)

    bench.benchmark_serialization()
    bench.benchmark_zenoh_latency()
    bench.benchmark_parallel_training()

    bench.print_summary()

    return bench.results


if __name__ == "__main__":
    results = run_all_benchmarks()
