"""
PID参数调优工具
自动化测试不同PID参数组合，找到最优配置
"""

import time
import json
from tcp_client import GodotClient
from pid_controller import BalanceController
from typing import List, Tuple, Dict

import logging

logger = logging.getLogger(__name__)


class PIDTuner:
    """PID参数自动调优器"""

    def __init__(self) -> None:
        self.client = GodotClient()
        self.test_results = []

    def test_pid_configuration(
        self,
        roll_params: Tuple[float, float, float],
        pitch_params: Tuple[float, float, float],
        duration: float = 30.0,
    ) -> Dict:
        """
        测试特定PID配置的性能

        Args:
            roll_params: (Kp, Ki, Kd) for Roll
            pitch_params: (Kp, Ki, Kd) for Pitch
            duration: 测试时长（秒）

        Returns:
            测试结果字典
        """
        if not self.client.connect():
            logger.info("❌ 无法连接到仿真器")
            return {}

        logger.info("\n🧪 测试配置:")
        logger.info(
            f"  Roll PID: Kp={roll_params[0]:.2f}, Ki={roll_params[1]:.2f}, Kd={roll_params[2]:.2f}"
        )
        logger.info(
            f"  Pitch PID: Kp={pitch_params[0]:.2f}, Ki={pitch_params[1]:.2f}, Kd={pitch_params[2]:.2f}"
        )

        # 创建平衡控制器
        balance = BalanceController(roll_params, pitch_params)

        # 统计数据
        total_deviation = 0.0
        max_deviation = 0.0
        samples = 0
        stable_time = 0.0
        last_time = time.time()

        start_time = time.time()

        while time.time() - start_time < duration:
            current_time = time.time()
            dt = current_time - last_time

            # 获取传感器数据
            sensor = self.client.get_latest_sensors()

            if sensor:
                # 计算平衡指令
                commands = balance.compute_balance(sensor, dt)

                # 发送指令
                self.client.send_motor_commands(commands)

                # 统计性能
                orient = sensor["sensors"]["imu"]["orient"]
                roll = abs(orient[0])
                pitch = abs(orient[1])
                deviation = roll + pitch

                total_deviation += deviation
                max_deviation = max(max_deviation, deviation)
                samples += 1

                # 稳定时间（偏差<5度）
                if deviation < 5.0:
                    stable_time += dt

                # 检测摔倒
                if deviation > 45:
                    logger.info(f"❌ 机器人摔倒! (t={time.time() - start_time:.2f}s)")
                    break

            last_time = current_time
            time.sleep(0.01)

        self.client.close()

        # 计算结果
        avg_deviation = total_deviation / samples if samples > 0 else 999
        stability_score = max(0, 100 - avg_deviation * 2)

        result = {
            "roll_params": roll_params,
            "pitch_params": pitch_params,
            "avg_deviation": avg_deviation,
            "max_deviation": max_deviation,
            "stable_time": stable_time,
            "stability_score": stability_score,
            "duration": time.time() - start_time,
        }

        logger.info("\n📊 测试结果:")
        logger.info(f"  平均偏差: {avg_deviation:.2f}°")
        logger.info(f"  最大偏差: {max_deviation:.2f}°")
        logger.info(f"  稳定时间: {stable_time:.1f}s / {duration:.1f}s")
        logger.info(f"  稳定性评分: {stability_score:.1f}/100")

        return result

    def grid_search(
        self,
        kp_range: List[float],
        ki_range: List[float],
        kd_range: List[float],
        test_duration: float = 20.0,
    ):
        """
        网格搜索最优PID参数

        Args:
            kp_range: Kp值列表
            ki_range: Ki值列表
            kd_range: Kd值列表
            test_duration: 每次测试时长
        """
        logger.info("\n" + "=" * 60)
        logger.info("🔬 PID参数网格搜索")
        logger.info("=" * 60)
        logger.info(f"参数空间: Kp={kp_range}, Ki={ki_range}, Kd={kd_range}")
        logger.info(f"总组合数: {len(kp_range) * len(ki_range) * len(kd_range)}")

        self.test_results = []
        test_count = 0

        for kp in kp_range:
            for ki in ki_range:
                for kd in kd_range:
                    test_count += 1
                    logger.info(f"\n--- 测试 {test_count} ---")

                    # 同时用于Roll和Pitch
                    result = self.test_pid_configuration(
                        (kp, ki, kd), (kp, ki, kd), test_duration
                    )

                    self.test_results.append(result)

                    # 等待一下，让仿真器重置
                    time.sleep(2)

        # 输出最佳结果
        self._print_best_results()

    def adaptive_search(
        self,
        initial_params: Tuple[float, float, float] = (5.0, 0.1, 2.0),
        step_sizes: Tuple[float, float, float] = (1.0, 0.05, 0.5),
        iterations: int = 10,
    ):
        """
        自适应搜索 - 从初始参数开始，逐步优化

        Args:
            initial_params: 初始PID参数
            step_sizes: 搜索步长
            iterations: 迭代次数
        """
        logger.info("\n" + "=" * 60)
        logger.info("🎯 PID参数自适应搜索")
        logger.info("=" * 60)

        best_params = initial_params
        best_score = 0.0

        for i in range(iterations):
            logger.info(f"\n=== 迭代 {i+1}/{iterations} ===")

            # 测试当前参数
            result = self.test_pid_configuration(
                best_params, best_params, duration=20.0
            )

            current_score = result["stability_score"]

            if current_score > best_score:
                best_score = current_score
                logger.info(f"✅ 发现更好的配置! 评分: {best_score:.1f}")

            # 尝试周围的参数
            neighbors = self._generate_neighbors(best_params, step_sizes)

            for neighbor in neighbors[:3]:  # 只测试3个邻居
                result = self.test_pid_configuration(neighbor, neighbor, duration=15.0)

                if result["stability_score"] > best_score:
                    best_params = neighbor
                    best_score = result["stability_score"]
                    logger.info(f"✨ 更新最佳参数: {best_params}, 评分: {best_score:.1f}")

                time.sleep(2)

        logger.info("\n🏆 最终最佳参数:")
        logger.info(
            f"  Kp={best_params[0]:.2f}, Ki={best_params[1]:.2f}, Kd={best_params[2]:.2f}"
        )
        logger.info(f"  评分: {best_score:.1f}/100")

    def _generate_neighbors(self, params: Tuple, steps: Tuple) -> List[Tuple]:
        """生成邻近参数组合"""
        kp, ki, kd = params
        kp_step, ki_step, kd_step = steps

        neighbors = [
            (kp + kp_step, ki, kd),
            (kp - kp_step, ki, kd),
            (kp, ki + ki_step, kd),
            (kp, ki - ki_step, kd),
            (kp, ki, kd + kd_step),
            (kp, ki, kd - kd_step),
        ]

        # 过滤负值
        neighbors = [(max(0, p[0]), max(0, p[1]), max(0, p[2])) for p in neighbors]

        return neighbors

    def _print_best_results(self, top_n: int = 5) -> List:
        """打印最佳结果"""
        if not self.test_results:
            return

        sorted_results = sorted(
            self.test_results, key=lambda x: x["stability_score"], reverse=True
        )

        logger.info("\n" + "=" * 60)
        logger.info(f"🏆 前{top_n}名配置")
        logger.info("=" * 60)

        for i, result in enumerate(sorted_results[:top_n], 1):
            kp, ki, kd = result["roll_params"]
            logger.info(f"\n#{i} 评分: {result['stability_score']:.1f}/100")
            logger.info(f"   PID参数: Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.2f}")
            logger.info(f"   平均偏差: {result['avg_deviation']:.2f}°")
            logger.info(f"   稳定时间: {result['stable_time']:.1f}s")

    def save_results(self, filename: str = "pid_tuning_results.json") -> List:
        """保存调优结果"""
        with open(filename, "w") as f:
            json.dump(self.test_results, f, indent=2)
        logger.info(f"\n💾 结果已保存到: {filename}")


# 使用示例
if __name__ == "__main__":
    tuner = PIDTuner()

    logger.info("PID参数调优工具")
    logger.info("=" * 60)

    # 选择调优方法
    method = input("\n选择调优方法 [1: 网格搜索, 2: 自适应搜索]: ").strip()

    if method == "1":
        # 网格搜索
        tuner.grid_search(
            kp_range=[4.0, 6.0, 8.0, 10.0],
            ki_range=[0.1, 0.3, 0.5],
            kd_range=[1.0, 2.0, 3.0, 4.0],
            test_duration=15.0,
        )
    else:
        # 自适应搜索
        tuner.adaptive_search(
            initial_params=(8.0, 0.5, 3.0), step_sizes=(1.0, 0.1, 0.5), iterations=10
        )

    # 保存结果
    tuner.save_results()

    logger.info("\n✅ 调优完成")
