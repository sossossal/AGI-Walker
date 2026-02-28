"""
AI驱动的机器人控制器
使用3B小模型进行实时控制
"""

import time
import json
from typing import Optional
from tcp_client import GodotClient
from ai_model import create_ai_model, BaseAIModel


class SafetyChecker:
    """安全检查器"""

    def __init__(self):
        # 关节限位
        self.joint_limits = {"hip_left": (-45, 90), "hip_right": (-45, 90)}

        # 速度限制
        self.max_angle_change = 20  # 度/帧
        self.last_angles = {}

    def check(self, action: dict) -> dict:
        """检查并修正动作"""
        safe_action = {"motors": {}}

        for joint, angle in action.get("motors", {}).items():
            # 类型检查
            if not isinstance(angle, (int, float)):
                print(f"⚠️ {joint} 角度类型错误: {type(angle)}")
                angle = 0.0

            # 限位检查
            if joint in self.joint_limits:
                min_angle, max_angle = self.joint_limits[joint]
                original_angle = angle
                angle = max(min_angle, min(max_angle, angle))

                if angle != original_angle:
                    print(f"⚠️ {joint} 角度限位: {original_angle:.1f}° → {angle:.1f}°")

            # 速度限制
            if joint in self.last_angles:
                max_change = self.max_angle_change
                change = angle - self.last_angles[joint]

                if abs(change) > max_change:
                    angle = self.last_angles[joint] + (
                        max_change if change > 0 else -max_change
                    )
                    print(f"⚠️ {joint} 速度限制: {change:.1f}° → {max_change:.1f}°")

            safe_action["motors"][joint] = angle
            self.last_angles[joint] = angle

        return safe_action


class AIController:
    """AI控制器主类"""

    def __init__(
        self,
        ai_model: Optional[BaseAIModel] = None,
        strategy: str = "保持躯干直立，站立稳定",
    ):
        """
        初始化AI控制器

        Args:
            ai_model: AI模型实例（如果为None，使用默认Ollama）
            strategy: 控制策略
        """
        self.client = GodotClient()
        self.ai_model = ai_model or create_ai_model(engine="ollama")
        self.safety = SafetyChecker()
        self.strategy = strategy

        # 统计
        self.loop_count = 0
        self.start_time = None
        self.fall_time = None

    def run(self, duration: float = 120.0, target_hz: float = 30.0):
        """
        运行AI控制循环

        Args:
            duration: 运行时长（秒）
            target_hz: 目标控制频率
        """
        if not self.client.connect():
            print("❌ 无法连接到Godot仿真器")
            return

        print("\n" + "=" * 60)
        print("🤖 AI控制器启动")
        print("=" * 60)
        print(f"模型: {self.ai_model.__class__.__name__}")
        print(f"策略: {self.strategy}")
        print(f"目标频率: {target_hz} Hz")
        print(f"运行时长: {duration} 秒")
        print("=" * 60 + "\n")

        self.start_time = time.time()
        loop_time_target = 1.0 / target_hz

        try:
            while time.time() - self.start_time < duration:
                loop_start = time.time()

                # 1. 获取传感器数据
                sensor_data = self.client.get_latest_sensors()
                if not sensor_data:
                    time.sleep(0.01)
                    continue

                # 2. 检查稳定性
                if not self._check_stability(sensor_data):
                    print("❌ 机器人摔倒!")
                    self.fall_time = time.time() - self.start_time
                    break

                # 3. AI推理
                ai_start = time.time()
                action = self.ai_model.predict(sensor_data, self.strategy)
                ai_time = time.time() - ai_start

                # 4. 安全检查
                safe_action = self.safety.check(action)

                # 5. 发送指令
                self.client.send_motor_commands(safe_action)

                # 6. 性能监控
                self.loop_count += 1
                loop_time = time.time() - loop_start

                # 每秒输出一次状态
                if self.loop_count % int(target_hz) == 0:
                    self._print_status(sensor_data, ai_time, loop_time)

                # 警告：推理太慢
                if ai_time > 0.05:  # 50ms
                    print(f"⚠️ AI推理延迟: {ai_time*1000:.1f}ms")

                # 7. 控制频率
                sleep_time = max(0, loop_time_target - loop_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n\n⏹️ 用户中断")

        finally:
            self._cleanup()

    def _check_stability(self, sensor_data: dict) -> bool:
        """检查机器人是否摔倒"""
        orient = sensor_data["sensors"]["imu"]["orient"]
        roll, pitch = orient[0], orient[1]

        # 倾斜超过45度视为摔倒
        if abs(roll) > 45 or abs(pitch) > 45:
            return False

        # 躯干高度过低
        if sensor_data.get("torso_height", 1.0) < 0.3:
            return False

        return True

    def _print_status(self, sensor_data: dict, ai_time: float, loop_time: float):
        """打印状态信息"""
        orient = sensor_data["sensors"]["imu"]["orient"]
        elapsed = time.time() - self.start_time

        print(
            f"[{elapsed:6.1f}s] "
            f"Roll: {orient[0]:5.1f}° | "
            f"Pitch: {orient[1]:5.1f}° | "
            f"高度: {sensor_data['torso_height']:.2f}m | "
            f"AI: {ai_time*1000:4.1f}ms | "
            f"循环: {loop_time*1000:4.1f}ms | "
            f"频率: {1/loop_time:4.1f}Hz"
        )

    def _cleanup(self):
        """清理和统计"""
        self.client.close()

        # 打印总结
        print("\n" + "=" * 60)
        print("📊 运行总结")
        print("=" * 60)

        elapsed = time.time() - self.start_time if self.start_time else 0

        print(f"运行时间: {elapsed:.1f}秒")
        print(f"总循环数: {self.loop_count}")
        print(f"平均频率: {self.loop_count/elapsed:.1f}Hz" if elapsed > 0 else "N/A")

        if self.fall_time:
            print(f"摔倒时间: {self.fall_time:.1f}秒")
        else:
            print("状态: ✅ 稳定站立")

        # AI统计
        ai_stats = self.ai_model.get_stats()
        print("\nAI推理统计:")
        print(f"  总次数: {ai_stats['total_predictions']}")
        print(f"  平均耗时: {ai_stats['avg_inference_time']*1000:.2f}ms")
        print(f"  错误次数: {ai_stats['errors']}")

        if ai_stats.get("error_rate"):
            print(f"  错误率: {ai_stats['error_rate']*100:.1f}%")

        print("=" * 60 + "\n")


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
