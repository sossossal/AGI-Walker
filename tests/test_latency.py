"""
通信延迟测试脚本
测试Godot仿真器与Python控制端之间的通信性能
"""

import time
import pytest

pytestmark = pytest.mark.integration

try:
    from python_controller.tcp_client import GodotClient
except ImportError:
    pytest.skip("python_controller.tcp_client 不可用", allow_module_level=True)


def test_latency(duration: float = 10.0):
    """测试通信延迟"""

    client = GodotClient()

    if not client.connect():
        print("❌ 连接失败")
        return

    print(f"\n🧪 开始延迟测试 (持续{duration}秒)...\n")

    latencies = []
    start_time = time.time()

    while time.time() - start_time < duration:
        t0 = time.time()

        # 发送测试指令
        client.send_motor_commands({"test": time.time()})

        # 等待传感器数据
        sensor = client.wait_for_sensors(timeout=0.1)

        if sensor:
            t1 = time.time()
            latency = (t1 - t0) * 1000  # 转换为毫秒
            latencies.append(latency)

        time.sleep(0.01)  # 100Hz

    client.close()

    # 统计结果
    if latencies:
        import statistics

        print("\n" + "=" * 50)
        print("📊 延迟测试结果")
        print("=" * 50)
        print(f"测试次数: {len(latencies)}")
        print(f"平均延迟: {statistics.mean(latencies):.2f} ms")
        print(f"中位延迟: {statistics.median(latencies):.2f} ms")
        print(f"最小延迟: {min(latencies):.2f} ms")
        print(f"最大延迟: {max(latencies):.2f} ms")
        print(f"标准差: {statistics.stdev(latencies):.2f} ms")

        # 延迟分布
        print("\n延迟分布:")
        print(
            f"  < 5ms:  {sum(1 for lat in latencies if lat < 5) / len(latencies) * 100:.1f}%"
        )
        print(
            f"  < 10ms: {sum(1 for lat in latencies if lat < 10) / len(latencies) * 100:.1f}%"
        )
        print(
            f"  < 20ms: {sum(1 for lat in latencies if lat < 20) / len(latencies) * 100:.1f}%"
        )
        print(
            f"  ≥ 20ms: {sum(1 for lat in latencies if lat >= 20) / len(latencies) * 100:.1f}%"
        )

        # 判断是否合格
        avg_latency = statistics.mean(latencies)
        if avg_latency < 10:
            print(f"\n✅ 测试通过! 平均延迟 {avg_latency:.2f}ms < 10ms")
        else:
            print(f"\n⚠️ 测试未达标! 平均延迟 {avg_latency:.2f}ms ≥ 10ms")
    else:
        print("❌ 未收到任何数据")


if __name__ == "__main__":
    test_latency(duration=10.0)
