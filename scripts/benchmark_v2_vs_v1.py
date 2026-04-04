import json
import time
import sys
import os

# 确保能导入 agi_walker
sys.path.append(os.getcwd())

from agi_walker.core.api.comm.proto import robot_protocol_pb2

def benchmark():
    iterations = 10000
    print(f"AGI-Walker V2.0 Performance Benchmark ({iterations} iterations)")
    print("-" * 60)

    # 准备测试数据: 12 个电机的复杂指令
    test_data = {
        "type": "COMMAND",
        "msg_id": 12345,
        "commands": [
            {
                "motor_id": i,
                "target_pos": 0.5 + i * 0.1,
                "target_vel": 1.0,
                "kp": 40.0,
                "kd": 1.5,
                "feed_forward_torque": 0.05
            } for i in range(12)
        ]
    }

    # --- V1: JSON Benchmark ---
    start_v1 = time.time()
    v1_size = 0
    for _ in range(iterations):
        v1_payload = json.dumps(test_data).encode()
        v1_size = len(v1_payload)
        _ = json.loads(v1_payload.decode())
    end_v1 = time.time()
    v1_time = (end_v1 - start_v1) * 1000

    # --- V2: Protobuf Benchmark ---
    # 构建 PB 消息
    msg = robot_protocol_pb2.RobotMessage()
    msg.type = robot_protocol_pb2.RobotMessage.COMMAND
    msg.msg_id = 12345
    for i in range(12):
        cmd = msg.commands.add()
        cmd.motor_id = i
        cmd.target_pos = 0.5 + i * 0.1
        cmd.target_vel = 1.0
        cmd.kp = 40.0
        cmd.kd = 1.5
        cmd.feed_forward_torque = 0.05

    start_v2 = time.time()
    v2_size = 0
    for _ in range(iterations):
        v2_payload = msg.SerializeToString()
        v2_size = len(v2_payload)
        new_msg = robot_protocol_pb2.RobotMessage()
        new_msg.ParseFromString(v2_payload)
    end_v2 = time.time()
    v2_time = (end_v2 - start_v2) * 1000

    # --- 结果对比 ---
    print(f"V1 (JSON)     | Avg Latency: {v1_time/iterations:6.4f} ms | Pkt Size: {v1_size:4d} bytes")
    print(f"V2 (Protobuf) | Avg Latency: {v2_time/iterations:6.4f} ms | Pkt Size: {v2_size:4d} bytes")
    print("-" * 60)
    print(f"Speedup: {v1_time/v2_time:.2f}x faster")
    print(f"Compression: {(1 - v2_size/v1_size)*100:.1f}% reduction in bandwidth")

if __name__ == "__main__":
    try:
        benchmark()
    except Exception as e:
        print(f"Benchmark failed: {e}")
