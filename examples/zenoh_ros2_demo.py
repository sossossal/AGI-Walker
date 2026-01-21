"""
Zenoh + ROS 2 集成演示
展示如何使用 OpenNeuro 通信框架与 AGI-Walker
"""

import sys
import os
import time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from python_api.zenoh_interface import ZenohInterface, ZENOH_AVAILABLE


def demo_zenoh_basic():
    """演示 1: Zenoh 基础通信"""
    print("\n" + "="*60)
    print("演示 1: Zenoh 基础 Pub/Sub")
    print("="*60)
    
    if not ZENOH_AVAILABLE:
        print("❌ Zenoh 未安装，跳过")
        return
    
    zenoh = ZenohInterface()
    
    # 订阅状态
    received_count = [0]
    def on_state(data):
        received_count[0] += 1
        print(f"  📥 [State] {data}")
    
    zenoh.declare_subscriber("demo/robot/state", on_state)
    
    # 发布命令
    zenoh.declare_publisher("demo/robot/cmd")
    
    print("\n发送 5 条命令...")
    for i in range(5):
        cmd = {
            "timestamp": time.time(),
            "joint_positions": [i * 0.1, i * 0.2, i * 0.3]
        }
        zenoh.publish("demo/robot/cmd", cmd)
        print(f"  📤 [Cmd] {cmd}")
        time.sleep(0.5)
    
    print(f"\n✅ 演示完成，接收到 {received_count[0]} 条消息")
    zenoh.close()


def demo_tcp_zenoh_bridge():
    """演示 2: TCP-Zenoh 桥接器"""
    print("\n" + "="*60)
    print("演示 2: TCP-Zenoh 桥接器")
    print("="*60)
    
    try:
        from python_api.tcp_zenoh_bridge import TcpZenohBridge
    except ImportError:
        print("❌ 桥接器模块未找到")
        return
    
    print("\n启动桥接器...")
    bridge = TcpZenohBridge(tcp_port=9091)  # 使用不同端口避免冲突
    bridge.start()
    
    print("\n桥接器运行中 (5秒)...")
    print("  - TCP 端口: 9091")
    print("  - Zenoh 键: rt/godot/state, rt/python/cmd")
    
    time.sleep(5)
    
    bridge.stop()
    print("✅ 桥接器已停止")


def demo_ros2_node():
    """演示 3: ROS 2 节点"""
    print("\n" + "="*60)
    print("演示 3: ROS 2 节点")
    print("="*60)
    
    try:
        import rclpy
        from python_api.ros2_robot_node import AGIWalkerNode
    except ImportError:
        print("❌ ROS 2 未安装，跳过")
        print("   安装方法: sudo apt install ros-jazzy-rclpy")
        return
    
    print("\n启动 ROS 2 节点...")
    rclpy.init()
    node = AGIWalkerNode()
    
    print("\n节点运行中 (5秒)...")
    print("  - Topic: /robot/joint_states")
    print("  - Topic: /robot/joint_commands")
    
    # 运行 5 秒
    start_time = time.time()
    while time.time() - start_time < 5:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    node.destroy_node()
    rclpy.shutdown()
    print("✅ ROS 2 节点已停止")


def main():
    print("\n🚀 AGI-Walker × OpenNeuro 集成演示")
    print("="*60)
    
    demos = [
        ("Zenoh 基础通信", demo_zenoh_basic),
        ("TCP-Zenoh 桥接器", demo_tcp_zenoh_bridge),
        ("ROS 2 节点", demo_ros2_node)
    ]
    
    for i, (name, func) in enumerate(demos, 1):
        print(f"\n[{i}/{len(demos)}] {name}")
        try:
            func()
        except Exception as e:
            print(f"❌ 演示失败: {e}")
        
        if i < len(demos):
            input("\n按 Enter 继续下一个演示...")
    
    print("\n" + "="*60)
    print("🎉 所有演示完成!")
    print("="*60)


if __name__ == "__main__":
    main()
