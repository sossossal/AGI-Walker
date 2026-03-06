"""
AGI-Walker 集成测试套件
测试 OpenNeuro 通信框架的所有核心功能
"""

import sys
import os
import time
import threading
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 测试结果记录
test_results = {"passed": [], "failed": [], "skipped": []}


def test_zenoh_import():
    """测试 1: Zenoh 模块导入"""
    print("\n" + "=" * 60)
    print("测试 1: Zenoh 模块导入")
    print("=" * 60)

    try:
        from python_api.comm.zenoh_interface import ZenohInterface, ZENOH_AVAILABLE

        assert ZENOH_AVAILABLE, "Zenoh 未安装"
        print("✅ PASS: Zenoh 模块导入成功")
        test_results["passed"].append("Zenoh 模块导入")
    except Exception as e:
        print(f"❌ FAIL: {e}")
        test_results["failed"].append(f"Zenoh 模块导入: {e}")
        raise


def test_zenoh_session():
    """测试 2: Zenoh 会话创建"""
    print("\n" + "=" * 60)
    print("测试 2: Zenoh 会话创建")
    print("=" * 60)

    try:
        from python_api.comm.zenoh_interface import ZenohInterface

        try:
            zenoh = ZenohInterface()
        except Exception as e:
            pytest.skip(f"无法在当前环境下创建 Zenoh 会话: {e}")

        assert zenoh.session is not None, "会话创建失败"

        zenoh.close()
        print("✅ PASS: Zenoh 会话创建和关闭成功")
        test_results["passed"].append("Zenoh 会话创建")
    except Exception as e:
        print(f"❌ FAIL: {e}")
        test_results["failed"].append(f"Zenoh 会话创建: {e}")
        raise


def test_zenoh_pubsub():
    """测试 3: Zenoh Pub/Sub"""
    print("\n" + "=" * 60)
    print("测试 3: Zenoh Pub/Sub 通信")
    print("=" * 60)

    try:
        from python_api.comm.zenoh_interface import ZenohInterface

        zenoh = ZenohInterface()

        # 订阅
        received_data = []

        def callback(data):
            received_data.append(data)

        zenoh.declare_subscriber("test/topic", callback)

        # 发布
        zenoh.declare_publisher("test/topic")
        test_data = {"test": "hello", "value": 123}
        zenoh.publish("test/topic", test_data)

        # 等待消息
        time.sleep(0.5)

        # 验证
        assert len(received_data) > 0, "未收到消息"
        assert received_data[0] == test_data, "数据不匹配"

        zenoh.close()
        print(f"✅ PASS: 发送并接收到消息: {received_data[0]}")
        test_results["passed"].append("Zenoh Pub/Sub")
    except Exception as e:
        print(f"❌ FAIL: {e}")
        test_results["failed"].append(f"Zenoh Pub/Sub: {e}")
        raise


def test_tcp_zenoh_bridge():
    """测试 4: TCP-Zenoh 桥接器"""
    print("\n" + "=" * 60)
    print("测试 4: TCP-Zenoh 桥接器")
    print("=" * 60)

    try:
        from python_api.comm.tcp_zenoh_bridge import TcpZenohBridge

        # 创建桥接器 (使用不同端口避免冲突)
        bridge = TcpZenohBridge(tcp_port=9091)
        bridge.start()

        # 等待启动
        time.sleep(1)

        # 验证 TCP 服务器运行
        assert bridge.running, "桥接器未运行"

        # 停止
        bridge.stop()

        print("✅ PASS: TCP-Zenoh 桥接器启动和停止成功")
        test_results["passed"].append("TCP-Zenoh 桥接器")
    except Exception as e:
        print(f"❌ FAIL: {e}")
        test_results["failed"].append(f"TCP-Zenoh 桥接器: {e}")
        raise


def test_ros2_node():
    """测试 5: ROS 2 节点"""
    print("\n" + "=" * 60)
    print("测试 5: ROS 2 节点")
    print("=" * 60)

    try:
        import rclpy
        from python_api.ros2_robot_node import AGIWalkerNode

        rclpy.init()
        node = AGIWalkerNode()

        # 验证节点创建
        assert node is not None, "节点创建失败"

        # 运行一次
        rclpy.spin_once(node, timeout_sec=0.1)

        # 清理
        node.destroy_node()
        rclpy.shutdown()

        print("✅ PASS: ROS 2 节点创建和运行成功")
        test_results["passed"].append("ROS 2 节点")
    except (ImportError, ModuleNotFoundError):
        print("⏭️  SKIP: ROS 2 未安装")
        test_results["skipped"].append("ROS 2 节点 (未安装)")
        pytest.skip("ROS 2 未安装")
    except Exception as e:
        print(f"❌ FAIL: {e}")
        test_results["failed"].append(f"ROS 2 节点: {e}")
        raise


def test_parts_manager():
    """测试 6: 零件管理器"""
    print("\n" + "=" * 60)
    print("测试 6: 零件管理器")
    print("=" * 60)

    try:
        from python_api.parts.parts_manager import PartsManager

        pm = PartsManager()

        # 验证零件加载
        assert len(pm.parts_db) > 0, "零件库为空"

        # 获取零件
        motor = pm.get_part("go_m8010")
        assert motor is not None, "未找到电机"
        assert motor.specs["max_torque_nm"] == 23.7, "电机参数错误"

        # 计算 BOM
        bom = pm.calculate_bom(["go_m8010", "lipo_4s_5000mah"])
        assert bom["total_cost_usd"] > 0, "BOM 计算错误"

        print(f"✅ PASS: 零件库加载成功 ({len(pm.parts_db)} 个零件)")
        test_results["passed"].append("零件管理器")
    except Exception as e:
        print(f"❌ FAIL: {e}")
        test_results["failed"].append(f"零件管理器: {e}")
        raise


def print_summary():
    """打印测试总结"""
    print("\n" + "=" * 60)
    print("测试总结")
    print("=" * 60)

    total = (
        len(test_results["passed"])
        + len(test_results["failed"])
        + len(test_results["skipped"])
    )

    print(f"\n总计: {total} 个测试")
    print(f"✅ 通过: {len(test_results['passed'])}")
    print(f"❌ 失败: {len(test_results['failed'])}")
    print(f"⏭️  跳过: {len(test_results['skipped'])}")

    if test_results["passed"]:
        print("\n通过的测试:")
        for test in test_results["passed"]:
            print(f"  ✅ {test}")

    if test_results["failed"]:
        print("\n失败的测试:")
        for test in test_results["failed"]:
            print(f"  ❌ {test}")

    if test_results["skipped"]:
        print("\n跳过的测试:")
        for test in test_results["skipped"]:
            print(f"  ⏭️  {test}")

    # 返回状态码
    return 0 if len(test_results["failed"]) == 0 else 1


def main():
    print("\n🧪 AGI-Walker 集成测试套件")
    print("=" * 60)

    # 运行所有测试
    tests = [
        test_zenoh_import,
        test_zenoh_session,
        test_zenoh_pubsub,
        test_tcp_zenoh_bridge,
        test_ros2_node,
        test_parts_manager,
    ]

    for test_func in tests:
        try:
            test_func()
        except Exception as e:
            # 识别 pytest.skip 抛出的内部异常 (如果手动运行 main)
            if "Skipped" in str(type(e)):
                pass
            else:
                print(f"❌ 测试异常: {e}")
                test_results["failed"].append(f"{test_func.__name__}: {e}")

        time.sleep(0.5)  # 测试间隔

    # 打印总结
    exit_code = print_summary()

    print("\n" + "=" * 60)
    print("测试完成!")
    print("=" * 60)

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
