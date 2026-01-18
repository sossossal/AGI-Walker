"""
完整的系统测试套件
涵盖所有核心功能的自动化测试
"""

import time
import json
import statistics
from typing import Dict, List, Tuple
from tcp_client import GodotClient


class TestRunner:
    """测试运行器基类"""
    
    def __init__(self):
        self.results = []
        self.client = None
    
    def setup(self):
        """测试前准备"""
        self.client = GodotClient()
        
    def teardown(self):
        """测试后清理"""
        if self.client:
            self.client.close()
    
    def record_result(self, test_name: str, passed: bool, details: Dict = None):
        """记录测试结果"""
        self.results.append({
            "test": test_name,
            "passed": passed,
            "details": details or {},
            "timestamp": time.time()
        })


class TCPCommunicationTest(TestRunner):
    """TCP通信测试"""
    
    def test_connection(self) -> bool:
        """测试1: 连接建立"""
        print("\n📡 测试TCP连接...")
        
        try:
            success = self.client.connect(timeout=5.0)
            if success:
                print("✅ 连接成功")
                self.record_result("TCP连接", True)
                return True
            else:
                print("❌ 连接失败")  
                self.record_result("TCP连接", False)
                return False
        except Exception as e:
            print(f"❌ 连接错误: {e}")
            self.record_result("TCP连接", False, {"error": str(e)})
            return False
    
    def test_latency(self, duration: float = 10.0) -> bool:
        """测试2: 通信延迟"""
        print(f"\n⏱️ 测试通信延迟 ({duration}秒)...")
        
        latencies = []
        start_time = time.time()
        
        while time.time() - start_time < duration:
            t0 = time.time()
            
            # 发送命令
            self.client.send_motor_commands({"test": time.time()})
            
            # 等待接收
            sensor = self.client.wait_for_sensors(timeout=0.1)
            
            if sensor:
                t1 = time.time()
                latency = (t1 - t0) * 1000
                latencies.append(latency)
            
            time.sleep(0.01)
        
        if latencies:
            avg = statistics.mean(latencies)
            median = statistics.median(latencies)
            max_lat = max(latencies)
            
            print(f"  平均延迟: {avg:.2f}ms")
            print(f"  中位数: {median:.2f}ms")
            print(f"  最大值: {max_lat:.2f}ms")
            
            passed = avg < 10.0  # 目标<10ms
            
            if passed:
                print(f"✅ 延迟测试通过 ({avg:.2f}ms < 10ms)")
            else:
                print(f"⚠️ 延迟偏高 ({avg:.2f}ms > 10ms)")
            
            self.record_result("通信延迟", passed, {
                "avg_ms": avg,
                "median_ms": median,
                "max_ms": max_lat,
                "samples": len(latencies)
            })
            
            return passed
        else:
            print("❌ 未收到数据")
            self.record_result("通信延迟", False)
            return False
    
    def test_data_integrity(self, samples: int = 100) -> bool:
        """测试3: 数据完整性"""
        print(f"\n🔍 测试数据完整性 ({samples}个样本)...")
        
        valid_count = 0
        
        for i in range(samples):
            sensor = self.client.get_latest_sensors()
            
            if sensor:
                try:
                    # 验证必需字段
                    assert "sensors" in sensor
                    assert "imu" in sensor["sensors"]
                    assert "joints" in sensor["sensors"]
                    assert "orient" in sensor["sensors"]["imu"]
                    assert len(sensor["sensors"]["imu"]["orient"]) == 3
                    
                    valid_count += 1
                    
                except AssertionError:
                    pass
            
            time.sleep(0.01)
        
        success_rate = valid_count / samples * 100
        passed = success_rate >= 95.0
        
        print(f"  有效数据: {valid_count}/{samples} ({success_rate:.1f}%)")
        
        if passed:
            print(f"✅ 数据完整性通过 ({success_rate:.1f}%)")
        else:
            print(f"❌ 数据完整性不足 ({success_rate:.1f}% < 95%)")
        
        self.record_result("数据完整性", passed, {
            "valid_count": valid_count,
            "total": samples,
            "success_rate": success_rate
        })
        
        return passed


class StabilityTest(TestRunner):
    """稳定性测试"""
    
    def test_standing_stability(self, duration: float = 30.0) -> bool:
        """测试4: 站立稳定性"""
        print(f"\n🧍 测试站立稳定性 ({duration}秒)...")
        
        start_time = time.time()
        tilts = []
        heights = []
        fell = False
        fall_time = None
        
        while time.time() - start_time < duration:
            sensor = self.client.get_latest_sensors()
            
            if sensor:
                orient = sensor['sensors']['imu']['orient']
                roll = abs(orient[0])
                pitch = abs(orient[1])
                tilt = roll + pitch
                height = sensor.get('torso_height', 0)
                
                tilts.append(tilt)
                heights.append(height)
                
                # 检测摔倒
                if tilt > 45 or height < 0.3:
                    fell = True
                    fall_time = time.time() - start_time
                    print(f"❌ 机器人摔倒 (t={fall_time:.1f}s)")
                    break
            
            time.sleep(0.033)
        
        if tilts:
            avg_tilt = statistics.mean(tilts)
            max_tilt = max(tilts)
            avg_height = statistics.mean(heights)
            
            print(f"  平均倾斜: {avg_tilt:.2f}°")
            print(f"  最大倾斜: {max_tilt:.2f}°")
            print(f"  平均高度: {avg_height:.2f}m")
            
            if not fell:
                print(f"✅ 稳定站立 {duration}秒")
                passed = True
            else:
                print(f"❌ 在{fall_time:.1f}秒时摔倒")
                passed = False
            
            self.record_result("站立稳定性", passed, {
                "duration": fall_time if fell else duration,
                "avg_tilt": avg_tilt,
                "max_tilt": max_tilt,
                "avg_height": avg_height,
                "fell": fell
            })
            
            return passed
        
        return False


class PerformanceTest(TestRunner):
    """性能测试"""
    
    def test_control_frequency(self, duration: float = 10.0) -> bool:
        """测试5: 控制频率"""
        print(f"\n🔄 测试控制频率 ({duration}秒)...")
        
        start_time = time.time()
        loop_count = 0
        
        while time.time() - start_time < duration:
            # 模拟控制循环
            sensor = self.client.get_latest_sensors()
            if sensor:
                self.client.send_motor_commands({
                    "motors": {"hip_left": 0, "hip_right": 0}
                })
                loop_count += 1
            
            time.sleep(0.01)  # 目标100Hz
        
        elapsed = time.time() - start_time
        frequency = loop_count / elapsed
        
        print(f"  循环数: {loop_count}")
        print(f"  频率: {frequency:.1f} Hz")
        
        passed = frequency >= 20.0  # 目标≥20Hz
        
        if passed:
            print(f"✅ 频率测试通过 ({frequency:.1f}Hz ≥ 20Hz)")
        else:
            print(f"❌ 频率不足 ({frequency:.1f}Hz < 20Hz)")
        
        self.record_result("控制频率", passed, {
            "loops": loop_count,
            "duration": elapsed,
            "frequency_hz": frequency
        })
        
        return passed


def run_full_test_suite():
    """运行完整测试套件"""
    print("="*60)
    print("🧪 AGI-Walker 系统测试套件")
    print("="*60)
    print("\n请确保:")
    print("1. Godot仿真器正在运行")
    print("2. 机器人已正确搭建")
    print("3. TCP服务器已启动 (127.0.0.1:9999)")
    
    input("\n按Enter键开始测试...")
    
    all_results = []
    
    # TCP通信测试
    print("\n" + "="*60)
    print("第1部分: TCP通信测试")
    print("="*60)
    
    tcp_test = TCPCommunicationTest()
    tcp_test.setup()
    
    try:
        tcp_test.test_connection()
        tcp_test.test_latency(duration=10.0)
        tcp_test.test_data_integrity(samples=100)
    finally:
        all_results.extend(tcp_test.results)
        tcp_test.teardown()
    
    # 稳定性测试
    print("\n" + "="*60)
    print("第2部分: 稳定性测试")
    print("="*60)
    
    stability_test = StabilityTest()
    stability_test.setup()
    
    try:
        if stability_test.client.connect():
            stability_test.test_standing_stability(duration=30.0)
    finally:
        all_results.extend(stability_test.results)
        stability_test.teardown()
    
    # 性能测试
    print("\n" + "="*60)
    print("第3部分: 性能测试")
    print("="*60)
    
    perf_test = PerformanceTest()
    perf_test.setup()
    
    try:
        if perf_test.client.connect():
            perf_test.test_control_frequency(duration=10.0)
    finally:
        all_results.extend(perf_test.results)
        perf_test.teardown()
    
    # 生成报告
    _generate_report(all_results)


def _generate_report(results: List[Dict]):
    """生成测试报告"""
    print("\n" + "="*60)
    print("📊 测试报告")
    print("="*60)
    
    total = len(results)
    passed = sum(1 for r in results if r["passed"])
    
    print(f"\n总测试数: {total}")
    print(f"通过: {passed}")
    print(f"失败: {total - passed}")
    print(f"成功率: {passed/total*100:.1f}%")
    
    print(f"\n详细结果:")
    for result in results:
        status = "✅" if result["passed"] else "❌"
        print(f"  {status} {result['test']}")
        
        # 显示详细信息
        if result["details"]:
            for key, value in result["details"].items():
                if isinstance(value, float):
                    print(f"      {key}: {value:.2f}")
                else:
                    print(f"      {key}: {value}")
    
    # 保存JSON报告
    report_file = f"test_report_{int(time.time())}.json"
    with open(report_file, 'w', encoding='utf-8') as f:
        json.dump(results, f, indent=2, ensure_ascii=False)
    
    print(f"\n💾 报告已保存: {report_file}")
    
    if passed == total:
        print("\n🎉 所有测试通过!")
    else:
        print(f"\n⚠️ {total - passed}个测试失败，请检查")


if __name__ == "__main__":
    try:
        run_full_test_suite()
    except KeyboardInterrupt:
        print("\n\n⏹️ 测试中断")
    except Exception as e:
        print(f"\n❌ 测试错误: {e}")
        import traceback
        traceback.print_exc()
