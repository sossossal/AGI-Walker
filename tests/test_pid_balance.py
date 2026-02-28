"""
PID平衡控制专项测试
测试不同PID参数下的平衡效果
"""

import time
import json
import statistics
from typing import List, Tuple, Dict
from tcp_client import GodotClient


class PIDBalanceTest:
    """PID平衡控制测试器"""
    
    def __init__(self):
        self.client = GodotClient()
        self.test_results = []
    
    def test_configuration(self, 
                          test_name: str,
                          duration: float = 30.0,
                          enable_pid: bool = True) -> Dict:
        """
        测试特定配置
        
        Args:
            test_name: 测试名称
            duration: 测试时长
            enable_pid: 是否启用PID控制
        """
        print(f"\n{'='*60}")
        print(f"测试: {test_name}")
        print(f"{'='*60}")
        print(f"PID控制: {'启用' if enable_pid else '禁用'}")
        print(f"时长: {duration}秒")
        
        if not self.client.connect():
            print("❌ 无法连接到仿真器")
            return {}
        
        # TODO: 向Godot发送PID启用/禁用指令
        # 现在只是测试数据收集
        
        start_time = time.time()
        
        # 统计数据
        tilts = []
        roll_values = []
        pitch_values = []
        heights = []
        fell = False
        fall_time = None
        
        print("\n开始监控...")
        
        while time.time() - start_time < duration:
            sensor = self.client.get_latest_sensors()
            
            if sensor:
                orient = sensor['sensors']['imu']['orient']
                roll = orient[0]
                pitch = orient[1]
                height = sensor.get('torso_height', 0)
                
                tilt = abs(roll) + abs(pitch)
                
                tilts.append(tilt)
                roll_values.append(abs(roll))
                pitch_values.append(abs(pitch))
                heights.append(height)
                
                # 检测摔倒
                if tilt > 45 or height < 0.3:
                    fell = True
                    fall_time = time.time() - start_time
                    print(f"\n❌ 机器人摔倒 (t={fall_time:.1f}s)")
                    break
                
                # 定期输出状态
                elapsed = time.time() - start_time
                if int(elapsed) % 5 == 0 and len(tilts) % 30 == 0:
                    print(f"[{elapsed:5.1f}s] Roll: {roll:5.1f}° Pitch: {pitch:5.1f}° 倾斜: {tilt:5.1f}°")
            
            time.sleep(0.033)
        
        self.client.close()
        
        # 计算统计
        result = self._calculate_stats(
            test_name, tilts, roll_values, pitch_values, 
            heights, fell, fall_time, duration
        )
        
        self.test_results.append(result)
        
        return result
    
    def _calculate_stats(self, test_name, tilts, rolls, pitches, 
                        heights, fell, fall_time, duration) -> Dict:
        """计算统计数据"""
        
        if not tilts:
            return {
                "test": test_name,
                "fell": True,
                "duration": 0,
                "score": 0
            }
        
        # 基本统计
        avg_tilt = statistics.mean(tilts)
        max_tilt = max(tilts)
        avg_roll = statistics.mean(rolls)
        avg_pitch = statistics.mean(pitches)
        avg_height = statistics.mean(heights)
        
        # 稳定时间（倾斜<5度）
        stable_count = sum(1 for t in tilts if t < 5.0)
        stable_ratio = stable_count / len(tilts)
        
        # 评分（0-100）
        if fell:
            # 摔倒了，根据持续时间评分
            score = min(50, (fall_time / duration) * 50)
        else:
            # 未摔倒，根据倾斜角评分
            tilt_score = max(0, 100 - avg_tilt * 5)
            stability_score = stable_ratio * 100
            score = (tilt_score * 0.6 + stability_score * 0.4)
        
        result = {
            "test": test_name,
            "fell": fell,
            "duration": fall_time if fell else duration,
            "avg_tilt": avg_tilt,
            "max_tilt": max_tilt,
            "avg_roll": avg_roll,
            "avg_pitch": avg_pitch,
            "avg_height": avg_height,
            "stable_ratio": stable_ratio,
            "score": score
        }
        
        # 打印结果
        print("\n📊 测试结果:")
        print(f"  持续时间: {result['duration']:.1f}秒")
        print(f"  平均倾斜: {avg_tilt:.2f}°")
        print(f"  最大倾斜: {max_tilt:.2f}°")
        print(f"  平均Roll: {avg_roll:.2f}°")
        print(f"  平均Pitch: {avg_pitch:.2f}°")
        print(f"  稳定比例: {stable_ratio*100:.1f}%")
        print(f"  综合评分: {score:.1f}/100")
        
        return result
    
    def compare_tests(self):
        """对比测试结果"""
        if len(self.test_results) < 2:
            print("需要至少2个测试结果进行对比")
            return
        
        print("\n" + "="*60)
        print("📊 测试对比")
        print("="*60)
        
        # 按评分排序
        sorted_results = sorted(self.test_results, 
                               key=lambda x: x['score'], 
                               reverse=True)
        
        print(f"\n{'排名':<5} {'测试名称':<30} {'评分':<10} {'平均倾斜':<12} {'持续':<10}")
        print("-" * 70)
        
        for i, result in enumerate(sorted_results, 1):
            print(f"{i:<5} {result['test']:<30} {result['score']:>6.1f}/100 "
                  f"{result['avg_tilt']:>8.2f}° {result['duration']:>7.1f}s")
        
        # 保存对比报告
        report_file = f"pid_comparison_{int(time.time())}.json"
        with open(report_file, 'w', encoding='utf-8') as f:
            json.dump(sorted_results, f, indent=2, ensure_ascii=False)
        
        print(f"\n💾 对比报告已保存: {report_file}")


def run_pid_tests():
    """运行PID测试"""
    print("="*60)
    print("🧪 PID平衡控制测试")
    print("="*60)
    
    tester = PIDBalanceTest()
    
    # 测试1: 无PID控制（基准）
    print("\n⚠️ 注意: 确保Godot中BalanceController节点的enabled=false")
    input("准备好后按Enter开始基准测试...")
    tester.test_configuration(
        "基准测试-无PID",
        duration=20.0,
        enable_pid=False
    )
    
    # 测试2: 启用PID控制
    print("\n⚠️ 现在在Godot中将BalanceController的enabled=true")
    input("准备好后按Enter开始PID测试...")
    tester.test_configuration(
        "PID控制-默认参数",
        duration=30.0,
        enable_pid=True
    )
    
    # 对比结果
    tester.compare_tests()
    
    print("\n✅ PID测试完成")


if __name__ == "__main__":
    try:
        run_pid_tests()
    except KeyboardInterrupt:
        print("\n\n⏹️ 测试中断")
    except Exception as e:
        print(f"\n❌ 测试错误: {e}")
        import traceback
        traceback.print_exc()
