"""
物理参数调优工具
用于测试不同参数组合对机器人稳定性的影响
"""

import time
import json
from tcp_client import GodotClient
from typing import Dict, List


class PhysicsTuner:
    """物理参数调优器"""
    
    def __init__(self):
        self.client = GodotClient()
        self.test_results = []
    
    def test_stability(self, duration: float = 10.0) -> Dict:
        """测试当前参数下的稳定性"""
        
        if not self.client.connect():
            print("❌ 无法连接到仿真器")
            return {}
        
        print(f"\n🧪 开始稳定性测试 (持续{duration}秒)...")
        
        start_time = time.time()
        
        # 统计数据
        total_tilt = 0.0
        max_tilt = 0.0
        samples = 0
        fall_time = None
        
        while time.time() - start_time < duration:
            sensor = self.client.get_latest_sensors()
            
            if sensor:
                # 计算总倾斜角
                roll = abs(sensor['sensors']['imu']['orient'][0])
                pitch = abs(sensor['sensors']['imu']['orient'][1])
                tilt = roll + pitch
                
                total_tilt += tilt
                max_tilt = max(max_tilt, tilt)
                samples += 1
                
                # 检测摔倒
                if tilt > 45 and fall_time is None:
                    fall_time = time.time() - start_time
                    print(f"❌ 机器人摔倒! (t={fall_time:.2f}s)")
                    break
            
            time.sleep(0.033)  # 30Hz
        
        self.client.close()
        
        # 计算结果
        avg_tilt = total_tilt / samples if samples > 0 else 0
        stability_score = 100 - min(avg_tilt, 100)  # 越低越好
        
        result = {
            "duration": fall_time if fall_time else duration,
            "avg_tilt": avg_tilt,
            "max_tilt": max_tilt,
            "stability_score": stability_score,
            "fell": fall_time is not None
        }
        
        print(f"\n📊 测试结果:")
        print(f"  持续时间: {result['duration']:.2f}s")
        print(f"  平均倾斜: {result['avg_tilt']:.2f}°")
        print(f"  最大倾斜: {result['max_tilt']:.2f}°")
        print(f"  稳定性评分: {result['stability_score']:.1f}/100")
        
        return result
    
    def test_motor_response(self) -> Dict:
        """测试电机响应速度"""
        
        if not self.client.connect():
            return {}
        
        print("\n🧪 测试电机响应速度...")
        
        # 发送目标角度
        target_angle = 30.0
        self.client.send_motor_commands({
            "motors": {
                "hip_left": target_angle,
                "hip_right": target_angle
            }
        })
        
        # 测量到达目标的时间
        start_time = time.time()
        reached = False
        response_time = None
        
        while time.time() - start_time < 5.0:
            sensor = self.client.get_latest_sensors()
            
            if sensor:
                left_angle = sensor['sensors']['joints']['hip_left']['angle']
                right_angle = sensor['sensors']['joints']['hip_right']['angle']
                
                # 检查是否接近目标（误差<5度）
                if abs(left_angle - target_angle) < 5 and abs(right_angle - target_angle) < 5:
                    if not reached:
                        response_time = time.time() - start_time
                        reached = True
                        break
            
            time.sleep(0.01)
        
        self.client.close()
        
        if reached:
            print(f"✅ 电机响应时间: {response_time:.3f}s")
            return {"response_time": response_time, "success": True}
        else:
            print("❌ 电机未能到达目标位置")
            return {"response_time": None, "success": False}
    
    def run_parameter_sweep(self, param_sets: List[Dict]):
        """批量测试多组参数"""
        
        print("\n" + "="*60)
        print("🔬 参数扫描测试")
        print("="*60)
        
        for i, params in enumerate(param_sets, 1):
            print(f"\n--- 测试组 {i}/{len(param_sets)} ---")
            print(f"参数: {params}")
            
            # 注意: 实际应用中需要动态修改Godot中的参数
            # 这里只是测试框架
            
            result = self.test_stability(duration=10.0)
            result["params"] = params
            self.test_results.append(result)
            
            time.sleep(2)  # 等待重置
        
        # 输出最佳结果
        self._print_best_results()
    
    def _print_best_results(self):
        """打印最佳测试结果"""
        
        if not self.test_results:
            return
        
        # 按稳定性评分排序
        sorted_results = sorted(self.test_results, 
                               key=lambda x: x['stability_score'], 
                               reverse=True)
        
        print("\n" + "="*60)
        print("🏆 最佳参数配置")
        print("="*60)
        
        for i, result in enumerate(sorted_results[:3], 1):
            print(f"\n#{i} 稳定性评分: {result['stability_score']:.1f}")
            print(f"   参数: {result.get('params', 'N/A')}")
            print(f"   持续: {result['duration']:.2f}s")
            print(f"   倾斜: {result['avg_tilt']:.2f}°")


# 使用示例
if __name__ == "__main__":
    tuner = PhysicsTuner()
    
    # 测试当前配置
    print("="*60)
    print("🎯 测试当前物理配置")
    print("="*60)
    
    # 稳定性测试
    tuner.test_stability(duration=30.0)
    
    # 电机响应测试
    # tuner.test_motor_response()
    
    print("\n✅ 测试完成")
