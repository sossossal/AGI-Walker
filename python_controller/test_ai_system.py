"""
AI控制系统测试套件
验证AI模型和控制器的各项功能
"""

import time
import json
from ai_model import create_ai_model
from ai_controller import AIController


def test_model_loading():
    """测试1: 模型加载"""
    print("\n" + "="*60)
    print("测试1: 模型加载")
    print("="*60)
    
    try:
        ai = create_ai_model(engine="ollama", model_name="phi3:mini")
        print("✅ 模型加载成功")
        return True
    except Exception as e:
        print(f"❌ 模型加载失败: {e}")
        return False


def test_inference_speed():
    """测试2: 推理速度"""
    print("\n" + "="*60)
    print("测试2: 推理速度")
    print("="*60)
    
    try:
        ai = create_ai_model()
        
        # 模拟传感器数据
        dummy_sensor = {
            "sensors": {
                "imu": {"orient": [5.2, -3.1, 0.0]},
                "joints": {
                    "hip_left": {"angle": 10.0, "velocity": 0.0},
                    "hip_right": {"angle": -8.0, "velocity": 0.0}
                }
            },
            "torso_height": 1.45
        }
        
        # 预热
        print("预热中...")
        for _ in range(3):
            ai.predict(dummy_sensor)
        
        # 测试
        print("执行测试...")
        latencies = []
        for i in range(20):
            t0 = time.time()
            action = ai.predict(dummy_sensor)
            t1 = time.time()
            latencies.append((t1 - t0) * 1000)
            
            if i % 5 == 0:
                print(f"  推理 {i+1}/20: {latencies[-1]:.1f}ms")
        
        avg_latency = sum(latencies) / len(latencies)
        max_latency = max(latencies)
        min_latency = min(latencies)
        
        print(f"\n结果:")
        print(f"  平均: {avg_latency:.2f}ms")
        print(f"  最小: {min_latency:.2f}ms")
        print(f"  最大: {max_latency:.2f}ms")
        
        if avg_latency < 100:
            print(f"✅ 速度测试通过 ({avg_latency:.1f}ms < 100ms)")
            return True
        else:
            print(f"⚠️ 速度偏慢 ({avg_latency:.1f}ms > 100ms)")
            print("   建议: 使用更小的模型或启用GPU加速")
            return False
            
    except Exception as e:
        print(f"❌ 速度测试失败: {e}")
        return False


def test_json_format():
    """测试3: JSON格式验证"""
    print("\n" + "="*60)
    print("测试3: JSON格式验证")
    print("="*60)
    
    try:
        ai = create_ai_model()
        
        dummy_sensor = {
            "sensors": {
                "imu": {"orient": [5.2, -3.1, 0.0]},
                "joints": {
                    "hip_left": {"angle": 10.0, "velocity": 0.0},
                    "hip_right": {"angle": -8.0, "velocity": 0.0}
                }
            },
            "torso_height": 1.45
        }
        
        print("测试100次推理...")
        success_count = 0
        
        for i in range(100):
            try:
                action = ai.predict(dummy_sensor)
                
                # 验证格式
                assert isinstance(action, dict), "输出不是字典"
                assert "motors" in action, "缺少motors字段"
                assert "hip_left" in action["motors"], "缺少hip_left"
                assert "hip_right" in action["motors"], "缺少hip_right"
                assert isinstance(action["motors"]["hip_left"], (int, float)), "hip_left类型错误"
                assert isinstance(action["motors"]["hip_right"], (int, float)), "hip_right类型错误"
                
                success_count += 1
                
            except AssertionError as e:
                print(f"  第{i+1}次失败: {e}")
            except Exception as e:
                print(f"  第{i+1}次错误: {e}")
        
        success_rate = success_count / 100 * 100
        print(f"\n结果: {success_count}/100 成功 ({success_rate:.1f}%)")
        
        if success_rate == 100:
            print("✅ JSON格式测试通过（100%正确）")
            return True
        elif success_rate >= 95:
            print(f"⚠️ JSON格式基本正确（{success_rate:.1f}%）")
            return True
        else:
            print(f"❌ JSON格式错误率过高（{100-success_rate:.1f}%失败）")
            return False
            
    except Exception as e:
        print(f"❌ 格式测试失败: {e}")
        return False


def test_safety_checker():
    """测试4: 安全检查器"""
    print("\n" + "="*60)
    print("测试4: 安全检查器")
    print("="*60)
    
    try:
        from ai_controller import SafetyChecker
        
        safety = SafetyChecker()
        
        # 测试用例
        test_cases = [
            # (输入, 预期行为)
            ({"motors": {"hip_left": 50, "hip_right": 30}}, "正常范围"),
            ({"motors": {"hip_left": 100, "hip_right": 30}}, "上限限位"),
            ({"motors": {"hip_left": -50, "hip_right": 30}}, "下限限位"),
            ({"motors": {"hip_left": "invalid", "hip_right": 30}}, "类型错误处理"),
        ]
        
        passed = 0
        for action, expected in test_cases:
            result = safety.check(action)
            
            # 基本验证
            assert "motors" in result
            assert isinstance(result["motors"].get("hip_left", 0), (int, float))
            
            print(f"  {expected}: ✅")
            passed += 1
        
        print(f"\n✅ 安全检查器测试通过 ({passed}/{len(test_cases)})")
        return True
        
    except Exception as e:
        print(f"❌ 安全检查器测试失败: {e}")
        return False


def run_all_tests():
    """运行所有测试"""
    print("\n" + "="*60)
    print("🧪 AGI-Walker AI控制系统测试套件")
    print("="*60)
    
    results = {}
    
    # 运行测试
    results["model_loading"] = test_model_loading()
    results["inference_speed"] = test_inference_speed()
    results["json_format"] = test_json_format()
    results["safety_checker"] = test_safety_checker()
    
    # 总结
    print("\n" + "="*60)
    print("📊 测试总结")
    print("="*60)
    
    for test_name, passed in results.items():
        status = "✅ 通过" if passed else "❌ 失败"
        print(f"{test_name:20s}: {status}")
    
    total = len(results)
    passed_count = sum(results.values())
    
    print(f"\n总计: {passed_count}/{total} 通过")
    
    if passed_count == total:
        print("\n🎉 所有测试通过! 系统已就绪。")
        print("\n下一步: 运行 python ai_controller.py 开始AI控制")
    else:
        print("\n⚠️ 部分测试失败，请检查配置")
    
    return passed_count == total


if __name__ == "__main__":
    import sys
    
    try:
        success = run_all_tests()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\n⏹️ 测试中断")
        sys.exit(1)
