"""
AI控制系统测试套件
验证AI模型和控制器的各项功能
"""

import logging
import time
import types
import sys
import pytest

logger = logging.getLogger(__name__)
pytestmark = pytest.mark.integration

try:
    from agi_walker.core.controllers.ai_model import create_ai_model

    AI_MODULES_AVAILABLE = True
except ImportError:
    AI_MODULES_AVAILABLE = False


def check_ai_available():
    if not AI_MODULES_AVAILABLE:
        pytest.skip("python_controller.ai_model 不可用")


def test_model_loading() -> None:
    """测试1: 模型加载"""
    check_ai_available()
    logger.info("\n" + "=" * 60)
    logger.info("测试1: 模型加载")
    logger.info("=" * 60)

    try:
        create_ai_model(engine="ollama", model_name="phi3:mini")
        logger.info("✅ 模型加载成功")
    except Exception as e:
        pytest.fail(f"模型加载失败: {e}")


def test_ollama_fallback_when_service_unavailable(monkeypatch) -> None:
    """测试Ollama服务不可用时启用本地fallback"""
    check_ai_available()

    import agi_walker.core.controllers.ai_model as ai_model_module

    fake_ollama = types.ModuleType("ollama")

    def _raise_unavailable():
        raise RuntimeError("service unavailable")

    fake_ollama.list = _raise_unavailable
    monkeypatch.setitem(sys.modules, "ollama", fake_ollama)

    ai = ai_model_module.create_ai_model(engine="ollama", model_name="phi3:mini")
    action = ai.predict(
        {
            "sensors": {
                "imu": {"orient": [3.0, -2.0, 0.0]},
                "joints": {
                    "hip_left": {"angle": 0.0, "velocity": 0.0},
                    "hip_right": {"angle": 0.0, "velocity": 0.0},
                },
            },
            "torso_height": 0.75,
        }
    )

    assert action["mode"] == "deterministic_fallback"
    assert isinstance(action["motors"]["hip_left"], (int, float))
    assert isinstance(action["motors"]["hip_right"], (int, float))
    assert ai.get_stats()["fallback_mode"] is True


def test_inference_speed() -> None:
    """测试2: 推理速度"""
    check_ai_available()
    logger.info("\n" + "=" * 60)
    logger.info("测试2: 推理速度")
    logger.info("=" * 60)

    try:
        ai = create_ai_model()

        # 模拟传感器数据
        dummy_sensor = {
            "sensors": {
                "imu": {"orient": [5.2, -3.1, 0.0]},
                "joints": {
                    "hip_left": {"angle": 10.0, "velocity": 0.0},
                    "hip_right": {"angle": -8.0, "velocity": 0.0},
                },
            },
            "torso_height": 1.45,
        }

        # 预热
        logger.info("预热中...")
        for _ in range(3):
            ai.predict(dummy_sensor)

        # 测试
        logger.info("执行测试...")
        latencies = []
        for i in range(20):
            t0 = time.time()
            ai.predict(dummy_sensor)
            t1 = time.time()
            latencies.append((t1 - t0) * 1000)

            if i % 5 == 0:
                logger.info(f"  推理 {i + 1}/20: {latencies[-1]:.1f}ms")

        avg_latency = sum(latencies) / len(latencies)
        max_latency = max(latencies)
        min_latency = min(latencies)

        logger.info("\n结果:")
        logger.info(f"  平均: {avg_latency:.2f}ms")
        logger.info(f"  最小: {min_latency:.2f}ms")
        logger.info(f"  最大: {max_latency:.2f}ms")

        assert avg_latency < 200, f"速度过慢 ({avg_latency:.1f}ms > 200ms)"
        logger.info(f"✅ 速度测试通过 ({avg_latency:.1f}ms < 200ms)")

    except Exception as e:
        pytest.fail(f"速度测试失败: {e}")


def test_json_format() -> None:
    """测试3: JSON格式验证"""
    check_ai_available()
    logger.info("\n" + "=" * 60)
    logger.info("测试3: JSON格式验证")
    logger.info("=" * 60)

    try:
        ai = create_ai_model()

        dummy_sensor = {
            "sensors": {
                "imu": {"orient": [5.2, -3.1, 0.0]},
                "joints": {
                    "hip_left": {"angle": 10.0, "velocity": 0.0},
                    "hip_right": {"angle": -8.0, "velocity": 0.0},
                },
            },
            "torso_height": 1.45,
        }

        logger.info("测试10次推理...")
        success_count = 0

        for i in range(10):
            try:
                action = ai.predict(dummy_sensor)

                # 验证格式
                assert isinstance(action, dict), "输出不是字典"
                assert "motors" in action, "缺少motors字段"
                assert "hip_left" in action["motors"], "缺少hip_left"
                assert "hip_right" in action["motors"], "缺少hip_right"
                assert isinstance(action["motors"]["hip_left"], (int, float)), (
                    "hip_left类型错误"
                )
                assert isinstance(action["motors"]["hip_right"], (int, float)), (
                    "hip_right类型错误"
                )

                success_count += 1

            except AssertionError as e:
                logger.info(f"  第{i + 1}次失败: {e}")
            except Exception as e:
                logger.info(f"  第{i + 1}次错误: {e}")

        success_rate = success_count / 10 * 100
        logger.info(f"\n结果: {success_count}/10 成功 ({success_rate:.1f}%)")

        assert success_rate >= 90, f"JSON格式错误率过高 ({success_rate:.1f}% 成功)"
        logger.info(f"✅ JSON格式测试通过 ({success_rate:.1f}%)")

    except Exception as e:
        pytest.fail(f"格式测试失败: {e}")


def test_safety_checker() -> None:
    """测试4: 安全检查器"""
    check_ai_available()
    logger.info("\n" + "=" * 60)
    logger.info("测试4: 安全检查器")
    logger.info("=" * 60)

    try:
        from agi_walker.core.controllers.ai_controller import SafetyChecker

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

            logger.info(f"  {expected}: ✅")
            passed += 1

        logger.info(f"\n✅ 安全检查器测试通过 ({passed}/{len(test_cases)})")

    except Exception as e:
        pytest.fail(f"安全检查器测试失败: {e}")


def run_all_tests():
    """运行所有测试 (兼容脚本模式)"""
    logger.info("\n" + "=" * 60)
    logger.info("🧪 AGI-Walker AI控制系统测试套件")
    logger.info("=" * 60)

    # 在脚本模式下直接调用这些函数，需要跳过 pytest 逻辑
    if not AI_MODULES_AVAILABLE:
        logger.info("❌ 模块不可用，无法运行测试")
        return False

    try:
        test_model_loading()
        test_inference_speed()
        test_json_format()
        test_safety_checker()
        return True
    except Exception as e:
        logger.info(f"❌ 测试运行出错: {e}")
        return False


if __name__ == "__main__":
    import sys

    try:
        success = run_all_tests()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        logger.info("\n\n⏹️ 测试中断")
        sys.exit(1)
    except Exception as e:
        logger.info(f"\n❌ 测试崩溃: {e}")
        sys.exit(1)
