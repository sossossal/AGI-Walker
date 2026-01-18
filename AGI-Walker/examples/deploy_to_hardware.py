"""
将训练好的策略部署到 IMC-22 硬件
演示完整的 Sim-to-Real 工作流
"""

import sys
import os
import torch
import onnx
from stable_baselines3 import PPO

# 添加父目录到路径
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from godot_robot_env import GodotRobotEnv
from godot_robot_env.hardware_controller import IMC22Controller, HardwareEnvironment


def step1_train_in_simulation():
    """步骤 1: 在仿真中训练策略"""
    print("=" * 60)
    print("步骤 1: 在 AGI-Walker 仿真中训练策略")
    print("=" * 60)
    
    # 创建仿真环境
    env = GodotRobotEnv()
    
    # 训练（这里用较少步数演示）
    print("\n训练策略中...")
    model = PPO("MultiInputPolicy", env, verbose=1)
    model.learn(total_timesteps=10000)  # 实际应用需要更多步数
    
    # 保存模型
    model.save("walker_policy")
    print("✓ 模型已保存: walker_policy.zip")
    
    return model


def step2_export_to_onnx(model):
    """步骤 2: 导出为 ONNX 格式"""
    print("\n" + "=" * 60)
    print("步骤 2: 导出模型为 ONNX")
    print("=" * 60)
    
    # 提取策略网络
    policy = model.policy
    
    # 创建虚拟输入
    dummy_input = torch.randn(1, policy.observation_space.shape[0])
    
    # 导出
    print("\n导出中...")
    torch.onnx.export(
        policy,
        dummy_input,
        "walker_policy.onnx",
        input_names=['observation'],
        output_names=['action'],
        opset_version=11,
        verbose=False
    )
    
    # 检查模型
    model_onnx = onnx.load("walker_policy.onnx")
    onnx.checker.check_model(model_onnx)
    
    # 显示大小
    size_kb = os.path.getsize("walker_policy.onnx") / 1024
    print(f"✓ ONNX 模型已导出: walker_policy.onnx ({size_kb:.2f} KB)")


def step3_quantize_model():
    """步骤 3: 量化为 INT8"""
    print("\n" + "=" * 60)
    print("步骤 3: 量化模型为 INT8")
    print("=" * 60)
    
    try:
        from onnxruntime.quantization import quantize_dynamic, QuantType
        
        print("\n量化中...")
        quantize_dynamic(
            "walker_policy.onnx",
            "walker_policy_int8.onnx",
            weight_type=QuantType.QInt8
        )
        
        # 对比大小
        fp32_size = os.path.getsize("walker_policy.onnx") / 1024
        int8_size = os.path.getsize("walker_policy_int8.onnx") / 1024
        compression = (1 - int8_size / fp32_size) * 100
        
        print(f"✓ 量化完成:")
        print(f"  FP32: {fp32_size:.2f} KB")
        print(f"  INT8: {int8_size:.2f} KB")
        print(f"  压缩率: {compression:.1f}%")
        
    except ImportError:
        print("⚠ 未安装 onnxruntime,跳过量化步骤")
        print("  安装命令: pip install onnxruntime")


def step4_test_on_hardware():
    """步骤 4: 在硬件上测试"""
    print("\n" + "=" * 60)
    print("步骤 4: 在 IMC-22 硬件上测试")
    print("=" * 60)
    
    try:
        # 创建硬件环境
        print("\n连接硬件中...")
        hw_env = HardwareEnvironment(num_joints=12, control_freq_hz=100)
        
        # 加载策略
        model = PPO.load("walker_policy")
        
        # 测试运行
        print("\n运行硬件测试 (10 步)...")
        obs = hw_env.reset()
        
        for step in range(10):
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = hw_env.step(action)
            
            print(f"  步骤 {step + 1}/10: 奖励 = {reward:.4f}")
            
            if terminated or truncated:
                obs = hw_env.reset()
        
        hw_env.close()
        print("✓ 硬件测试完成")
        
    except Exception as e:
        print(f"⚠ 硬件测试失败: {e}")
        print("  请检查:")
        print("  - CAN 适配器是否连接")
        print("  - IMC-22 节点是否上电")
        print("  - CAN 总线配置是否正确")


def main():
    """完整部署流程"""
    print("\n" + "=" * 60)
    print("AGI-Walker → IMC-22 部署演示")
    print("=" * 60)
    
    # 询问用户要执行哪些步骤
    print("\n选择执行步骤:")
    print("  1. 训练新模型（在仿真中）")
    print("  2. 导出现有模型到 ONNX")
    print("  3. 量化模型")
    print("  4. 在硬件上测试")
    print("  5. 执行全部步骤")
    
    choice = input("\n请选择 (1-5): ").strip()
    
    if choice == "1":
        model = step1_train_in_simulation()
    
    elif choice == "2":
        # 加载已有模型
        if os.path.exists("walker_policy.zip"):
            model = PPO.load("walker_policy")
            step2_export_to_onnx(model)
        else:
            print("✗ 未找到 walker_policy.zip，请先训练模型")
    
    elif choice == "3":
        step3_quantize_model()
    
    elif choice == "4":
        step4_test_on_hardware()
    
    elif choice == "5":
        # 全部执行
        model = step1_train_in_simulation()
        step2_export_to_onnx(model)
        step3_quantize_model()
        step4_test_on_hardware()
    
    else:
        print("✗ 无效选择")
        return
    
    print("\n" + "=" * 60)
    print("部署流程完成！")
    print("=" * 60)
    print("\n下一步:")
    print("  1. 使用 Hive-Reflex SDK 编译固件")
    print("  2. 烧录到 IMC-22:")
    print("     cd ../hive-reflex")
    print("     make APP_SRCS=examples/example_reflex_node.c")
    print("     make flash")
    print("  3. 详细说明请参考: HARDWARE_INTEGRATION_GUIDE.md")


if __name__ == "__main__":
    main()
