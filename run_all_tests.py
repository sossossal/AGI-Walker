"""
全面功能测试脚本
依次运行各个模块的自测功能，验证系统完整性
"""

import subprocess
import sys
import time
from pathlib import Path

# 定义要测试的模块及其路径
MODULES_TO_TEST = [
    {
        "name": "Reward Designer (RL)",
        "path": "python_controller/reward_designer.py",
        "desc": "验证奖励函数配置与计算"
    },
    {
        "name": "Auto Labeler (Data)",
        "path": "training/auto_labeler.py",
        "desc": "验证轨迹自动标记功能"
    },
    {
        "name": "Robot Models (Infra)",
        "path": "robot_models/base_robot.py",
        "desc": "验证多机器人模型配置与工厂"
    },
    {
        "name": "Cloud Sim (Infra)",
        "path": "python_controller/cloud_sim.py",
        "desc": "验证云仿真接口与并行管理"
    },
    {
        "name": "Sim2Real Analyzer (Sim2Real)",
        "path": "python_controller/sim2real_analyzer.py",
        "desc": "验证功率差异与滞后分析"
    },
    {
        "name": "Physics Calibrator (Sim2Real)",
        "path": "python_controller/physics_calibrator.py",
        "desc": "验证参数校准逻辑"
    },
    {
        "name": "Terrain Mapper (Multimodal)",
        "path": "python_controller/terrain_mapper.py",
        "desc": "验证局部高程图构建"
    },
    {
        "name": "Vision Processor (Multimodal)",
        "path": "python_controller/vision_processor.py",
        "desc": "验证视觉编码器 (Mock/Real)"
    },
    {
        "name": "Evolution Logic (System)",
        "path": "python_controller/verify_mocked.py",
        "desc": "验证自动化进化循环逻辑流"
    }
]

def run_test(module):
    print(f"\n{'='*60}")
    print(f"🧪 Testing: {module['name']}")
    print(f"   Desc: {module['desc']}")
    print(f"{'='*60}")
    
    file_path = Path("d:/新建文件夹/AGI-Walker") / module['path']
    if not file_path.exists():
        print(f"❌ File not found: {file_path}")
        return False, "File Missing"
        
    start_time = time.time()
    try:
        # 强制使用 UTF-8 环境
        env = sys.modules['os'].environ.copy()
        env['PYTHONIOENCODING'] = 'utf-8'
        
        # 使用当前 python 解释器运行
        # 设置 cwd 为项目根目录，确保导入正确
        result = subprocess.run(
            [sys.executable, str(file_path)],
            cwd="d:/新建文件夹/AGI-Walker",
            capture_output=True,
            text=True,
            encoding='utf-8',
            errors='replace', # 防止编码错误
            env=env # 传递环境变量
        )
        
        duration = time.time() - start_time
        
        # 打印输出 (截断过长的输出)
        print("--- Output ---")
        lines = result.stdout.splitlines()
        # 打印前10行和后5行
        if len(lines) > 20:
             print("\n".join(lines[:10]))
             print(f"\n... (skipped {len(lines)-15} lines) ...\n")
             print("\n".join(lines[-5:]))
        else:
             print(result.stdout)
             
        if result.stderr:
            print("--- Stderr ---")
            print(result.stderr)
            
        if result.returncode == 0:
            print(f"✅ PASS ({duration:.2f}s)")
            return True, f"Pass ({duration:.2f}s)"
        else:
            print(f"❌ FAIL (Exit Code: {result.returncode})")
            return False, f"Fail (Code {result.returncode})"
            
    except Exception as e:
        print(f"❌ ERROR: {e}")
        return False, str(e)

def main():
    print("🚀 AGI-Walker 全面功能自检启动")
    print(f"Time: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    
    results = []
    
    for module in MODULES_TO_TEST:
        success, msg = run_test(module)
        results.append({
            "name": module['name'],
            "success": success,
            "msg": msg
        })
        
    print("\n" + "="*60)
    print("📊 测试结果汇总")
    print("="*60)
    
    all_passed = True
    for res in results:
        icon = "✅" if res['success'] else "❌"
        print(f"{icon} {res['name']:<30} | {res['msg']}")
        if not res['success']:
            all_passed = False
            
    print("-" * 60)
    if all_passed:
        print("🎉 所有模块功能验证通过！系统符合预期。")
    else:
        print("⚠️ 部分模块存在问题，请检查日志。")

if __name__ == "__main__":
    main()
