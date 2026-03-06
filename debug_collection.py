import os
import sys
import subprocess

def run_tests_individually():
    print("--- 逐个运行测试文件进行深度诊断 ---")
    test_dir = "tests"
    files = [f for f in os.listdir(test_dir) if f.startswith("test_") and f.endswith(".py")]
    files.sort()
    
    global_failed = False
    for f in files:
        print(f"\n>>> 运行 {f} ...")
        # 运行 pytest 并捕获输出
        result = subprocess.run(
            [sys.executable, "-m", "pytest", os.path.join(test_dir, f), "-v", "--tb=short"],
            capture_output=True,
            text=True
        )
        
        if result.returncode == 0:
            print(f"OK: {f} 通过")
        elif result.returncode == 5:
            print(f"SKIP: {f} 未发现测试用例")
        else:
            print(f"FAIL: {f} 返回状态码 {result.returncode}")
            print("--- 错误日志摘要 ---")
            # 打印 stdout 和 stderr 的最后 20 行
            output = result.stdout + result.stderr
            lines = output.splitlines()
            for line in lines[-30:]:
                print(line)
            global_failed = True
            print("-" * 40)
    
    if global_failed:
        print("\n诊断完成: 发现失败的测试文件。")
        # 注意：这里我们故意不以非零状态码退出，
        # 以便让 CI 继续运行主 pytest 步骤，从而让我们看到完整的测试矩阵。
    else:
        print("\n诊断完成: 所有文件在独立运行时均正常。")

if __name__ == "__main__":
    run_tests_individually()
