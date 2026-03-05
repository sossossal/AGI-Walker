import os
import sys
import traceback
import importlib.util

def debug_imports():
    print("--- 调试测试文件导入 ---")
    # 显式设置路径
    root_dir = os.getcwd()
    if root_dir not in sys.path:
        sys.path.insert(0, root_dir)
    
    test_dir = "tests"
    if not os.path.exists(test_dir):
        print(f"错误: 找不到目录 {test_dir}")
        return

    files = [f for f in os.listdir(test_dir) if f.startswith("test_") and f.endswith(".py")]
    files.sort()
    
    failed = False
    for f in files:
        module_name = f"tests.{f[:-3]}"
        file_path = os.path.join(test_dir, f)
        print(f"尝试导入 {f}...", end=" ", flush=True)
        try:
            # 使用 importlib 动态加载
            spec = importlib.util.spec_from_file_location(module_name, file_path)
            module = importlib.util.module_from_spec(spec)
            # 模拟 pytest 的行为，将模块加入 sys.modules
            sys.modules[module_name] = module
            spec.loader.exec_module(module)
            print("OK")
        except Exception:
            print("FAILED!")
            traceback.print_exc()
            failed = True
            print("-" * 40)
    
    if failed:
        print("
结论: 部分测试文件在导入阶段崩溃，这正是 Exit Code 2 的原因。")
    else:
        print("
结论: 所有测试文件导入正常，请检查 pytest 插件冲突。")

if __name__ == "__main__":
    debug_imports()
