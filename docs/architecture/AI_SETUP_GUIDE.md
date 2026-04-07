# AGI-Walker AI模型集成快速指�?

本指南帮助您快速完成AI模型的安装和配置�?

---

## 🚀 快速开始（5分钟�?

### 步骤1: 安装Ollama

**Windows**:
```powershell
winget install Ollama.Ollama
```

**或下载安装器**: https://ollama.com/download

### 步骤2: 下载AI模型

```bash
ollama pull phi3:mini
```

等待下载完成（约2.3GB�?

### 步骤3: 安装Python依赖

```bash
cd python_controller
pip install ollama
```

### 步骤4: 测试AI模型

```bash
python ai_model.py
```

应该看到:
```
�?模型 phi3:mini 已加�?
执行推理...

预测动作:
{
  "motors": {
    "hip_left": -2.3,
    "hip_right": 1.8
  },
  "confidence": 0.92
}
```

### 步骤5: 运行AI控制�?

1. **启动Godot仿真**（按F5�?
2. **运行AI控制�?*:
   ```bash
   python ai_controller.py --duration 60
   ```

---

## 📋 详细安装步骤

### 方案A: Ollama（推荐）

#### 1. 安装Ollama

**Windows PowerShell**:
```powershell
# 方法1: 使用winget
winget install Ollama.Ollama

# 方法2: 使用Chocolatey
choco install ollama

# 方法3: 手动下载
# 访问 https://ollama.com/download
```

安装后，Ollama会自动启动为后台服务�?

#### 2. 验证安装

```bash
ollama --version
```

应该显示版本号，�? `ollama version 0.1.x`

#### 3. 下载模型

**Phi-3-mini**（推荐）:
```bash
ollama pull phi3:mini
```

**其他选项**:
```bash
ollama pull gemma:2b      # 更小更快
ollama pull qwen2:3b      # 中文优化
```

#### 4. 测试模型

```bash
ollama run phi3:mini
```

输入: `你好，介绍一下你自己`

如果模型正常响应，说明安装成功！

#### 5. 安装Python客户�?

```bash
pip install ollama
```

---

### 方案B: llama.cpp（高级用户）

#### 1. 编译llama.cpp

**Windows (使用CMake)**:
```powershell
git clone https://github.com/ggerganov/llama.cpp
cd llama.cpp

# 编译
cmake -B build
cmake --build build --config Release
```

**启用GPU加�?* (如果有NVIDIA GPU):
```powershell
cmake -B build -DLLAMA_CUDA=ON
cmake --build build --config Release
```

#### 2. 下载模型

从HuggingFace下载GGUF格式:

```bash
# 使用huggingface-cli
pip install huggingface-hub

huggingface-cli download \
  microsoft/Phi-3-mini-4k-instruct-gguf \
  Phi-3-mini-4k-instruct-q4.gguf \
  --local-dir ./models
```

**或手动下�?*:
https://huggingface.co/microsoft/Phi-3-mini-4k-instruct-gguf

#### 3. 测试模型

```bash
.\build\bin\Release\main.exe \
  -m weights/Phi-3-mini-4k-instruct-q4.gguf \
  -p "Hello, introduce yourself" \
  -n 50
```

#### 4. 安装Python绑定

```bash
pip install llama-cpp-python
```

**GPU版本**:
```bash
CMAKE_ARGS="-DLLAMA_CUDA=ON" pip install llama-cpp-python
```

---

## 🧪 测试与验�?

### 测试1: 模型响应速度

```bash
python -c "
from ai_model import create_ai_model
import time

ai = create_ai_model()
dummy = {
    'sensors': {
        'imu': {'orient': [5, -3, 0]},
        'joints': {
            'hip_left': {'angle': 10},
            'hip_right': {'angle': -8}
        }
    },
    'torso_height': 1.45
}

# 测试10�?
times = []
for _ in range(10):
    t0 = time.time()
    ai.predict(dummy)
    times.append(time.time() - t0)

print(f'平均: {sum(times)/len(times)*1000:.1f}ms')
"
```

**目标**: < 100ms

### 测试2: JSON格式

```bash
python ai_model.py
```

检查输出是否为有效JSON�?

### 测试3: 集成控制

```bash
# 1. 启动Godot (另一个终�?
# 2. 运行AI控制�?
python ai_controller.py --duration 30
```

**验收标准**:
- �?无连接错�?
- �?控制频率 > 20Hz
- �?无JSON解析错误
- �?机器人保持站�?

---

## 🛠�?故障排除

### 问题1: Ollama服务未启�?

**症状**:
```
�?无法连接到Ollama服务
```

**解决**:
```bash
# Windows: 在开始菜单搜�?Ollama"并启�?

# 或命令行启动
ollama serve
```

### 问题2: 模型下载失败

**解决**:
```bash
# 使用国内镜像（如果需要）
export OLLAMA_HOST=https://ollama.mirror.cn

# 或手动下载后导入
ollama create phi3:mini -f Modelfile
```

### 问题3: 推理太慢

**症状**: 推理 > 200ms

**解决方案**:
1. 使用GPU加�?
2. 使用更小的模型（gemma:2b�?
3. 减少`num_predict`参数
4. 升级硬件

### 问题4: JSON格式错误

**症状**: 
```
�?JSON解析错误
```

**临时解决**:
AI模型会返回默认安全动作（角度0�?

**长期解决**:
1. 使用Grammar约束（llama.cpp�?
2. 优化Prompt
3. 增加后处理修�?

---

## ⚙️ 配置优化

### 降低延迟

�?`ai_model.py` 中调�?
```python
options={
    'temperature': 0.05,  # 降低温度
    'num_predict': 30,    # 减少生成token�?
    'top_p': 0.85
}
```

### 提高稳定�?

```python
options={
    'temperature': 0.01,  # 极低温度
    'top_k': 10,          # 限制采样范围
    'repeat_penalty': 1.1
}
```

---

## 📊 性能基准

| 硬件配置 | 模型 | 推理速度 | 控制频率 |
|---------|------|---------|---------|
| i7-12700 | Phi-3 Q4 | 40-60ms | 25-30Hz �?|
| i5-10400 | Phi-3 Q4 | 80-120ms | 12-18Hz ⚠️ |
| RTX 4060 | Phi-3 Q4 | 15-25ms | 40-50Hz 🔥 |

---

## 🎓 下一�?

完成安装�?

1. **调优Prompt** - 改进 `_build_prompt()` 方法
2. **收集数据** - 记录成功的控制轨�?
3. **集成PID** - 结合PID控制�?
4. **添加70B优化�?* - 实现策略优化

---

> 💡 **提示**: 第一次运行可能需要一些时间让模型"热身"，后续推理会更快�?
