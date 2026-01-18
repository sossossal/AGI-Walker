"""
AI模型包装器
支持Ollama和llama.cpp两种推理引擎
"""

import json
import time
from typing import Dict, Optional, Literal
from abc import ABC, abstractmethod


class BaseAIModel(ABC):
    """AI模型基类"""
    
    @abstractmethod
    def predict(self, sensor_data: Dict, strategy: str = "") -> Dict:
        """预测动作"""
        pass
    
    @abstractmethod
    def get_stats(self) -> Dict:
        """获取统计信息"""
        pass


class OllamaModel(BaseAIModel):
    """Ollama AI模型"""
    
    def __init__(self, model_name: str = "phi3:mini"):
        try:
            import ollama
            self.ollama = ollama
        except ImportError:
            raise ImportError("请安装ollama: pip install ollama")
        
        self.model = model_name
        self.schema = self._create_schema()
        
        # 统计
        self.total_predictions = 0
        self.total_time = 0.0
        self.errors = 0
        
        # 验证模型可用
        self._verify_model()
    
    def _verify_model(self):
        """验证模型是否可用"""
        try:
            models = self.ollama.list()
            available = [m['name'] for m in models.get('models', [])]
            
            if not any(self.model in m for m in available):
                print(f"⚠️ 模型 {self.model} 未找到")
                print(f"可用模型: {available}")
                print(f"\n请运行: ollama pull {self.model}")
                raise ValueError(f"模型 {self.model} 不可用")
            
            print(f"✅ 模型 {self.model} 已加载")
            
        except Exception as e:
            print(f"❌ 无法连接到Ollama服务")
            print(f"请确保Ollama正在运行: https://ollama.com/")
            raise
    
    def _create_schema(self) -> Dict:
        """创建JSON Schema"""
        return {
            "type": "object",
            "properties": {
                "motors": {
                    "type": "object",
                    "properties": {
                        "hip_left": {
                            "type": "number",
                            "minimum": -45,
                            "maximum": 90,
                            "description": "左髋关节目标角度（度）"
                        },
                        "hip_right": {
                            "type": "number",
                            "minimum": -45,
                            "maximum": 90,
                            "description": "右髋关节目标角度（度）"
                        }
                    },
                    "required": ["hip_left", "hip_right"]
                },
                "confidence": {
                    "type": "number",
                    "minimum": 0,
                    "maximum": 1,
                    "description": "预测置信度"
                }
            },
            "required": ["motors"]
        }
    
    def predict(self, sensor_data: Dict, strategy: str = "") -> Dict:
        """
        AI推理预测动作
        
        Args:
            sensor_data: 传感器数据
            strategy: 策略提示
        
        Returns:
            动作字典
        """
        start_time = time.time()
        
        try:
            prompt = self._build_prompt(sensor_data, strategy)
            
            response = self.ollama.generate(
                model=self.model,
                prompt=prompt,
                format=self.schema,
                options={
                    'temperature': 0.1,    # 低温度保证稳定
                    'top_p': 0.9,
                    'num_predict': 100,    # 最大token数
                    'stop': ['\n\n']       # 停止符
                }
            )
            
            # 解析JSON
            action = json.loads(response['response'])
            
            # 更新统计
            self.total_predictions += 1
            self.total_time += time.time() - start_time
            
            return action
            
        except json.JSONDecodeError as e:
            print(f"❌ JSON解析错误: {e}")
            print(f"   原始响应: {response.get('response', 'N/A')[:200]}")
            self.errors += 1
            return self._default_action()
            
        except Exception as e:
            print(f"❌ AI推理错误: {e}")
            self.errors += 1
            return self._default_action()
    
    def _build_prompt(self, sensor_data: Dict, strategy: str) -> str:
        """构建Prompt"""
        orient = sensor_data['sensors']['imu']['orient']
        joints = sensor_data['sensors']['joints']
        height = sensor_data.get('torso_height', 0)
        
        return f"""你是一个双足机器人的运动控制AI。根据传感器数据，输出电机控制指令以维持平衡和稳定站立。

## 当前策略
{strategy if strategy else "保持躯干直立，站立稳定"}

## 传感器数据
- Roll（左右倾斜）: {orient[0]:.2f}度
- Pitch（前后倾斜）: {orient[1]:.2f}度  
- Yaw（旋转）: {orient[2]:.2f}度
- 左髋关节当前角度: {joints['hip_left']['angle']:.2f}度
- 右髋关节当前角度: {joints['hip_right']['angle']:.2f}度
- 躯干高度: {height:.2f}米

## 控制目标
1. 保持躯干直立（Roll和Pitch尽可能接近0度）
2. 维持稳定站立姿态
3. 躯干高度保持在0.8米以上

## 输出要求
输出JSON格式的电机目标角度。左髋和右髋的角度范围: -45度到90度。
如果机器人向左倾斜（Roll>0），应该调整左右髋关节产生反向力矩。
如果机器人向前倾斜（Pitch<0），两个髋关节应协同向后调整。

请输出你的控制指令:"""
    
    def _default_action(self) -> Dict:
        """默认安全动作"""
        return {
            "motors": {
                "hip_left": 0.0,
                "hip_right": 0.0
            },
            "confidence": 0.0
        }
    
    def get_stats(self) -> Dict:
        """获取统计信息"""
        avg_time = self.total_time / self.total_predictions if self.total_predictions > 0 else 0
        
        return {
            "total_predictions": self.total_predictions,
            "avg_inference_time": avg_time,
            "errors": self.errors,
            "error_rate": self.errors / self.total_predictions if self.total_predictions > 0 else 0
        }


class LlamaCppModel(BaseAIModel):
    """llama.cpp AI模型（高级）"""
    
    def __init__(self, model_path: str, grammar_path: Optional[str] = None):
        try:
            from llama_cpp import Llama
            self.Llama = Llama
        except ImportError:
            raise ImportError("请安装llama-cpp-python: pip install llama-cpp-python")
        
        print(f"加载模型: {model_path}")
        self.model = self.Llama(
            model_path=model_path,
            n_ctx=2048,
            n_threads=4,
            n_gpu_layers=0,  # 设置>0启用GPU
            verbose=False
        )
        
        self.grammar_path = grammar_path
        self.total_predictions = 0
        self.total_time = 0.0
        self.errors = 0
        
        print("✅ 模型加载成功")
    
    def predict(self, sensor_data: Dict, strategy: str = "") -> Dict:
        """AI推理"""
        start_time = time.time()
        
        try:
            prompt = self._build_prompt(sensor_data, strategy)
            
            # 生成参数
            gen_params = {
                'max_tokens': 100,
                'temperature': 0.1,
                'top_p': 0.9,
                'stop': ['\n\n'],
                'echo': False
            }
            
            # 如果有语法文件，使用语法约束
            if self.grammar_path:
                gen_params['grammar_path'] = self.grammar_path
            
            output = self.model(prompt, **gen_params)
            response_text = output['choices'][0]['text']
            
            # 解析JSON
            action = json.loads(response_text)
            
            self.total_predictions += 1
            self.total_time += time.time() - start_time
            
            return action
            
        except Exception as e:
            print(f"❌ 推理错误: {e}")
            self.errors += 1
            return {"motors": {"hip_left": 0.0, "hip_right": 0.0}}
    
    def _build_prompt(self, sensor_data: Dict, strategy: str) -> str:
        """构建Prompt（复用OllamaModel的实现）"""
        orient = sensor_data['sensors']['imu']['orient']
        joints = sensor_data['sensors']['joints']
        
        return f"""Robot control AI. Output motor angles in JSON format.

Current state:
- Roll: {orient[0]:.2f}°
- Pitch: {orient[1]:.2f}°  
- Hip Left: {joints['hip_left']['angle']:.2f}°
- Hip Right: {joints['hip_right']['angle']:.2f}°

Strategy: {strategy}

Output JSON:"""
    
    def get_stats(self) -> Dict:
        """统计信息"""
        avg_time = self.total_time / self.total_predictions if self.total_predictions > 0 else 0
        return {
            "total_predictions": self.total_predictions,
            "avg_inference_time": avg_time,
            "errors": self.errors
        }


def create_ai_model(
    engine: Literal["ollama", "llamacpp"] = "ollama",
    **kwargs
) -> BaseAIModel:
    """
    工厂函数：创建AI模型
    
    Args:
        engine: 引擎类型（"ollama" 或 "llamacpp"）
        **kwargs: 引擎特定参数
    
    Returns:
        AI模型实例
    """
    if engine == "ollama":
        model_name = kwargs.get("model_name", "phi3:mini")
        return OllamaModel(model_name=model_name)
    
    elif engine == "llamacpp":
        model_path = kwargs.get("model_path")
        if not model_path:
            raise ValueError("llamacpp需要model_path参数")
        
        grammar_path = kwargs.get("grammar_path")
        return LlamaCppModel(model_path=model_path, grammar_path=grammar_path)
    
    else:
        raise ValueError(f"未知引擎: {engine}")


# 测试代码
if __name__ == "__main__":
    print("AI模型测试\n")
    
    # 创建模型
    ai = create_ai_model(engine="ollama", model_name="phi3:mini")
    
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
    
    # 测试推理
    print("执行推理...")
    action = ai.predict(dummy_sensor, "保持平衡")
    
    print(f"\n预测动作:")
    print(json.dumps(action, indent=2, ensure_ascii=False))
    
    # 统计
    stats = ai.get_stats()
    print(f"\n统计信息:")
    print(f"  推理次数: {stats['total_predictions']}")
    print(f"  平均耗时: {stats['avg_inference_time']*1000:.2f}ms")
