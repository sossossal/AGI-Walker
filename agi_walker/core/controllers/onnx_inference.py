import onnxruntime as ort
import numpy as np
import logging
from typing import Dict, Any, List, Optional

logger = logging.getLogger(__name__)

class ONNXInferenceEngine:
    """
    AGI-Walker V2.0 High-Performance ONNX Inference Engine.
    Optimized for low-latency robot control on edge devices.
    """
    def __init__(self, model_path: str, use_gpu: bool = False):
        self.model_path = model_path
        
        # 选择执行提供者 (Execution Providers)
        providers = ['CPUExecutionProvider']
        if use_gpu:
            providers = ['CUDAExecutionProvider'] + providers
        
        try:
            # 启用优化器
            sess_options = ort.SessionOptions()
            sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            sess_options.intra_op_num_threads = 4  # 边缘端 CPU 核心数
            
            self.session = ort.InferenceSession(
                model_path, 
                sess_options=sess_options, 
                providers=providers
            )
            
            # 自动获取输入输出节点名称
            self.input_name = self.session.get_inputs()[0].name
            self.output_name = self.session.get_outputs()[0].name
            
            logger.info(f"ONNX Session loaded from {model_path} with providers {providers}")
        except Exception as e:
            logger.error(f"Failed to load ONNX model: {e}")
            raise

    def predict(self, input_data: np.ndarray) -> np.ndarray:
        """
        执行单次同步推理。
        """
        try:
            # 确保输入数据维度正确 [Batch, Input_Dim]
            if len(input_data.shape) == 1:
                input_data = input_data.reshape(1, -1)
            
            # 推理执行
            outputs = self.session.run(
                [self.output_name], 
                {self.input_name: input_data.astype(np.float32)}
            )
            return outputs[0]
        except Exception as e:
            logger.error(f"Inference prediction error: {e}")
            return np.zeros((1, 12)) # 失败时返回默认静止扭矩

    def benchmark(self, iterations: int = 100):
        """性能基准测试"""
        import time
        dummy_input = np.random.randn(1, self.session.get_inputs()[0].shape[1]).astype(np.float32)
        
        start = time.time()
        for _ in range(iterations):
            _ = self.predict(dummy_input)
        end = time.time()
        
        avg_latency = (end - start) / iterations * 1000
        logger.info(f"Benchmark: {iterations} iterations, Avg Latency: {avg_latency:.2f} ms")
        return avg_latency
