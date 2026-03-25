"""
ONNX推理引擎
高性能推理，支持CPU和GPU，最大兼容性
"""

import time
import numpy as np
from typing import Dict, List, Optional, Tuple, Union
from pathlib import Path
from dataclasses import dataclass
from collections import deque
import logging

logger = logging.getLogger(__name__)


@dataclass
class ONNXConfig:
    """ONNX推理配置"""

    use_gpu: bool = False  # 默认使用CPU以保证最大兼容性
    num_threads: int = 4
    enable_profiling: bool = False
    graph_optimization_level: str = "all"  # "basic", "extended", "all"


class ONNXInferenceEngine:
    """
    高性能ONNX推理引擎

    特点：
    - 自动检测可用Provider
    - 最大兼容性（优先使用CPU）
    - 延迟统计和性能监控
    """

    def __init__(
        self, model_path: Optional[str] = None, config: Optional[ONNXConfig] = None
    ):
        self.config = config or ONNXConfig()
        self.model_path = model_path
        self.session = None
        self.input_names: List[str] = []
        self.output_names: List[str] = []
        self.input_shapes: Dict[str, Tuple] = {}

        # 延迟初始化ONNX Runtime
        self._ort = None
        self._init_onnx_runtime()

        # 性能统计
        self.latency_history: deque = deque(maxlen=1000)
        self.total_inferences = 0
        self.total_time = 0.0

        # 加载模型
        if model_path and Path(model_path).exists():
            self.load_model(model_path)

    def _init_onnx_runtime(self) -> None:
        """初始化ONNX Runtime"""
        try:
            import onnxruntime as ort

            self._ort = ort

            # 检测可用Providers
            available_providers = ort.get_available_providers()
            logger.info(f"📦 ONNX Runtime版本: {ort.__version__}")
            logger.info(f"可用Providers: {available_providers}")

        except ImportError:
            logger.info(" ONNX Runtime未安装")
            logger.info("请运行: pip install onnxruntime")
            self._ort = None

    def load_model(self, model_path: str) -> bool:
        """加载ONNX模型"""
        if self._ort is None:
            logger.info(" ONNX Runtime不可用")
            return False

        path = Path(model_path)
        if not path.exists():
            logger.info(f" 模型文件不存在: {model_path}")
            return False

        logger.info(f"加载模型: {model_path}")

        # 配置Session选项
        sess_options = self._ort.SessionOptions()
        sess_options.intra_op_num_threads = self.config.num_threads

        # 图优化级别
        opt_levels = {
            "basic": self._ort.GraphOptimizationLevel.ORT_ENABLE_BASIC,
            "extended": self._ort.GraphOptimizationLevel.ORT_ENABLE_EXTENDED,
            "all": self._ort.GraphOptimizationLevel.ORT_ENABLE_ALL,
        }
        sess_options.graph_optimization_level = opt_levels.get(
            self.config.graph_optimization_level,
            self._ort.GraphOptimizationLevel.ORT_ENABLE_ALL,
        )

        if self.config.enable_profiling:
            sess_options.enable_profiling = True

        # 选择Provider（最大兼容性）
        providers = self._get_providers()

        try:
            self.session = self._ort.InferenceSession(
                str(path), sess_options, providers=providers
            )

            # 获取输入输出信息
            self.input_names = [inp.name for inp in self.session.get_inputs()]
            self.output_names = [out.name for out in self.session.get_outputs()]

            for inp in self.session.get_inputs():
                self.input_shapes[inp.name] = inp.shape

            logger.info("✅ 模型加载成功")
            logger.info(f"输入: {self.input_names}")
            logger.info(f"输出: {self.output_names}")

            return True

        except Exception as e:
            logger.info(f" 模型加载失败: {e}")
            return False

    def _get_providers(self) -> List[str]:
        """获取可用的Providers（最大兼容性）"""
        if self._ort is None:
            return []

        available = self._ort.get_available_providers()

        # 按优先级排序（CPU优先以保证兼容性）
        priority = [
            "CPUExecutionProvider",
            "CUDAExecutionProvider",
            "TensorrtExecutionProvider",
            "DmlExecutionProvider",  # Windows DirectML
            "CoreMLExecutionProvider",  # macOS
        ]

        if self.config.use_gpu:
            # GPU优先
            priority = [
                "CUDAExecutionProvider",
                "TensorrtExecutionProvider",
                "DmlExecutionProvider",
                "CPUExecutionProvider",
            ]

        providers = []
        for p in priority:
            if p in available:
                providers.append(p)

        if not providers:
            providers = ["CPUExecutionProvider"]

        logger.info(f"使用Providers: {providers}")
        return providers

    def predict(
        self, inputs: Union[np.ndarray, Dict[str, np.ndarray]]
    ) -> Union[np.ndarray, Dict[str, np.ndarray]]:
        """
        执行推理

        Args:
            inputs: 输入数据，可以是单个数组或字典

        Returns:
            推理结果
        """
        if self.session is None:
            raise RuntimeError("模型未加载")

        start_time = time.time()

        # 准备输入
        if isinstance(inputs, np.ndarray):
            # 单输入
            feed_dict = {self.input_names[0]: inputs.astype(np.float32)}
        else:
            # 多输入
            feed_dict = {name: data.astype(np.float32) for name, data in inputs.items()}

        # 执行推理
        outputs = self.session.run(self.output_names, feed_dict)

        # 统计
        latency = time.time() - start_time
        self.latency_history.append(latency)
        self.total_inferences += 1
        self.total_time += latency

        # 返回结果
        if len(outputs) == 1:
            return outputs[0]
        else:
            return {name: out for name, out in zip(self.output_names, outputs)}

    def predict_batch(self, inputs: np.ndarray, batch_size: int = 32) -> np.ndarray:
        """批量推理"""
        num_samples = inputs.shape[0]
        results = []

        for i in range(0, num_samples, batch_size):
            batch = inputs[i : i + batch_size]
            result = self.predict(batch)
            results.append(result)

        return np.concatenate(results, axis=0)

    def warmup(self, num_runs: int = 10) -> None:
        """模型预热"""
        if self.session is None:
            return

        logger.info(f"模型预热 ({num_runs}次)...")

        # 创建dummy输入
        dummy_inputs = {}
        for name, shape in self.input_shapes.items():
            # 替换动态维度为1
            static_shape = [s if isinstance(s, int) and s > 0 else 1 for s in shape]
            dummy_inputs[name] = np.random.randn(*static_shape).astype(np.float32)

        for _ in range(num_runs):
            self.predict(dummy_inputs)

        # 清除预热统计
        self.latency_history.clear()
        self.total_inferences = 0
        self.total_time = 0.0

        logger.info("✅ 预热完成")

    def get_latency_stats(self) -> dict:
        """获取延迟统计"""
        if not self.latency_history:
            return {
                "avg_ms": 0,
                "min_ms": 0,
                "max_ms": 0,
                "p50_ms": 0,
                "p95_ms": 0,
                "p99_ms": 0,
                "total_inferences": 0,
            }

        latencies = list(self.latency_history)
        latencies.sort()

        return {
            "avg_ms": sum(latencies) / len(latencies) * 1000,
            "min_ms": min(latencies) * 1000,
            "max_ms": max(latencies) * 1000,
            "p50_ms": latencies[len(latencies) // 2] * 1000,
            "p95_ms": latencies[int(len(latencies) * 0.95)] * 1000,
            "p99_ms": latencies[int(len(latencies) * 0.99)] * 1000,
            "total_inferences": self.total_inferences,
            "throughput_fps": (
                self.total_inferences / self.total_time if self.total_time > 0 else 0
            ),
        }

    def get_model_info(self) -> dict:
        """获取模型信息"""
        if self.session is None:
            return {"loaded": False}

        return {
            "loaded": True,
            "model_path": self.model_path,
            "inputs": {
                name: {"shape": self.input_shapes.get(name)}
                for name in self.input_names
            },
            "outputs": self.output_names,
            "providers": self.session.get_providers(),
        }


class DummyONNXEngine:
    """虚拟ONNX引擎（当ONNX Runtime不可用时使用）"""

    def __init__(self, *args, **kwargs) -> None:
        logger.info(" 使用虚拟ONNX引擎")
        self.total_inferences = 0

    def load_model(self, model_path: str) -> bool:
        return False

    def predict(self, inputs: np.ndarray) -> np.ndarray:
        self.total_inferences += 1
        # 返回零输出
        return np.zeros((inputs.shape[0], 1), dtype=np.float32)

    def get_latency_stats(self) -> dict:
        return {"dummy": True, "total_inferences": self.total_inferences}

    def get_model_info(self) -> dict:
        return {"loaded": False, "dummy": True}


def create_onnx_engine(
    model_path: Optional[str] = None, use_gpu: bool = False
) -> ONNXInferenceEngine:
    """
    工厂函数：创建ONNX推理引擎

    Args:
        model_path: 模型路径
        use_gpu: 是否使用GPU

    Returns:
        ONNX推理引擎实例
    """
    try:
        import onnxruntime

        config = ONNXConfig(use_gpu=use_gpu)
        return ONNXInferenceEngine(model_path, config)
    except ImportError:
        return DummyONNXEngine()


# 测试代码
if __name__ == "__main__":
    import json

    logger.info("ONNX推理引擎测试\n")

    # 创建引擎
    engine = create_onnx_engine(use_gpu=False)

    logger.info("\n=== 模型信息 ===")
    logger.info(json.dumps(engine.get_model_info(), indent=2))

    # 如果有现有ONNX模型，测试推理
    project_root = Path(__file__).resolve().parent.parent
    reflex_model = project_root / "hive-reflex" / "reflex_net.onnx"

    if reflex_model.exists():
        logger.info("\n=== 测试ReflexNet模型 ===")

        if engine.load_model(str(reflex_model)):
            # 预热
            engine.warmup(10)

            # 测试推理
            dummy_input = np.random.randn(1, 5, 12).astype(np.float32)
            h0 = np.zeros((1, 1, 16), dtype=np.float32)
            c0 = np.zeros((1, 1, 16), dtype=np.float32)

            for _ in range(100):
                output = engine.predict({"input": dummy_input, "h_in": h0, "c_in": c0})

            logger.info("\n延迟统计:")
            logger.info(json.dumps(engine.get_latency_stats(), indent=2))
    else:
        logger.info("\n ReflexNet ONNX模型不存在")
        logger.info("请运行: cd hive-reflex && python reflex_net.py")

        # 测试虚拟推理
        if isinstance(engine, DummyONNXEngine):
            logger.info("\n使用虚拟引擎测试...")
            dummy_input = np.random.randn(1, 12).astype(np.float32)
            output = engine.predict(dummy_input)
            logger.info(f"输出形状: {output.shape}")
