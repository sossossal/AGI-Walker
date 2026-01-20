"""
模型编排器（Model Orchestrator）
协调三层AI模型：小模型（3B）、中模型（7B）、大模型（70B）
实现动态模型选择和智能路由
"""

import time
from typing import Dict, Optional, Literal, Callable
from dataclasses import dataclass
from enum import Enum
from ai_model import BaseAIModel, OllamaModel, create_ai_model
from medium_model import MediumModel


class ModelTier(Enum):
    """模型层级"""
    SMALL = "small"      # 3B - 实时控制
    MEDIUM = "medium"    # 7B - 半实时任务
    LARGE = "large"      # 70B - 离线优化


@dataclass
class ModelConfig:
    """模型配置"""
    small_model: str = "phi3:mini"      # 3B小模型
    medium_model: str = "mistral:7b"    # 7B中模型
    large_model: str = "llama2:70b"     # 70B大模型
    
    # 延迟阈值（ms）
    small_latency_threshold: float = 50.0
    medium_latency_threshold: float = 500.0
    
    # 调用频率限制
    medium_min_interval: float = 0.5    # 中模型最小调用间隔（秒）
    large_min_interval: float = 60.0    # 大模型最小调用间隔（秒）


class ModelOrchestrator:
    """
    三层模型编排器
    
    负责：
    - 模型选择和路由
    - 延迟监控和降级
    - 日志收集和上报
    - 策略同步
    """
    
    def __init__(self, config: Optional[ModelConfig] = None):
        self.config = config or ModelConfig()
        
        # 模型实例（延迟加载）
        self._small_model: Optional[BaseAIModel] = None
        self._medium_model: Optional[MediumModel] = None
        self._large_model: Optional[BaseAIModel] = None
        
        # 状态跟踪
        self.last_medium_call = 0.0
        self.last_large_call = 0.0
        self.current_tier = ModelTier.SMALL
        
        # 统计
        self.tier_usage = {
            ModelTier.SMALL: 0,
            ModelTier.MEDIUM: 0,
            ModelTier.LARGE: 0
        }
        self.total_requests = 0
        self.fallback_count = 0
        
        # 回调函数
        self.on_escalate: Optional[Callable] = None
        self.on_tier_change: Optional[Callable] = None
    
    # =================== 模型访问器（延迟加载） ===================
    
    @property
    def small_model(self) -> BaseAIModel:
        """获取小模型（按需加载）"""
        if self._small_model is None:
            print("正在加载小模型...")
            self._small_model = create_ai_model(
                engine="ollama",
                model_name=self.config.small_model
            )
        return self._small_model
    
    @property
    def medium_model(self) -> MediumModel:
        """获取中模型（按需加载）"""
        if self._medium_model is None:
            print("正在加载中模型...")
            self._medium_model = MediumModel(
                model_name=self.config.medium_model
            )
        return self._medium_model
    
    @property
    def large_model(self) -> Optional[BaseAIModel]:
        """获取大模型（按需加载，可能为None）"""
        if self._large_model is None:
            try:
                print("正在加载大模型...")
                self._large_model = create_ai_model(
                    engine="ollama",
                    model_name=self.config.large_model
                )
            except Exception as e:
                print(f"⚠️ 大模型加载失败: {e}")
                print("大模型功能将被禁用")
                return None
        return self._large_model
    
    # =================== 核心处理接口 ===================
    
    def process(self, sensor_data: dict, context: str = "realtime") -> dict:
        """
        根据上下文选择合适的模型处理请求
        
        Args:
            sensor_data: 传感器数据
            context: 上下文类型 ("realtime", "adjustment", "optimization")
        
        Returns:
            处理结果
        """
        self.total_requests += 1
        
        if context == "realtime":
            return self._process_realtime(sensor_data)
        elif context == "adjustment":
            return self._process_adjustment(sensor_data)
        elif context == "optimization":
            return self._process_optimization(sensor_data)
        else:
            print(f"⚠️ 未知上下文: {context}，使用实时处理")
            return self._process_realtime(sensor_data)
    
    def _process_realtime(self, sensor_data: dict) -> dict:
        """实时控制处理（小模型）"""
        start_time = time.time()
        
        try:
            result = self.small_model.predict(sensor_data)
            latency = (time.time() - start_time) * 1000
            
            # 记录延迟用于监控
            result['_latency_ms'] = latency
            result['_tier'] = ModelTier.SMALL.value
            
            self.tier_usage[ModelTier.SMALL] += 1
            
            # 检查是否需要调整
            if self._should_trigger_adjustment(sensor_data, latency):
                self._schedule_adjustment(sensor_data)
            
            return result
            
        except Exception as e:
            print(f"❌ 小模型处理失败: {e}")
            self.fallback_count += 1
            return self._get_fallback_action()
    
    def _process_adjustment(self, sensor_data: dict) -> dict:
        """环境感知调整（中模型）"""
        current_time = time.time()
        
        # 检查调用间隔
        if current_time - self.last_medium_call < self.config.medium_min_interval:
            return {"skip": True, "reason": "调用间隔过短"}
        
        start_time = time.time()
        
        try:
            result = self.medium_model.adjust_environment(sensor_data)
            latency = (time.time() - start_time) * 1000
            
            result['_latency_ms'] = latency
            result['_tier'] = ModelTier.MEDIUM.value
            
            self.last_medium_call = current_time
            self.tier_usage[ModelTier.MEDIUM] += 1
            
            # 检查是否需要上报大模型
            if result.get('escalate', False):
                self._schedule_optimization(sensor_data, result)
            
            return result
            
        except Exception as e:
            print(f"❌ 中模型处理失败: {e}")
            return {"error": str(e), "skip": True}
    
    def _process_optimization(self, sensor_data: dict) -> dict:
        """离线策略优化（大模型）"""
        current_time = time.time()
        
        # 检查调用间隔
        if current_time - self.last_large_call < self.config.large_min_interval:
            return {"skip": True, "reason": "调用间隔过短"}
        
        if self.large_model is None:
            return {"skip": True, "reason": "大模型未加载"}
        
        start_time = time.time()
        
        try:
            # 收集日志
            logs = self.medium_model.get_logs_for_large_model()
            
            # 构建优化请求
            optimization_data = {
                **sensor_data,
                "logs": logs,
                "environment_state": self.medium_model.environment_state
            }
            
            result = self.large_model.predict(
                optimization_data,
                strategy="深度策略优化模式"
            )
            
            latency = (time.time() - start_time) * 1000
            
            result['_latency_ms'] = latency
            result['_tier'] = ModelTier.LARGE.value
            
            self.last_large_call = current_time
            self.tier_usage[ModelTier.LARGE] += 1
            
            # 触发回调
            if self.on_escalate:
                self.on_escalate(result)
            
            return result
            
        except Exception as e:
            print(f"❌ 大模型处理失败: {e}")
            return {"error": str(e), "skip": True}
    
    # =================== 辅助方法 ===================
    
    def _should_trigger_adjustment(self, sensor_data: dict, latency: float) -> bool:
        """判断是否需要触发环境调整"""
        # 延迟超标
        if latency > self.config.small_latency_threshold:
            return True
        
        # 姿态不稳定
        orient = sensor_data['sensors']['imu']['orient']
        if abs(orient[0]) > 15 or abs(orient[1]) > 15:
            return True
        
        # 高度异常
        height = sensor_data.get('torso_height', 1.0)
        if height < 0.5:
            return True
        
        return False
    
    def _schedule_adjustment(self, sensor_data: dict):
        """调度环境调整（异步）"""
        # 简单实现：直接调用
        # 完整实现应使用线程池或异步队列
        self._process_adjustment(sensor_data)
    
    def _schedule_optimization(self, sensor_data: dict, adjustment_result: dict):
        """调度离线优化（异步）"""
        print("📤 调度大模型优化...")
        # 完整实现应使用后台线程
        self._process_optimization(sensor_data)
    
    def _get_fallback_action(self) -> dict:
        """获取fallback动作"""
        return {
            "motors": {"hip_left": 0.0, "hip_right": 0.0},
            "confidence": 0.0,
            "_tier": "fallback"
        }
    
    # =================== 日志管理 ===================
    
    def add_log(self, log_entry: dict):
        """添加日志"""
        self.medium_model.add_log(log_entry)
    
    def filter_logs(self, logs: list) -> list:
        """过滤日志"""
        return self.medium_model.filter_logs(logs)
    
    # =================== 统计和监控 ===================
    
    def get_stats(self) -> dict:
        """获取统计信息"""
        return {
            "total_requests": self.total_requests,
            "tier_usage": {k.value: v for k, v in self.tier_usage.items()},
            "fallback_count": self.fallback_count,
            "current_tier": self.current_tier.value,
            "small_model_stats": self.small_model.get_stats() if self._small_model else None,
            "medium_model_stats": self.medium_model.get_stats() if self._medium_model else None,
        }
    
    def reset_stats(self):
        """重置统计"""
        self.tier_usage = {tier: 0 for tier in ModelTier}
        self.total_requests = 0
        self.fallback_count = 0


def create_orchestrator(
    small_model: str = "phi3:mini",
    medium_model: str = "mistral:7b",
    large_model: str = "llama2:70b"
) -> ModelOrchestrator:
    """
    工厂函数：创建模型编排器
    
    Args:
        small_model: 小模型名称
        medium_model: 中模型名称
        large_model: 大模型名称
    
    Returns:
        ModelOrchestrator实例
    """
    config = ModelConfig(
        small_model=small_model,
        medium_model=medium_model,
        large_model=large_model
    )
    return ModelOrchestrator(config)


# 测试代码
if __name__ == "__main__":
    import json
    
    print("模型编排器测试\n")
    
    # 创建编排器
    orchestrator = create_orchestrator(
        small_model="phi3:mini",
        medium_model="mistral:7b"
    )
    
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
    
    # 测试实时处理
    print("1. 测试实时处理（小模型）...")
    result = orchestrator.process(dummy_sensor, context="realtime")
    print(f"结果: {json.dumps(result, indent=2, ensure_ascii=False)}")
    
    # 测试环境调整
    print("\n2. 测试环境调整（中模型）...")
    result = orchestrator.process(dummy_sensor, context="adjustment")
    print(f"结果: {json.dumps(result, indent=2, ensure_ascii=False)}")
    
    # 统计
    print("\n3. 统计信息:")
    stats = orchestrator.get_stats()
    print(json.dumps(stats, indent=2, ensure_ascii=False, default=str))
