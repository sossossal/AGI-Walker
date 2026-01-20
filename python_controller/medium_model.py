"""
中模型层（Medium Model Layer）
8B-13B参数模型，作为小模型和大模型之间的桥梁
处理半实时任务：日志过滤、环境感知调整、策略微调
"""

import json
import time
from typing import Dict, List, Optional
from collections import deque
from ai_model import BaseAIModel


class MediumModel(BaseAIModel):
    """
    中模型包装器（8B-13B参数）
    
    用途：
    - 日志过滤：快速过滤失败事件，只上传关键日志给大模型
    - 环境感知调整：即时调整控制策略
    - 策略微调：在小模型和大模型之间进行策略过渡
    
    响应时间目标：100-500ms
    """
    
    def __init__(self, model_name: str = "mistral:7b"):
        try:
            import ollama
            self.ollama = ollama
        except ImportError:
            raise ImportError("请安装ollama: pip install ollama")
        
        self.model = model_name
        self.schema = self._create_schema()
        
        # 日志缓冲区
        self.log_buffer: deque = deque(maxlen=1000)
        self.critical_events: List[dict] = []
        
        # 环境状态缓存
        self.environment_state: Dict = {}
        self.last_adjustment_time: float = 0
        self.adjustment_interval: float = 0.5  # 500ms最小调整间隔
        
        # 统计
        self.total_predictions = 0
        self.total_time = 0.0
        self.errors = 0
        self.logs_filtered = 0
        self.events_escalated = 0
        
        # 验证模型
        self._verify_model()
    
    def _verify_model(self):
        """验证模型是否可用"""
        try:
            models = self.ollama.list()
            available = [m['name'] for m in models.get('models', [])]
            
            if not any(self.model in m for m in available):
                print(f"⚠️ 中模型 {self.model} 未找到")
                print(f"可用模型: {available}")
                print(f"\n请运行: ollama pull {self.model}")
                raise ValueError(f"模型 {self.model} 不可用")
            
            print(f"✅ 中模型 {self.model} 已加载")
            
        except Exception as e:
            print(f"❌ 无法连接到Ollama服务: {e}")
            raise
    
    def _create_schema(self) -> Dict:
        """创建JSON Schema"""
        return {
            "type": "object",
            "properties": {
                "adjustment": {
                    "type": "object",
                    "properties": {
                        "strategy_modifier": {
                            "type": "string",
                            "description": "策略调整建议"
                        },
                        "pid_tuning": {
                            "type": "object",
                            "properties": {
                                "kp_factor": {"type": "number", "minimum": 0.5, "maximum": 2.0},
                                "ki_factor": {"type": "number", "minimum": 0.5, "maximum": 2.0},
                                "kd_factor": {"type": "number", "minimum": 0.5, "maximum": 2.0}
                            }
                        },
                        "compliance_factor": {
                            "type": "number",
                            "minimum": 0,
                            "maximum": 1,
                            "description": "柔顺系数调整"
                        }
                    }
                },
                "escalate": {
                    "type": "boolean",
                    "description": "是否需要上报大模型"
                },
                "confidence": {
                    "type": "number",
                    "minimum": 0,
                    "maximum": 1
                }
            },
            "required": ["adjustment", "escalate"]
        }
    
    def predict(self, sensor_data: Dict, strategy: str = "") -> Dict:
        """
        中模型推理预测
        
        Args:
            sensor_data: 传感器数据
            strategy: 策略提示
        
        Returns:
            调整建议字典
        """
        start_time = time.time()
        
        try:
            prompt = self._build_prompt(sensor_data, strategy)
            
            response = self.ollama.generate(
                model=self.model,
                prompt=prompt,
                format=self.schema,
                options={
                    'temperature': 0.2,
                    'top_p': 0.9,
                    'num_predict': 200,
                    'stop': ['\n\n']
                }
            )
            
            result = json.loads(response['response'])
            
            self.total_predictions += 1
            self.total_time += time.time() - start_time
            
            return result
            
        except Exception as e:
            print(f"❌ 中模型推理错误: {e}")
            self.errors += 1
            return self._default_action()
    
    def _build_prompt(self, sensor_data: Dict, strategy: str) -> str:
        """构建环境感知调整Prompt"""
        orient = sensor_data['sensors']['imu']['orient']
        joints = sensor_data['sensors']['joints']
        height = sensor_data.get('torso_height', 0)
        
        # 获取最近日志摘要
        recent_errors = self._summarize_recent_errors()
        
        return f"""你是一个机器人运动控制的环境感知系统。根据当前状态和历史数据，输出控制策略调整建议。

## 当前策略
{strategy if strategy else "保持躯干直立，站立稳定"}

## 传感器数据
- Roll: {orient[0]:.2f}度
- Pitch: {orient[1]:.2f}度
- 左髋角度: {joints['hip_left']['angle']:.2f}度
- 右髋角度: {joints['hip_right']['angle']:.2f}度
- 躯干高度: {height:.2f}米

## 最近错误事件
{recent_errors if recent_errors else "无"}

## 环境状态
{json.dumps(self.environment_state, ensure_ascii=False, indent=2)}

## 任务
1. 分析当前状态是否稳定
2. 如果需要调整，输出PID参数调整因子
3. 判断是否需要上报大模型进行深度优化

请输出JSON格式的调整建议:"""
    
    def _default_action(self) -> Dict:
        """默认安全动作"""
        return {
            "adjustment": {
                "strategy_modifier": "",
                "pid_tuning": {"kp_factor": 1.0, "ki_factor": 1.0, "kd_factor": 1.0},
                "compliance_factor": 0.5
            },
            "escalate": False,
            "confidence": 0.0
        }
    
    def _summarize_recent_errors(self) -> str:
        """总结最近的错误事件"""
        if not self.critical_events:
            return ""
        
        recent = self.critical_events[-5:]  # 最近5个关键事件
        summary = []
        for event in recent:
            summary.append(f"- [{event.get('timestamp', 'N/A')}] {event.get('type', 'Unknown')}: {event.get('message', '')}")
        
        return "\n".join(summary)
    
    # =================== 日志过滤功能 ===================
    
    def add_log(self, log_entry: dict):
        """添加日志条目到缓冲区"""
        self.log_buffer.append(log_entry)
        self.logs_filtered += 1
        
        # 检查是否为关键事件
        if self._is_critical_event(log_entry):
            self.critical_events.append(log_entry)
    
    def _is_critical_event(self, log_entry: dict) -> bool:
        """判断是否为关键事件（需要上报大模型）"""
        # 关键事件条件
        critical_conditions = [
            log_entry.get('level') == 'ERROR',
            log_entry.get('level') == 'CRITICAL',
            log_entry.get('type') == 'FALL_DETECTED',
            log_entry.get('type') == 'EMERGENCY_STOP',
            abs(log_entry.get('roll', 0)) > 45,  # 严重倾斜
            abs(log_entry.get('pitch', 0)) > 45,
            log_entry.get('height', 1.0) < 0.3,  # 高度过低（可能摔倒）
        ]
        
        return any(critical_conditions)
    
    def filter_logs(self, logs: List[dict]) -> List[dict]:
        """
        过滤日志，只保留关键失败事件
        
        Args:
            logs: 原始日志列表
        
        Returns:
            过滤后的关键事件列表
        """
        critical_logs = []
        
        for log in logs:
            self.add_log(log)
            if self._is_critical_event(log):
                critical_logs.append(log)
        
        return critical_logs
    
    def get_logs_for_large_model(self) -> List[dict]:
        """获取需要上传给大模型的日志"""
        # 返回所有关键事件
        events = self.critical_events.copy()
        self.critical_events.clear()  # 清空已上传的事件
        self.events_escalated += len(events)
        return events
    
    # =================== 环境感知调整 ===================
    
    def adjust_environment(self, sensor_data: dict) -> dict:
        """
        即时环境感知调整
        
        Args:
            sensor_data: 传感器数据
        
        Returns:
            调整建议
        """
        current_time = time.time()
        
        # 检查调整间隔
        if current_time - self.last_adjustment_time < self.adjustment_interval:
            return {"skip": True, "reason": "调整间隔过短"}
        
        # 更新环境状态
        self._update_environment_state(sensor_data)
        
        # 获取调整建议
        adjustment = self.predict(sensor_data, "环境感知调整模式")
        
        self.last_adjustment_time = current_time
        
        return adjustment
    
    def _update_environment_state(self, sensor_data: dict):
        """更新环境状态缓存"""
        orient = sensor_data['sensors']['imu']['orient']
        height = sensor_data.get('torso_height', 0)
        
        # 计算稳定性指标
        stability = 1.0 - (abs(orient[0]) + abs(orient[1])) / 90.0
        stability = max(0, min(1, stability))
        
        self.environment_state = {
            "last_update": time.time(),
            "stability_score": stability,
            "roll_trend": orient[0],
            "pitch_trend": orient[1],
            "height": height,
            "critical_event_count": len(self.critical_events)
        }
    
    def should_escalate(self, event: dict) -> bool:
        """
        判断是否需要上报给大模型
        
        Args:
            event: 事件字典
        
        Returns:
            是否需要上报
        """
        # 严重事件立即上报
        if self._is_critical_event(event):
            return True
        
        # 稳定性过低上报
        if self.environment_state.get('stability_score', 1.0) < 0.3:
            return True
        
        # 关键事件积累过多上报
        if len(self.critical_events) >= 10:
            return True
        
        return False
    
    def get_stats(self) -> Dict:
        """获取统计信息"""
        avg_time = self.total_time / self.total_predictions if self.total_predictions > 0 else 0
        
        return {
            "total_predictions": self.total_predictions,
            "avg_inference_time": avg_time,
            "errors": self.errors,
            "logs_filtered": self.logs_filtered,
            "events_escalated": self.events_escalated,
            "environment_state": self.environment_state
        }


# 测试代码
if __name__ == "__main__":
    print("中模型层测试\n")
    
    # 创建中模型
    medium = MediumModel(model_name="mistral:7b")
    
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
    
    # 测试环境感知调整
    print("测试环境感知调整...")
    adjustment = medium.adjust_environment(dummy_sensor)
    print(f"调整建议: {json.dumps(adjustment, indent=2, ensure_ascii=False)}")
    
    # 测试日志过滤
    print("\n测试日志过滤...")
    test_logs = [
        {"level": "INFO", "message": "正常运行"},
        {"level": "WARNING", "message": "轻微倾斜", "roll": 10},
        {"level": "ERROR", "message": "控制异常", "type": "CONTROL_ERROR"},
        {"level": "CRITICAL", "message": "检测到跌倒", "type": "FALL_DETECTED"},
    ]
    
    critical = medium.filter_logs(test_logs)
    print(f"关键事件数量: {len(critical)}")
    
    # 统计
    stats = medium.get_stats()
    print(f"\n统计信息:")
    print(f"  推理次数: {stats['total_predictions']}")
    print(f"  平均耗时: {stats['avg_inference_time']*1000:.2f}ms")
    print(f"  日志过滤: {stats['logs_filtered']}")
    print(f"  上报事件: {stats['events_escalated']}")
