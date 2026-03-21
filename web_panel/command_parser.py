"""
AGI-Walker Agent 指令解析器 (Day 7 增强版)
将自然语言指令转换为结构化 Robot Configuration JSON。

扩展规则：
  - 支持中文/英文关键词
  - 支持数值提取（躯干高度、质量、腿长）
  - 输出 skills_params 供 /api/skills/pipeline 直接消费
"""
import re
import json
from typing import Dict, Any, Optional


class CommandParser:
    """
    解析 Agent 指令，生成机器人配置。
    当前为规则式实现（Day 7），后续可接入 LLM 替换。
    """

    def parse(self, command: str) -> Dict[str, Any]:
        """
        解析文本指令，返回结构化配置。

        示例：
          "创建一个双足机器人，躯干 0.5m，两腿各 0.3m，质量 6kg"
          "create quadruped height 0.4 mass 10"
        """
        cmd_lower = command.lower()

        # 默认参数
        skills_params: Dict[str, Any] = {
            "name": "ai_generated_robot",
            "type": "biped",
            "torso_height": 0.5,
            "torso_mass": 5.0,
            "thigh_length": 0.3,
            "shin_length": 0.3,
            "target_com_height": 0.22,
        }

        # 1. 类型提取
        if any(k in cmd_lower for k in ["四足", "quadruped", "狗", "dog", "quad"]):
            skills_params["type"] = "quadruped"
            skills_params["name"] = "ai_quadruped"
            skills_params["target_com_height"] = 0.15
        elif any(k in cmd_lower for k in ["六足", "hexapod", "hex"]):
            skills_params["type"] = "quadruped"   # 当前 builder 映射到 quadruped 扩展
            skills_params["name"] = "ai_hexapod"
            skills_params["target_com_height"] = 0.12
        elif any(k in cmd_lower for k in ["双足", "biped", "人形", "humanoid"]):
            skills_params["type"] = "biped"
            skills_params["name"] = "ai_biped"
            skills_params["target_com_height"] = 0.25

        # 2. 躯干高度：匹配 "躯干 0.5m" / "torso 0.5" / "高度 0.5m"
        h_match = re.search(
            r'(?:躯干|torso|高度?|height)\s*[：:=]?\s*([\d.]+)\s*m?',
            cmd_lower
        )
        if h_match:
            skills_params["torso_height"] = float(h_match.group(1))

        # 3. 质量：匹配 "质量 6kg" / "mass 10" / "重 10kg"
        m_match = re.search(
            r'(?:质量|重量?|mass|weight)\s*[：:=]?\s*([\d.]+)\s*k?g?',
            cmd_lower
        )
        if m_match:
            skills_params["torso_mass"] = float(m_match.group(1))

        # 4. 大腿长度：匹配 "大腿 0.3m" / "thigh 0.3"
        thigh_match = re.search(
            r'(?:大腿|thigh)\s*[：:=]?\s*([\d.]+)\s*m?',
            cmd_lower
        )
        if thigh_match:
            skills_params["thigh_length"] = float(thigh_match.group(1))

        # 5. 小腿长度："小腿 0.3m" / "shin 0.3" / "两腿各 0.3m"
        shin_match = re.search(
            r'(?:小腿|shin|腿各|each\s*leg)\s*[：:=]?\s*([\d.]+)\s*m?',
            cmd_lower
        )
        if shin_match:
            skills_params["shin_length"] = float(shin_match.group(1))
            # 若提到"两腿各"，大腿同样推断为相同长度（除非已单独指定）
            if not thigh_match and "各" in cmd_lower:
                skills_params["thigh_length"] = float(shin_match.group(1))

        # 6. 名称关键词提取（"叫 xxx" / "命名为 xxx"）
        name_match = re.search(
            r'(?:叫|命名为|named?)\s*([a-z_\u4e00-\u9fa5]{2,20})',
            cmd_lower
        )
        if name_match:
            skills_params["name"] = name_match.group(1).strip()

        # 构建统一返回格式
        config = {
            "parts": [],         # 保留字段（Godot 直接加载 legacy）
            "connections": [],
            "metadata": {
                "source_command": command,
                "type": skills_params["type"],
                "parser": "CommandParser-v2",
            },
            "skills_params": skills_params,
        }

        return config
