"""
AGI-Walker Agent 指令解析器 (Day 7 增强版 -> Milestone 3 LLM 融合版)
将自然语言指令转换为结构化 Robot Configuration JSON。

扩展规则：
  - 优先尝试通过神盾局系统配置的 LLM（Ollama/OpenAI）提取 JSON 格式参数
  - 当 LLM 环境未配置或请求失败时，安全回退到本地的强正则机制
  - 输出 skills_params 供 /api/skills/pipeline 直接消费
"""
import re
import json
import urllib.request
import urllib.error
import os
from typing import Dict, Any, Optional


class CommandParser:
    """
    解析 Agent 指令，生成机器人配置。
    引入 LLM 的智能语义抽取，极大提升泛化能力。
    """

    def parse(self, command: str) -> Dict[str, Any]:
        """
        解析文本指令，返回结构化配置。
        先尝试 LLM，失败则 fallback 到 Regex。
        """
        prompt = f"""
你是一个专业的机器人属性提取器。请从用户的指令中提取以下参数，并严格以JSON格式返回：
- type (仅限 "biped" 或 "quadruped", 默认为 "biped")
- name (机器人名字，例如 "ai_biped" 或用户指定的英文名, 默认 "ai_generated_robot")
- torso_height (躯干高度，浮点数，默认 0.5)
- torso_mass (躯干质量，浮点数，默认 5.0)
- thigh_length (大腿长度，浮点数，默认 0.3)
- shin_length (小腿长度，浮点数，默认 0.3)
- target_com_height (质心高度，双足默认0.25，四足默认0.15)

用户指令: "{command}"
请仅输出纯JSON，没有任何markdown代码块（不要包裹```json），也不要有任何解释。
"""
        
        skills_params = None
        
        # 1. 尝试 LLM 获取
        try:
            skills_params = self._llm_parse(prompt)
        except Exception as e:
            print(f"⚠️ LLM 解析失败，正在回退到正则机制: {e}")

        # 2. 回退机制：正则表达式匹配
        if not skills_params:
            skills_params = self._regex_parse(command)

        # 构建统一返回格式
        config = {
            "parts": [],         
            "connections": [],
            "metadata": {
                "source_command": command,
                "type": skills_params.get("type", "biped"),
                "parser": "LLM" if skills_params.get("_from_llm") else "Regex",
            },
            "skills_params": {k: v for k, v in skills_params.items() if not k.startswith("_")},
        }

        return config

    def _llm_parse(self, prompt: str) -> Optional[Dict[str, Any]]:
        """调用本地 Ollama 或 OpenAI API 进行解析"""
        api_key = os.environ.get("OPENAI_API_KEY", "")
        
        # 如果配置了 OpenAI 密钥，优先走 OpenAI
        if api_key:
            req = urllib.request.Request(
                "https://api.openai.com/v1/chat/completions",
                headers={
                    "Authorization": f"Bearer {api_key}",
                    "Content-Type": "application/json"
                },
                data=json.dumps({
                    "model": "gpt-4o-mini",
                    "messages": [{"role": "user", "content": prompt}],
                    "temperature": 0.1
                }).encode("utf-8")
            )
            with urllib.request.urlopen(req, timeout=10) as response:
                result = json.loads(response.read().decode("utf-8"))
                content = result["choices"][0]["message"]["content"].strip()
                # 清理可能的 markdown codeblock
                if content.startswith("```"):
                    content = content.replace("```json", "").replace("```", "").strip()
                data = json.loads(content)
                data["_from_llm"] = True
                return data

        # 否则尝试本地 Ollama (默认 11434 端口)
        req = urllib.request.Request(
            "http://localhost:11434/api/generate",
            headers={"Content-Type": "application/json"},
            data=json.dumps({
                "model": "qwen2.5:latest", # 假设用户可能装了 qwen 或者 llama3，如果报错会走 fallback
                "prompt": prompt,
                "stream": False,
                "format": "json"
            }).encode("utf-8")
        )
        try:
            with urllib.request.urlopen(req, timeout=5) as response:
                result = json.loads(response.read().decode("utf-8"))
                data = json.loads(result["response"])
                data["_from_llm"] = True
                return data
        except urllib.error.URLError:
            return None # 网络不通，让上层去 fallback

    def _regex_parse(self, command: str) -> Dict[str, Any]:
        """原有的纯正则提取逻辑"""
        cmd_lower = command.lower()
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
        elif any(k in cmd_lower for k in ["双足", "biped", "人形", "humanoid"]):
            skills_params["type"] = "biped"
            skills_params["name"] = "ai_biped"
            skills_params["target_com_height"] = 0.25

        # 2. 局部属性提取
        h_match = re.search(r'(?:躯干|torso|高度?|height)\s*[：:=]?\s*([\d.]+)\s*m?', cmd_lower)
        if h_match: skills_params["torso_height"] = float(h_match.group(1))

        m_match = re.search(r'(?:质量|重量?|mass|weight)\s*[：:=]?\s*([\d.]+)\s*k?g?', cmd_lower)
        if m_match: skills_params["torso_mass"] = float(m_match.group(1))

        thigh_match = re.search(r'(?:大腿|thigh)\s*[：:=]?\s*([\d.]+)\s*m?', cmd_lower)
        if thigh_match: skills_params["thigh_length"] = float(thigh_match.group(1))

        shin_match = re.search(r'(?:小腿|shin|腿各|each\s*leg)\s*[：:=]?\s*([\d.]+)\s*m?', cmd_lower)
        if shin_match:
            skills_params["shin_length"] = float(shin_match.group(1))
            if not thigh_match and "各" in cmd_lower:
                skills_params["thigh_length"] = float(shin_match.group(1))

        name_match = re.search(r'(?:叫|命名为|named?)\s*([a-z_\u4e00-\u9fa5]{2,20})', cmd_lower)
        if name_match: skills_params["name"] = name_match.group(1).strip()

        return skills_params
