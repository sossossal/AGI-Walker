import abc
from typing import Any, Dict, List, Optional


class GodotAgentBackend(abc.ABC):
    """
    Godot Studio Agent 的抽象集成后端接口。
    AGI-Walker 核心通过此契约层使用高级的编辑与创作能力，而不需要强依赖整套上帝视角的编排网络。
    """

    @abc.abstractmethod
    def execute_command(
        self,
        command: str,
        context: Optional[Dict[str, Any]] = None,
        project_path: Optional[str] = None,
    ) -> Dict[str, Any]:
        """
        抛出一条文本指令进入 Godot Agent 执行上下文。
        """
        pass

    @abc.abstractmethod
    def execute_pipeline(
        self, commands: List[str], context: Optional[Dict[str, Any]] = None
    ) -> List[Dict[str, Any]]:
        """
        执行一条由多个原子任务组成的 Pipeline，必须保证上下文的流转与成功态。
        """
        pass

    @abc.abstractmethod
    def get_roles_info(self) -> List[Dict[str, Any]]:
        """
        向外界汇报自身系统装载的可用 Role/Agent 算力矩阵。
        """
        pass

    @abc.abstractmethod
    def list_skills(self) -> Dict[str, Any]:
        """
        向外围展示可用的（通过模板和提示词注入）的预设技能或 Prompt 模块。
        返回值应当如 `{"status": "success", "skills": [{"id": ..., "name": ...}]}`。
        """
        pass

    @abc.abstractmethod
    def apply_skill(self, skill_id: str) -> Dict[str, Any]:
        """
        依据要求加载技能配置的具体内容详情。
        """
        pass

    @abc.abstractmethod
    def list_templates(self) -> Dict[str, Any]:
        """
        列出当前 backend 暴露的模板资源。
        返回值应当如 `{"status": "success", "templates": [{"id": ..., "type": ...}]}`。
        """
        pass

    @abc.abstractmethod
    def get_template(self, template_id: str) -> Dict[str, Any]:
        """
        获取单个模板详情。
        """
        pass

    @abc.abstractmethod
    def plan_command(
        self,
        command: str,
        context: Optional[Dict[str, Any]] = None,
        project_path: Optional[str] = None,
    ) -> Dict[str, Any]:
        """
        生成可供前端展示或编辑的任务计划。
        """
        pass

    @abc.abstractmethod
    def get_history(self, limit: int = 20) -> Dict[str, Any]:
        """
        获取后端保存的最近任务历史。
        """
        pass

    @abc.abstractmethod
    def doctor(self, project_path: Optional[str] = None) -> Dict[str, Any]:
        """
        执行后端能力的环境与配置自检。
        """
        pass

    @abc.abstractmethod
    def launch_editor(
        self,
        project_path: Optional[str] = None,
        scene_path: Optional[str] = None,
    ) -> Dict[str, Any]:
        """
        请求启动 Godot 编辑器。
        """
        pass
