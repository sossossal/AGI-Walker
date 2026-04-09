import logging
from typing import Optional, Dict, Any
from .task_planning import BasePlanner, TaskGraph, TaskNode

logger = logging.getLogger(__name__)


class SimplePlanner(BasePlanner):
    """
    A rule-based planner that generates TaskGraphs from mission keywords.
    Serves as a foundation for future LLM-based planning.
    """

    def plan(
        self, instruction: str, context: Optional[Dict[str, Any]] = None
    ) -> TaskGraph:
        """
        Translates a mission instruction into a TaskGraph.
        Supported instructions: 'patrol', 'optimize_robot'
        """
        logger.info(f"Planning mission for instruction: '{instruction}'")

        graph = TaskGraph()

        if "patrol" in instruction.lower():
            return self._build_patrol_graph(graph)
        elif "optimize" in instruction.lower():
            return self._build_optimization_graph(graph)
        else:
            # Default: Simple identity graph
            node = TaskNode(name="identity", skill="system", action="ping")
            graph.add_node(node)
            return graph

    def _build_patrol_graph(self, graph: TaskGraph) -> TaskGraph:
        """Move -> (Success) -> Inspect | (Failure) -> Alarm"""

        move_node = TaskNode(
            name="move_to_point",
            skill="robot_modeling",
            action="move",
            params={"target": [5.0, 5.0]},
        )

        inspect_node = TaskNode(
            name="inspect_area", skill="vision_processor", action="capture_and_analyze"
        )

        alarm_node = TaskNode(
            name="emergency_alarm", skill="system", action="broadcast_error"
        )

        # Add nodes
        m_id = graph.add_node(move_node)
        i_id = graph.add_node(inspect_node)
        a_id = graph.add_node(alarm_node)

        # Add conditional edges
        graph.add_edge(m_id, i_id, condition="on_success")
        graph.add_edge(m_id, a_id, condition="on_failure")

        return graph

    def _build_optimization_graph(self, graph: TaskGraph) -> TaskGraph:
        """Build -> (Success) -> Optimize | (Failure) -> Fix -> Retry"""

        build_node = TaskNode(
            name="create_model",
            skill="robot_modeling",
            action="create_from_template",
            params={"template": "biped_basic"},
        )

        optimize_node = TaskNode(
            name="optimize_params",
            skill="parameter_optimizer",
            action="optimize_mass_distribution",
        )

        fix_node = TaskNode(
            name="fix_config", skill="system", action="auto_correct_params"
        )

        # Add nodes
        b_id = graph.add_node(build_node)
        o_id = graph.add_node(optimize_node)
        f_id = graph.add_node(fix_node)

        graph.add_edge(b_id, o_id, condition="on_success")
        graph.add_edge(b_id, f_id, condition="on_failure")
        # Retry logic is now handled via StepPolicy in V2.1+,
        # so we remove the cyclic edge back to b_id.

        return graph
