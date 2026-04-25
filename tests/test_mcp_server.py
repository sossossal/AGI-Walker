import asyncio
import ast
import json
from pathlib import Path

from agi_walker.mcp.server import (
    build_initialization_options,
    build_tool_list,
    call_tool,
    create_server,
    get_tool_definitions,
)


class FakeProvider:
    async def execute_mission(self, instruction: str):
        return {"status": "success", "instruction": instruction}

    def get_telemetry(self):
        return {"status": "success", "telemetry": {"cpu_percent": 12.5}}

    def query_rag(self, orient, top_k=1):
        return {"status": "success", "query": {"orient": orient, "top_k": top_k}}

    def list_workflows(self):
        return {"status": "success", "count": 1, "workflows": [{"name": "wf"}]}

    def get_workflow(self, name: str):
        return {"status": "success", "workflow": {"name": name}}

    def execute_workflow(self, name: str, parameters=None, use_real=None):
        return {"status": "success", "workflow_name": name, "parameters": parameters}

    def list_skills(self):
        return {
            "status": "success",
            "count": 1,
            "skills": [{"name": "robot-modeling"}],
        }

    def get_skill(self, name: str, include_doc: bool = False):
        return {
            "status": "success",
            "skill": {"name": name, "include_doc": include_doc},
        }

    def get_godot_agent_status(self):
        return {"status": "ready", "backend_mode": "fake"}

    def list_godot_templates(self):
        return {"status": "success", "templates": [{"id": "ai/patrol.gd"}]}

    def plan_godot_command(self, command: str, context=None, project_path=None):
        return {
            "status": "awaiting_confirmation",
            "command": command,
            "project_path": project_path,
        }

    def doctor_godot_agent(self, project_path=None):
        return {"status": "success", "ok": True, "project_path": project_path}

    def get_godot_history(self, limit=20):
        return {"status": "success", "count": 1, "limit": limit}

    def get_capability_matrix(self):
        return {
            "schema_version": "1.0",
            "artifact_type": "capability_matrix",
            "summary": {"total_domains": 5},
        }

    def get_release_control_plane_surface(
        self,
        project_root=None,
        manifest_path=None,
        release_ops_execution_report_path=None,
    ):
        return {
            "status": "success",
            "route": "/api/release/control-plane/surface",
            "source": "release_manifest",
            "manifest_path": manifest_path or "test_env/release/release_manifest.json",
            "control_plane_surface": {
                "status": "passed",
                "event_count": 3,
                "release_ops_execution": {"status": "passed", "event_count": 3},
                "control_plane_event_stream": {
                    "path": "test_env/release_ops/mcp.jsonl",
                    "event_count": 3,
                },
            },
        }

    def get_release_closeout(
        self,
        project_root=None,
        external_mainline_execution_plan_path=None,
        security_release_preflight_report_path=None,
        vulnerability_exception_review_report_path=None,
        release_readiness_report_path=None,
        worktree_release_blocker_report_path=None,
    ):
        return {
            "status": "success",
            "route": "/api/release/closeout",
            "release_closeout": {
                "status": "action_required",
                "blocked_components": 1,
                "waiting_external_input_components": 1,
                "ready_to_run_components": 0,
                "missing_components": 0,
                "action_items": [
                    {
                        "component": "external_mainline",
                        "status": "waiting_external_input",
                        "command": "python tools/run_customer_external_bindings_closure.py ...",
                        "component_route": "/static/release-closeout.html?component=external_mainline",
                        "component_api_route": "/api/release/closeout/component?component=external_mainline",
                    }
                ],
                "external_mainline": {"status": "waiting_external_input"},
                "vulnerability_exception_review": {
                    "status": "waiting_external_input",
                    "review_candidate_count": 31,
                },
                "worktree_release_blocker": {"status": "blocked"},
            },
        }

    def get_release_closeout_component(
        self,
        component,
        project_root=None,
        external_mainline_execution_plan_path=None,
        security_release_preflight_report_path=None,
        vulnerability_exception_review_report_path=None,
        release_readiness_report_path=None,
        worktree_release_blocker_report_path=None,
    ):
        payload = self.get_release_closeout()["release_closeout"]
        action_item = next(
            (
                item
                for item in payload["action_items"]
                if item.get("component") == component
            ),
            {},
        )
        return {
            "status": "success",
            "route": "/api/release/closeout/component",
            "component": component,
            "component_route": f"/static/release-closeout.html?component={component}",
            "component_api_route": f"/api/release/closeout/component?component={component}",
            "action_item": action_item,
            "closeout_component": payload.get(component, {}),
        }

    def get_release_closeout_next(
        self,
        project_root=None,
        external_mainline_execution_plan_path=None,
        security_release_preflight_report_path=None,
        vulnerability_exception_review_report_path=None,
        release_readiness_report_path=None,
        worktree_release_blocker_report_path=None,
    ):
        component = "external_mainline"
        component_payload = self.get_release_closeout_component(component)
        return {
            "status": "success",
            "route": "/api/release/closeout/next",
            "closeout_route": "/api/release/closeout",
            "next_component": component,
            "next_component_status": "waiting_external_input",
            "next_command": "python tools/run_customer_external_bindings_closure.py ...",
            "component_route": component_payload["component_route"],
            "component_api_route": component_payload["component_api_route"],
            "action_item": component_payload["action_item"],
            "closeout_component": component_payload["closeout_component"],
            "component_payload": component_payload,
        }

    def get_release_closeout_plan(
        self,
        project_root=None,
        external_mainline_execution_plan_path=None,
        external_mainline_inputs_path=None,
        external_mainline_input_checklist_report_path=None,
        security_release_preflight_report_path=None,
        vulnerability_exception_review_report_path=None,
        release_readiness_report_path=None,
        worktree_release_blocker_report_path=None,
    ):
        return {
            "status": "success",
            "route": "/api/release/closeout/plan",
            "portal_route": "/static/release-closeout-plan.html",
            "plan_status": "action_required",
            "next_stage_id": "customer_external_bindings_inputs",
            "next_stage_route": "/static/release-closeout-plan.html?stage=customer_external_bindings_inputs",
            "next_stage_api_route": "/api/release/closeout/plan?stage=customer_external_bindings_inputs",
            "stages": [
                {
                    "id": "customer_external_bindings_inputs",
                    "label": "Close Customer External Bindings Inputs",
                    "status": "waiting_external_input",
                    "component": "external_mainline",
                }
            ],
            "release_closeout": self.get_release_closeout()["release_closeout"],
        }

    def get_release_closeout_plan_stage(
        self,
        stage,
        project_root=None,
        external_mainline_execution_plan_path=None,
        external_mainline_inputs_path=None,
        external_mainline_input_checklist_report_path=None,
        security_release_preflight_report_path=None,
        vulnerability_exception_review_report_path=None,
        release_readiness_report_path=None,
        worktree_release_blocker_report_path=None,
    ):
        return {
            "status": "success",
            "route": "/api/release/closeout/plan/stage",
            "plan_route": "/api/release/closeout/plan",
            "plan_portal_route": "/static/release-closeout-plan.html",
            "plan_next_route": "/api/release/closeout/plan/next",
            "plan_status": "action_required",
            "stage": stage,
            "is_next_stage": stage == "customer_external_bindings_inputs",
            "stage_label": "Close Customer External Bindings Inputs",
            "stage_status": "waiting_external_input",
            "stage_component": "external_mainline",
            "stage_route": f"/static/release-closeout-plan.html?stage={stage}",
            "stage_api_route": f"/api/release/closeout/plan/stage?stage={stage}",
            "stage_payload": {
                "id": stage,
                "label": "Close Customer External Bindings Inputs",
                "status": "waiting_external_input",
                "component": "external_mainline",
                "stage_route": f"/static/release-closeout-plan.html?stage={stage}",
                "stage_api_route": f"/api/release/closeout/plan/stage?stage={stage}",
            },
            "release_closeout": self.get_release_closeout()["release_closeout"],
        }

    def get_release_closeout_plan_next(
        self,
        project_root=None,
        external_mainline_execution_plan_path=None,
        external_mainline_inputs_path=None,
        external_mainline_input_checklist_report_path=None,
        security_release_preflight_report_path=None,
        vulnerability_exception_review_report_path=None,
        release_readiness_report_path=None,
        worktree_release_blocker_report_path=None,
    ):
        return {
            "status": "success",
            "route": "/api/release/closeout/plan/next",
            "plan_route": "/api/release/closeout/plan",
            "plan_portal_route": "/static/release-closeout-plan.html",
            "plan_status": "action_required",
            "next_stage_id": "customer_external_bindings_inputs",
            "next_stage_label": "Close Customer External Bindings Inputs",
            "next_stage_status": "waiting_external_input",
            "next_stage_component": "external_mainline",
            "next_stage_route": "/static/release-closeout-plan.html?stage=customer_external_bindings_inputs",
            "next_stage_api_route": "/api/release/closeout/plan/stage?stage=customer_external_bindings_inputs",
            "next_stage_commands": [
                {
                    "label": "Generate customer-specific config",
                    "command": "python tools/build_customer_external_bindings_config.py ...",
                }
            ],
            "stage_payload": {
                "id": "customer_external_bindings_inputs",
                "label": "Close Customer External Bindings Inputs",
                "status": "waiting_external_input",
                "component": "external_mainline",
                "stage_route": "/static/release-closeout-plan.html?stage=customer_external_bindings_inputs",
                "stage_api_route": "/api/release/closeout/plan/stage?stage=customer_external_bindings_inputs",
            },
            "release_closeout": self.get_release_closeout()["release_closeout"],
        }

    def get_release_ops_catalog(self):
        return {
            "status": "success",
            "actions_count": 2,
            "policy_profiles_count": 2,
            "actions": [
                {
                    "action": "release_readiness",
                    "description": "Build release readiness.",
                    "policy_level": "local_safe_refresh",
                    "request_type": "ReleaseReadinessRequest",
                    "action_route": "/api/release/control-plane/action?action=release_readiness",
                    "request_template_route": "/api/release/control-plane/request-templates?action=release_readiness",
                    "portal_route": "/static/release-control-plane.html?action=release_readiness",
                },
                {
                    "action": "external_mainline_execution",
                    "description": "Run external mainline execution.",
                    "policy_level": "requires_attestation",
                    "request_type": "ExternalMainlineExecutionRequest",
                    "action_route": "/api/release/control-plane/action?action=external_mainline_execution",
                    "request_template_route": "/api/release/control-plane/request-templates?action=external_mainline_execution",
                    "portal_route": "/static/release-control-plane.html?action=external_mainline_execution",
                },
            ],
            "policy_profiles": [
                {
                    "policy_profile": "local_safe_refresh",
                    "description": "Allow local safe refresh actions.",
                    "default": "true",
                },
                {
                    "policy_profile": "requires_attestation",
                    "description": "Allow actions that require attestation.",
                    "default": "false",
                },
            ],
        }

    def get_release_ops_request_templates(self, action: str | None = None):
        templates = [
            {
                "action": "release_readiness",
                "description": "Build release readiness.",
                "policy_level": "local_safe_refresh",
                "default_policy_profile": "local_safe_refresh",
                "request_type": "ReleaseReadinessRequest",
                "required_fields": ["current_version", "stable_version", "project_root", "source_root"],
                "optional_fields": ["changelog", "output_root"],
                "action_route": "/api/release/control-plane/action?action=release_readiness",
                "request_template_route": "/api/release/control-plane/request-templates?action=release_readiness",
                "request_file_route": "/api/release/control-plane/request-file?action=release_readiness",
                "request_file_download_route": "/api/release/control-plane/request-file?action=release_readiness&download=1",
                "request_file_name": "release_ops.release_readiness.request.json",
                "portal_route": "/static/release-control-plane.html?action=release_readiness",
                "request_template": {
                    "current_version": None,
                    "stable_version": None,
                    "project_root": ".",
                    "source_root": ".",
                },
            },
            {
                "action": "external_mainline_execution",
                "description": "Run external mainline execution.",
                "policy_level": "requires_attestation",
                "default_policy_profile": "local_safe_refresh",
                "request_type": "ExternalMainlineExecutionRequest",
                "required_fields": [
                    "project_root",
                    "inputs_file",
                    "skip_managed_inputs",
                    "output",
                    "external_mainline_input_checklist_report",
                    "customer_config",
                    "customer_external_bindings_closure_report",
                    "vulnerability_exception_review_report",
                    "industrial_delivery_rehearsal_report",
                ],
                "optional_fields": ["customer_confirmed_by", "customer_confirmation_ticket"],
                "action_route": "/api/release/control-plane/action?action=external_mainline_execution",
                "request_template_route": "/api/release/control-plane/request-templates?action=external_mainline_execution",
                "request_file_route": "/api/release/control-plane/request-file?action=external_mainline_execution",
                "request_file_download_route": "/api/release/control-plane/request-file?action=external_mainline_execution&download=1",
                "request_file_name": "release_ops.external_mainline_execution.request.json",
                "portal_route": "/static/release-control-plane.html?action=external_mainline_execution",
                "request_template": {
                    "project_root": ".",
                    "inputs_file": "deployment/external_mainline.inputs.json",
                    "skip_managed_inputs": False,
                    "output": "test_env/release_evidence/operations/external_mainline_execution_plan.json",
                },
            },
        ]
        if action is not None:
            templates = [item for item in templates if item["action"] == action]
        return {
            "status": "success",
            "request_templates_count": len(templates),
            "request_templates": templates,
        }

    def get_release_control_plane_action(self, action: str):
        templates = self.get_release_ops_request_templates(action=action)[
            "request_templates"
        ]
        actions = [
            item for item in self.get_release_ops_catalog()["actions"] if item["action"] == action
        ]
        return {
            "status": "success",
            "route": "/api/release/control-plane/action",
            "action": action,
            "action_route": f"/api/release/control-plane/action?action={action}",
            "portal_route": f"/static/release-control-plane.html?action={action}",
            "action_definition": actions[0] if actions else {},
            "request_template": templates[0] if templates else {},
            "request_template_route": f"/api/release/control-plane/request-templates?action={action}",
            "request_file_route": f"/api/release/control-plane/request-file?action={action}",
            "request_file_download_route": f"/api/release/control-plane/request-file?action={action}&download=1",
            "request_file_name": f"release_ops.{action}.request.json",
        }

    def get_release_control_plane_request_file(self, action: str):
        action_payload = self.get_release_control_plane_action(action)
        template = action_payload.get("request_template", {})
        return {
            "status": "success",
            "route": "/api/release/control-plane/request-file",
            "action": action,
            "action_route": action_payload.get("action_route"),
            "portal_route": action_payload.get("portal_route"),
            "request_template_route": action_payload.get("request_template_route"),
            "request_file_route": f"/api/release/control-plane/request-file?action={action}",
            "request_file_download_route": f"/api/release/control-plane/request-file?action={action}&download=1",
            "request_file_name": f"release_ops.{action}.request.json",
            "content_type": "application/json",
            "request_file": template.get("request_template", {}),
            "request_file_pretty_json": json.dumps(
                template.get("request_template", {}), ensure_ascii=False, indent=2
            )
            + "\n",
        }

    def get_release_control_plane_next(
        self,
        project_root=None,
        manifest_path=None,
        release_ops_execution_report_path=None,
    ):
        action = "release_readiness"
        action_payload = self.get_release_control_plane_action(action)
        request_file_payload = self.get_release_control_plane_request_file(action)
        return {
            "status": "success",
            "route": "/api/release/control-plane/next",
            "control_plane_index_route": "/api/release/control-plane",
            "control_plane_surface_route": "/api/release/control-plane/surface",
            "release_closeout_route": "/api/release/closeout",
            "control_plane_surface_source": "release_manifest",
            "next_action": action,
            "next_action_default_policy_profile": "local_safe_refresh",
            "action_route": action_payload["action_route"],
            "portal_route": action_payload["portal_route"],
            "request_template_route": action_payload["request_template_route"],
            "request_file_route": request_file_payload["request_file_route"],
            "request_file_download_route": request_file_payload[
                "request_file_download_route"
            ],
            "request_file_name": request_file_payload["request_file_name"],
            "action_definition": action_payload["action_definition"],
            "request_template": action_payload["request_template"],
            "request_file": request_file_payload["request_file"],
            "request_file_pretty_json": request_file_payload[
                "request_file_pretty_json"
            ],
            "content_type": "application/json",
            "action_payload": action_payload,
            "request_file_payload": request_file_payload,
        }

    def get_release_control_plane_index(
        self,
        project_root=None,
        manifest_path=None,
        release_ops_execution_report_path=None,
    ):
        return {
            "status": "success",
            "control_plane_surface_source": "release_manifest",
            "manifest_path": manifest_path or "test_env/release/release_manifest.json",
            "release_ops_execution_report_path": release_ops_execution_report_path,
            "control_plane_surface": {
                "status": "passed",
                "event_count": 3,
                "release_ops_execution": {"status": "passed", "event_count": 3},
            },
            "actions_count": 2,
            "policy_profiles_count": 2,
            "next_action": "release_readiness",
            "next_action_default_policy_profile": "local_safe_refresh",
            "next_action_route": "/static/release-control-plane.html?action=release_readiness",
            "next_action_request_route": "/api/release/control-plane/action?action=release_readiness",
            "next_action_request_file_route": "/api/release/control-plane/request-file?action=release_readiness",
            "next_action_request_file_download_route": "/api/release/control-plane/request-file?action=release_readiness&download=1",
            "next_action_request_file_name": "release_ops.release_readiness.request.json",
            "actions": self.get_release_ops_catalog()["actions"],
            "policy_profiles": self.get_release_ops_catalog()["policy_profiles"],
            "request_templates_count": 2,
            "request_templates": self.get_release_ops_request_templates()[
                "request_templates"
            ],
        }

    def get_release_next(
        self,
        project_root=None,
        manifest_path=None,
        release_ops_execution_report_path=None,
        external_mainline_execution_plan_path=None,
        security_release_preflight_report_path=None,
        vulnerability_exception_review_report_path=None,
        release_readiness_report_path=None,
        worktree_release_blocker_report_path=None,
    ):
        return {
            "status": "success",
            "route": "/api/release/next",
            "portal_route": "/static/release-next.html",
            "control_plane_next": self.get_release_control_plane_next(),
            "release_closeout_next": self.get_release_closeout_next(),
            "primary_kind": "closeout_component",
            "primary_name": "external_mainline",
            "primary_status": "waiting_external_input",
            "primary_portal_route": "/static/release-closeout.html?component=external_mainline",
            "primary_api_route": "/api/release/closeout/component?component=external_mainline",
            "primary_next_route": "/api/release/closeout/next",
            "primary_command": "python tools/run_customer_external_bindings_closure.py ...",
            "primary_payload": {
                "status": "success",
                "route": "/api/release/next/primary",
                "primary_payload_route": "/api/release/next/primary",
                "release_next_route": "/api/release/next",
                "release_next_portal_route": "/static/release-next.html",
                "primary_kind": "closeout_component",
                "primary_name": "external_mainline",
                "primary_status": "waiting_external_input",
                "primary_portal_route": "/static/release-closeout.html?component=external_mainline",
                "primary_api_route": "/api/release/closeout/component?component=external_mainline",
                "primary_next_route": "/api/release/closeout/next",
                "primary_command": "python tools/run_customer_external_bindings_closure.py ...",
                "primary_follow_up_kind": "command",
                "primary_follow_up_label": "建议命令",
                "primary_follow_up_route": "/api/release/closeout/next",
                "primary_follow_up_text": "python tools/run_customer_external_bindings_closure.py ...",
            },
            "follow_up_payload": {
                "status": "success",
                "route": "/api/release/next/follow-up",
                "release_next_route": "/api/release/next",
                "release_next_primary_route": "/api/release/next/primary",
                "release_next_portal_route": "/static/release-next.html",
                "follow_up_kind": "command",
                "follow_up_label": "建议命令",
                "follow_up_route": "/api/release/closeout/next",
                "follow_up_text": "python tools/run_customer_external_bindings_closure.py ...",
            },
            "request_file_payload": {
                "status": "missing",
                "route": "/api/release/next/request-file",
                "release_next_route": "/api/release/next",
                "release_next_primary_route": "/api/release/next/primary",
                "release_next_follow_up_route": "/api/release/next/follow-up",
                "release_next_portal_route": "/static/release-next.html",
                "request_file_download_route": None,
            },
        }

    def get_release_next_primary(
        self,
        project_root=None,
        manifest_path=None,
        release_ops_execution_report_path=None,
        external_mainline_execution_plan_path=None,
        security_release_preflight_report_path=None,
        vulnerability_exception_review_report_path=None,
        release_readiness_report_path=None,
        worktree_release_blocker_report_path=None,
    ):
        return {
            "status": "success",
            "route": "/api/release/next/primary",
            "primary_payload_route": "/api/release/next/primary",
            "release_next_route": "/api/release/next",
            "release_next_portal_route": "/static/release-next.html",
            "primary_kind": "closeout_component",
            "primary_name": "external_mainline",
            "primary_status": "waiting_external_input",
            "primary_portal_route": "/static/release-closeout.html?component=external_mainline",
            "primary_api_route": "/api/release/closeout/component?component=external_mainline",
            "primary_next_route": "/api/release/closeout/next",
            "primary_command": "python tools/run_customer_external_bindings_closure.py ...",
            "primary_follow_up_kind": "command",
            "primary_follow_up_label": "建议命令",
            "primary_follow_up_route": "/api/release/closeout/next",
            "primary_follow_up_text": "python tools/run_customer_external_bindings_closure.py ...",
        }

    def get_release_next_follow_up(
        self,
        project_root=None,
        manifest_path=None,
        release_ops_execution_report_path=None,
        external_mainline_execution_plan_path=None,
        security_release_preflight_report_path=None,
        vulnerability_exception_review_report_path=None,
        release_readiness_report_path=None,
        worktree_release_blocker_report_path=None,
    ):
        return {
            "status": "success",
            "route": "/api/release/next/follow-up",
            "release_next_route": "/api/release/next",
            "release_next_primary_route": "/api/release/next/primary",
            "release_next_portal_route": "/static/release-next.html",
            "primary_kind": "closeout_component",
            "primary_name": "external_mainline",
            "primary_status": "waiting_external_input",
            "follow_up_kind": "command",
            "follow_up_label": "建议命令",
            "follow_up_route": "/api/release/closeout/next",
            "follow_up_text": "python tools/run_customer_external_bindings_closure.py ...",
        }

    def get_release_next_request_file(
        self,
        project_root=None,
        manifest_path=None,
        release_ops_execution_report_path=None,
        external_mainline_execution_plan_path=None,
        security_release_preflight_report_path=None,
        vulnerability_exception_review_report_path=None,
        release_readiness_report_path=None,
        worktree_release_blocker_report_path=None,
    ):
        return {
            "status": "missing",
            "route": "/api/release/next/request-file",
            "release_next_route": "/api/release/next",
            "release_next_primary_route": "/api/release/next/primary",
            "release_next_follow_up_route": "/api/release/next/follow-up",
            "release_next_portal_route": "/static/release-next.html",
            "request_file_download_route": None,
        }


def _string_keys(dict_node):
    return [
        key.value
        for key in dict_node.keys
        if isinstance(key, ast.Constant) and isinstance(key.value, str)
    ]


def test_mcp_server_source_has_no_duplicate_tool_registry_keys() -> None:
    source = Path("agi_walker/mcp/server.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    registry_keys = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.FunctionDef) or node.name != "get_tool_definitions":
            continue
        for child in ast.walk(node):
            if isinstance(child, ast.Return) and isinstance(child.value, ast.Dict):
                registry_keys.extend(_string_keys(child.value))

    assert registry_keys
    assert len(registry_keys) == len(set(registry_keys))


def test_mcp_tool_provider_source_has_no_duplicate_methods() -> None:
    source = Path("agi_walker/core/api/mcp_tools.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    method_names = []
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == "MCPToolProvider":
            method_names = [
                child.name for child in node.body if isinstance(child, ast.FunctionDef)
            ]
            break

    assert method_names
    assert len(method_names) == len(set(method_names))


def test_build_tool_list_contains_extended_surface() -> None:
    tool_definitions = get_tool_definitions(FakeProvider())
    tools = build_tool_list(tool_definitions)
    names = {tool.name for tool in tools}

    assert "mission_execute" in names
    assert "workflow_execute" in names
    assert "skills_list" in names
    assert "godot_agent_status" in names
    assert "capability_matrix_get" in names
    assert "release_control_plane_surface_get" in names
    assert "release_closeout_get" in names
    assert "release_closeout_component_get" in names
    assert "release_closeout_next_get" in names
    assert "release_closeout_plan_get" in names
    assert "release_closeout_plan_stage_get" in names
    assert "release_closeout_plan_next_get" in names
    assert "release_ops_catalog_get" in names
    assert "release_ops_request_templates_get" in names
    assert "release_control_plane_next_get" in names
    assert "release_control_plane_request_file_get" in names
    assert "release_control_plane_action_get" in names
    assert "release_control_plane_index_get" in names
    assert "release_next_get" in names
    assert "release_next_primary_get" in names
    assert "release_next_follow_up_get" in names
    assert "release_next_request_file_get" in names
    assert len(names) == 31


def test_call_tool_returns_json_text_content() -> None:
    tool_definitions = get_tool_definitions(FakeProvider())

    result = asyncio.run(
        call_tool(
            tool_definitions,
            "godot_agent_plan",
            {"command": "生成巡逻脚本", "project_path": "D:/tmp/project"},
        )
    )

    assert len(result) == 1
    payload = json.loads(result[0].text)
    assert payload["status"] == "awaiting_confirmation"
    assert payload["project_path"] == "D:/tmp/project"


def test_capability_matrix_tool_returns_contract_payload() -> None:
    tool_definitions = get_tool_definitions(FakeProvider())

    result = asyncio.run(call_tool(tool_definitions, "capability_matrix_get", {}))

    payload = json.loads(result[0].text)
    assert payload["artifact_type"] == "capability_matrix"
    assert payload["summary"]["total_domains"] == 5


def test_release_control_plane_surface_tool_returns_contract_payload() -> None:
    tool_definitions = get_tool_definitions(FakeProvider())

    result = asyncio.run(
        call_tool(
            tool_definitions,
            "release_control_plane_surface_get",
            {"manifest_path": "test_env/release/release_manifest.json"},
        )
    )

    payload = json.loads(result[0].text)
    assert payload["status"] == "success"
    assert payload["route"] == "/api/release/control-plane/surface"
    assert payload["source"] == "release_manifest"
    assert payload["control_plane_surface"]["status"] == "passed"
    assert payload["control_plane_surface"]["event_count"] == 3


def test_release_ops_catalog_tool_returns_structured_payload() -> None:
    tool_definitions = get_tool_definitions(FakeProvider())

    result = asyncio.run(call_tool(tool_definitions, "release_ops_catalog_get", {}))

    payload = json.loads(result[0].text)
    assert payload["status"] == "success"
    assert payload["actions_count"] >= 1
    assert payload["policy_profiles_count"] >= 1


def test_release_closeout_tool_returns_structured_payload() -> None:
    tool_definitions = get_tool_definitions(FakeProvider())

    result = asyncio.run(call_tool(tool_definitions, "release_closeout_get", {}))

    payload = json.loads(result[0].text)
    assert payload["status"] == "success"
    assert payload["route"] == "/api/release/closeout"
    assert payload["release_closeout"]["status"] == "action_required"
    assert payload["release_closeout"]["blocked_components"] == 1


def test_release_closeout_component_tool_returns_structured_payload() -> None:
    tool_definitions = get_tool_definitions(FakeProvider())

    result = asyncio.run(
        call_tool(
            tool_definitions,
            "release_closeout_component_get",
            {"component": "external_mainline"},
        )
    )

    payload = json.loads(result[0].text)
    assert payload["status"] == "success"
    assert payload["route"] == "/api/release/closeout/component"
    assert payload["component"] == "external_mainline"
    assert payload["closeout_component"]["status"] == "waiting_external_input"
    assert payload["action_item"]["component"] == "external_mainline"
    assert payload["component_api_route"].endswith(
        "/api/release/closeout/component?component=external_mainline"
    )
    assert payload["action_item"]["component_route"].endswith(
        "/static/release-closeout.html?component=external_mainline"
    )


def test_release_closeout_next_tool_returns_structured_payload() -> None:
    tool_definitions = get_tool_definitions(FakeProvider())

    result = asyncio.run(call_tool(tool_definitions, "release_closeout_next_get", {}))

    payload = json.loads(result[0].text)
    assert payload["status"] == "success"
    assert payload["route"] == "/api/release/closeout/next"
    assert payload["next_component"] == "external_mainline"
    assert payload["component_api_route"].endswith(
        "/api/release/closeout/component?component=external_mainline"
    )
    assert payload["action_item"]["component"] == "external_mainline"
    assert payload["closeout_component"]["status"] == "waiting_external_input"


def test_release_closeout_plan_tool_returns_structured_payload() -> None:
    tool_definitions = get_tool_definitions(FakeProvider())

    result = asyncio.run(call_tool(tool_definitions, "release_closeout_plan_get", {}))

    payload = json.loads(result[0].text)
    assert payload["status"] == "success"
    assert payload["route"] == "/api/release/closeout/plan"
    assert payload["portal_route"] == "/static/release-closeout-plan.html"
    assert payload["plan_status"] == "action_required"
    assert payload["next_stage_id"] == "customer_external_bindings_inputs"
    assert payload["stages"][0]["component"] == "external_mainline"


def test_release_closeout_plan_stage_and_next_tools_return_structured_payload() -> None:
    tool_definitions = get_tool_definitions(FakeProvider())

    stage_result = asyncio.run(
        call_tool(
            tool_definitions,
            "release_closeout_plan_stage_get",
            {"stage": "customer_external_bindings_inputs"},
        )
    )
    next_result = asyncio.run(
        call_tool(tool_definitions, "release_closeout_plan_next_get", {})
    )

    stage_payload = json.loads(stage_result[0].text)
    next_payload = json.loads(next_result[0].text)

    assert stage_payload["status"] == "success"
    assert stage_payload["route"] == "/api/release/closeout/plan/stage"
    assert stage_payload["stage"] == "customer_external_bindings_inputs"
    assert stage_payload["stage_api_route"].endswith(
        "/api/release/closeout/plan/stage?stage=customer_external_bindings_inputs"
    )

    assert next_payload["status"] == "success"
    assert next_payload["route"] == "/api/release/closeout/plan/next"
    assert next_payload["next_stage_id"] == "customer_external_bindings_inputs"
    assert next_payload["next_stage_api_route"].endswith(
        "/api/release/closeout/plan/stage?stage=customer_external_bindings_inputs"
    )


def test_release_control_plane_index_tool_returns_structured_payload() -> None:
    tool_definitions = get_tool_definitions(FakeProvider())

    result = asyncio.run(
        call_tool(
            tool_definitions,
            "release_control_plane_index_get",
            {"manifest_path": "test_env/release/release_manifest.json"},
        )
    )

    payload = json.loads(result[0].text)
    assert payload["status"] == "success"
    assert payload["control_plane_surface_source"] == "release_manifest"
    assert payload["control_plane_surface"]["status"] == "passed"
    assert payload["actions_count"] >= 1
    assert payload["policy_profiles_count"] >= 1
    assert payload["request_templates_count"] >= 1
    assert payload["next_action"] == "release_readiness"
    assert payload["next_action_request_file_route"].endswith(
        "/api/release/control-plane/request-file?action=release_readiness"
    )


def test_release_next_tool_returns_structured_payload() -> None:
    tool_definitions = get_tool_definitions(FakeProvider())

    result = asyncio.run(call_tool(tool_definitions, "release_next_get", {}))

    payload = json.loads(result[0].text)
    assert payload["status"] == "success"
    assert payload["route"] == "/api/release/next"
    assert payload["portal_route"] == "/static/release-next.html"
    assert payload["control_plane_next"]["route"] == "/api/release/control-plane/next"
    assert payload["release_closeout_next"]["route"] == "/api/release/closeout/next"
    assert payload["primary_kind"] == "closeout_component"
    assert payload["primary_name"] == "external_mainline"
    assert payload["primary_next_route"] == "/api/release/closeout/next"
    assert payload["primary_payload"]["route"] == "/api/release/next/primary"
    assert payload["follow_up_payload"]["route"] == "/api/release/next/follow-up"


def test_release_next_primary_tool_returns_structured_payload() -> None:
    tool_definitions = get_tool_definitions(FakeProvider())

    result = asyncio.run(call_tool(tool_definitions, "release_next_primary_get", {}))

    payload = json.loads(result[0].text)
    assert payload["status"] == "success"
    assert payload["route"] == "/api/release/next/primary"
    assert payload["primary_payload_route"] == "/api/release/next/primary"
    assert payload["release_next_route"] == "/api/release/next"
    assert payload["release_next_portal_route"] == "/static/release-next.html"
    assert payload["primary_kind"] == "closeout_component"
    assert payload["primary_name"] == "external_mainline"
    assert payload["primary_follow_up_kind"] == "command"
    assert payload["primary_follow_up_route"] == "/api/release/closeout/next"


def test_release_next_follow_up_tool_returns_structured_payload() -> None:
    tool_definitions = get_tool_definitions(FakeProvider())

    result = asyncio.run(call_tool(tool_definitions, "release_next_follow_up_get", {}))

    payload = json.loads(result[0].text)
    assert payload["status"] == "success"
    assert payload["route"] == "/api/release/next/follow-up"
    assert payload["release_next_route"] == "/api/release/next"
    assert payload["release_next_primary_route"] == "/api/release/next/primary"
    assert payload["follow_up_kind"] == "command"
    assert payload["follow_up_route"] == "/api/release/closeout/next"


def test_release_next_request_file_tool_returns_structured_payload() -> None:
    tool_definitions = get_tool_definitions(FakeProvider())

    result = asyncio.run(call_tool(tool_definitions, "release_next_request_file_get", {}))

    payload = json.loads(result[0].text)
    assert payload["status"] == "missing"
    assert payload["route"] == "/api/release/next/request-file"
    assert payload["release_next_route"] == "/api/release/next"
    assert payload["release_next_primary_route"] == "/api/release/next/primary"


def test_release_ops_request_templates_tool_returns_structured_payload() -> None:
    tool_definitions = get_tool_definitions(FakeProvider())

    result = asyncio.run(
        call_tool(
            tool_definitions,
            "release_ops_request_templates_get",
            {"action": "external_mainline_execution"},
        )
    )

    payload = json.loads(result[0].text)
    assert payload["status"] == "success"
    assert payload["request_templates_count"] == 1
    assert payload["request_templates"][0]["action"] == "external_mainline_execution"
    assert payload["request_templates"][0]["policy_level"] == "requires_attestation"
    assert payload["request_templates"][0]["portal_route"].endswith(
        "/static/release-control-plane.html?action=external_mainline_execution"
    )
    assert payload["request_templates"][0]["request_file_route"].endswith(
        "/api/release/control-plane/request-file?action=external_mainline_execution"
    )
    assert payload["request_templates"][0]["request_file_download_route"].endswith(
        "/api/release/control-plane/request-file?action=external_mainline_execution&download=1"
    )


def test_release_control_plane_next_tool_returns_structured_payload() -> None:
    tool_definitions = get_tool_definitions(FakeProvider())

    result = asyncio.run(
        call_tool(
            tool_definitions,
            "release_control_plane_next_get",
            {},
        )
    )

    payload = json.loads(result[0].text)
    assert payload["status"] == "success"
    assert payload["route"] == "/api/release/control-plane/next"
    assert payload["next_action"] == "release_readiness"
    assert payload["action_definition"]["action"] == "release_readiness"
    assert payload["request_template"]["action"] == "release_readiness"
    assert payload["request_file_route"].endswith(
        "/api/release/control-plane/request-file?action=release_readiness"
    )
    assert payload["request_file_download_route"].endswith(
        "/api/release/control-plane/request-file?action=release_readiness&download=1"
    )


def test_release_control_plane_request_file_tool_returns_structured_payload() -> None:
    tool_definitions = get_tool_definitions(FakeProvider())

    result = asyncio.run(
        call_tool(
            tool_definitions,
            "release_control_plane_request_file_get",
            {"action": "external_mainline_execution"},
        )
    )

    payload = json.loads(result[0].text)
    assert payload["status"] == "success"
    assert payload["route"] == "/api/release/control-plane/request-file"
    assert payload["action"] == "external_mainline_execution"
    assert payload["request_file_route"].endswith(
        "/api/release/control-plane/request-file?action=external_mainline_execution"
    )
    assert payload["request_file_download_route"].endswith(
        "/api/release/control-plane/request-file?action=external_mainline_execution&download=1"
    )
    assert payload["request_file_name"] == (
        "release_ops.external_mainline_execution.request.json"
    )
    assert payload["request_file"]["project_root"] == "."


def test_release_control_plane_action_tool_returns_structured_payload() -> None:
    tool_definitions = get_tool_definitions(FakeProvider())

    result = asyncio.run(
        call_tool(
            tool_definitions,
            "release_control_plane_action_get",
            {"action": "external_mainline_execution"},
        )
    )

    payload = json.loads(result[0].text)
    assert payload["status"] == "success"
    assert payload["route"] == "/api/release/control-plane/action"
    assert payload["action"] == "external_mainline_execution"
    assert payload["action_definition"]["policy_level"] == "requires_attestation"
    assert payload["request_template"]["action"] == "external_mainline_execution"
    assert payload["action_route"].endswith(
        "/api/release/control-plane/action?action=external_mainline_execution"
    )
    assert payload["portal_route"].endswith(
        "/static/release-control-plane.html?action=external_mainline_execution"
    )
    assert payload["request_file_route"].endswith(
        "/api/release/control-plane/request-file?action=external_mainline_execution"
    )
    assert payload["request_file_download_route"].endswith(
        "/api/release/control-plane/request-file?action=external_mainline_execution&download=1"
    )


def test_call_tool_rejects_unknown_name() -> None:
    tool_definitions = get_tool_definitions(FakeProvider())

    try:
        asyncio.run(call_tool(tool_definitions, "missing_tool", {}))
    except ValueError as exc:
        assert "Unknown tool" in str(exc)
    else:
        raise AssertionError("Expected ValueError for unknown tool")


def test_build_initialization_options_exposes_tools_capability() -> None:
    server = create_server(FakeProvider())

    options = build_initialization_options(server)

    assert options.server_name == "agi-walker-control"
    assert options.server_version == "3.0.0"
    assert options.capabilities.tools is not None
