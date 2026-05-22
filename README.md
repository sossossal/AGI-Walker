# AGI-Walker

AGI-Walker is a Python platform for robot design, simulation workflow orchestration, Web/Godot integration and delivery evidence automation.

The current repository focuses on a contract-first robotics workflow: robot JSON schemas, generated Godot mechanical node trees, static and live verification reports, Web operator surfaces, release evidence, and hardware-aware acceptance gates.

## What Is Included

- Robot modeling and workflow orchestration through `agi_walker/`, skills and workflow definitions.
- FastAPI Web Panel under `web_panel/` for workflow runs, Godot sessions, release evidence and operator surfaces.
- Godot runtime assets under `godot_project/`, including dynamic robot assembly and smoke-test support.
- JSON-to-Godot dynamic robot generation tooling, reports, delivery gates and release bundles.
- Docker Compose deployment assets under `deployment/`.
- MCP server entry points for Codex/MCP-compatible clients.
- Hardware, ROS2 and no-hardware acceptance helpers with explicit external blockers.
- Local deterministic biped mountain demo under `biped_robot/`.

## Quick Start

Requirements:

- Python `>=3.10`
- Python `3.12` is recommended for local validation.

Install in editable mode:

```bash
pip install -e .
```

Install development dependencies:

```bash
pip install -e ".[dev]"
```

Run a quick environment check:

```bash
python -m agi_walker.cli doctor
```

List available skills:

```bash
python -m agi_walker.cli skills list
```

Start the Web Panel:

```bash
python -m web_panel.server
```

Default URL:

```text
http://localhost:8000
```

## Dynamic Godot Robot Generation

The dynamic Godot path converts robot JSON into a Godot mechanical node tree, validates the generated manifest, and can optionally compare static evidence against live Godot runtime evidence.

Primary guide:

- [Dynamic Godot Robot Generation](docs/guides/DYNAMIC_GODOT_ROBOT_GENERATION.md)

Useful local checks:

```powershell
py -3.12 tools\build_static_godot_node_tree_evidence.py tests\fixtures\robot_dynamic_fixed_pair.json tests\fixtures\robot_dynamic_biped.json tests\fixtures\robot_dynamic_quadruped.json --output-root test_env\static_godot_node_tree_manifest_ci --manifest-dir test_env\static_godot_node_tree_manifests
py -3.12 -m pytest tests\test_dynamic_godot_robot_generation.py -q
```

Optional live Godot smoke requires a local Godot executable:

```powershell
py -3.12 tools\run_dynamic_godot_robot_smoke.py tests\fixtures\robot_dynamic_biped.json --godot-exe "D:\迅雷下载\Godot\Godot.exe" --dry-run-discovery --live-profile local --live-artifact-root test_env\dynamic_godot_live\local --output test_env\dynamic_godot_live\local\discovery.json
```

## Local Biped Mountain Demo

`biped_robot/` is an isolated local demo workspace for a humanoid biped in a mountain environment. It has its own Godot project, deterministic simulation tools and hardware-free acceptance checks.

```powershell
py -3.12 biped_robot\tools\validate_biped_workspace.py
py -3.12 biped_robot\tools\run_local_acceptance.py
```

Open the standalone Godot demo:

```powershell
& "D:\迅雷下载\Godot\Godot.exe" --path biped_robot\godot
```

## Docker Compose Deployment

The supported local/customer deployment entry point is `deployment/docker-compose.yml`.

```powershell
Copy-Item deployment/compose.env.example deployment/compose.env
Copy-Item deployment/web_panel.env.example deployment/web_panel.env
docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml up -d --build zenoh-router web-panel
```

Deployment docs:

- [Deployment Matrix](docs/guides/DEPLOYMENT_MATRIX.md)
- [部署矩阵](docs/guides/DEPLOYMENT_MATRIX.md)
- [Customer Installation Guide](docs/guides/CUSTOMER_INSTALLATION_GUIDE.md)
- [客户安装指南](docs/guides/CUSTOMER_INSTALLATION_GUIDE.md)
- [Support Matrix](docs/guides/SUPPORT_MATRIX.md)
- [支持矩阵](docs/guides/SUPPORT_MATRIX.md)
- [容量与规模声明](docs/guides/CAPACITY_AND_SCALE.md)
- [客户验收清单](docs/guides/CUSTOMER_ACCEPTANCE_CHECKLIST.md)
- [已知限制](docs/guides/KNOWN_LIMITATIONS.md)
- [Production Deployment Runbook](PRODUCTION_DEPLOYMENT_RUNBOOK.md)
- [生产部署 Runbook](PRODUCTION_DEPLOYMENT_RUNBOOK.md)

## Active Operator and Delivery Links

The following paths are active user-facing or delivery-facing surfaces and are kept in the root README for discoverability:

- Web operator timeline: `/static/operator-history-timeline.html`
- [Instruction Control Demo Runbook](docs/guides/INSTRUCTION_CONTROL_DEMO_RUNBOOK.md)
- [Web Browser Manual Validation Checklist](docs/guides/WEB_BROWSER_MANUAL_VALIDATION_CHECKLIST_20260426.md)
- [Web Browser Manual Validation Template](deployment/web_browser_manual_validation.template.json)
- `tools/run_web_browser_playwright_smoke.py`
- `tools/build_web_browser_manual_validation_report.py`
- `tools/build_web_browser_validation_closeout.py`
- [Next Stage Execution Plan](docs/guides/NEXT_STAGE_EXECUTION_PLAN_20260426.md)

Release handoff evidence keeps these canonical fields visible for reviewers:

- `extension_execution_evidence`
- `extension_execution_instance`
- `extension_execution_schedule`
- `extension_execution_actuals`
- `approval_identity_source_path`
- `archive_target_binding_type`
- `due_trigger_binding_type`
- `due_trigger_checked_at`
- `closure_archive/index.json`

## MCP

Start the MCP server:

```bash
agi-walker-mcp
```

or:

```bash
python -m agi_walker.mcp.server
```

MCP documentation:

- [MCP Guide](docs/mcp.md)

## Validation

High-signal local checks:

```powershell
py -3.12 -m pytest tests\test_docs_utf8.py -q
py -3.12 -m pytest tests\test_mcp_tools.py tests\test_mcp_server.py -q
py -3.12 tests\run_smoke_tests.py
```

Dynamic Godot focused checks:

```powershell
py -3.12 -m pytest tests\test_dynamic_godot_robot_generation.py -q
py -3.12 -m pytest tests\test_workflow_contracts.py -q
py -3.12 -m pytest -m "not live" --collect-only -q
```

Security release preflight:

```powershell
py -3.12 tools\run_security_release_preflight.py --output-root test_env\release_evidence --run-python-vuln-scan --run-container-vuln-scan --container-image-ref deployment-zenoh-router --container-image-ref deployment-web-panel-distributed
```

## Repository Map

| Path | Purpose |
| --- | --- |
| `agi_walker/` | Core Python package, APIs, workflows, integrations and MCP server. |
| `web_panel/` | FastAPI Web Panel, static operator pages and Web APIs. |
| `godot_project/` | Main Godot project used by dynamic robot generation and smoke tests. |
| `biped_robot/` | Isolated mountain biped demo workspace with local Godot and hardware-free checks. |
| `tools/` | CLI tools for evidence generation, validation, release gates and smoke helpers. |
| `tests/` | Unit, contract, smoke and integration tests. |
| `deployment/` | Docker Compose, deployment env templates and security exception inputs. |
| `docs/` | User, operator, deployment, release, hardware, Web, Godot and architecture docs. |
| `plans/` | Codex project plan module subplans. |
| `.agi_data/` | Small tracked workflow/sample artifacts; runtime data should remain ignored. |
| `offline_data/`, `weights/` | Small sample/demo data and model-adjacent assets, not production model stores. |
| `archive/` | Historical snapshots and legacy planning notes. |

More detail:

- [Repository Layout](docs/REPOSITORY_LAYOUT.md)

## Documentation Index

Start here:

- [Documentation Index](docs/README.md)
- [Current Status](docs/CURRENT_STATUS.md)
- [Testing Guide](docs/guides/TESTING_GUIDE.md)
- [Web Panel Guide](docs/guides/WEB_PANEL_GUIDE.md)
- [Release Guide](docs/guides/RELEASE_GUIDE.md)
- [Known Limitations](docs/guides/KNOWN_LIMITATIONS.md)

Planning and Codex workflow:

- [Project Plan](PROJECT_PLAN.md)
- [Codex Working Rules](AGENTS.md)
- [Contributing Guide](CONTRIBUTING.md)
- [Security Policy](SECURITY.md)
- [Dynamic Godot Future Plan](docs/guides/DYNAMIC_GODOT_ROBOT_GENERATION_FUTURE_PLAN.md)

## License

[MIT](LICENSE)
