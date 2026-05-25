# Module Goal

Make the opt-in production Docker Compose smoke test executable against the repository's supported deployment entrypoint.

# Ownership

- `deployment/docker-compose.yml`
- `deployment/compose.env`
- `deployment/compose.env.example`
- `tests/test_prod_compose_smoke.py`

# Inputs and Outputs

Inputs:

- Host Docker Engine and Docker Compose v2.
- `AGI_WALKER_ENABLE_PROD_COMPOSE_SMOKE=1` for the live smoke test.
- Free host ports for Web, Redis and Zenoh.

Outputs:

- A temporary Compose project running `redis`, `zenoh-router`, `web-panel` and `workflow-worker`.
- Authenticated Web API workflow execution evidence from `robot_creation_pipeline`.

# Contract Checklist

- Public surface this module exposes: `deployment/docker-compose.yml` as the supported Docker Compose entrypoint.
- Inputs this module accepts: host port overrides and `AGI_WALKER_REDIS_URL`.
- Outputs this module produces: Web API status, auth token flow, terminal workflow run status and live log download.
- Shared contracts touched: Web workflow API, Celery broker configuration and deployment env template.
- Backward compatibility requirements: existing `zenoh-router`, `web-panel`, distributed profile services and host port overrides remain valid.
- Integration tests required: opt-in compose smoke must run the supported stack and clean up containers.

# Non-Goals

- Do not reintroduce root-level `docker-compose.prod.yml`; current docs identify `deployment/docker-compose.yml` as the supported entrypoint.
- Do not require real hardware, live Godot or ROS2 for this compose smoke.
- Do not add Prometheus/Grafana assertions unless those services are added to the supported compose stack.

# Tasks

- [x] Add Redis service for Celery broker/backend.
- [x] Add `workflow-worker` service using the Web Panel image and shared runtime volumes.
- [x] Run Alembic migrations before Web Panel startup so auth tables exist.
- [x] Add Redis env defaults to compose env files.
- [x] Update opt-in compose smoke to use supported services and current workflow response contract.
- [x] Run targeted live compose smoke.

# Validation

```powershell
py -3.12 -m py_compile tests\test_prod_compose_smoke.py
py -3.12 -m pytest tests\test_active_path_references.py -q
$env:AGI_WALKER_ENABLE_PROD_COMPOSE_SMOKE='1'; py -3.12 -m pytest tests\test_prod_compose_smoke.py -q -rs
```

# Notes

- 2026-05-21: The live compose smoke now starts the supported compose entrypoint, registers and logs in a Web user, starts `robot_creation_pipeline` in mock mode, waits for `completed`, verifies three completed steps and downloads the live log.
- 2026-05-21: Remaining full-suite skips after enabling Godot and production compose are expected to be limited to real hardware and real ROS2 runtime prerequisites.
