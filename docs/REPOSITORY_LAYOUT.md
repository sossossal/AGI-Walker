# Repository Layout

This page explains the top-level repository tree for GitHub readers and release reviewers. It is presentation guidance only; it does not change runtime contracts.

## Primary Source Areas

| Path | Role |
| --- | --- |
| `agi_walker/` | Core Python package, API contracts, workflow orchestration, integrations and MCP server code. |
| `web_panel/` | FastAPI Web Panel, operator pages, workflow APIs and Godot session surfaces. |
| `godot_project/` | Main Godot project for dynamic robot generation and smoke validation. |
| `biped_robot/` | Isolated mountain biped demo with local Godot and hardware-free acceptance checks. |
| `tools/` | Release, validation, evidence, Godot smoke, security and acceptance CLI tools. |
| `tests/` | Unit, contract, smoke and integration tests. |
| `deployment/` | Docker Compose deployment entry points, env templates and release/security inputs. |
| `docs/` | User, operator, deployment, release, hardware, Web, Godot and architecture documentation. |
| `plans/` | Codex project plan module subplans. |

## Generated, Sample and Historical Areas

Some tracked directories look like local artifacts but remain in Git because tests, docs or release evidence reference small sample files from them.

| Path | Why It Exists | Handling Policy |
| --- | --- | --- |
| `.agi_data/` | Small workflow/sample artifacts used by examples and smoke coverage. | Keep small curated files; runtime sessions and large artifacts stay ignored. |
| `offline_data/` | Small mock datasets for local and no-hardware flows. | Keep compatibility samples; do not treat as production data storage. |
| `weights/` | Small model-adjacent demo outputs and mock artifacts. | Keep only lightweight fixtures; large model binaries remain ignored. |
| `archive/` | Historical snapshots and legacy planning notes. | Keep for audit context; first-run docs should link through `README.md` or `docs/README.md`. |
| `htmlcov/`, `coverage.xml`, `.coverage` | Legacy tracked coverage outputs. | Marked as generated in `.gitattributes`; removal needs separate compatibility review. |
| `godot_project/.godot/` | Godot editor cache files currently present in history. | Marked as generated in `.gitattributes`; new cache files are ignored. |

## GitHub Presentation Rules

- Root `README.md` stays short and points to the most important runnable paths.
- `docs/README.md` is the grouped documentation index.
- Public collaboration entry points live at `CONTRIBUTING.md`, `CODE_OF_CONDUCT.md`, `SECURITY.md`, `.github/PULL_REQUEST_TEMPLATE.md` and `.github/ISSUE_TEMPLATE/`.
- `.gitattributes` marks generated/sample-heavy paths so GitHub language statistics and source archives are less noisy.
- `.gitignore` prevents new local runtime data, caches and large artifacts from entering future commits.
- Do not delete tracked sample/evidence directories without a dedicated compatibility review and path-reference validation.
