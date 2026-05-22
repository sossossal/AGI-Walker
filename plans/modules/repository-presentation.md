# Module Goal

Make the GitHub-facing repository entry points easier to understand without changing runtime contracts or deleting tracked sample artifacts.

# Ownership

- `README.md`
- `docs/README.md`
- `docs/REPOSITORY_LAYOUT.md`
- `.gitattributes`
- `.github/PULL_REQUEST_TEMPLATE.md`
- `CONTRIBUTING.md`
- `CODE_OF_CONDUCT.md`
- `SECURITY.md`
- `PROJECT_PLAN.md`

# Contract Checklist

- Public surface this module exposes: GitHub root README and documentation index.
- Inputs this module accepts: existing docs, plans, package entry points, Web/Godot run commands and tracked sample directories.
- Outputs this module produces: concise root README, grouped docs index and explicit repository tree guidance.
- GitHub metadata this module produces: generated/sample-heavy path attributes and PR validation prompts.
- Public collaboration surfaces this module keeps aligned: contribution, conduct and security reporting entry points.
- Shared contracts touched: documentation entry points only.
- Backward compatibility requirements: do not rename or remove existing runtime, test, Web, Godot, deployment or evidence contract files.
- Integration tests required: documentation link/reference smoke where feasible.

# Non-Goals

- Do not delete tracked `.agi_data`, `offline_data`, `weights`, `archive` or coverage artifacts in this pass.
- Do not reorganize source directories.
- Do not change package, API, CLI, Web, Godot, Docker or CI behavior.

# Tasks

- [x] Shorten the root README into a GitHub-friendly quick entry.
- [x] Add a grouped docs index under `docs/README.md`.
- [x] Document top-level directories, including sample/artifact-like directories.
- [x] Record the cross-module documentation presentation change in `PROJECT_PLAN.md`.
- [x] Add GitHub presentation attributes for generated/sample-heavy paths.
- [x] Add a repository layout guide for first-time GitHub readers.
- [x] Align the PR template with AGENTS.md validation and residual-risk expectations.
- [x] Add a security policy and link it from public documentation entry points.
- [x] Refresh contribution/conduct guidance to avoid stale validation commands and placeholder contacts.

# Validation

```powershell
py -3.12 -m pytest tests\test_docs_utf8.py -q
py -3.12 -m pytest tests\test_active_path_references.py -q
```

# Residual Risks

- Some tracked sample/artifact directories remain visible in the GitHub tree; removal needs a separate compatibility review because tests and docs currently reference them.
