# Module Goal

Make the GitHub-facing repository entry points easier to understand without changing runtime contracts or deleting tracked sample artifacts.

# Ownership

- `README.md`
- `docs/README.md`
- `docs/REPOSITORY_LAYOUT.md`
- `.gitattributes`
- `.github/PULL_REQUEST_TEMPLATE.md`
- `.github/ISSUE_TEMPLATE/*.md`
- `.github/ISSUE_TEMPLATE/config.yml`
- `.github/CODEOWNERS`
- `CONTRIBUTING.md`
- `SUPPORT.md`
- `CODE_OF_CONDUCT.md`
- `SECURITY.md`
- `PROJECT_PLAN.md`

# Contract Checklist

- Public surface this module exposes: GitHub root README and documentation index.
- Inputs this module accepts: existing docs, plans, package entry points, Web/Godot run commands and tracked sample directories.
- Outputs this module produces: concise root README, grouped docs index and explicit repository tree guidance.
- GitHub metadata this module produces: generated/sample-heavy path attributes and PR validation prompts.
- Public collaboration surfaces this module keeps aligned: contribution, conduct and security reporting entry points.
- Issue triage surfaces this module keeps aligned: bug, feature and nightly regression templates plus issue contact links.
- Governance metadata this module keeps aligned: CODEOWNERS routing for GitHub review ownership.
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
- [x] Align issue templates with module scope, contract impact, validation evidence and residual-risk triage.
- [x] Add issue template config to route security reports and docs questions to the right entry points.
- [x] Add a support policy that routes usage, bug, release, live-environment and security requests.
- [x] Add CODEOWNERS routing for source, docs, deployment, Godot, hardware and GitHub governance surfaces.

# Validation

```powershell
py -3.12 -m pytest tests\test_docs_utf8.py -q
py -3.12 -m pytest tests\test_active_path_references.py -q
```

# Residual Risks

- Some tracked sample/artifact directories remain visible in the GitHub tree; removal needs a separate compatibility review because tests and docs currently reference them.

# Notes

- 2026-05-24: PR CI showed README optimization had removed active operator/delivery links that are protected by docs contract tests. Root README now keeps a compact active-link block for those canonical tools, templates and guides while preserving the concise GitHub entry structure.
- 2026-05-26: Root README now exposes the Phase 3 control/communication simulation guide plus non-live report/closeout commands so the completed simulation evidence lane is discoverable from the GitHub entry point.
- 2026-05-26: Active-path tests now protect the Phase 3 control/communication guide link in both the root README and `docs/README.md` documentation index.
- 2026-05-26: Active-path tests now verify `PROJECT_PLAN.md` Module Index rows point to existing complete subplans with no unchecked module tasks.
- 2026-05-26: Active-path tests now also verify every `plans/modules/*.md` subplan is indexed by `PROJECT_PLAN.md` so module plans cannot become orphaned.
- 2026-05-26: Active-path tests now require each `PROJECT_PLAN.md` Module Index module name to match its subplan filename stem.
- 2026-05-26: Active-path tests now protect required `PROJECT_PLAN.md` governance sections, including `Decision Log` and `Change Control`.
- 2026-05-26: Active-path tests now prevent unchecked root `PROJECT_PLAN.md` tasks from reappearing while the Module Index is complete.
- 2026-05-26: Active-path tests now require Module Index rows to keep non-empty module, responsibility and dependency cells plus canonical backticked `plans/modules/*.md` subplan paths.
- 2026-05-26: Active-path tests now prevent complete module plans from keeping stale implementation-before wording that implies unfinished increments.
- 2026-05-26: Active-path tests now prevent complete module plans from keeping stale wording that implies another immediate implementation step after root scope is closed.
- 2026-05-26: Active-path tests now require `PROJECT_PLAN.md` Integration Plan numbering to remain contiguous.
