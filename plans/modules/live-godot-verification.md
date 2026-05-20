# Module Goal

把 manual live Godot smoke 升级为可复用的本地、手动和定时 CI 验证 profile，输出可归档、可诊断、可被 release/readiness 工具消费的 live evidence。

# Ownership

- `tools/run_dynamic_godot_robot_smoke.py`
- `tools/build_dynamic_robot_generation_report.py`
- `.github/workflows/ci.yml`
- Godot executable discovery and environment profile documentation
- live smoke artifacts under `test_env/` and CI uploaded artifacts

# Inputs and Outputs

Inputs:

- Robot fixture JSON and normalized Godot-ready config.
- Optional user-provided Godot executable path.
- Static node-tree manifest and report/gate settings.

Outputs:

- Live smoke JSON with executable resolution, profile name, environment mode and artifact paths.
- Classified failure details for missing executable, launch failure, load failure, timeout, runtime mismatch, motion failure and flaky retry exhaustion.
- Retained report, gate, smoke and readiness artifacts for local/manual/scheduled runs.

# Contract Checklist

- Public surface this module exposes: live verification profile names, CLI flags, smoke JSON fields, CI artifact names.
- Inputs this module accepts: fixture paths, Godot executable path, profile mode, retry/flaky settings, artifact output root.
- Outputs this module produces: smoke report, delivery gate, readiness summary and retention metadata.
- Shared types/schemas/config touched: delivery gate metadata, release/readiness evidence, CI workflow config.
- Backward compatibility requirements: live validation remains opt-in and must not become required for default PR static CI.
- Integration tests required: dry-run/discovery tests, report/gate contract tests, scheduled/manual workflow shape tests.

# Local Context

Current docs already define a manual live smoke checklist and required `godot_verified` fields. The next increment should turn that checklist into a reusable profile while preserving static CI as the mandatory default.

# Non-Goals

- Do not require Godot on every PR.
- Do not make flaky live runs silently pass.
- Do not replace static manifest gates.

# Tasks

- [x] Define live verification profile contract: local, manual CI, scheduled CI.
- [x] Add Godot executable discovery metadata and explicit failure category output.
- [x] Add artifact retention metadata for report, gate, smoke and readiness outputs.
- [x] Define retry/flaky policy and expose final flaky classification in smoke evidence.
- [x] Add CI workflow entry or documented scheduled/manual profile without changing default PR requirements.
- [x] Add tests for discovery failure, dry-run profile output and artifact metadata.
- [x] Update live smoke documentation and release/readiness references.

# Risks and Mitigations

- Risk: Local machines and CI images expose Godot differently.
  Mitigation: Record executable resolution source and distinguish missing executable from runtime failures.

- Risk: Retry policy hides real failures.
  Mitigation: Keep all retry attempts in retained artifacts and report final classification separately from raw failures.

# Validation

```powershell
py -3.12 -m py_compile tools\run_dynamic_godot_robot_smoke.py tools\build_dynamic_robot_generation_report.py
py -3.12 -m pytest tests\test_dynamic_godot_robot_generation.py -q
py -3.12 -m pytest -m "not live" --collect-only -q
```

When Godot is available, run the documented live profile and archive report, gate, smoke and readiness artifacts.

# Completion Criteria

- Live verification has a documented reusable profile.
- Missing executable and runtime failures are classified differently.
- Artifacts are retained with stable paths and metadata.
- Default non-live CI behavior remains unchanged.

# Notes

- Start with dry-run/discovery behavior so the profile is testable without Godot.
- 2026-05-19: Added `--live-profile`, discovery metadata, artifact retention metadata, flaky policy metadata, and `--dry-run-discovery` to the smoke runner; report builder now forwards the profile fields.
- 2026-05-19: Added manual/scheduled `dynamic-godot-live-verification` CI job. It always uploads discovery artifacts and only runs full live verification when `GODOT_EXECUTABLE` is provided.
- 2026-05-19: Ran the local live profile against `tests/fixtures/robot_dynamic_biped.json` with `D:\迅雷下载\Godot\Godot.exe`. Discovery succeeded, the full mechanical restoration live report/gate passed at `godot_verified`, and readiness summary reported `status=ready`. Artifacts were written under `test_env/dynamic_godot_live/local/`.
- 2026-05-19: Readiness summary accepts the live generation report and gate as evidence inputs; raw smoke JSON is retained as a required artifact but is not a direct readiness input.
- 2026-05-19: Fixed static-vs-runtime manifest comparison for live Godot absolute NodePath output. Runtime paths such as `/root/RLServer/<robot>/<node>` now compare by suffix against the static manifest's robot-relative node paths for node path fields only; class, transform, physical and parameter comparisons remain exact/tolerance-based.
- 2026-05-19: Re-ran local live profiles for the golden fixture set `robot_dynamic_biped.json`, `robot_dynamic_fixed_pair.json` and `robot_dynamic_quadruped.json` with `D:\迅雷下载\Godot\Godot.exe`. Each report/gate passed at `godot_verified`, each static manifest comparison reported `mismatch_count=0`, and `test_env/dynamic_godot_live/local/dynamic_godot_golden_live_readiness.json` reported `status=ready` and `proven_level=godot_verified`.
- 2026-05-19: Changed the report builder's live smoke default from a fixed TCP port to auto-selected free localhost ports. Explicit `--port <port>` still preserves fixed-port diagnostics, while local/manual CI profiles avoid accidental same-port collisions when multiple smoke reports run near each other.
- 2026-05-19: Added report-level live smoke attempt metadata and one retry for automatic-port TCP startup response failures. Fixed-port diagnostics remain single-attempt so explicit port conflicts stay visible.
- 2026-05-20: Direct smoke runner now writes structured error reports for Godot launch failures, TCP startup timeouts and unexpected runtime command failures. Report-builder auto-port retry also recognizes structured `godot_tcp_timeout` smoke reports.
- 2026-05-20: Report-builder live evidence now mirrors wrapper-level attempt count, max attempts and retry result into `godot_smoke.live_verification.flaky_policy`, so retained report artifacts expose `passed_after_retry` or `failed_after_retry` instead of only the final child smoke process classification.
- 2026-05-20: Report-builder wrapper retries now also synchronize the final retained smoke JSON's `live_verification.flaky_policy` fields with the report-level evidence, preventing archived smoke artifacts from showing stale child-process retry metadata.
- 2026-05-20: Retained smoke JSON now stores wrapper retry attempt summaries in `live_verification.flaky_policy.wrapper_attempts` after wrapper retries, so archived smoke evidence remains self-contained even without the parent generation report.

# Drift Check

Before implementation, verify this module still supports `PROJECT_PLAN.md` Phase 2 live verification scope and does not make live Godot mandatory for default PR CI.
