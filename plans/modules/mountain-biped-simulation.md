# Module Goal

Create a local, deterministic mountain-running simulation example for a humanoid biped robot, with reusable robot JSON, terrain/run evidence, tests and handoff docs.

# Ownership

- `configs/mountain_humanoid_biped.json`
- `configs/mountain_terrain.json`
- `examples/mountain_biped_simulation.py`
- `examples/mountain_biped/README.md`
- `godot_project/scripts/tcp_server.gd`
- `tools/run_dynamic_godot_robot_smoke.py`
- `tools/build_dynamic_robot_generation_report.py`
- `tests/test_mountain_biped_simulation.py`

# Inputs and Outputs

Inputs:

- Godot-compatible robot mechanical JSON.
- Mountain terrain parameters: seed, length, width, resolution, roughness, ridge height and step count.

Outputs:

- Machine-readable run report JSON under `test_env/mountain_biped/`.
- Optional full trace JSON with per-step position, terrain, gait, contact and stability fields.
- Console summary for local operator feedback.

# Contract Checklist

- Public surface this module exposes: CLI command `py -3.12 examples/mountain_biped_simulation.py`.
- Inputs this module accepts: robot config path, output path, optional trace path, seed, steps and terrain scale parameters.
- Outputs this module produces: `dynamic_mountain_biped_simulation.v1` report and `dynamic_mountain_biped_trace.v1` trace.
- Shared types/schemas/config touched: robot config uses dynamic Godot robot schema `1.5`.
- Backward compatibility requirements: no changes to existing schema, report builders or live Godot gates.
- Integration tests required: robot config validates, simulation report is deterministic and reports completed mountain running state.

# Local Context

The repository already has rough-terrain and slope-walking Gym demos plus dynamic Godot robot validation. Those examples are not a single local entry point that creates a humanoid biped, generates mountain terrain and records run evidence in one command.

# Non-Goals

- Do not require Godot, MuJoCo, PyBullet, GPU, browser or hardware for this local example.
- Do not modify dynamic Godot acceptance levels or make this example a release gate.
- Do not train a policy; the gait controller is a deterministic smoke controller for state simulation.

# Tasks

- [x] Add humanoid biped robot JSON with torso, head, arms, thighs, shins and feet.
- [x] Add deterministic mountain terrain and gait simulation CLI.
- [x] Add local run report and optional trace artifact contracts.
- [x] Add tests for config validation, determinism and output shape.
- [x] Run targeted and feasible non-live validation.
- [x] Add live Godot mountain terrain configuration and returned terrain evidence.

# Risks and Mitigations

- Risk: This could be mistaken for full physics proof.
  Mitigation: Report and docs state it is deterministic local simulation evidence, not live Godot physics verification.

- Risk: Future Godot integration needs terrain node support.
  Mitigation: Keep robot JSON schema-compatible and terrain/run evidence additive.

# Validation

```powershell
py -3.12 -m py_compile examples\mountain_biped_simulation.py
py -3.12 -m pytest tests\test_mountain_biped_simulation.py -q
py -3.12 examples\mountain_biped_simulation.py --output test_env\mountain_biped\mountain_biped_simulation_report.json --trace-output test_env\mountain_biped\mountain_biped_trace.json
py -3.12 -m pytest -q -rs
```

# Completion Criteria

- Local command creates the humanoid biped mountain simulation artifacts.
- Tests prove the robot config validates and simulation output is stable.
- Final handoff names the artifacts, skipped live checks and residual risk.

# Notes

- 2026-05-20: Added as an example module outside the existing dynamic Godot release gate scope.
- 2026-05-20: Ran live Godot full mechanical restoration acceptance for
  `configs/mountain_humanoid_biped.json` with `D:\迅雷下载\Godot\Godot.exe`.
  The live generation report and delivery gate passed at `godot_verified`, and
  `test_env/mountain_biped/live_godot_readiness.json` reported `status=ready`.
  Static-vs-runtime path-map mismatch count was `0`. Non-blocking mechanical
  behavior evidence reported `complete=false` with `joint_limit_violation` but
  no violation details; this remains a documented behavior-evidence risk.
- 2026-05-20: Added `configs/mountain_terrain.json` and live smoke
  `--terrain-json` support. The Godot TCP server now handles
  `configure_terrain`, creates `MountainTerrain` with `HeightMapShape3D` and
  `ArrayMesh`, and returns terrain evidence in live smoke observations.
- 2026-05-20: Re-ran live Godot full mechanical restoration acceptance with
  `--terrain-json configs\mountain_terrain.json`. The generation gate passed
  at `godot_verified`, readiness reported `status=ready`, and smoke evidence
  recorded `HeightMapShape3D`, `ArrayMesh`, `height_samples=2304` and
  `height_range_m≈0.694`.
- 2026-05-20: Reduced previously unrun local checks by installing `scipy` and
  `d3rlpy`, then re-ran the affected skills/offline-RL tests. Re-ran full
  pytest with explicit Godot headless smoke enabled:
  `1446 passed, 5 skipped`. Remaining skips required real hardware, ROS2
  runtime, or production compose execution; production compose follow-up moved
  to `plans/modules/production-compose-smoke.md`.

# Drift Check

The module is additive and preserves the root plan's dynamic Godot contracts. It does not redefine `static_only`, `godot_load_verified` or `godot_verified`.
