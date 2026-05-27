# Module Goal

Build a staged, hardware-safe robot control and communication simulation stack: deterministic Python/asyncio timing first, Godot script/log replay for retained simulation evidence, then Zenoh/OpenNeuro-like transport simulation, EtherCAT cycle modeling, motor/joint physical response, optional simulator adapters, and finally explicit live CAN/EtherCAT/TSN migration gates.

# Ownership

- Future `agi_walker/core/simulation/` timing and trace modules.
- Future `agi_walker/core/api/comm/` canonical message envelope and local bus adapters.
- Future Godot-side control communication replay script and retained structured log artifacts.
- Existing `agi_walker/core/api/comm/zenoh_interface.py`.
- Existing `agi_walker/core/api/comm/tcp_zenoh_bridge.py`.
- Future `agi_walker/core/fieldbus/` EtherCAT cycle/PDO/watchdog model.
- Future `agi_walker/core/actuation/` motor and joint response model.
- Future `agi_walker/core/sim_adapters/` Gazebo, MuJoCo and Isaac Sim adapters.
- Tests under future `tests/test_control_communication_simulation.py` and focused adapter tests.
- Related documentation under `docs/hardware/` and `docs/guides/` when behavior becomes user-visible.
- Related live/headless Godot smoke tooling when Godot execution becomes available.

# Inputs and Outputs

Inputs:

- Robot schema 1.5 actuator, sensor, joint-limit, controller and material/physics metadata when available.
- Deterministic virtual clock configuration: cycle period, duration, jitter/drop/fault injection and deadline budget.
- Local asyncio message endpoints and future Zenoh/OpenNeuro-like topic definitions.
- Godot script/headless replay configuration using the same canonical message envelope and scenario fixture.
- Future EtherCAT PDO map, cycle period, watchdog and fault-class configuration.
- Future simulator adapter configuration for Gazebo, MuJoCo or Isaac Sim.
- Future live transport evidence for CAN/EtherCAT/TSN only when explicitly supplied.

Outputs:

- `control_comm_simulation_report.v1` with status, transport mode, clock mode, timing metrics, message integrity metrics, trace paths and residual risks.
- `godot_control_comm_simulation_log.v1` with script/profile metadata, replay scenario identity, cycle/message events, status, artifact paths and residual risks.
- Deterministic timing trace artifacts recording cycle index, expected timestamp, actual timestamp, jitter, deadline status and task outcome.
- Message trace artifacts recording topic, sequence, source, target, timestamp, payload type, delivery outcome and fault injection result.
- Godot script log artifacts recording the same envelope fields plus Godot node path or script source when available.
- EtherCAT model trace artifacts recording cycle, PDO inputs/outputs, deadline miss, watchdog state and simulated fault class.
- Motor/joint response trace artifacts recording command, position, velocity, torque, saturation, limit state and fault state.
- `live_hardware_migration_gate.v1` recording CAN/EtherCAT/TSN external evidence requirements and fail-closed release blockers.

# Contract Checklist

- Public surface this module exposes: future Python APIs, CLI/report builders and Godot replay script/log artifact contracts for deterministic simulation evidence.
- Inputs this module accepts: local config dictionaries/files for clock, transport, Godot replay scenario, EtherCAT model and motor/joint model; future robot schema actuator metadata.
- Outputs this module produces: versioned reports, Python trace artifacts and Godot structured log artifacts that can be included in release evidence without claiming live hardware validation.
- Shared types/schemas/config touched: robot schema 1.5 optional actuator/controller/joint-limit metadata, hardwareless acceptance evidence, distributed/Zenoh transport contracts.
- Backward compatibility requirements: no existing Godot, distributed smoke, hardwareless acceptance or release evidence contract may be weakened.
- Integration tests required: virtual clock determinism, message envelope validation, asyncio bus ordering/drop/jitter, Godot log artifact shape, Zenoh adapter shape, EtherCAT cycle/watchdog model and motor/joint response trace.

# Local Context

- Existing distributed runtime lives under `agi_walker/core/distributed/`.
- Existing Zenoh-related code lives under `agi_walker/core/api/comm/`.
- Existing hardwareless acceptance keeps real hardware validation blocked without external evidence.
- Existing dynamic Godot and mechanical behavior evidence already tracks joint limits, torque/velocity response and step traces; Phase 3 should reuse those concepts rather than invent incompatible names.
- Existing live Godot verification already has executable discovery, profile metadata and artifact retention patterns; Phase 3 should reuse that evidence style for Godot script/headless replay.
- Current project has no dedicated deterministic control-loop simulation contract, no canonical message envelope shared across asyncio/Zenoh/future fieldbus, and no EtherCAT cycle model.
- Current Phase 3 plan must not stop at Python traces; Godot-side replay logs are required before claiming Godot simulation evidence.

# Non-Goals

- Do not connect to real CAN, EtherCAT or TSN hardware in the first implementation stage.
- Do not require Gazebo, MuJoCo, Isaac Sim, GPU, ROS2 or real-time OS support for local CI.
- Do not claim OpenNeuro compatibility until an explicit topic/payload contract is defined and tested.
- Do not replace existing Godot dynamic generation or hardwareless acceptance contracts.
- Do not use wall-clock timing as the primary correctness oracle for deterministic unit tests.
- Do not require Godot executable availability for default non-live CI; artifact-shape validation must remain possible without Godot.

# Tasks

- [x] Define `control_comm_simulation_report.v1` and canonical message envelope fields.
- [x] Add deterministic virtual clock and periodic task scheduler design.
- [x] Add local asyncio bus design with ordering, latency, jitter, drop and duplicate simulation.
- [x] Add trace schema for cycle timing and message delivery.
- [x] Add first local fixture scenario: controller command stream to simulated joint endpoint with deterministic timing.
- [x] Add Godot replay script/log contract using the same fixture scenario and canonical envelope.
- [x] Add retained Godot log artifact shape validation for `godot_control_comm_simulation_log.v1`.
- [x] Add optional Godot headless/live smoke plan that runs the replay script and archives logs when executable discovery succeeds.
- [x] Add Zenoh/OpenNeuro-like topic mapping plan over the canonical envelope.
- [x] Add EtherCAT cycle model plan: cycle period, PDO input/output, deadline miss, watchdog and fault class.
- [x] Add motor/joint model plan: position, velocity, torque, limits, saturation, friction/backlash placeholders and fault state.
- [x] Define simulator adapter boundary for Gazebo, MuJoCo and Isaac Sim without runtime dependency.
- [x] Define live CAN/EtherCAT/TSN migration gates and required external evidence.
- [x] Add documentation links once APIs or CLI become user-visible.

# Risks and Mitigations

- Risk: Simulated timing is mistaken for real-time hardware proof.
  Mitigation: Every report must include `clock_mode`, `transport_mode` and explicit residual risks; live hardware mode remains blocked without external evidence.

- Risk: Wall-clock tests are flaky on CI.
  Mitigation: Use deterministic virtual clock for unit and contract tests; real-time smoke stays opt-in.

- Risk: Zenoh, EtherCAT and simulator adapters diverge into incompatible payloads.
  Mitigation: Define one canonical envelope first and keep adapters as serialization/transport layers.

- Risk: Godot replay becomes a second, divergent simulation instead of a view of the same scenario.
  Mitigation: Godot replay must consume or emit the same scenario identity and envelope fields as the Python trace, and validation must compare the artifact shape before aggregation.

- Risk: Scope expands into hardware driver implementation too early.
  Mitigation: Phase 3 starts with simulation contracts and no-hardware evidence; real transport gates are planned but not implemented until local simulation is stable.

# Validation

Initial planning validation:

```powershell
py -3.12 -m py_compile agi_walker\core\api\comm\zenoh_interface.py agi_walker\core\api\comm\tcp_zenoh_bridge.py
py -3.12 -m pytest tests\test_distributed_smoke_runner.py tests\test_hardwareless_acceptance_report.py -q
```

Implemented non-live validation:

```powershell
py -3.12 -m pytest tests\test_control_communication_simulation.py -q
py -3.12 -m pytest tests\test_godot_control_comm_simulation_artifacts.py -q
py -3.12 tools\run_godot_control_comm_replay.py --dry-run-discovery --output-root test_env\godot_control_comm_replay
py -3.12 -m pytest tests\test_hardwareless_acceptance_report.py -q
py -3.12 -m pytest -m "not live" --collect-only -q
```

When Godot is available, the opt-in headless replay command archives `godot_control_comm_simulation_log.v1` and validates the retained log shape against the canonical envelope contract.

# Completion Criteria

- Root plan records Phase 3 scope, cross-module contracts, risks and acceptance criteria.
- Module plan defines staged work from deterministic asyncio simulation to live hardware gates.
- Godot-side script/log replay is a required evidence lane for simulation results that claim Godot participation.
- Local implementation proceeds without requiring external simulators or hardware.
- No existing dynamic Godot, distributed, hardwareless or security release gates are weakened.
- The approved non-live Phase 3 scope is complete; external simulator validation and real CAN/EtherCAT/TSN hardware validation remain separate evidence lanes.

# Notes

- 2026-05-26: Module plan created from requested staged roadmap: Python/asyncio timing, Zenoh/OpenNeuro-like communication, EtherCAT cycle model, motor/joint model, Gazebo/MuJoCo/Isaac Sim adapters, and eventual CAN/EtherCAT/TSN hardware migration.
- 2026-05-26: Treat "OpenNeuro" as an OpenNeuro-like communication contract placeholder until topic names, payload schema and compatibility target are specified.
- 2026-05-26: User clarified simulation must also run or replay through Godot scripts/logs and retain real generated information/results as artifacts.
- 2026-05-26: Added `agi_walker/core/simulation/control_comm_simulation.py`, `tools/run_control_comm_simulation.py`, `godot_project/scripts/control_comm_replay.gd`, `tests/test_control_communication_simulation.py`, `tests/test_godot_control_comm_simulation_artifacts.py` and `docs/guides/CONTROL_COMMUNICATION_SIMULATION.md`. This closed the first non-live contract slice and established the shared report, envelope and Godot log contracts.
- 2026-05-26: Extended the local bus slice with deterministic asyncio transport simulation for latency, per-cycle jitter, dropped sequences, duplicated sequences, delivery order, deadline misses and message integrity metrics.
- 2026-05-26: Added `tools/run_godot_control_comm_replay.py` to run dry-run discovery by default and execute `godot_project/scripts/control_comm_replay.gd` when a Godot executable is supplied. Tests cover missing executable discovery and retained log validation through a mocked successful execution.
- 2026-05-26: Local Godot 4.5.1 headless replay succeeded with `test_env/godot_control_comm_replay_live/godot_control_comm_replay_report.json`, `status=success`, `message_event_count=4` and `log_validation_errors=[]`. Real hardware transport remains not run.
- 2026-05-26: Added simulated Zenoh/OpenNeuro-like mapping and trace artifacts over the canonical envelope. This is a deterministic routing/serialization contract only: no real Zenoh session is opened and OpenNeuro compatibility is not claimed.
- 2026-05-26: Added non-live EtherCAT cycle model trace over virtual timing and message traces. It records PDO inputs/outputs, deadline misses, missing frames, watchdog state and fault class without starting a real EtherCAT master or claiming hardware validation.
- 2026-05-26: Added non-live motor/joint response trace over EtherCAT model output. It records command, position, velocity, torque, saturation, limit state, friction/backlash placeholders and fieldbus-derived fault state mapped to schema actuator, joint-limit and controller concepts.
- 2026-05-26: Added simulator adapter boundary artifact for Gazebo, MuJoCo and Isaac Sim. It declares canonical input/output contracts only; all adapters remain `not_run` and no runtime dependency is introduced.
- 2026-05-26: Added live hardware migration gate artifact for CAN, EtherCAT and TSN. It remains `blocked` with `simulation_substitute_allowed=false` until external hardware evidence, hardware role permission and operator confirmation are supplied.
- 2026-05-26: Closed approved non-live Phase 3 module scope after contract, CLI, Godot replay, simulator adapter boundary, live migration gate, tests and docs were added. Remaining live hardware and external simulator execution is not claimed by this module.
- 2026-05-26: Active-path tests now lock the control/communication guide's non-live commands, report/closeout/log contracts, bundle handoff flag and live hardware blocked boundary.

# Drift Check

Before implementation, verify this module still matches `PROJECT_PLAN.md`: deterministic simulation before live hardware, Godot script/log evidence before Godot simulation claims, canonical contract before adapters, no hard dependency on external simulators, and fail-closed hardware acceptance.
