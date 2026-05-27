# Control Communication Simulation

This guide describes the first Phase 3 non-live evidence path for robot control and communication simulation.

## Scope

The current implementation proves the contract and deterministic artifact path only:

- canonical `control_message_envelope.v1`
- deterministic `control_comm_simulation_report.v1`
- local asyncio virtual bus metrics for latency, jitter, drop, duplicate and delivery order
- simulated Zenoh/OpenNeuro-like topic mapping over the same canonical envelope
- non-live EtherCAT cycle/PDO/watchdog model over the same virtual timing trace
- non-live motor/joint response model mapped to actuator, joint-limit and controller fields
- Gazebo/MuJoCo/Isaac Sim adapter boundary without runtime dependency
- fail-closed CAN/EtherCAT/TSN live hardware migration gate
- retained timing and message trace artifacts
- retained `zenoh_openneuro_topic_mapping.json` and `zenoh_simulated_trace.json`
- retained `ethercat_model_trace.json`
- retained `motor_joint_response_trace.json`
- retained `simulator_adapter_boundary.json`
- retained `live_hardware_migration_gate.json`
- retained `godot_control_comm_simulation_log.v1` contract preview
- Godot replay script at `godot_project/scripts/control_comm_replay.gd`
- Godot replay runner at `tools/run_godot_control_comm_replay.py`

It does not claim live Godot execution, real Zenoh transport, OpenNeuro compatibility, live EtherCAT validation, real motor driver validation, simulator adapter validation, or real hardware validation.

## Local Command

```powershell
py -3.12 tools\run_control_comm_simulation.py --output-root test_env\control_comm_simulation
```

Fault-injection example:

```powershell
py -3.12 tools\run_control_comm_simulation.py --output-root test_env\control_comm_simulation_faults --cycles 4 --cycle-period-ns 100 --jitter-budget-ns 5 --bus-latency-ns 3 --bus-jitter-by-cycle-ns "[0, 10, -2, 0]" --bus-drop-sequences "[2]" --bus-duplicate-sequences "[1]"
```

Expected artifacts:

- `control_comm_simulation_report.json`
- `control_comm_simulation_closeout.json` when the closeout command is run
- `timing_trace.json`
- `message_trace.json`
- `zenoh_openneuro_topic_mapping.json`
- `zenoh_simulated_trace.json`
- `ethercat_model_trace.json`
- `motor_joint_response_trace.json`
- `simulator_adapter_boundary.json`
- `live_hardware_migration_gate.json`
- `godot_replay_scenario.json`
- `godot_control_comm_simulation_log.json`

The Godot log artifact written by this command is a non-live contract preview with `status=not_run`. It exists so CI can validate the retained log shape before a Godot executable is available.

The local asyncio bus does not sleep on wall-clock time. It computes virtual delivery timestamps so tests remain deterministic while still recording scheduling and transport effects in the trace artifacts.

## Evidence Closeout

After generating the simulation report, build a reusable closeout artifact:

```powershell
py -3.12 tools\build_control_comm_simulation_closeout.py --report test_env\control_comm_simulation\control_comm_simulation_report.json
```

The closeout writes `control_comm_simulation_closeout.v1`. A healthy local simulation closeout uses `status=accepted_with_documented_external_blockers`, `evidence_level=non_live_simulation` and `closeout_validation_errors=[]`. It validates retained artifacts, records each present artifact's `size_bytes` and `sha256`, and carries live hardware blockers forward; it does not mark CAN, EtherCAT, TSN, external simulator or real Zenoh/OpenNeuro validation as passed.

## CI Evidence Profile

The default GitHub Actions workflow includes a non-live `control-communication-simulation-evidence` job after smoke tests. It runs:

```bash
python tools/run_control_comm_simulation.py --output-root test_env/control_comm_simulation_ci
python tools/build_control_comm_simulation_closeout.py --report test_env/control_comm_simulation_ci/control_comm_simulation_report.json --output test_env/control_comm_simulation_ci/control_comm_simulation_closeout.json
```

The job uploads `control-communication-simulation-artifacts` from `test_env/control_comm_simulation_ci`. This CI profile is deterministic and local-only: it does not install Godot, open Zenoh, start external simulators, use Docker or validate real CAN/EtherCAT/TSN hardware.

## Release Bundle Handoff

When a dynamic Godot release evidence bundle should include Phase 3 evidence, pass the closeout as an optional artifact:

```powershell
py -3.12 tools\build_dynamic_godot_release_evidence_bundle.py --static-closeout test_env\static_godot_node_tree_manifest_ci\static_godot_node_tree_evidence_closeout.json --delivery-gate test_env\static_godot_node_tree_manifest_ci\gate.json --readiness-summary test_env\dynamic_godot_release_readiness.json --control-comm-closeout test_env\control_comm_simulation_ci\control_comm_simulation_closeout.json --output-root test_env\dynamic_godot_release_evidence_bundle
```

The bundle validator treats this artifact as optional. If present, it must be a healthy `control_comm_simulation_closeout.v1` with `evidence_level=non_live_simulation`, no closeout or artifact errors, and `live_hardware_release_gate_status=blocked`.

## Zenoh/OpenNeuro-Like Simulation

The local command also writes a simulated Zenoh/OpenNeuro-like layer by default:

- `zenoh_openneuro_topic_mapping.json` maps the canonical envelope topic to a deterministic Zenoh key expression and OpenNeuro-like topic string.
- `zenoh_simulated_trace.json` mirrors every message trace event with the same envelope, delivery outcome, duplicate flag and fault injection label.

This layer is a serialization and routing contract check only. It keeps `transport_mode=zenoh_simulated` and `compatibility_claim=simulation_only`; it does not open a Zenoh session and does not claim OpenNeuro compatibility.

To suppress these artifacts for narrow debugging:

```powershell
py -3.12 tools\run_control_comm_simulation.py --output-root test_env\control_comm_simulation --no-zenoh-openneuro-simulation
```

## EtherCAT Cycle Model

The local command also writes `ethercat_model_trace.json` by default. This is a deterministic non-live model of:

- cycle period and virtual cycle jitter
- PDO input and output fields
- per-cycle deadline miss status
- missing PDO frame status when the upstream message is dropped
- watchdog state and simulated fault class

This layer keeps `transport_mode=ethercat_model` and `compatibility_claim=simulation_only`. It does not start an EtherCAT master, access a network interface or claim fieldbus hardware validation.

Useful options:

```powershell
py -3.12 tools\run_control_comm_simulation.py --output-root test_env\control_comm_simulation --ethercat-watchdog-timeout-cycles 2 --ethercat-deadline-budget-ns 500000
py -3.12 tools\run_control_comm_simulation.py --output-root test_env\control_comm_simulation --no-ethercat-cycle-model
```

## Motor/Joint Response Model

The local command also writes `motor_joint_response_trace.json` by default. This is a deterministic non-live model of:

- position, velocity and torque response
- velocity and torque saturation
- joint position limit clamping
- fault state propagated from the EtherCAT cycle model
- friction and backlash placeholders
- schema mapping back to actuator, joint-limit and controller fields

This layer keeps `compatibility_claim=simulation_only`. It does not drive a real motor controller and does not claim physical robot validation.

Useful options:

```powershell
py -3.12 tools\run_control_comm_simulation.py --output-root test_env\control_comm_simulation --motor-velocity-limit-rad-s 1.0 --motor-torque-limit-nm 2.0 --joint-position-lower-rad -1.0 --joint-position-upper-rad 1.0
py -3.12 tools\run_control_comm_simulation.py --output-root test_env\control_comm_simulation --no-motor-joint-model
```

## Simulator Adapter Boundary

The local command also writes `simulator_adapter_boundary.json` by default. This is a contract-only adapter boundary for:

- Gazebo
- MuJoCo
- Isaac Sim

The boundary declares which canonical artifacts an external simulator adapter must consume, including timing trace, message trace, EtherCAT model trace and motor/joint response trace. It also declares expected future output contracts such as simulator step and contact traces.

This artifact keeps `runtime_dependency_required=false`, each adapter `runtime_status=not_run`, and `compatibility_claim=adapter_contract_only`. It does not import or launch Gazebo, MuJoCo or Isaac Sim.

To suppress this artifact for narrow debugging:

```powershell
py -3.12 tools\run_control_comm_simulation.py --output-root test_env\control_comm_simulation --no-simulator-adapter-boundary
```

## Live Hardware Migration Gate

The local command also writes `live_hardware_migration_gate.json` by default. This is a fail-closed gate for future:

- CAN
- EtherCAT
- TSN

The artifact records required external evidence, hardware role permission and operator confirmation for each transport. It also records the local simulation artifacts that can support migration planning, while marking every simulation input as `accepted_for_live=false`.

This artifact keeps `status=blocked`, `release_gate.status=blocked`, `simulation_substitute_allowed=false` and `compatibility_claim=migration_gate_only`. It does not claim real CAN, EtherCAT or TSN transport validation.

To suppress this artifact for narrow debugging:

```powershell
py -3.12 tools\run_control_comm_simulation.py --output-root test_env\control_comm_simulation --no-live-hardware-migration-gate
```

## Godot Replay Script

Discovery can run without requiring Godot:

```powershell
py -3.12 tools\run_godot_control_comm_replay.py --dry-run-discovery --output-root test_env\godot_control_comm_replay
```

When a Godot executable is available, run the replay through the runner:

```powershell
py -3.12 tools\run_godot_control_comm_replay.py --godot-exe "<path-to-godot>" --scenario test_env\control_comm_simulation\godot_replay_scenario.json --output-root test_env\godot_control_comm_replay
```

Equivalent direct Godot command:

```powershell
godot --headless --path godot_project --script res://scripts/control_comm_replay.gd -- --control-comm-scenario <scenario.json> --control-comm-log <log.json>
```

The produced log must use the same envelope fields as the Python trace. The runner archives `godot_control_comm_simulation_log.json`, validates its shape and writes `godot_control_comm_replay_report.json`.
