# Biped Robot Local Plan

## Scope

This folder is the isolated workspace for the biped humanoid robot mountain-environment prototype.

In scope:

- A standalone Godot 4 demo that renders a biped humanoid robot walking over generated mountain terrain.
- Local robot and terrain configuration files.
- Hardware-free deterministic simulation evidence.
- Public dataset-shaped offline replay evidence for Godot IO, communication, actuator and contact simulation.
- Part-parameter-driven system-level pure software simulation evidence.
- Sensor, scenario matrix, robot description mapping, fault injection, ROS2 topic contract, and Godot visual frame-summary simulation evidence.
- Local validation scripts and generated acceptance artifacts under this folder.

Out of scope:

- Changes to repository root plans, shared project configuration, existing Godot projects, hardware drivers, ROS2 integrations, CI, or deployment files.
- Real motor, sensor, serial, CAN, or ROS2 validation.

## Ownership Boundary

All task files are owned by `biped_robot/`.

Changing files outside this directory requires explicit user approval.

## Contracts

- `config/robot.json` is the local biped robot contract.
- `config/mountain_terrain.json` is the local mountain terrain contract.
- `godot/project.godot` is a standalone Godot project.
- `godot/scenes/biped_mountain_demo.tscn` is the entry scene.
- `config/godot_io_input.json` is the Godot-side command/input contract.
- `tools/validate_godot_io.py` validates Godot-generated telemetry and summary output.
- `tools/simulate_biped.py` writes deterministic JSON evidence for hardware-free behavior.
- `tools/validate_biped_workspace.py` checks the local folder contract.
- `tools/check_contact_stability.py` checks terrain contact and stability margins from the deterministic trace.
- `tools/simulate_actuator_physics.py` runs a deterministic actuator-level physics approximation with torque, current, thermal, saturation, energy, and tracking telemetry.
- `tools/build_component_parameter_log.py` joins body segment, joint, actuator, and observed simulation metrics into a per-component parameter log.
- `tools/build_retention_manifest.py` records generated evidence files, hashes, schemas, and retention policy.
- `tools/build_hardware_gap_report.py` records unavailable hardware, serial/CAN, and ROS2 checks as explicit blocked gates.
- `config/public_real_data_sources.json` records public source metadata used for offline replay evidence.
- `fixtures/public_real_data_replay_fixture.json` stores small normalized public dataset-shaped samples for deterministic replay.
- `tools/run_public_real_data_replay.py` maps public dataset-shaped samples into local Godot IO, communication, actuator and contact evidence without claiming hardware validation.
- `tools/run_part_driven_system_simulation.py` aggregates component parameters, actuator response, terrain/contact, Godot IO, communication, public replay, and hardware validation options into one software readiness report.
- `tools/validate_part_driven_system_simulation.py` validates the system report, retained trace, and hardware-blocked semantics.
- `config/sensor_model.json`, `config/scenario_matrix.json`, `config/fault_injection.json`, and `config/ros2_topic_contract.json` define simulator-grade software capability contracts.
- `tools/run_sensor_simulation.py`, `tools/run_scenario_matrix.py`, `tools/export_robot_description_mapping.py`, `tools/run_fault_injection.py`, `tools/run_ros2_topic_contract_simulation.py`, and `tools/run_godot_visual_acceptance.py` generate additional software simulation evidence.
- `tools/build_simulation_capability_upgrade_report.py` and `tools/validate_simulation_capability_upgrade.py` aggregate and validate the simulation capability upgrade.
- `tools/run_local_acceptance.py` runs the folder-scoped acceptance suite.
- `contracts/hardware_interface.json` defines the future hardware boundary without changing shared hardware modules.

## Verification

Target checks:

- Python syntax check for local tools.
- Local deterministic biped simulation report generation.
- Local workspace contract validation.
- Godot headless launch if a local Godot executable is available.
- Godot input/output simulation with generated telemetry and summary validation.
- Folder-scoped acceptance report generation.
- Hardware gap report generation for unavailable live checks.
- Actuator physics simulation report and telemetry retention manifest.
- Per-component parameter log generation.
- Public real-data replay report and retained trace generation.
- Part-parameter-driven system simulation report and retained trace generation.
- System report contract validation, including hardware blocked and command-disabled checks.
- Sensor simulation, scenario matrix, robot description mapping, fault injection, ROS2 topic contract simulation, Godot visual frame-summary acceptance, and aggregate capability upgrade validation.

Regression checks:

- Repository-wide regression is intentionally replaced by `tools/run_local_acceptance.py` for this isolated folder.
- Running shared repository tests or CI requires explicit approval because it validates modules outside `biped_robot/`.

## Acceptance Criteria

- The Godot project opens from `biped_robot/godot` and starts at the mountain biped demo scene.
- The demo creates a visible mountain terrain, camera, lighting, and animated humanoid biped.
- The Godot demo reads `config/godot_io_input.json` and writes `test_env/godot_io_telemetry.jsonl` plus `test_env/godot_io_report.json`.
- Local scripts generate acceptance evidence without hardware.
- Actuator telemetry includes command, measured position, velocity, torque, current, temperature, power, and saturation state per simulated joint.
- Component parameter logs include segment dimensions, mass, joint limits, actuator model constants, and latest observed actuator metrics.
- Generated acceptance artifacts are indexed with hashes in `test_env/retention_manifest.json`.
- Public real-data replay artifacts are indexed with hashes in `test_env/retention_manifest.json`.
- Part-driven system simulation artifacts are indexed with hashes in `test_env/retention_manifest.json`.
- Simulation capability upgrade artifacts are indexed with hashes in `test_env/retention_manifest.json`.
- Validation fails closed if required local files or contracts are missing.
- Missing hardware, serial/CAN, or ROS2 evidence is reported as blocked with prerequisites instead of being treated as passed.

## Residual Risks

- Full rigid-body actuator dynamics remain deferred until the user approves hardware or shared engine integration work; this folder now includes a deterministic actuator approximation for controller and data-retention rehearsal.
- Hardware behavior remains blocked, with explicit prerequisites recorded in `test_env/hardware_gap_report.json`.
- Terrain contact is still approximate, but now has a local trace-based contact and stability gate.
- Public real-data replay uses offline normalized samples and source metadata; it is not a substitute for live hardware, real serial/CAN, or ROS2 runtime evidence.
- The system-level software report aggregates existing deterministic/Godot evidence and remains lower fidelity than hardware-in-the-loop or full rigid-body simulation.
- New simulator-grade capability reports remain software-only; URDF/SDF/MJCF mappings are previews, ROS2 topics are contract simulations, and visual acceptance uses telemetry-derived frame summaries by default.

## Decision Log

- 2026-05-21: Keep all new work inside `biped_robot/` to honor the user constraint. Do not update root `PROJECT_PLAN.md` or shared module plans without approval.
- 2026-05-21: Use a standalone Godot project instead of modifying the existing project under `godot_project/`.
- 2026-05-21: Add folder-scoped acceptance because repository-wide regression would exceed the approved edit and validation boundary.
- 2026-06-12: Add public dataset-shaped offline replay evidence after user approval for project-level plan integration; keep real hardware and ROS2 blockers unchanged.
- 2026-06-12: Add part-parameter-driven system software simulation closeout before hardware work; hardware remains an optional blocked validation lane.
- 2026-06-12: Add simulator-grade software capability upgrade evidence for sensors, scenario matrix, description mapping, faults, ROS2 topic contracts and Godot visual frame summaries without external simulator or hardware runtime dependencies.
