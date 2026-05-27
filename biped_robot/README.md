# Biped Robot Mountain Demo

This folder is an isolated Godot and hardware-free simulation workspace for a biped humanoid robot walking in a mountain environment.

## Contents

- `config/robot.json`: local biped robot structure and gait limits.
- `config/mountain_terrain.json`: mountain terrain generation parameters.
- `godot/`: standalone Godot 4 project.
- `tools/simulate_biped.py`: deterministic local simulation evidence generator.
- `tools/validate_biped_workspace.py`: local contract validator.
- `test_env/`: generated local acceptance artifacts.

## Run Local Simulation

```powershell
py -3.12 biped_robot\tools\simulate_biped.py --output biped_robot\test_env\biped_sim_report.json --trace-output biped_robot\test_env\biped_sim_trace.json
```

## Validate Workspace

```powershell
py -3.12 biped_robot\tools\validate_biped_workspace.py
```

## Run Folder-Scoped Acceptance

```powershell
py -3.12 biped_robot\tools\run_local_acceptance.py
```

This command runs only checks owned by `biped_robot/`: Python syntax, local contract validation, deterministic simulation, contact stability, hardware gap reporting, and Godot headless loading when `Godot.exe` is available.

## Run Actuator Physics Simulation

```powershell
py -3.12 biped_robot\tools\simulate_actuator_physics.py
```

Outputs:

- `test_env/actuator_physics_report.json`: aggregate torque, current, thermal, tracking, saturation, and energy metrics.
- `test_env/actuator_telemetry.jsonl`: per-step retained telemetry suitable for later replay or comparison against real hardware.
- `test_env/component_parameter_log.json`: per-part and per-joint parameter log.
- `test_env/retention_manifest.json`: generated artifact index with hashes and retention policy.

## Open Godot

From the repository root:

```powershell
& "D:\迅雷下载\Godot\Godot.exe" --path biped_robot\godot
```

The Godot project is standalone and starts at `res://scenes/biped_mountain_demo.tscn`.

## Godot Data Input and Output

Input:

- `config/godot_io_input.json`: simulation duration, speed scale, gait cycle, telemetry interval, and timed command schedule.

Outputs after a Godot run:

- `test_env/godot_io_telemetry.jsonl`: frame-level telemetry from Godot.
- `test_env/godot_io_report.json`: Godot input/output summary report.

Validate output:

```powershell
py -3.12 biped_robot\tools\validate_godot_io.py
```

## Communication Simulation

The local communication test simulates the controller-to-Godot command path and Godot-to-controller telemetry path.

```powershell
py -3.12 biped_robot\tools\simulate_communication.py
```

Outputs:

- `test_env/communication_events.jsonl`: command, ACK, telemetry, latency, and drop events.
- `test_env/communication_report.json`: delivery, ACK, telemetry, latency, jitter, and loss summary.

## Hardware Boundary

This folder does not connect to real hardware. It is intended to complete the Godot-side structure first, so real hardware transport and ROS2 integration can be added later after explicit approval to touch the relevant project modules.

The current hardware status is machine-readable:

```powershell
py -3.12 biped_robot\tools\build_hardware_gap_report.py --output biped_robot\test_env\hardware_gap_report.json
```
