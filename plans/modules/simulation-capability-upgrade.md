# Module Goal

Upgrade the biped software simulation capability toward mainstream robotics simulator expectations without adding external simulator or hardware runtime dependencies.

# Ownership

- `biped_robot/config/sensor_model.json`
- `biped_robot/config/scenario_matrix.json`
- `biped_robot/config/fault_injection.json`
- `biped_robot/config/ros2_topic_contract.json`
- New simulation capability tools under `biped_robot/tools/`
- Generated reports and traces under `biped_robot/test_env/`

# Inputs and Outputs

Inputs:

- Existing robot, actuator, Godot IO, communication, public replay and part-driven system reports.
- New sensor, scenario, fault and ROS2 topic contract configs.

Outputs:

- `sensor_simulation_report.v1`
- `scenario_matrix_report.v1`
- `robot_description_mapping_report.v1`
- `fault_injection_report.v1`
- `ros2_topic_contract_simulation_report.v1`
- `godot_visual_acceptance_report.v1`
- `simulation_capability_upgrade_report.v1`

# Contract Checklist

- Public surface exposed: new biped CLI tools for each capability and an aggregate validator.
- Inputs accepted: default local config/report paths plus optional output paths.
- Outputs produced: report JSON and trace/event JSONL artifacts.
- Backward compatibility: no hardware, ROS2 runtime, external simulator or existing Godot contract changes.
- Integration tests required: biped workspace validation, local acceptance and frontend-to-Godot project acceptance.

# Local Context

The biped folder already has Godot, actuator, communication, public data replay and system-level software closeout evidence. This module adds the missing simulator-grade layers: sensor modeling, scenario matrix, description mapping, fault injection, ROS2 topic contract simulation and visual frame-summary acceptance.

# Non-Goals

- Do not install or require Gazebo, MuJoCo, Isaac Sim or Webots.
- Do not start real ROS2 runtime.
- Do not connect to hardware or claim hardware validation.

# Tasks

- [x] Add sensor simulation config, report and trace.
- [x] Add scenario matrix config, report and trace.
- [x] Add URDF/SDF/MJCF mapping report and preview artifact.
- [x] Add fault injection config, report and trace.
- [x] Add ROS2 topic contract simulation report and events.
- [x] Add Godot visual frame-summary acceptance report.
- [x] Add aggregate capability upgrade report and validator.
- [x] Wire all new domains into biped local acceptance and coverage.

# Risks and Mitigations

- Risk: Mapping reports are mistaken for simulator-native models.
  Mitigation: Reports mark mappings as previews with unsupported fields.

- Risk: ROS2 topic simulation is mistaken for ROS2 runtime validation.
  Mitigation: Runtime mode is `software_contract_simulation` and hardware/ROS2 gates remain blocked.

- Risk: Visual frame summaries are mistaken for screenshot evidence.
  Mitigation: Report states visual evidence mode as telemetry-derived frame summary.

# Validation

```powershell
D:\actions-tools\Python312\python.exe -m py_compile biped_robot\tools\run_sensor_simulation.py biped_robot\tools\run_scenario_matrix.py biped_robot\tools\export_robot_description_mapping.py biped_robot\tools\run_fault_injection.py biped_robot\tools\run_ros2_topic_contract_simulation.py biped_robot\tools\run_godot_visual_acceptance.py biped_robot\tools\build_simulation_capability_upgrade_report.py biped_robot\tools\validate_simulation_capability_upgrade.py
D:\actions-tools\Python312\python.exe biped_robot\tools\run_local_acceptance.py
D:\actions-tools\Python312\python.exe tools\run_frontend_godot_project_acceptance.py
```

# Completion Criteria

- All new capability reports pass.
- Full coverage marks all new simulation capability domains covered.
- Retention manifest contains each report and trace/event artifact.
- Real-world hardware/ROS2 coverage remains blocked without external runtime.

# Drift Check

This module strengthens simulation-before-hardware readiness and preserves the project-level hardware validation boundary.
