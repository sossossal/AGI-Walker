# ROS2 Typed IDL Migration Runbook

更新日期：`2026-04-26`

本页用于把 ROS2 bridge 的下游调用方从 legacy JSON topic / Trigger service 迁移到 typed `agi_walker_msgs` IDL。目标是降低字符串 payload 漂移，同时保留可回滚路径。

## 当前兼容策略

- JSON topic / Trigger service 保持可用，作为回滚和旧客户端兼容面。
- typed topic / typed service 作为新集成默认面。
- 迁移期间，同一语义只允许一个 owner 负责写入，避免 JSON 与 typed 双写导致指令重复。

## Topic / Service 映射

| Legacy surface | Typed surface | Contract |
| --- | --- | --- |
| `/instruction_set/json` | `/instruction_set` | `agi_walker_msgs/msg/InstructionSet` |
| `/simulated_circuit/json` | `/simulated_circuit` | `agi_walker_msgs/msg/SimulatedCircuit` |
| `/instruction_set/replay_last` | `/instruction_set/apply` | `agi_walker_msgs/srv/ApplyInstructionSet` |
| `/simulated_circuit/apply_default` | `/simulated_circuit/configure` | `agi_walker_msgs/srv/ConfigureSimulatedCircuit` |
| `/hardware/recovery_plan`, `/hardware/recover_by_fault_class`, `/hardware/clear_faults` | `/hardware/recovery` | `agi_walker_msgs/srv/HardwareRecovery` |

New typed-only surfaces:

- `/behavior_command` as `agi_walker_msgs/msg/BehaviorCommand`
- `/navigation_goal` as `agi_walker_msgs/msg/NavigationGoal`
- `/perception_snapshot` as `agi_walker_msgs/msg/PerceptionSnapshot`

## Migration Phases

### Phase 0：Inventory

1. Search downstream repos and launch files for:
   - `/instruction_set/json`
   - `/simulated_circuit/json`
   - `/hardware/recovery_plan`
   - `/hardware/recover_by_fault_class`
   - `/hardware/clear_faults`
2. Record each publisher/client owner, launch profile, and rollback owner.

### Phase 1：Shadow Typed Path

1. Build `agi_walker_msgs`.
2. Start `agi_walker_ros2` with the existing JSON producers unchanged.
3. Add typed publishers/services in shadow mode, but keep them disabled by default in live runs.
4. Validate with:

```bash
python -m pytest tests/test_ros2_bridge_runtime.py tests/test_ros2_workspace.py -q
```

### Phase 2：Single Writer Cutover

1. Select one surface at a time.
2. Disable the matching JSON writer.
3. Enable the typed publisher or service client.
4. Verify bridge runtime telemetry still emits:
   - `typed_instruction_set_applied`
   - `typed_simulated_circuit_configured`
   - `typed_hardware_*`
   - `behavior_command_applied`
   - `navigation_goal_applied`
   - `perception_snapshot_applied`

### Phase 3：Live Humble Verification

Run in the target ROS2 Humble environment:

```bash
export AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE=1
python -m pytest tests/test_ros2_bridge_smoke.py -q -m "integration and live"
```

Archive:

- `test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json`
- selected launch profile
- typed topic/service inventory
- rollback owner

Then build the cutover closeout report:

```bash
python tools/build_ros2_typed_idl_cutover_report.py \
  --input deployment/ros2_typed_idl_cutover.template.json \
  --output test_env/ros2_typed_idl_cutover/ros2_typed_idl_cutover_report.json
```

Use `--require-evidence-files` when the downstream typed inventory and rollback plan must be present on disk. The report remains `blocked` while JSON writers are still enabled, any typed surface is not `passed / ready`, or the live Humble smoke report is missing/non-passing.

## Deprecation Policy

- JSON surfaces are deprecated for new downstream integrations immediately after typed IDL build passes.
- JSON surfaces remain supported until at least one live Humble smoke and one downstream typed cutover pass.
- Removal requires:
  - updated downstream inventory with zero JSON writers
  - archived live smoke evidence
  - release note calling out removed topics/services

## Rollback

If typed cutover fails:

1. Disable the typed writer/client.
2. Re-enable the previous JSON writer/client.
3. Re-run fake-runtime tests.
4. Capture failure payload and map it to the corresponding typed IDL field.

Rollback is valid only if the JSON writer remains a single writer for that semantic surface.
