# AGI-Walker 2026.04.12-rc6 Release Notes

发布日期：`2026-04-12`
发布通道：`rc`
发布状态：`ready`
发布摘要：所有 live 证据已附带，release gate 已提升为 ready。

## 概览

这是当前仓库的正式候选发布说明，覆盖阶段一到阶段五的闭环结果，而不是历史版本宣传页。

当前发布结论：

- `release_gate_status=ready`
- `blocked_evidence=0`
- `opt_in_evidence=0`
- `distributed_runtime_live=passed`
- `godot_headless_live=passed`
- `ros2_bridge_live=passed`

当前发布产物位于：

- `test_env/release/release_manifest.json`

## 本次发布收口内容

### 1. 主链契约完成

- `workflow` artifact v1 已稳定，CLI、Workflow、Web 和 Godot delivery 使用同一组基础契约。
- `training_run` artifact v1 已覆盖 `mock_training`、`offline_dataset_training`、`sim_training`、`hardware_in_the_loop`。
- `capability_matrix` artifact v1 与 `release_manifest` artifact v1 已成为正式发布门禁。

### 2. 分布式与仿真证据补齐

- distributed smoke 已通过，并留存 `test_env/distributed_smoke/distributed_smoke_report.json`。
- Godot headless live smoke 已通过，并留存 `test_env/godot_headless_smoke/headless_smoke_report.json`。
- ROS2 bridge live smoke 已在 `ros:humble-ros-base` 容器化 ROS2 Humble runtime 中通过，并留存 `test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json`。

### 3. 发布门禁完成

- `python tests/run_smoke_tests.py` 通过。
- `python -m pytest -m "not live" -q` 当前为 `792 passed, 3 skipped, 3 deselected`。
- `tools/build_release_artifact.py` 现在会在 live 证据齐全时输出 `release_gate_status=ready`。
- `stable` 通道现在要求显式签核，需附带 `release_approval.status=approved`、`approved_by`、`approved_at` 和 `commit_sha`。
- `stable` 通道的签核 SHA 现在还会绑定当前 Git HEAD，并写入 `release_source`。
- `stable` 通道现在还要求 Git worktree 为 clean，dirty worktree 会直接阻塞 promotion。
- `stable` 通道当前还要求 HEAD 上存在与发布版本匹配的 Git tag，否则 gate 会保持 `blocked`。
- 已新增 `tools/check_release_readiness.py`，用于输出当前 rc/stable 预览门禁状态和下一步动作。
- 已新增 `tools/build_worktree_cleanup_report.py`，用于把 dirty worktree 转成结构化清理报告，而不是只看 `git status --short`。
- 已新增 `tools/build_tracked_artifact_review_report.py`，用于继续聚焦 tracked 的 runtime/generated 候选并输出 diff 摘要。
- 已新增 `tools/build_stable_promotion_checklist.py`，用于把当前 HEAD 的 stable 阻塞项转成结构化 promotion checklist。
- `tools/check_release_readiness.py` 和 `tools/build_stable_promotion_checklist.py` 现在都支持 `--approval-manifest`，可直接复用已有 stable manifest 的签核元数据。
- 已新增 `tools/run_release_rehearsal.py`，用于显式演练“匹配版本 tag 的 stable 发布会变为 ready”。
- 当前真实仓库的 cleanup report 已显示 `189` 个待处理路径，其中运行时产物 `6`、生成物候选 `1`、源码/文档人工审查项 `182`；新增 `.gitignore` 规则后，untracked runtime / generated 噪音已先被压缩。
- 当前 tracked artifact review report 已锁定 `7` 个 tracked 候选，供下一步人工决策。

## 操作命令

安装开发环境：

```bash
pip install -e ".[dev]"
```

执行默认 smoke：

```bash
python tests/run_smoke_tests.py
```

执行默认非 live 门禁：

```bash
python -m pytest -m "not live" -q
```

生成 release artifact：

```bash
python tools/build_release_artifact.py --version 2026.04.12-rc6 --channel rc --build-id build-20260412-006 --release-summary "Release gate now resolves to ready when all live evidence is attached." --output test_env/release/release_manifest.json
```

## 已知信息性边界

这些边界仍会出现在 `known_limitations` 中，但不再自动把 release gate 降级为非 `ready`：

- MCP 当前只声明 `stdio` 传输面。
- live Godot / ROS2 验证仍刻意排除在默认 `not live` 门禁之外。
- ROS2 live 证据当前来自容器化 `ros:humble-ros-base` 运行时，而不是宿主 Python 环境。

## 参考文档

- `docs/CURRENT_STATUS.md`
- `docs/FEATURE_COMPLETION_PLAN.md`
- `docs/guides/RELEASE_GUIDE.md`
- `docs/guides/CLI_GUIDE.md`
- `docs/guides/WEB_PANEL_GUIDE.md`
