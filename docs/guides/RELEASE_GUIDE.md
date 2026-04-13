# AGI-Walker Release Guide

更新日期：`2026-04-12`

本页是当前有效的发布门禁说明。历史 release checklist 和 Git release 备忘仍保留在 `docs/archive_and_reports/`，但不应再作为一线发布手册使用。

## 发布产物

阶段五当前固定的发布产物是 `release_manifest`：

- `schema_version=1.0`
- `artifact_type=release_manifest`
- 包含 `build_id`
- 包含 `version`
- 包含 `channel`
- 包含 `release_policy`
- 包含 `release_approval`
- 包含 `release_source`
- 包含 `release_summary`
- 包含 `contract_versions`
- 包含 `capability_matrix`
- 包含 `test_evidence`
- 包含 `known_limitations`

当前生成命令：

```bash
python tools/build_release_artifact.py --version 2026.04.12-rc6 --channel rc --build-id build-20260412-006
```

默认输出：

```text
test_env/release/release_manifest.json
```

`build_release_artifact.py` 现在会自动尝试读取以下默认证据文件：

- `test_env/distributed_smoke/distributed_smoke_report.json`
- `test_env/godot_headless_smoke/headless_smoke_report.json`
- `test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json`

如果这些文件存在，builder 会自动把对应 evidence 状态聚合进 `release_manifest`。如果你要从别的目录读取证据，可以显式传入：

```bash
python tools/build_release_artifact.py --version 2026.04.12-rc1 --channel rc --build-id build-20260412-001 --release-summary "阶段五发布门禁闭环。" --project-root D:/tmp/release_evidence_root --distributed-report D:/tmp/release_evidence_root/test_env/distributed_smoke/distributed_smoke_report.json
```

如果 `--release-summary` 未显式传入，builder 会尝试从 changelog 中提取：

- 第一行 `# ...` 作为 `changelog.title`
- `发布摘要：...` 或 `Release Summary: ...` 作为 `release_summary`

这意味着一线发布流程只需要先更新 `RELEASE_NOTES.md`，再执行 builder；只有需要临时覆盖时才传 `--release-summary` 或 `--changelog-title`。

## 发布前检查

1. 确认 `README.md`、`docs/CURRENT_STATUS.md` 和本页仍代表当前真实入口。
2. 运行 targeted tests 覆盖本次改动区域。
3. 运行默认 smoke：

```bash
python tests/run_smoke_tests.py
```

4. 运行默认非 live 门禁：

```bash
python -m pytest -m "not live" -q
```

5. 如本次发布涉及 live 环境声明，再单独收集对应证据：

```bash
python tests/run_distributed_smoke.py --build --stop-after --report-file test_env/distributed_smoke/distributed_smoke_report.json
AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1 python -m pytest tests/test_godot_headless_smoke.py -q -m "integration and live" --tb=short -vv
AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE=1 python -m pytest tests/test_ros2_bridge_smoke.py -q -m "integration and live" --tb=short -vv
```

## capability matrix

发布面矩阵当前通过两条稳定入口公开：

- Web：`GET /api/capabilities/matrix`
- MCP：`capability_matrix_get`

当前矩阵覆盖的发布面：

- `cli`
- `web_panel`
- `mcp`
- `distributed_runtime`
- `godot_integration`

`distributed_runtime` 默认基线仍从 `diagnostic_ready` 起步；一旦 `test_env/distributed_smoke/distributed_smoke_report.json` 存在且状态为 `passed`，release builder 会自动把该 domain 提升为 `ready`。

## test evidence 口径

`release_manifest` 中的 `test_evidence` 现在使用这三类状态：

- `passed`
- `blocked`
- `opt_in`

当前默认语义：

- `passed`：默认 smoke、默认非 live 门禁、release/capability targeted tests 已通过。
- `blocked`：当前环境存在明确外部阻塞，或真实 live smoke 报告显示失败。
- `opt_in`：测试需要显式环境准备，不进入默认非 live 门禁；如果结构化 live report 已存在且状态为 `passed`，builder 会自动把该证据提升为 `passed`。

## release channel policy

`release_manifest` 现在会附带机器可读的 `release_policy`，用于表达 `dev / rc / stable` 的不同门禁语义。

- `dev`
  - 允许 `opt_in` evidence 仍未闭合
  - 允许 `diagnostic_ready` domain 仍未闭合
  - 只要 required evidence 全部通过，就可以得到 `release_gate_status=ready`
- `rc`
  - 不允许 `opt_in` evidence 未闭合
  - 不允许 `diagnostic_ready` domain 未闭合
  - optional live 证据和 distributed diagnostic state 都必须收口后才会得到 `ready`
- `stable`
  - 不允许 `opt_in` evidence 未闭合
  - 不允许 `diagnostic_ready` domain 未闭合
  - 额外要求 `release_approval.status=approved`
  - 额外要求 `release_source` 成功解析当前 Git HEAD
  - 额外要求当前 Git worktree 为 clean
  - 额外要求当前 HEAD 至少存在一个与 `version` 或 `v{version}` 匹配的 Git tag
  - `approved_by`、`approved_at`、`commit_sha` 缺一不可
  - `release_approval.commit_sha` 必须与 `release_source.commit_sha` 对齐

`known_limitations` 仍然保留，但只作为信息字段；它们不会自动把 gate 从 `ready` 降级为 `ready_with_limitations`。

## stable signoff

稳定通道现在要求显式签核。builder 会把签核记录写入 `release_approval`，字段包括：

- `status`
- `required`
- `approved_by`
- `approved_at`
- `commit_sha`
- `notes`

同时，builder 会把当前仓库 HEAD 写入 `release_source`：

- `resolved_from_git`
- `commit_sha`
- `short_commit_sha`
- `git_tag`
- `matched_version_tag`
- `worktree_clean`
- `worktree_status_summary`
- `version_tag_matches`

如果 `--approval-status approved` 且未显式传 `--commit-sha`，builder 会自动把当前 HEAD commit 写入 `release_approval.commit_sha`。如果显式传入的 `--commit-sha` 与当前 HEAD 不一致，stable gate 会保持 `blocked`。如果当前 worktree 不是 clean，stable gate 也会保持 `blocked`。如果当前 HEAD 没有匹配 `version` 或 `v{version}` 的 Git tag，stable gate 同样会保持 `blocked`。

稳定通道示例：

```bash
python tools/build_release_artifact.py --version 2026.04.12 --channel stable --build-id build-20260412-stable --approval-status approved --approved-by release-manager --approved-at 2026-04-12T12:30:00+00:00 --approval-notes "stable signoff"
```

如果你要显式演练这条路径，而不是直接在当前仓库打 tag，可以运行：

```bash
python tools/run_release_rehearsal.py --version 2026.04.12-rehearsal --build-id release-rehearsal
```

该脚本会：

- 创建临时 Git repo
- 提交当前 rehearsal seed
- 创建与 `version` 匹配的 tag
- 写入通过态 distributed / Godot / ROS2 证据
- 调用 builder 生成 stable manifest
- 要求 `release_gate_status=ready`
- 输出 `release_rehearsal_report.json`

如果你要先看“当前仓库离 stable 还差什么”，而不是直接演练通过态，可以运行：

```bash
python tools/check_release_readiness.py
```

该脚本会：

- 预览当前 `rc` manifest
- 预览目标 `stable` manifest
- 写出 `release_readiness_report.json`
- 明确列出下一步命令，例如补签核、补 tag、生成 worktree cleanup report、重跑 live evidence

如果当前 stable 阻塞项包含 dirty worktree，建议先运行：

```bash
python tools/build_worktree_cleanup_report.py
```

该脚本会：

- 写出 `worktree_cleanup_report.json`
- 非破坏性分类当前 Git worktree 改动
- 区分运行时产物、生成物候选、源码/文档人工审查项和未知分类
- 给出下一步清理计划，而不是直接删除文件

如果 cleanup report 里仍有 tracked 的运行时产物或生成物候选，建议继续运行：

```bash
python tools/build_tracked_artifact_review_report.py
```

该脚本会：

- 聚焦 tracked 的 runtime / generated 候选
- 汇总每个文件的 diff 行数和 diff 预览
- 给出“回退或重建基线”和“审查是否继续跟踪”的建议动作
- 避免直接对 tracked 文件做破坏性清理

如果你要把这些阻塞项直接转换成当前 HEAD 的结构化执行清单，可以运行：

```bash
python tools/build_stable_promotion_checklist.py
```

该脚本会：

- 读取或刷新当前 `release_readiness_report.json`
- 写出 `stable_promotion_checklist.json`
- 固定列出 evidence、diagnostic domain、stable approval、Git HEAD 绑定、clean worktree、版本 tag 和最终 builder step
- 区分 `blocking` 前置项与“已可执行但尚未执行”的最终 stable builder
- 在 dirty worktree 场景下，`clean_worktree` step 会直接给出 `build_worktree_cleanup_report.py` 命令

## 验收标准

阶段五完成的标准不是“所有 live 环境都永远绿色”，而是：

- 发布产物有正式契约
- capability matrix 有正式契约
- release checklist 能关联 contract version、test evidence 和 known limitations
- 当前一线文档不再依赖 archive 里的 release 模板
- 默认 smoke 和默认非 live 门禁都覆盖到发布门禁主链

## 当前结果

当前仓库已具备：

- `capability_matrix` contract v1
- `release_manifest` contract v1
- `tools/build_release_artifact.py`
- Web `/api/capabilities/matrix`
- MCP `capability_matrix_get`
- smoke 中的 capability matrix probe

当前最新 release 口径是：

- `distributed_runtime_live=passed`
- `godot_headless_live=passed`
- `ros2_bridge_live=passed`
- `blocked_evidence=0`
- `blocked_optional_evidence=0`
- `opt_in_evidence=0`
- `release_gate_status=ready`

仓库仍会保留 `known_limitations` 作为信息字段，例如 MCP 仅声明 `stdio` 传输面，以及 live Godot / ROS2 验证不并入默认 `not live` 门禁；但这些已接受的产品边界不再自动把 release gate 降级为 `ready_with_limitations`。只有 `diagnostic_ready` domain、`blocked` evidence 或仍未闭合的 `opt_in` evidence 会让 gate 保持在非 `ready` 状态。

如果需要生成新的 release artifact，只需要替换 `version`、`channel`、`build_id` 和 `release_summary`，然后归档生成的 `release_manifest.json`。stable 通道额外需要确认签核记录绑定的是当前 Git HEAD，并且当前 HEAD 已打上匹配发布版本的 tag。
