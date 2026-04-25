# First Batch Verification Sequence 2026-04-24

用于配合 `FIRST_BATCH_CANDIDATE_CHECKLIST_20260424.md`，在你完成第一批文件的人工作业后，按固定顺序验证这批改动是否仍然闭环。

目标：

- 先做最快的回归
- 再做 closeout 主线验证
- 最后看全局状态有没有回退

## 建议执行顺序

### 1. 文档 / 路径入口校验

```bash
python -m pytest tests/test_active_path_references.py -q
```

通过标准：

- 路径引用测试全部通过

### 2. `clean_checkout_smoke` 相关验证

```bash
python -m pytest tests/test_clean_checkout_smoke.py -q
python tools/run_clean_checkout_smoke.py --output-root test_env/release_evidence/clean_checkout_smoke --report-file test_env/release_evidence/clean_checkout_smoke_report.json
```

通过标准：

- `tests/test_clean_checkout_smoke.py` 通过
- `clean_checkout_smoke_status=passed`

### 3. `worktree_cleanup / blocker` 验证

```bash
python -m pytest tests/test_worktree_cleanup_report.py tests/test_worktree_ops.py tests/test_worktree_release_blocker_report.py -q
python tools/run_worktree_release_blocker.py
```

通过标准：

- 三组 worktree 相关测试通过
- `worktree_release_blocker_report.json` 成功刷新
- 你能清楚看到当前 blocker 是真实改动还是误报

注意：

- 这一步不要求 `status=ready`
- 这一步主要确认脚本仍然可信

### 4. `customer bindings / extension execution / external mainline` 验证

```bash
python -m pytest tests/test_customer_external_bindings_config_builder.py tests/test_external_mainline_execution_plan.py -q
python tools/run_external_mainline_execution_plan.py --output test_env/release_evidence/operations/external_mainline_execution_plan.json
```

通过标准：

- 相关测试通过
- `external_mainline_execution_plan.json` 成功刷新

### 5. `customer acceptance / industrial readiness / promotion` 验证

```bash
python -m pytest tests/test_customer_acceptance_bundle.py tests/test_industrial_delivery_rehearsal_report.py tests/test_industrial_promotion_checklist.py tests/test_industrial_release_readiness.py tests/test_release_rehearsal.py tests/test_release_readiness.py tests/test_stable_promotion_checklist.py -q
python tools/check_release_readiness.py
```

通过标准：

- 这组主线测试通过
- `release_readiness_report.json` 成功刷新

### 6. `security preflight` 相关验证

```bash
python -m pytest tests/test_security_posture_reports.py tests/test_security_release_preflight.py tests/test_vulnerability_exception_review_report.py -q
python tools/run_security_release_preflight.py
```

通过标准：

- 相关测试通过
- `security_release_preflight_report.json` 成功刷新

注意：

- 这条命令可能耗时很长
- 如果工具层超时，不要直接据此判断失败，要检查报告时间戳和内容是否已刷新

### 7. 最后做一次全局状态确认

```bash
python tools/check_release_readiness.py
```

重点关注：

- `stable_security_preflight`
- `stable_customer_delivery`
- `stable_industrial_delivery`
- `stable_external_mainline_input_checklist`
- `stable_worktree_release_blocker`
- `stable_release_gate`

## 推荐阅读顺序

跑完后建议按这个顺序看报告：

1. `test_env/release_evidence/clean_checkout_smoke_report.json`
2. `test_env/worktree_cleanup/worktree_release_blocker_report.json`
3. `test_env/release_evidence/operations/external_mainline_execution_plan.json`
4. `test_env/release_evidence/security_release_preflight_report.json`
5. `test_env/release_readiness/release_readiness_report.json`

## 一句话用法

当你完成第一批文件勾选和人工审查后，就按这份文档从上往下执行；不要一开始就跑最重的 `security_preflight`，先用轻量和中量验证把问题收窄。

