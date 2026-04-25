# External Mainline Remaining Inputs 2026-04-24

用于把当前 `external_mainline` 剩余的 12 个阻塞项，压成一张可以逐项销项的执行表。

来源报告：

- `test_env/release_evidence/operations/external_mainline_input_checklist_report.json`

当前状态：

- `customer_missing=2`
- `vulnerability_missing=3`
- `industrial_missing=7`
- `ready_to_run_steps=1`
- `waiting_external_input_steps=2`

---

## 当前结论

- `customer_external_bindings_closure` 已经是 `ready_to_run`
- 还差 `confirmed_by` 和 `confirmation_ticket`
- `vulnerability_exception_replacement` 仍在等真实安全输入
- `industrial_delivery_live_evidence` 仍在等真实客户环境输入

---

## 剩余输入明细

| 类别 | 缺口 | 目标文件 / 字段 | 提供方 | 补齐后动作 |
|---|---|---|---|---|
| customer | `confirmed_by` | `deployment/external_mainline.inputs.json` → `customer_external_bindings.confirmed_by` | 客户侧 / 交付侧共同确认 | 重跑 customer closure |
| customer | `confirmation_ticket` | `deployment/external_mainline.inputs.json` → `customer_external_bindings.confirmation_ticket` | 客户侧 / 交付侧共同确认 | 重跑 customer closure |
| vulnerability | 在 `2026-05-15T00:00:00+01:00` 前完成 replacement / review | `deployment/security/vulnerability_exceptions.input.json` | 安全 / 发布侧 | 重建 vulnerability review + 重跑 preflight |
| vulnerability | 更新异常输入文件 | `deployment/security/vulnerability_exceptions.input.json` | 安全 / 发布侧 | 重建 vulnerability review + 重跑 preflight |
| vulnerability | 最新 upstream fix 版本或重算 scanner 结果 | `deployment/security/vulnerability_exceptions.input.json` 的来源证据 | 安全 / 发布侧 | 重建 vulnerability review + 重跑 preflight |
| industrial | 真实客户环境标识 | `deployment/external_mainline.inputs.json` → `industrial_live_evidence.target_environment` | 客户侧 | 重跑 external-mainline |
| industrial | 真实客户环境访问方式 | `deployment/external_mainline.inputs.json` → `industrial_live_evidence.access_method` | 客户侧 / 交付侧 | 重跑 external-mainline |
| industrial | install 实际命令或入口 | `deployment/external_mainline.inputs.json` → `industrial_live_evidence.install_entrypoint` | 交付侧 | 重跑 external-mainline |
| industrial | upgrade 实际命令或入口 | `deployment/external_mainline.inputs.json` → `industrial_live_evidence.upgrade_entrypoint` | 交付侧 | 重跑 external-mainline |
| industrial | rollback 实际命令或入口 | `deployment/external_mainline.inputs.json` → `industrial_live_evidence.rollback_entrypoint` | 交付侧 | 重跑 external-mainline |
| industrial | backup-restore 实际命令或入口 | `deployment/external_mainline.inputs.json` → `industrial_live_evidence.backup_restore_entrypoint` | 交付侧 | 重跑 external-mainline |
| industrial | closure archive 与现场留痕目录 | `deployment/external_mainline.inputs.json` → `industrial_live_evidence.closure_archive_root` | 交付侧 | 重跑 external-mainline |

---

## 推荐补齐顺序

### 1. 先补 customer 两项

因为这一步已经是 `ready_to_run`，补完后可以最快把一个 waiting 点转成已执行态。

需要同时核对：

- `deployment/customer_delivery.external_bindings.customer.overrides.json`
- `deployment/external_mainline.inputs.json`

完成后执行：

```powershell
python tools/run_customer_external_bindings_closure.py --config deployment/customer_delivery.external_bindings.customer.json --overrides-file deployment/customer_delivery.external_bindings.customer.overrides.json --section approval_identity --section archive_target --section due_trigger --confirmed-by <real-user> --confirmation-ticket <real-ticket> --skip-collect-release-evidence
```

### 2. 再补 industrial 七项

建议先在下面两个文件里整理：

- `deployment/industrial_live_evidence.customer.template.json`
- `deployment/external_mainline.inputs.customer_draft.json`

确认无误后再同步到：

- `deployment/external_mainline.inputs.json`

完成后执行：

```powershell
python tools/run_external_mainline_execution_plan.py --output test_env/release_evidence/operations/external_mainline_execution_plan.json
```

### 3. 最后补 vulnerability 三项

这一步需要真实安全输入，不建议现在凭空猜值。

完成后执行：

```powershell
python tools/build_vulnerability_exception_review_report.py --output test_env/release_evidence/security/vulnerability_exception_review_report.json --exception-report test_env/release_evidence/security/vulnerability_exception_report.json
python tools/run_security_release_preflight.py
```

---

## 完成判定

补齐后，下面两个报告应该一起变化：

- `test_env/release_evidence/operations/external_mainline_execution_plan.json`
- `test_env/release_evidence/operations/external_mainline_input_checklist_report.json`

理想目标：

- `customer_missing_inputs=[]`
- `industrial_missing_inputs=[]`
- `waiting_external_input_steps` 不再包含 `industrial_delivery_live_evidence`
- `missing_input_count` 明显下降，最终收敛为只剩安全侧真实输入或直接归零
