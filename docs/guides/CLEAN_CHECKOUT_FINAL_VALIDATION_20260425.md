# Clean Checkout Final Validation 2026-04-25

用于执行方案 B：在一个**临时 clean checkout** 中验证当前 staged 快照，而不是继续在开发工作区里判断 `stable_worktree_release_blocker`。

---

## 什么时候用

当当前仓库已经满足下面条件时：

- 所有改动都已经进入 staged
- `unstaged_tracked_paths=0`
- `untracked_paths=0`
- 当前 blocker 只剩 `clean_worktree=false`

这时，继续在当前工作区看 readiness 意义已经不大；更合适的动作是把 staged 快照 materialize 到一个临时 clean checkout，再在那里面跑最终 gate。

---

## 推荐命令

```bash
python tools/run_clean_checkout_final_validation.py
```

默认行为：

1. 从当前 staged changes 导出二进制 patch
2. 复制当前工作树快照到 `test_env/clean_checkout_final_validation/<run-id>/checkout`
3. 在这个临时 checkout 中初始化一个临时本地 Git 仓库并生成一条 snapshot commit
4. 运行：
   - `python tools/run_worktree_release_blocker.py`
   - `python tools/check_release_readiness.py`
5. 输出结构化报告

---

## 主要产物

- `test_env/clean_checkout_final_validation/<run-id>/clean_checkout_final_validation_report.json`
- `test_env/clean_checkout_final_validation/<run-id>/staged.patch`
- `test_env/clean_checkout_final_validation/<run-id>/checkout/`

---

## 成功判定

- `clean_checkout_final_validation_status=passed`
- 临时 checkout 中的 `worktree_release_blocker` 不再因为 staged/untracked 路径阻塞
- 临时 checkout 中的 `release_readiness_report.json` 能反映真实 gate 状态，而不是开发工作区状态

---

## 可选参数

只跑 readiness：

```bash
python tools/run_clean_checkout_final_validation.py --skip-worktree-blocker
```

只跑 blocker：

```bash
python tools/run_clean_checkout_final_validation.py --skip-readiness
```

改输出目录：

```bash
python tools/run_clean_checkout_final_validation.py --output-root test_env/clean_checkout_final_validation_manual
```

---

## 现在最重要的结论

当前仓库已经完成了“人工收口”和“staged 收口”。  
方案 B 的重点不再是继续清工作区，而是：

1. 把 staged 快照转成临时 clean checkout
2. 在 clean checkout 上判断最终 gate
3. 让 `stable_worktree_release_blocker` 的结论回到正确环境里
