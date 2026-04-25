# Staged And Unstaged Priority Checklist 2026-04-24

用于单独处理当前 `worktree blocker` 里最容易把第一批/第二批意图混在一起的 `staged_and_unstaged` 路径。

当前一共 `8` 个：

- `README.md`
- `agi_walker/ops/worktree.py`
- `deployment/customer_delivery.external_bindings.customer.json`
- `deployment/external_mainline.inputs.json`
- `tests/test_worktree_cleanup_report.py`
- `tests/test_worktree_ops.py`
- `tests/test_worktree_release_blocker_report.py`
- `tools/build_worktree_cleanup_report.py`

---

## 为什么先处理这 8 个

这批路径已经有内容进入暂存区，但工作区里还有新的未暂存变化。

如果不先处理它们：

- 第一批 closeout 的边界会一直模糊；
- `worktree blocker` 会持续同时计算 staged 和 unstaged；
- 下一轮审查时很容易看不清“哪些是已经确认的第一批、哪些是后来新增的调整”。

---

## 逐项建议

| 路径 | 当前归属 | 建议动作 | 原因 |
|---|---|---|---|
| `README.md` | 第一批文档入口 | 先比对 staged 与 working tree 差异，再决定是否把最新入口补充一并纳入第一批 | 它承载 closeout 文档入口，通常应保持和本轮新增文档一致 |
| `agi_walker/ops/worktree.py` | 第一批主线实现 | 保留最新版本并更新暂存区 | 这是本轮 blocker 可读性改进的核心实现 |
| `deployment/customer_delivery.external_bindings.customer.json` | 第一批输入面 | 保留最新版本并更新暂存区 | 这是 customer closure 主线输入 |
| `deployment/external_mainline.inputs.json` | 第一批输入面 | 保留最新版本并更新暂存区 | 这里刚补过 customer / industrial / vulnerability 关键值 |
| `tests/test_worktree_cleanup_report.py` | 第一批必要测试 | 保留最新版本并更新暂存区 | 覆盖 blocker 新增的 staged / unstaged 拆分 |
| `tests/test_worktree_ops.py` | 第一批必要测试 | 保留最新版本并更新暂存区 | 覆盖 worktree ops 聚合行为 |
| `tests/test_worktree_release_blocker_report.py` | 第一批必要测试 | 保留最新版本并更新暂存区 | 覆盖统一 blocker 报告行为 |
| `tools/build_worktree_cleanup_report.py` | 第一批主线实现 | 保留最新版本并更新暂存区 | 当前 blocker 新增状态拆分就是在这里完成的 |

---

## 推荐处理顺序

### 1. 先处理实现

- `tools/build_worktree_cleanup_report.py`
- `agi_walker/ops/worktree.py`

### 2. 再处理输入

- `deployment/customer_delivery.external_bindings.customer.json`
- `deployment/external_mainline.inputs.json`

### 3. 再处理测试

- `tests/test_worktree_cleanup_report.py`
- `tests/test_worktree_ops.py`
- `tests/test_worktree_release_blocker_report.py`

### 4. 最后处理入口文档

- `README.md`

---

## 推荐操作方式

对这 8 个路径，最稳的做法不是回滚，而是：

1. 看 staged 和 working tree 是否都属于同一轮 closeout；
2. 如果属于同一轮，就把**最新工作区版本**更新进暂存区；
3. 如果不是同一轮，再把新增部分拆出去。

---

## 完成后建议重跑

```powershell
python -m pytest tests/test_worktree_cleanup_report.py tests/test_worktree_release_blocker_report.py tests/test_worktree_ops.py -q
python tools/run_worktree_release_blocker.py
python tools/check_release_readiness.py
```

---

## 成功判定

如果这 8 个路径处理得当，下一轮你应该看到：

- `staged_and_unstaged_paths` 下降
- `staged_paths` 与 `unstaged_tracked_paths` 的边界更清楚
- `worktree blocker` 的人工处理压力进一步收缩到第二批扩展面和剩余未跟踪文件
