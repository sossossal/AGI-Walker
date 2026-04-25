# Worktree Blocker Closeout Plan 2026-04-24

用于把当前 `stable_worktree_release_blocker=blocked` 收敛成一条**人工可执行**的 closeout 路径。

来源报告：

- `test_env/worktree_cleanup/worktree_cleanup_report.json`

---

## 当前状态

最新报告显示：

- `clean_worktree=false`
- `total_paths=168`
- `tracked_paths=47`
- `untracked_paths=121`
- `tracked_review_candidate_count=0`
- 当前所有路径都被归类到 `manual_review`

这说明现在的 `worktree blocker` 是**真实阻塞**，不是脚本误报。

---

## 这一步为什么不能自动做

当前工作区里混有：

- release / readiness / external-mainline 主线改动
- customer / industrial closeout 相关输入与文档
- web panel / MCP / distributed / CI 等第二批改动

如果现在自动执行删除、回滚或大范围 `git restore`，很容易把你真正需要保留的 closeout 主线也一起清掉。

所以这里的正确做法不是“自动清理”，而是：

1. 先人工确定第一批要保留的面；
2. 再把不属于第一批的内容拆出去、暂缓或留待第二批；
3. 然后重跑 blocker 和 readiness。

---

## 推荐收口顺序

### 1. 先按第一批勾选结果锁主线

先使用：

- `docs/guides/FIRST_BATCH_CANDIDATE_CHECKLIST_20260424.md`
- `docs/guides/FIRST_BATCH_REVIEW_DECISIONS_20260424.md`

优先锁定这些内容为第一批：

- release / readiness / worktree / preflight
- external-mainline / extension-execution
- customer / industrial / rehearsal
- must_keep tests
- 必要输入 / 文档

### 2. 明确暂缓面

先不要让这些内容影响第一批 closeout：

- `web_panel/*`
- `.github/workflows/ci.yml`
- MCP / Portal / distributed 扩展
- 第二批工具与扩展测试

### 3. 只围绕第一批做人工判断

推荐动作只有三种：

- 保留
- 暂缓
- 第二批

如果某个文件不能明确支持第一批 closeout 主线，就不要先混进第一批判定。

---

## 建议你现在如何人工处理

### 方案 A：先做“保留面确认”

适合你当前最想先推进 stable closeout。

动作：

1. 把第一批主线文件视为保留；
2. 暂时不要处理第二批扩展面；
3. 完成后重跑 blocker 和 readiness 看阻塞是否缩小。

### 方案 B：先做“第二批剥离”

适合你已经很确定 `web_panel` / `MCP` / `CI` 这批不该进第一批。

动作：

1. 先人工把第二批文件从你的第一批意图中排除；
2. 再回头看第一批主线；
3. 完成后重跑 blocker 和 readiness。

---

## 重跑命令

在你完成一轮人工判断后，执行：

```powershell
python tools/run_worktree_release_blocker.py
python tools/check_release_readiness.py
```

如果你同时补了 external-mainline 输入，建议再串上：

```powershell
python tools/run_external_mainline_execution_plan.py --output test_env/release_evidence/operations/external_mainline_execution_plan.json
python tools/check_release_readiness.py
```

---

## 完成判定

这一步不要求“所有改动都消失”，而是要求：

- 第一批 closeout 主线已经明确；
- 第二批扩展面已经被显式暂缓；
- `worktree_cleanup_report.json` 的人工不确定性被缩小；
- `stable_worktree_release_blocker` 最终在你确认的 clean/controlled worktree 上解除。

---

## 和当前主阻塞的关系

现在稳定收口还剩两类主阻塞：

1. `external_mainline` 剩余真实输入未补齐；
2. `worktree blocker` 还没有经过人工拆分收口。

这两件事是并行可推进的，但**最终 stable gate** 只有在两边都收住后才会真正放行。
