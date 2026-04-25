# Shortest Next Actions · 2026-04-25

## 现在最短可执行路径

如果你现在就要开始真正推进提交与收口，建议只做下面这几步：

### 1. 先看当前状态

```powershell
git status --short
```

### 2. 执行第一批主线提交

直接按这里执行：

- `docs/guides/FIRST_BATCH_ACTUAL_COMMAND_CHAIN_20260425.md`

### 3. 执行第二批扩展面提交

第一批完成后，直接按这里执行：

- `docs/guides/SECOND_BATCH_ACTUAL_COMMAND_CHAIN_20260425.md`

### 4. 每批都先跑对应验证

不要跳过各批次文档里的 `pytest` 验证命令。  
如果某一批验证失败，就先停在那一批修，不要继续往后提交。

### 5. 最后处理 docs-only 收口

docs-only 建议放到最后一批，避免和实现批次混在一起。

### 6. 提交标题直接从这里拿

- `docs/guides/RECOMMENDED_COMMIT_MESSAGES_20260425.md`

## 一句话版本

最短执行顺序就是：

1. `git status --short`
2. 跑第一批命令串
3. 跑第二批命令串
4. 每批先测再提
5. 最后 docs-only 收口

## 你现在已经可以直接用的 4 份文件

- `docs/guides/FIRST_BATCH_ACTUAL_COMMAND_CHAIN_20260425.md`
- `docs/guides/SECOND_BATCH_ACTUAL_COMMAND_CHAIN_20260425.md`
- `docs/guides/RECOMMENDED_GIT_ADD_COMMANDS_20260425.md`
- `docs/guides/RECOMMENDED_COMMIT_MESSAGES_20260425.md`
