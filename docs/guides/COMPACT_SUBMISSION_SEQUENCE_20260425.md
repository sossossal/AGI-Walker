# Compact Submission Sequence · 2026-04-25

## 最短提交顺序

1. 提交第一批主线  
   `feat(release): add release closeout, external-mainline, and evidence automation`

2. 提交 MCP  
   `feat(mcp): expose release closeout and external-mainline MCP surfaces`

3. 提交 Web Panel  
   `feat(web-panel): add release closeout and control-plane surfaces`

4. 提交 Deployment Runtime  
   `feat(deployment): add distributed runtime and deployment validation assets`

5. 提交 Final Validation / Signoff  
   `feat(signoff): add clean-checkout final validation and signoff helpers`

6. 提交 CI  
   `ci: align workflow coverage with release closeout validation`

7. 最后提交 docs-only 收口  
   `docs(release): add closeout, signoff, and submission guides`

## 执行时只记住这 3 份文档

- 第一批命令串  
  `docs/guides/FIRST_BATCH_ACTUAL_COMMAND_CHAIN_20260425.md`

- 第二批命令串  
  `docs/guides/SECOND_BATCH_ACTUAL_COMMAND_CHAIN_20260425.md`

- Commit message 草稿  
  `docs/guides/RECOMMENDED_COMMIT_MESSAGES_20260425.md`
