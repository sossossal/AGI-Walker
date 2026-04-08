# Developer Guide

更新日期：`2026-04-08`

本页面向需要修改 AGI-Walker 代码或文档的贡献者。目标不是完整架构论文，而是告诉你当前主线路在哪里、修改后怎么验证。

## 1. 先理解主线

当前最值得先理解的模块是：

- `agi_walker/cli`
- `agi_walker/skills_loader.py`
- `agi_walker/workflow_orchestrator.py`
- `agi_walker/mcp`
- `web_panel/`

这几块共同构成当前仓库的用户主路径。

## 2. 常用本地命令

```bash
python -m agi_walker.cli --help
python -m agi_walker.cli skills list
python -m agi_walker.cli workflows list
python -m agi_walker.cli doctor
python -m web_panel.server
agi-walker-mcp
```

## 3. 仓库结构

### CLI 与 skills

- `agi_walker/cli/`
- `agi_walker/skills/`
- `agi_walker/skills_loader.py`
- `agi_walker/skill_executors.py`

### 编排与 API

- `agi_walker/workflow_orchestrator.py`
- `agi_walker/core/api/`
- `agi_walker/core/controllers/`

### Web 与 Godot

- `web_panel/`
- `agi_walker/integrations/godot_agent/`
- `godot_project/`

### 文档与测试

- `docs/`
- `tests/`

## 4. 新改动优先落在哪

推荐原则：

- 新的命令行能力优先落到 CLI
- 新的可编排任务优先落到 workflow / skill executor
- 新的外部调用面优先落到 Web 或 MCP
- 文档入口优先维护 `README.md` 和顶层 guide

避免继续扩散一次性脚本式入口。

## 5. 修改后最小验证

文档或主入口改动后，优先跑：

```bash
python -m pytest tests/test_docs_utf8.py -q
python -m pytest tests/test_mcp_tools.py tests/test_mcp_server.py -q
python tests/run_smoke_tests.py
```

如果你只改 workflow 或 skills，也建议补跑：

```bash
python -m pytest tests/test_workflow_orchestrator.py tests/test_integration_workflows.py -q
```

## 6. Skills 开发约定

当前 skill 目录至少应包含：

- `SKILL.md`

常见扩展目录：

- `scripts/`
- `references/`
- `assets/`

`SKILL.md` 的 YAML frontmatter 至少要有：

- `name`
- `description`

推荐补齐：

- `category`
- `emoji`
- `inputs`
- `outputs`
- `metadata.agi_walker.requires`

## 7. 文档维护约定

当前文档修复任务的目标是：

- 主入口文档全部为 UTF-8
- 不再引用失效的 `python_api/` 路径
- 不把实验模块写成稳定默认能力

如果你新增高频入口页，记得同步更新：

- `tests/test_docs_utf8.py`

## 8. Godot 与 distributed 的开发心态

这两块都不是最低门槛路径。开发时应明确区分：

- 便宜、稳定、可本地跑的测试
- 依赖 Godot、Zenoh、Docker 或额外仓库的测试

不要把重型集成路径塞进默认开发循环。

## 9. 提交前检查

提交前至少确认：

- 代码能被当前入口调用到
- 文档不再引用旧路径
- 关键测试通过
- 新增环境变量有默认值或回退路径
- 失败场景有明确错误信息

## 结论

对当前贡献者来说，最重要的是守住主线一致性：CLI、workflow、Web、MCP、docs 和 tests 口径一致。只要这条线稳，扩展模块就还能持续收敛。
