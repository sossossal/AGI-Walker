# Code Quality

更新日期：`2026-04-08`

本页记录 AGI-Walker 当前代码和文档维护的最低质量基线，以及本仓库中已经落地的检查方式。

## 当前质量基线

### 1. 文档必须是正常 UTF-8

入口文档不得出现已知的 mojibake 片段或 Unicode replacement character。

当前已纳入自动检查的入口文档包括：

- `README.md`
- `docs/mcp.md`
- `docs/CURRENT_STATUS.md`
- `docs/MIGRATION_GUIDE.md`
- `docs/API_REFERENCE.md`
- `docs/CODE_QUALITY.md`
- `docs/SKILLS_INDEX.md`
- `docs/guides/CLI_GUIDE.md`
- `docs/guides/WEB_PANEL_GUIDE.md`

对应测试：

```bash
python -m pytest tests/test_docs_utf8.py -q
```

### 2. 主入口必须可运行

当前要求至少保证这些入口不失效：

- `python -m agi_walker.cli`
- `python -m web_panel.server`
- `agi-walker-mcp`
- `python -m agi_walker.mcp.server`

这意味着：

- 文档示例必须和真实入口一致
- 新依赖升级后，MCP 初始化参数必须保持兼容

### 3. 最小高信号测试必须可执行

本仓库当前最重要的低成本回归集包括：

```bash
python -m pytest tests/test_docs_utf8.py tests/test_mcp_tools.py tests/test_mcp_server.py -q
python tests/run_smoke_tests.py
```

可选的真实 Godot 集成检查：

```powershell
$env:AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE='1'
python -m pytest tests/test_godot_headless_smoke.py -q -m integration --tb=short -vv
```

## 当前已落地的仓库配置

### Black

`pyproject.toml` 中当前配置：

- 行宽：`88`
- 目标版本：`py310`

如果本地安装了 Black，可使用：

```bash
black --check agi_walker web_panel tests
```

### Pytest

`pyproject.toml` 中当前 pytest 配置：

- `minversion = 7.0`
- `addopts = "-ra -q --cov=agi_walker"`
- `testpaths = ["tests"]`

这说明默认测试运行会同时对 `agi_walker` 生成 coverage。

### 类型与日志

本仓库当前的推荐方向是：

- 新代码优先补类型标注
- 用 `logging` 取代随手 `print`
- 对入口与桥接层优先做错误处理

这不是“全仓库已经完全一致”，而是当前维护方向。

## 推荐的提交前检查

### 文档或 MCP 改动

```bash
python -m pytest tests/test_docs_utf8.py tests/test_mcp_tools.py tests/test_mcp_server.py -q
```

### CLI / workflow / Web 改动

```bash
python -m agi_walker.cli skills list
python -m agi_walker.cli workflows list
python tests/run_smoke_tests.py
```

### Godot / session bridge 改动

```powershell
$env:AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE='1'
python -m pytest tests/test_godot_headless_smoke.py -q -m integration --tb=short -vv
```

### Web workflow 改动

至少检查：

- `/api/system/status`
- `/api/workflows/`
- `/api/workflows/runs`
- `/api/godot/capabilities`
- `/api/godot-agent/status`

## 文档质量要求

文档变更至少满足以下约束：

- 入口命令必须与当前代码一致
- 环境变量名必须和代码中真实名称一致
- 不把历史归档文档当作当前操作手册
- 变更 README 时，要同步考虑 `CURRENT_STATUS`、`MIGRATION_GUIDE`、`CLI_GUIDE`、`WEB_PANEL_GUIDE`

## 代码质量检查清单

提交前建议逐项确认：

1. 命令或路由名没有写错。
2. 新增文档是 UTF-8，且没有历史乱码。
3. 关键入口仍然能启动。
4. 关键测试至少跑过最小回归集。
5. 文档中的例子是当前仓库可执行的，不是历史残留。

## 当前限制

以下内容仍未统一到同一质量水平：

- `docs/archive_and_reports/` 下的历史归档
- 部分非入口文档页
- 某些大型设计文档和旧方案页

因此当前的策略是：

- 先保住主入口和高频操作路径
- 再逐步清理历史页面
