# AGI-Walker CLI Guide

更新日期：`2026-04-08`

本页只记录当前代码中真实存在、且已经过最小验证的 CLI 入口。

## 标准入口

推荐入口：

```bash
python -m agi_walker.cli
```

Windows 下也可以使用包装脚本：

```bat
scripts\agi_walker.bat skills list
```

当前顶层子命令：

- `skills`
- `workflows`
- `doctor`

查看帮助：

```bash
python -m agi_walker.cli --help
python -m agi_walker.cli skills --help
python -m agi_walker.cli workflows --help
```

## Skills 命令

### 列出 skills

```bash
python -m agi_walker.cli skills list
python -m agi_walker.cli skills list -v
python -m agi_walker.cli skills list --category 建模
```

`2026-04-08` 当前检出的 skills 至少包括：

- `robot-modeling`
- `parameter-optimizer`
- `urdf-generator`
- `model-distiller`

### 查看 skill 详情

```bash
python -m agi_walker.cli skills info robot-modeling
python -m agi_walker.cli skills info robot-modeling -d
```

说明：

- `info` 显示元数据、路径、脚本目录和参考文档。
- `-d` 会额外输出完整 `SKILL.md` 正文。

### 搜索与分类

```bash
python -m agi_walker.cli skills search 优化
python -m agi_walker.cli skills search URDF
python -m agi_walker.cli skills categories
```

### 验证 skills 配置

```bash
python -m agi_walker.cli skills validate
python -m agi_walker.cli skills validate -v
```

返回值语义：

- 全部有效时返回 `0`
- 存在依赖或配置问题时返回非 `0`

## Workflows 命令

当前推荐使用顶层 alias：

```bash
python -m agi_walker.cli workflows ...
```

兼容写法仍然可用：

```bash
python -m agi_walker.cli skills workflows ...
```

### 列出 workflows

```bash
python -m agi_walker.cli workflows list
```

`2026-04-08` 当前检出至少包含：

- `robot_creation_pipeline`
- `simulation_ready_robot`

### 执行 workflow

基本写法：

```bash
python -m agi_walker.cli workflows run robot_creation_pipeline
```

常用选项：

- `--mock`：使用 mock executors
- `--params key=value ...`：传入工作流参数
- `--force`：强制重跑
- `--resume`：遇到已有产物时跳过对应步骤
- `--output-root <path>`：将输出重定向到指定目录

示例：

```bash
python -m agi_walker.cli workflows run robot_creation_pipeline --mock --resume
python -m agi_walker.cli workflows run robot_creation_pipeline --mock --output-root test_env/manual_run --resume
python -m agi_walker.cli workflows run robot_creation_pipeline --params template=biped_basic mass=42
```

已验证示例：

```bash
python -m agi_walker.cli workflows run robot_creation_pipeline --mock --output-root test_env/doc_check_workflow --resume
```

该命令在 `2026-04-08` 当前工作区执行完成，结果为：

- 状态 `completed`
- 3 个步骤全部成功
- 输出写入 `test_env/doc_check_workflow`

### 验证 workflow 定义

```bash
python -m agi_walker.cli workflows validate robot_creation_pipeline
```

## Doctor 命令

环境自检入口：

```bash
python -m agi_walker.cli doctor
```

当前检查项包括：

- Core Python packages
- Optional / training packages
- 默认端口可用性
- 关键目录读写权限

`doctor` 适合用来快速判断“当前环境是否可以开始本地调试”。

## 常见工作流

### 1. 先确认当前仓库能用

```bash
python -m agi_walker.cli skills list
python -m agi_walker.cli workflows list
python -m agi_walker.cli doctor
```

### 2. 跑一遍最小 workflow

```bash
python -m agi_walker.cli workflows run robot_creation_pipeline --mock --output-root test_env/quick_check --resume
```

### 3. 检查文档和 MCP

```bash
python -m pytest tests/test_mcp_tools.py tests/test_mcp_server.py -q
```

## Smoke 命令

主 smoke：

```bash
python tests/run_smoke_tests.py
```

可选的 Godot headless smoke：

```powershell
$env:AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE='1'
python -m pytest tests/test_godot_headless_smoke.py -q -m integration --tb=short -vv
```

## 常见问题

### 为什么文档里不用 `agi_walker ...` 作为主命令？

因为当前 `pyproject.toml` 没有注册通用 `agi_walker` console script。最稳妥、与当前仓库一致的方式是：

```bash
python -m agi_walker.cli ...
```

### `workflows` 和 `skills workflows` 有什么区别？

当前没有本质区别。`workflows` 是更短的 alias，推荐优先使用。

### 如何查看完整 skill 文档？

```bash
python -m agi_walker.cli skills info <skill-name> -d
```
