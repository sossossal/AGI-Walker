# AGI-Walker Release Checklist

在使用 `main` 分支发布新版本前，请务必确认以下事项：

## 1. 代码质量门禁 (CI Gates)
- [ ] **Lint Check**: `ruff` 和 `black` 检查全通过 (无 `continue-on-error`)。
- [ ] **Unit Tests**: 所有单元测试在 Ubuntu 和 Windows 上均为绿色。
- [ ] **Coverage**: 核心包 `agi_walker/` 覆盖率达到基线 (当前建议 > 60%)。

## 2. 功能回归验证
- [ ] **URDF Export**: 运行 `verify_pipeline.py`，确保生成的 URDF 结构完整（含 Thigh/Shin）。
- [ ] **Web Panel**: 启动 `server.py`，确保 `design.html` 能成功生成机器人。
- [ ] **Examples**: 运行 `python examples/skills_demo.py` 无报错。

## 3. 依赖与环境
- [ ] **Requirements**: `requirements.txt` 已锁定且无冲突。
- [ ] **PyTest**: 本地运行 `pytest` (不带参数) 应默认为 Pass。

## 4. 文档与版本
- [ ] **VERSION**: `VERSION` 文件内容已更新。
- [ ] **Changelog**: `CHANGELOG.md` 已记录本次变更。
- [ ] **Docs**: 文档中的示例代码已同步更新。

---
> **注意**: 如果 CI 为红色，**严禁合并**到 `main` 分支！
