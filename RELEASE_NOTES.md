# AGI-Walker v1.0.0 发版说明

**发布日期**: 2026-04-02  
**状态**: 🚀 **V1 计划全量收口 (RC)**

---

## 🎯 版本概览

AGI-Walker v1.0.0 是项目从实验性原型向“标准化机器人工作流平台”转型的关键里程碑。本版本完成了 V1 计划中定义的所有核心主线，确立了 Skills、Workflow、Web 与 Godot 的官方集成标准。

## 🚀 核心更新

### 1. Workflow 闭环产品化
- **Real Executor**: 官方 `robot_creation_pipeline` 已全面接入真实 Skills，不再仅作为 Mock 演示。
- **端到端追踪**: 从 CLI 触发到 Web 端查看 SSE 实时日志，实现了完整的任务追踪闭环。
- **策略控制**: 支持 `resume` / `force` 执行策略，并引入参数覆盖机制。

### 2. Web 运维看板 (Nightly)
- **Nightly 看板**: 新增 `/static/nightly.html`，实时对接 GitHub Actions，展示核心 Job (`smoke`, `distributed-smoke`, `godot-headless-smoke`) 的运行状态。
- **本地复现**: 提供标准化的复现命令与 Artifact 关联，大幅缩短故障排查链路。

### 3. Godot 官方链路
- **Session Bridge**: 正式确立为 Workflow 产物送往 Godot 的标准路径，支持一键同步机器人配置。
- **自动化验证**: Headless Smoke 烟雾测试已稳定，确保仿真环境的 API 兼容性。

### 4. 仓库质量体系
- **V1 门禁**: 建立了包含质量检查、基础烟雾测试与 Godot 专项回归的完整流水线。
- **环境隔离**: 默认采用 `test_env/` 作为所有测试与 Web 运行的产物根目录，保持仓库整洁。

---

## 🛠️ 安装与升级

### 新用户
```bash
pip install -r requirements.txt
python tests/run_smoke_tests.py
```

### 现有用户 (从 v0.9.x / v2.0.0 升级)
1. 建议清理旧的 `.output/` 目录。
2. 配置 `AGI_WALKER_WEB_ENV_FILE` 以启用新的归档保留策略。
3. 参考 `PRODUCTION_DEPLOYMENT_RUNBOOK.md` 更新部署环境。

---

## 📚 关键文档

- [V1 状态宣言](docs/CURRENT_STATUS.md)
- [部署手册](PRODUCTION_DEPLOYMENT_RUNBOOK.md)
- [CLI 使用指南](docs/guides/CLI_GUIDE.md)

---

**发布团队**: AGI-Walker Core Team  
**质量评级**: ⭐⭐⭐⭐⭐ (V1 闭环验收通过)
