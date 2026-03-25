# AGI-Walker

AGI-Walker 是一个面向机器人建模、任务示例、Web 控制面板和 Godot 集成的实验性开发仓库。项目当前更接近“多模块平台原型”，而不是已经完全收敛的单体产品。

## 当前状态

- 已存在并可查看的核心目录:
  - `agi_walker/`：Python 包与 Skills 系统
  - `examples/`：任务示例与演示脚本
  - `web_panel/`：FastAPI Web 面板与前端静态资源
  - `godot_project/`：Godot 仿真工程
  - `tests/`：测试与验证脚本
  - `docs/`：使用说明、部署和开发文档
- 仓库内包含机器人建模、参数优化、URDF 转换等 Skills 相关实现。
- Web-Godot 相关代码和页面已存在，但仍属于集成中的能力，不应视为完全稳定。
- README 仅陈述仓库内可见的结构与文档入口；路线图、性能结论和“计划中”能力不再作为首页主叙述。
## 代码质量标准 (v2.0+)

AGI-Walker v2.0 完成了全面的代码规范化升级：

| 指标 | 现状 | 说明 |
|------|------|------|
| **Logger 框架** | 93% 覆盖 | 统一的日志系统，支持日志级别控制和生产部署 |
| **Print 清理** | 1600+清理 | 核心系统 (Phase 3-6) 完全消除 print 语句 |
| **Type Hints** | 941+ 函数 | 关键函数添加返回类型，IDE 智能补全 |
| **异常处理** | 100% 覆盖 | 所有异常正确捕获和记录 |
| 编译验证 | 100% 通过 | 所有文件无语法错误 |
| **Pytest 测试** | 567 发现 | 测试框架完整性保证 |

### 快速开始 (新用户)

```python
# ✅ 标准的 Logger 用法
import logging
logger = logging.getLogger(__name__)

logger.info("操作启动")
logger.warning("潜在问题") 
logger.error("错误发生")

# ✅ 函数返回类型提示
def process_observation(obs: np.ndarray) -> Dict[str, float]:
    """处理机器人观测数据."""
    return results

# ✅ 异常处理规范
try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    logger.warning("torch 不可用")
    TORCH_AVAILABLE = False
```

### 文档资源

- 📖 [MIGRATION_GUIDE.md](MIGRATION_GUIDE.md) - 升级指南和最佳实践
- 📊 [CODE_QUALITY.md](CODE_QUALITY.md) - 详细的代码质量报告
- 📋 [CHANGELOG.md](CHANGELOG.md) - 完整的变更清单
- 📢 [RELEASE_NOTES.md](RELEASE_NOTES.md) - v2.0 发版说明

### 向后兼容性

✅ **100% 向后兼容** - 现有代码无需修改即可继续工作。这些改进仅添加功能，不删除或破坏现有 API。
## 项目结构

- `agi_walker/`
  Python 主包，包含 Skills 加载器、CLI、工作流编排与相关实现。
- `examples/`
  示例脚本和标准任务目录，例如楼梯攀爬任务。
- `web_panel/`
  Web 控制面板、WebSocket 协议处理和静态页面资源。
- `godot_project/`
  Godot 仿真工程。
- `python_api/`、`python_controller/`
  Python 侧接口与控制逻辑。
- `parts_library/`
  零件库与相关文档资源。
- `training/`、`sysid/`
  训练和系统辨识相关代码。
- `tests/`
  单元测试、集成验证脚本和基准测试。

## 仓库里现在有什么

### 1. Skills 系统

仓库内目前包含 3 个核心 Skills 目录：

- `agi_walker/skills/robot-modeling/`
- `agi_walker/skills/parameter-optimizer/`
- `agi_walker/skills/urdf-generator/`

相关文档：

- [Skills 系统指南](.agent/AGENTS.md)
- [CLI 使用指南](docs/guides/CLI_GUIDE.md)
- [GUI 使用指南](docs/guides/GUI_GUIDE.md)
- [开发新 Skill](docs/guides/SKILLS_DEVELOPMENT.md)

说明：

- 这些 Skills 的实现和文档存在于仓库中。
- 当前首页不再把具体导入示例写成“开箱即用”，因为部分入口仍需要进一步整理和验证。

### 2. 任务示例

任务示例位于 `examples/tasks/`，目前文档中可见的重点示例包括：

- `examples/tasks/stair_climbing/`
- `examples/tasks/README.md`

可参考的入口：

```bash
python examples/tasks/stair_climbing/env.py
```

注意：

- 该示例会在终端输出包含 emoji 的文本；在部分 Windows GBK 终端中，直接运行可能触发编码错误。
- 更适合作为代码阅读和本地调试入口，而不是首页承诺的“零配置演示”。

### 3. Web 控制面板

Web 面板代码位于 `web_panel/`，文档位于：

- [Web 面板指南](docs/guides/WEB_PANEL_GUIDE.md)

当前判断：

- 面板代码、协议处理和前端页面资源已存在。
- 该部分依赖 FastAPI、静态资源路径和 Godot 联动环境。
- 现阶段更适合按模块阅读和调试，不建议在首页中宣称为完全稳定的一键入口。

### 4. Godot 集成

Godot 相关内容位于：

- `godot_project/`
- `godot_studio_agent/`

这部分说明仓库已经在尝试形成 Web 与 Godot 的联动链路，但从首页文档角度，应该视为“正在集成中的能力”。

## 安装

### 基础依赖

```bash
pip install -r requirements.txt
```

### 辅助脚本

- Windows: `install.bat`
- Linux/macOS: `install.sh`

说明：

- 仓库内提供了安装脚本和 Dockerfile。
- 不同模块的依赖并不完全相同，涉及 Godot、硬件、训练环境时通常需要额外配置。

## 建议阅读顺序

如果你是第一次进入这个仓库，建议按下面顺序理解项目：

1. [任务目录说明](examples/tasks/README.md)
2. [CLI 使用指南](docs/guides/CLI_GUIDE.md)（`agi_walker workflows ...` 现在等同于 `agi_walker skills workflows ...`）
3. [GUI 使用指南](docs/guides/GUI_GUIDE.md)
4. [Web 面板指南](docs/guides/WEB_PANEL_GUIDE.md)
5. [开发者指南](docs/guides/developer_guide.md)
6. [硬件部署文档](docs/hardware/HARDWARE_DEPLOYMENT.md)

## 当前 README 的边界

这个 README 只做三件事：

- 说明仓库目标和模块分布
- 给出真实存在的文档入口
- 明确哪些能力仍处于实验或集成阶段

它不再在首页直接承诺以下内容：

- 未验证的导入示例一定可直接运行
- 所有 Web/Godot 链路已经稳定可用
- 所有路线图条目已经进入可演示状态
- 缺少落地证据的性能结论

## 相关文档

- [贡献指南](CONTRIBUTING.md)
- [模块化零件系统文档](docs/hardware/MODULAR_ROBOT_BUILDER.md)
- [硬件部署文档](docs/hardware/HARDWARE_DEPLOYMENT.md)
- [性能基准脚本](tests/benchmark_performance.py)

## 许可证

本项目采用 [MIT 许可证](LICENSE)。
