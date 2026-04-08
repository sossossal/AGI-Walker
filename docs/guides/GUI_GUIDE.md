# GUI Guide

更新日期：`2026-04-08`

AGI-Walker 当前并没有单一“官方桌面 GUI”。现有图形界面大致分成三类：Web 控制台、PyQt skills 浏览器，以及实验性的 Tk 工具。

## 1. 优先使用 Web GUI

当前最稳定的 GUI 入口是 Web Panel：

```bash
python -m web_panel.server
```

启动后常用页面包括：

- `/`
- `/static/workflows.html`
- `/static/godot-control.html`
- `/static/nightly.html`
- `/static/distributed.html`

对大多数用户和维护者，这就是默认 GUI。

## 2. PyQt Skills Browser

文件：

- `agi_walker/gui/skills_browser.py`

作用：

- 浏览已加载 skills
- 搜索和按分类过滤
- 查看 `SKILL.md`
- 打开 skill 目录

依赖：

- `PyQt6`

运行方式：

```bash
python -m agi_walker.gui.skills_browser
```

如果没有安装 `PyQt6`，程序会直接退出并提示安装。

## 3. 实验性 Tk 配置器

文件：

- `tools/robot_configurator_gui.py`

这个工具仍然保留，但它带有明显的历史遗留痕迹：

- 依赖旧导入路径
- 依赖额外桌面图形库
- 并不属于当前主线回归入口

因此建议把它视为实验性工具，而不是默认支持的正式 GUI。

## 4. GUI 与 workflow 的关系

当前更推荐的 GUI 设计方向是：

- 用 Web 展示 workflow runs
- 用 Web 下载 artifacts
- 用 Web 发起 Godot sync

而不是继续为每个单独功能新增一套桌面窗口。

## 5. GUI 维护建议

如果你要维护或新增 GUI，请优先遵守下面的顺序：

1. 先确认 Web 是否已经能承载该功能
2. 再考虑是否需要桌面 GUI
3. 如果是桌面 GUI，明确它是主线工具还是实验工具

## 6. 测试建议

GUI 相关改动优先配套：

```bash
python -m pytest tests/test_gui_layer.py -q
python -m pytest tests/test_web_panel_integration_routes.py -q
```

如果你改的是 Web GUI，额外建议跑：

```bash
python tests/run_smoke_tests.py
```

## 结论

当前 GUI 主线是 Web Panel。PyQt skills 浏览器是可选补充，Tk 配置器属于实验性遗留工具。新功能应优先往 Web 收口。
