# GUI User Guide

更新日期：`2026-04-08`

本页从使用者视角说明 AGI-Walker 当前有哪些可用图形界面，以及推荐从哪里开始。

## 1. 最推荐的入口

启动 Web Panel：

```bash
python -m web_panel.server
```

然后在浏览器中打开：

```text
http://127.0.0.1:8000/
```

## 2. 常用页面

### 主页

- `/`

适合先确认服务已启动。

### Workflow 页面

- `/static/workflows.html`

适合查看工作流运行、日志、产物和后续 Godot 同步。

### Godot 控制页面

- `/static/godot-control.html`

适合查看 Godot 相关控制入口。

### Distributed 页面

- `/static/distributed.html`

仅在分布式环境里有明显价值。

### Nightly 页面

- `/static/nightly.html`

适合查看 nightly / regression 状态聚合。

## 3. 什么时候用桌面 GUI

如果你只是：

- 查看 skills
- 浏览技能文档

可以使用 PyQt skills 浏览器：

```bash
python -m agi_walker.gui.skills_browser
```

前提是已安装 `PyQt6`。

## 4. 当前不建议依赖的桌面工具

`tools/robot_configurator_gui.py` 仍然存在，但它更像实验性或遗留工具，不是当前推荐主入口。除非你明确知道自己在修这条线，否则不要把它当作默认使用界面。

## 5. 遇到界面打不开时先检查什么

### Web Panel 打不开

先运行：

```bash
python -m agi_walker.cli doctor
```

重点检查：

- 端口是否被占用
- 核心依赖是否已安装

### PyQt GUI 打不开

常见原因：

- 没安装 `PyQt6`
- 运行在无图形环境

## 6. 推荐使用顺序

1. 先用 Web Panel
2. 需要浏览 skills 时再开 PyQt skills 浏览器
3. 不要默认使用历史 Tk 工具

## 结论

对当前用户来说，AGI-Walker 的 GUI 主入口就是 Web Panel。桌面 GUI 只适合作为补充工具，而不是默认工作台。
