# OpenNeuro Integration

更新日期：`2026-04-08`

本页说明仓库里的 `openneuro/` 相关内容。当前它更适合被理解为实验性集成与 MVP 资料区，而不是 AGI-Walker 主 CLI / Web / workflow 路径的一部分。

## 1. 当前仓库里有什么

重点目录：

- `openneuro/mvp/`
- `openneuro/cortex/`
- `openneuro/docs/`

其中 `openneuro/mvp/README.md` 已经明确说明它是最小可行性验证路径。

## 2. MVP 模拟链

当前最直接的实验入口是：

- `openneuro/mvp/simulation/virtual_ganglion.py`

它模拟：

- Zone Controller / Ganglion
- Zenoh 通信
- 模拟关节状态
- 模拟 IMC-22 推理输出

主题结构大致是：

- `zone/<id>/cmd`
- `zone/<id>/state`
- `zone/<id>/imc22/output`

## 3. 固件参考

仓库里同时保留了若干 firmware 参考目录，例如：

- `openneuro/mvp/firmware/stm32/`
- `openneuro/neuron/firmware/esp32_demo/`

这些更像硬件或通信方向的参考材料，不应被写成当前 Web/CLI 的默认依赖。

## 4. 与 AGI-Walker 主线的关系

当前 OpenNeuro 相关内容和主线的关系更像“边缘研究分支”：

- 使用 Zenoh 主题通信
- 探索控制器/分区节点架构
- 补充硬件和边缘控制方向的原型

但它没有直接进入：

- `python -m agi_walker.cli`
- `python -m web_panel.server`
- `agi-walker-mcp`

## 5. 什么时候值得看它

适合：

- 做 Zenoh 通信实验
- 做分区控制器原型
- 看硬件 / 固件方向的参考实现

不适合：

- 普通用户上手
- workflow 文档主线
- MCP 或 Web 功能排错

## 6. 最小实验路径

如果你确实要验证 OpenNeuro MVP，建议：

1. 先准备 Zenoh router
2. 跑 `virtual_ganglion.py`
3. 用 Zenoh CLI 订阅和发布测试消息

但这属于独立实验路径，不应与 AGI-Walker 主 smoke 混在一起。

## 7. 当前边界

- 文档和代码仍偏 MVP
- 与主线回归测试耦合较弱
- 部分内容面向未来硬件集成，而非当前默认安装路径

## 结论

OpenNeuro 集成在当前仓库里是实验性研究资产，而不是主产品面。最合理的定位，是把它当作 Zenoh / firmware / 分区控制方向的资料和原型区。
