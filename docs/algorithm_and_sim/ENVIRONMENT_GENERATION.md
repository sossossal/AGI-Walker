# Environment Generation

更新日期：`2026-04-08`

本页说明 AGI-Walker 当前仓库里的环境生成相关资源。这里的“生成”主要发生在 Godot 侧，而不是 Python CLI 主线。

## 1. 当前相关资源

Godot 环境脚本主要位于：

- `godot_project/scripts/environment/procedural_terrain.gd`
- `godot_project/scripts/environment/environment_controller.gd`
- `godot_project/scripts/environment/environment_ui.gd`
- `godot_project/scripts/environment/ground_material.gd`
- `godot_project/scripts/environment/ground_material_library.gd`
- `godot_project/scripts/environment/ground_tilt_controller.gd`

测试场景：

- `godot_project/scenes/test_environment.tscn`

## 2. 当前能力边界

从现有文件看，环境生成目前主要覆盖：

- 环境参数预设
- 重力、风、温度、摩擦等环境因素
- 程序化地形脚本
- 地面材质管理
- 测试 UI / 控制器

它不是一个成熟的 Python 环境生成 API。

## 3. Python 侧的对应点

Python 侧与环境复杂度更相关的模块是：

- `GodotRobotEnv`
- `RandomizationManager`
- `ElevationMapBuilder`

作用分别是：

- 与 Godot 环境通信
- 做域随机化
- 对局部地形进行建图分析

## 4. 推荐理解方式

把当前环境生成链路理解成两层：

### Godot 层

- 负责场景和物理环境本体

### Python 层

- 负责参数随机化、传感器消费和算法侧分析

## 5. 适合的使用场景

- 地形与材质测试
- 环境扰动实验
- Sim2Real 训练前的随机化增强
- Godot 侧快速场景验证

## 6. 当前不应夸大的部分

- 没有统一环境生成 CLI
- 没有标准化资产市场式环境模板系统
- 没有保证所有 Godot 环境脚本都已经纳入主线回归

## 7. 推荐实践

如果你要用环境生成能力：

1. 先从 `test_environment.tscn` 入手
2. 用 `environment_controller.gd` 验证预设切换
3. 再把随机化参数映射到 Python 训练环境

## 结论

AGI-Walker 当前的环境生成能力主要是 Godot 侧脚本资产，而不是标准 Python 产品接口。对仓库主线来说，它属于增强层，不是最先依赖的基础层。
