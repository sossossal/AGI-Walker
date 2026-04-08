# Modular Robot Builder

更新日期：`2026-04-08`

本页说明 AGI-Walker 当前的“模块化机器人构建”能力。当前这条能力的核心不是 CAD 建模器，而是 parts library、装配清单和 JSON 配置。

## 1. 当前核心资产

### Parts Database

- `parts_library/complete_parts_database.json`

### 管理工具

- `parts_library/parts_manager.py`

### 参考参数表

- `parts_library/PARTS_SPECIFICATIONS.md`

### Godot 侧测试脚本

- `godot_project/scripts/test_parts_library.gd`

## 2. 当前可构建的对象

从现有 parts database 看，当前更像是：

- 零件索引
- 装配套件定义
- BOM 生成基础

而不是复杂的几何约束装配系统。

当前 assembly 分区包括：

- `complete_robot_kits`
- `low_cost_robotd`

其中已有的套件名包括：

- `双足行走机器人完整套件`
- `四足机器人完整套件`
- `低成本双足机器人套件 (DIY版)`

## 3. 当前构建方式

最现实的“builder”路径是：

1. 从 parts database 挑零件
2. 读取 assembly 定义
3. 计算成本和 BOM
4. 结合 workflow / 配置文件导出机器人描述

## 4. PartsManager 提供什么

`PartsLibrary` 当前提供：

- `search_by_type`
- `search_by_name`
- `get_part_by_id`
- `calculate_assembly_cost`
- `generate_bom`

这意味着当前 builder 更偏向：

- 选型
- 成本估算
- BOM 输出

## 5. 与 workflow 的关系

当前模块化构建并没有单独形成独立 CLI builder 命令。更现实的主线是：

- 先用 parts library 做部件与 BOM 层决策
- 再用 skill / workflow 做机器人配置生成与导出

## 6. 不应误解的地方

- 当前没有完整可视化 CAD 装配主线
- Tk GUI 不是当前稳定 builder 主界面
- parts database 不是自动物理求解器

## 7. 推荐使用方式

如果你要做模块化构建：

1. 先从 `parts_library/complete_parts_database.json` 理解分类
2. 用 `parts_manager.py` 搜索和生成 BOM
3. 再将结果映射到 robot config / workflow

## 结论

AGI-Walker 当前的模块化机器人构建能力，本质上是“部件库 + 套件定义 + BOM 工具链”。它适合作为机器人配置和选型前端，而不是完整机械设计平台。
