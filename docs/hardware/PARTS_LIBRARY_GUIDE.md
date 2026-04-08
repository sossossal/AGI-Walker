# Parts Library Guide

更新日期：`2026-04-08`

本页说明 AGI-Walker 当前的零件库系统。对当前仓库来说，这是最成熟的硬件数据入口之一。

## 1. 核心文件

- `parts_library/complete_parts_database.json`
- `parts_library/parts_manager.py`
- `parts_library/PARTS_SPECIFICATIONS.md`
- `parts_library/cloud_repo_mock/manifest.json`

## 2. 当前数据库结构

`complete_parts_database.json` 当前主要包含两块：

- `parts`
- `assemblies`

### 当前零件分类

- `accessories`
- `communication`
- `controllers`
- `joints`
- `motors`
- `motors_diy`
- `power`
- `sensors`
- `structure`

### 当前 assembly 分区

- `complete_robot_kits`
- `low_cost_robotd`

## 3. 当前工具能力

`parts_manager.py` 提供：

- 按类型搜索
- 按名称搜索
- 按 ID 获取零件
- 计算组件成本
- 生成 BOM
- 列出电机和传感器

这套工具对文档、BOM、选型和 demo 都很有用。

## 4. BOM 生成

仓库里已有一份生成后的参考 BOM：

- `docs/hardware/BOM_biped.txt`

你也可以直接用 `parts_manager.py` 生成，例如：

```bash
python parts_library/parts_manager.py --bom 双足
```

## 5. 云端市场模拟

当前有一个 mock 市场目录：

- `parts_library/cloud_repo_mock/`

其中 `manifest.json` 定义了：

- 名称
- 描述
- 维护者
- license

Web 端也暴露了对应的 mock API：

- `GET /api/parts/market`
- `POST /api/parts/import`

## 6. 当前角色定位

当前零件库主要承担：

- 部件参数数据源
- 套件成本与 BOM 支撑
- Web parts market mock 数据源
- 机器人构建前的选型入口

它不是：

- 实时库存系统
- CAD 装配求解器
- 自动采购平台

## 7. 与其他模块的关系

零件库与这些路径关系最密切：

- 模块化机器人构建
- Web `parts` / `sim2real` 相关接口
- Godot parts 测试脚本
- 文档与 BOM

## 8. 推荐用法

如果要用零件库做实际工作，建议顺序：

1. 先读 `PARTS_SPECIFICATIONS.md`
2. 再用 `parts_manager.py` 搜索和估算
3. 再决定 assembly / robot config
4. 最后结合 workflow 或导出链

## 结论

AGI-Walker 当前的 parts library 已经足够支撑选型、BOM 和套件级构建说明。它是当前硬件文档里最应该优先依赖的结构化数据源。
