# Skills Development

更新日期：`2026-04-08`

本页说明如何为 AGI-Walker 当前 skills 系统新增或维护一个 skill。目标是对齐 `SkillsLoader` 的真实解析规则，而不是延续旧目录约定。

## 1. 目录结构

一个最小 skill 目录至少包含：

- `SKILL.md`

常见扩展目录：

- `scripts/`
- `references/`
- `assets/`

推荐位置：

- `agi_walker/skills/<skill-name>/`

目录名建议使用 kebab-case。

## 2. `SKILL.md` 最小模板

```md
---
name: demo-skill
description: 简短描述
category: 示例
emoji: 🧩
inputs:
  source_file:
    type: file_path
    description: 输入文件
    required: true
outputs:
  result_file:
    type: file_path
    description: 输出文件
metadata:
  agi_walker:
    requires:
      python_modules:
        - yaml
---

# Demo Skill

这里写正文说明。
```

## 3. Loader 当前会解析什么

`agi_walker/skills_loader.py` 当前重点解析：

- `name`
- `description`
- `category`
- `emoji`
- `inputs`
- `outputs`
- `metadata.agi_walker.requires`

其中：

- `inputs` 和 `outputs` 会被转成 `SkillParameter`
- `requires` 会用于依赖检查

## 4. 输入输出类型

当前 `ParameterType` 支持：

- `string`
- `number`
- `boolean`
- `dict`
- `list`
- `file_path`

写 skill 文档时，参数类型尽量只使用这套枚举。

## 5. 文档正文怎么写

建议正文至少回答这些问题：

- 这个 skill 做什么
- 输入是什么
- 输出是什么
- 依赖什么环境
- 有没有示例命令或脚本

如果 skill 有额外 API 说明，优先放到：

- `references/api.md`

## 6. 新 skill 如何被发现

当前 loader 会扫描 `agi_walker/skills/` 下所有包含 `SKILL.md` 的目录，并跳过：

- 非目录项
- 以 `.` 或 `__` 开头的目录

## 7. 验证命令

新增或修改 skill 后，至少运行：

```bash
python -m agi_walker.cli skills list
python -m agi_walker.cli skills info <skill-name>
python -m agi_walker.cli skills validate
```

推荐再跑：

```bash
python -m pytest tests/test_skills_loader.py tests/test_skill_validation.py tests/test_skill_docs_utf8.py -q
```

## 8. 与 workflow 的关系

如果一个 skill 需要进入主路径，通常还要补：

- 对应 executor
- workflow step 定义
- 必要的测试或 smoke 覆盖

不要只加 `SKILL.md`，却没有真正可调用的执行路径。

## 9. 编写建议

- `name` 保持稳定，避免频繁改名
- `description` 说清动作，不要写口号
- `requires` 只写真实依赖
- 示例路径尽量使用当前仓库真实入口

## 结论

当前 skills 系统的核心不是目录数量，而是元数据、执行器和 workflow 之间的一致性。写 skill 时先保证 `SKILL.md` 能被 loader 正确理解，再考虑扩展文档和脚本。
