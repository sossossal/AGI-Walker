# Skills Index

更新日期：`2026-04-08`

本页列出当前 `agi_walker/skills/` 中实际存在并可被 `SkillsLoader` 扫描到的 skills。

## 当前 Skills

### 建模

#### `robot-modeling`

- 分类：`建模`
- 描述：快速创建双足和四足机器人配置，支持模板加载、参数化建模和 JSON 导出。
- 路径：[agi_walker/skills/robot-modeling](../agi_walker/skills/robot-modeling)

说明：

- 当前目录下没有独立 `scripts/`，主要通过 skill 文档和系统集成使用。

### 优化

#### `parameter-optimizer`

- 分类：`优化`
- 描述：自动优化机器人质量分布与 PID 控制参数，支持梯度法、遗传算法和批量调优。
- 路径：[agi_walker/skills/parameter-optimizer](../agi_walker/skills/parameter-optimizer)
- 脚本：
  - [batch_optimize.py](../agi_walker/skills/parameter-optimizer/scripts/batch_optimize.py)

#### `model-distiller`

- 分类：`优化`
- 描述：将大型教师模型的决策能力蒸馏到轻量学生模型中，用于边缘侧实时推理部署。
- 路径：[agi_walker/skills/model-distiller](../agi_walker/skills/model-distiller)
- 脚本：
  - [distill_model.py](../agi_walker/skills/model-distiller/scripts/distill_model.py)

### 转换

#### `urdf-generator`

- 分类：`转换`
- 描述：将 AGI-Walker 机器人配置转换为 URDF 或 SDF 文件，用于仿真器和 ROS 生态集成。
- 路径：[agi_walker/skills/urdf-generator](../agi_walker/skills/urdf-generator)
- 脚本：
  - [batch_convert.py](../agi_walker/skills/urdf-generator/scripts/batch_convert.py)

## CLI 查看方式

列出所有 skills：

```bash
python -m agi_walker.cli skills list
```

查看单个 skill：

```bash
python -m agi_walker.cli skills info robot-modeling
python -m agi_walker.cli skills info parameter-optimizer -d
```

按分类过滤：

```bash
python -m agi_walker.cli skills list --category 建模
python -m agi_walker.cli skills list --category 优化
python -m agi_walker.cli skills list --category 转换
```

## Python 访问方式

```python
from agi_walker.skills_loader import get_skills_loader

loader = get_skills_loader()
skills = loader.get_skills_list()
robot_modeling = loader.get_skill("robot-modeling")
```

常用能力：

- `get_skills_list()`
- `get_skill(name)`
- `get_categories()`
- `search_skills(query)`
- `get_skill_doc(name)`

## 与 MCP / Web 的关系

Skills 会通过多条路径暴露：

- CLI：`python -m agi_walker.cli skills ...`
- MCP：`skills_list` / `skill_get`
- Web：`GET /api/skills/catalog`、`GET /api/skills/list`

如果某个 skill 的 metadata、分类或描述需要变更，优先检查该 skill 目录下的 `SKILL.md` frontmatter。
