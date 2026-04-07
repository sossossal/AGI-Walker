# AGI-Walker CLI 使用指南

## 快速开�?

### 安装

CLI工具已内置在AGI-Walker�?无需额外安装�?

### 基本用法

# CLI 支持两套入口，等效执�?```bash
# 通用 Skills 命令
python -m agi_walker.cli skills <command>

# 直接调用 workflows 子命令（等价�?skills workflows�?python -m agi_walker.cli workflows <subcommand>

# 或使用快捷脚�?agi_walker.bat skills <command>
agi_walker.bat workflows <subcommand>
```

---

## 命令参�?
### 1. 列出所有Skills

```bash
python -m agi_walker.cli skills list
```

**输出示例:**
```
可用 Skills (3 �?:

【建模�?
  🤖 robot-modeling
    快速创建双�?四足/轮式机器人模�?..

【优化�?
  ⚙️ parameter-optimizer
    自动优化机器人参�?质量分布/PID增益)...

【转换�?
  📄 urdf-generator
    将配置转换为URDF/SDF格式...
```

**选项:**
- `-v, --verbose` - 显示详细信息
- `--category <分类>` - 按分类过�?

**示例:**
```bash
# 详细列表
python -m agi_walker.cli skills list -v

# 只看建模类skills
python -m agi_walker.cli skills list --category 建模
```

---

### 2. 查看Skill详情

```bash
python -m agi_walker.cli skills info <skill名称>
```

**示例:**
```bash
python -m agi_walker.cli skills info robot-modeling
```

**输出:**
```
🤖 robot-modeling
============================================================
名称: robot-modeling
分类: 建模
描述: 快速创建双�?四足...
路径: agi_walker\skills\robot-modeling

依赖:
  python_modules: numpy, pydantic

可用脚本:
  - (�?

参考文�?
  - api.md
```

**选项:**
- `-d, --doc` - 显示完整SKILL.md文档

**示例:**
```bash
# 查看完整文档
python -m agi_walker.cli skills info robot-modeling -d
```

---

### 3. 搜索Skills

```bash
python -m agi_walker.cli skills search <关键�?
```

**示例:**
```bash
python -m agi_walker.cli skills search 优化
python -m agi_walker.cli skills search URDF
python -m agi_walker.cli skills search 机器�?
```

**输出:**
```
搜索结果 (2 �?:

⚙️ parameter-optimizer
  自动优化机器人参�?..

📄 urdf-generator
  将配置转换为URDF/SDF格式...
```

---

### 4. 列出分类

```bash
python -m agi_walker.cli skills categories
```

**输出:**
```
Skill 分类 (3 �?:

  优化 (1 个skills)
  建模 (1 个skills)
  转换 (1 个skills)
```

---

### 5. 验证配置

检查所有skills的依赖是否满足�?

```bash
python -m agi_walker.cli skills validate
```

**输出:**
```
验证 Skills 配置...

�?所有skills配置有效
```

**选项:**
- `-v, --verbose` - 显示所有验证结�?

**示例:**
```bash
python -m agi_walker.cli skills validate -v
```

---

### 6. Workflows 快速通道

```bash
python -m agi_walker workflows list
python -m agi_walker workflows run simulation_ready_robot
python -m agi_walker workflows validate robot_creation_pipeline
```

�?alias 直接映射�?`skills workflows`，所以之前的 workflow 子命令所有选项行为一模一样�?
---

### 7. Workflow 执行策略

`workflows run` 现在支持显式指定执行策略，避免“已有产物时到底是重跑还是跳过”这种隐式行为�?
#### 默认行为: `resume`

```bash
python -m agi_walker.cli skills workflows run robot_creation_pipeline
python -m agi_walker.cli workflows run robot_creation_pipeline --resume
```

- 遇到已经存在且非空的 `output_file` 时，步骤会标记为 `SKIPPED`
- `SKIPPED` 现在算成功终态，所以不会再出现 “completed + 0.0% success�?这种误导性结�?
#### 强制重跑: `force`

```bash
python -m agi_walker.cli skills workflows run robot_creation_pipeline --force
```

- 忽略已有产物
- 所有步骤都会重新执�?- 适合回归测试和重新生成产�?
#### 隔离输出目录

如果你不想污染仓库默认的 `.output/` �?`exports/`，可以把相对输出路径重定向到新的根目录：

```bash
python -m agi_walker.cli skills workflows run robot_creation_pipeline ^
  --force ^
  --output-root test_env/workflow_runs/run_001
```

效果�?
- `.output/created_robot.json` 会写�?`test_env/workflow_runs/run_001/.output/created_robot.json`
- `exports/robot.urdf` 会写�?`test_env/workflow_runs/run_001/exports/robot.urdf`
- workflow log 也会进入该隔离目录下�?`.output/`

---

### 8. 最�?Smoke 验收

推荐把下面这个脚本作为最小可信验收入口：

```bash
python tests/run_smoke_tests.py
```

这个脚本会检查：

- Skills CLI 是否可列出内�?- Skills 配置是否有效
- mock workflow 是否可运�?- real workflow 是否可运�?- Web 面板导入�?`WsMessage(type="ping")` 兼容性是否正�?- Godot Agent fake backend 的最�?Web/API 可用�?- 如果存在 external `godot-agent` 目录，再追加 modern backend 的模�?计划/自检 smoke

自定�?smoke 产物目录�?
```bash
python tests/run_smoke_tests.py --output-root test_env/smoke_runs/manual
```

如果要显式指�?modern `godot-agent` 目录�?
```bash
AGI_WALKER_SMOKE_GODOT_AGENT_DIR=/path/to/godot-agent python tests/run_smoke_tests.py
```


## 使用场景

### 场景1: 快速查找Skill

```bash
# 1. 搜索相关skill
python -m agi_walker.cli skills search 建模

# 2. 查看详情
python -m agi_walker.cli skills info robot-modeling

# 3. 查看完整文档
python -m agi_walker.cli skills info robot-modeling -d
```

### 场景2: 检查系统状�?
```bash
# 列出所有skills
python -m agi_walker.cli skills list

# 验证配置
python -m agi_walker.cli skills validate -v

# 运行最�?smoke 验收
python tests/run_smoke_tests.py
```

### 场景3: 浏览文档

```bash
# 列出建模类skills
python -m agi_walker.cli skills list --category 建模

# 查看每个skill的详�?
python -m agi_walker.cli skills info robot-modeling
python -m agi_walker.cli skills info parameter-optimizer
python -m agi_walker.cli skills info urdf-generator
```

---

## 快捷脚本

### Windows

使用 `agi_walker.bat`:

```bash
agi_walker.bat skills list
agi_walker.bat skills info robot-modeling
agi_walker.bat skills search 优化
```

### Linux/macOS (TODO)

创建 `agi_walker.sh`:

```bash
#!/bin/bash
python -m agi_walker.cli "$@"
```

使用:
```bash
chmod +x agi_walker.sh
./agi_walker.sh skills list
```

---

## 提示和技�?

### 1. 快速查看帮�?

```bash
python -m agi_walker.cli --help
python -m agi_walker.cli skills --help
python -m agi_walker.cli skills list --help
```

### 2. 输出重定�?

```bash
# 保存skills列表
python -m agi_walker.cli skills list > skills_list.txt

# 保存skill文档
python -m agi_walker.cli skills info robot-modeling -d > robot_modeling_doc.md
```

### 3. 管道操作

```bash
# 搜索并统�?
python -m agi_walker.cli skills search 机器�?| find /c "skill"

# 过滤结果
python -m agi_walker.cli skills list -v | findstr "建模"
```

---

## 常见问题

**Q: 如何查看skill的脚�?**
A: 使用 `info` 命令会列出可用脚�?然后�?`agi_walker/skills/<skill-name>/scripts/` 目录查看�?

**Q: 如何运行skill的脚�?**
A: 直接使用Python运行,例如:
```bash
python agi_walker/skills/parameter-optimizer/scripts/batch_optimize.py --help
```

**Q: 如何添加新的skill?**
A: 参�?`.agent/AGENTS.md` 中的Skills开发指南�?

---

## 下一�?

- **GUI工具**: 使用图形界面浏览和使用Skills
- **集成到IDE**: 在代码编辑器中直接访问Skills
- **CI/CD集成**: 在自动化流程中使用CLI工具

---

**相关文档:**
- `.agent/AGENTS.md` - Skills系统完整指南
- `agi_walker/skills/*/SKILL.md` - 各Skill的使用文�?
- `examples/*.py` - Python代码示例
