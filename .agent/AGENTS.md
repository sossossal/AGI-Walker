# AGI-Walker Agent 规范文档

## 项目概述

AGI-Walker 是一个参数化机器人设计和AI训练平台,支持从概念设计到强化学习训练的完整工作流。

## 核心模块

### 1. 参数化控制系统 (`agi_walker/parametric/`)
- 机器人部件的参数化定义
- 模块化组件系统 (Torso/Limb/Joint)
- 配置序列化和验证

### 2. GUI 配置器 (`tools/robot_configurator_gui.py`)
- 可视化拖拽界面
- 实时参数调整
- Godot 仿真预览集成

### 3. Godot 仿真引擎 (`godot_project/`)
- 3D 实时物理仿真
- TCP 通信接口
- 机器人可视化和动画

### 4. AI 数据生成 (`agi_walker/ai_data_generation/`)
- 自动生成训练数据集
- 多种步态模式库
- 环境随机化

### 5. OpenNeuro 通信系统 (`openneuro/`)
- Zenoh 高性能通信
- 分布式 Zone 架构
- IMC-22 芯片集成

### 6. ROS 2 桥接 (`ros2_ws/`)
- ROS 2 消息转换
- 标准机器人接口
- 外部工具集成

---

## 机器人建模工作流

### 标准流程
1. **参数设计阶段**
   - 使用 parametric API 定义机器人部件
   - 或使用 GUI 可视化设计

2. **仿真验证**
   - Godot 实时预览运动学
   - 参数微调和优化

3. **数据生成**
   - 自动创建训练数据集
   - 步态模式生成

4. **模型训练**
   - DeepPole 强化学习
   - 策略优化

5. **部署验证**
   - ROS 2 硬件接口
   - OpenNeuro 通信测试

---

## Skills 系统

### 什么是 Skills?

Skills 是模块化的知识包,让 AI Agent 拥有特定领域的专业能力。每个 skill 包含:
- `SKILL.md`: 文档和使用指南
- `scripts/`: 可执行工具脚本 (可选)
- `references/`: 参考文档 (可选)
- `assets/`: 模板和资源 (可选)

### Skills 位置
所有 skills 存放在: `agi_walker/skills/`

### Skills 分类
- **建模**: 机器人创建和配置
- **优化**: 参数调优和性能提升
- **转换**: 格式转换和导出
- **仿真**: 仿真执行和分析
- **数据生成**: 训练数据创建

### 使用规则
1. Skill 必须包含 `SKILL.md` 文件
2. SKILL.md 必须有 YAML frontmatter (name/description)
3. 依赖通过 `requirements.txt` 声明
4. 遵循渐进式加载原则 (保持文档简洁)

---

## 代码规范

### Python 风格
- 遵循 PEP 8
- 使用 type hints
- 函数和类添加 docstrings

### 文件组织
- 一个文件不超过 500 行
- 相关功能放在同一模块
- 避免循环依赖

### 命名约定
- 类名: `PascalCase`
- 函数/变量: `snake_case`
- 常量: `UPPER_SNAKE_CASE`
- Skills: `kebab-case` (例: `robot-modeling`)

---

## 测试规范

### 单元测试
- 位置: `tests/` 目录
- 命名: `test_<module_name>.py`
- 使用 pytest 框架
- 目标覆盖率: 80%+

### 集成测试
- GUI 测试: 手动测试清单
- 仿真测试: Godot 场景验证
- 端到端: 完整工作流测试

---

## 文档规范

### 代码文档
- 所有公开 API 必须有 docstrings
- 复杂算法添加注释
- README 提供快速开始指南

### Skills 文档
- SKILL.md 保持简洁 (<500行)
- 提供代码示例
- 包含常见问题解答

### 更新规范
- 添加新功能时更新 docs/
- Breaking changes 在 CHANGELOG 中标注
- API 变更更新相应文档

---

## Skills 开发指南

### 创建新 Skill

1. **规划阶段**
   - 明确 skill 的功能和目标用户
   - 确定需要哪些资源 (脚本/文档/模板)

2. **创建目录**
   ```bash
   mkdir agi_walker/skills/my-skill
   cd agi_walker/skills/my-skill
   ```

3. **编写 SKILL.md**
   ```yaml
   ---
   name: my-skill
   description: "详细描述功能和使用场景"
   ---
   
   # My Skill
   
   ## 快速开始
   [代码示例]
   
   ## API 文档
   [详细说明]
   ```

4. **添加资源** (可选)
   - `scripts/`: Python/Bash 脚本
   - `references/`: 详细参考文档
   - `assets/`: 模板文件

5. **测试验证**
   - 确保所有示例代码可运行
   - 添加单元测试
   - 文档审查

### Skills 设计原则

1. **简洁至上**
   - 只包含必要信息
   - 避免重复
   - 使用简单语言

2. **渐进式详细**
   - SKILL.md 保持概览级别
   - 详细内容放入 references/
   - 复杂脚本独立文件

3. **实用导向**
   - 提供可直接使用的代码
   - 包含真实世界示例
   - 清晰的错误处理

---

## 版本控制

### Git 工作流
- 主分支: `main`
- 功能分支: `feature/skill-name`
- 修复分支: `fix/issue-description`

### 提交规范
```
<type>(<scope>): <subject>

<body>

<footer>
```

类型 (type):
- `feat`: 新功能
- `fix`: 修复
- `docs`: 文档
- `refactor`: 重构
- `test`: 测试
- `chore`: 构建/工具

### 发布流程
1. 更新版本号
2. 更新 CHANGELOG.md
3. 创建 release tag
4. 发布到 PyPI (可选)

---

## 依赖管理

### Python 依赖
- 核心依赖: `requirements.txt`
- 开发依赖: `requirements-dev.txt`
- Skill 特定依赖: 在 SKILL.md 中声明

### 版本固定
- 生产环境: 使用确定版本
- 开发环境: 可使用范围版本

---

## 性能考虑

### Skills 加载
- 延迟加载: 仅在需要时读取
- 缓存机制: 避免重复解析
- 最小上下文: 只加载必要部分

### 仿真优化
- 物理步长: 平衡精度和速度
- 渲染优化: LOD 和剔除
- 数据传输: 压缩和批处理

---

## 安全和隐私

### 数据处理
- 不在示例中包含真实数据
- 配置文件使用 .gitignore
- API 密钥使用环境变量

### 代码安全
- 输入验证
- 避免代码注入
- 依赖安全审计

---

## 社区贡献

### 贡献流程
1. Fork 仓库
2. 创建功能分支
3. 提交 Pull Request
4. 代码审查
5. 合并到主分支

### 贡献指南
- 遵循代码规范
- 添加测试
- 更新文档
- 描述清晰的 commit message

---

## 资源链接

- **主仓库**: https://github.com/yourusername/AGI-Walker
- **文档站**: docs/ 目录
- **示例项目**: examples/ 目录
- **问题追踪**: GitHub Issues

---

## 更新日志

### 2026-02-06
- 创建 AGENTS.md 规范文档
- 整合 Moltbot Skills 系统
- 定义 Skills 开发流程

---

**维护者**: AGI-Walker 开发团队
**最后更新**: 2026-02-06
