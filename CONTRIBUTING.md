# 贡献指南

感谢您考虑为 AGI-Walker 项目做出贡献！

## 如何贡献

### 1. 报告 Bug
如果您发现了 bug，请在 GitHub Issues 中创建一个新 issue，并提供以下信息：
- 复现步骤
- 预期行为
- 实际行为
- 您的环境 (OS, Python版本等)

### 2. 提交新功能
如果您有新功能的想法：
1. 请先在 Issues 中讨论
2. Fork 这个仓库
3. 创建您的特性分支 (`git checkout -b feature/AmazingFeature`)
4. 提交您的更改 (`git commit -m 'Add some AmazingFeature'`)
5. 推送到分支 (`git push origin feature/AmazingFeature`)
6. 开启一个 Pull Request

### 3. 代码风格
- 此项目遵循 PEP 8 规范
- 请确保添加了适当的注释和文档
- 运行测试以确保没有破坏现有功能

## 开发环境设置

```bash
# 克隆仓库
git clone https://github.com/sossossal/AGI-Walker.git

# 安装依赖
pip install -r requirements.txt

# 运行测试
python -m unittest discover tests
```

感谢您的支持！
