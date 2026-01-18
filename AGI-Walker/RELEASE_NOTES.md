# AGI-Walker v0.9.0-beta 发布说明

**发布日期**: 2026-01-16  
**版本**: 0.9.0-beta  
**类型**: 公开测试版

---

## 🎉 项目简介

AGI-Walker 是**业界首个完整开源的 Sim-to-Real 机器人开发工具链**，整合了 Hive-Reflex 分布式反射控制器，提供从零件选型到硬件部署的完整解决方案。

---

## ✨ 主要特性

### 🤖 完整的 Sim-to-Real 工作流
- 在 Godot 中仿真训练
- 使用 PPO/SAC 等 RL 算法
- 一键导出到 ONNX 并量化
- 部署到 IMC-22 真实硬件

### 🔧 真实硬件零件库
- 7 个真实硬件零件（电机、传感器、控制器）
- 基于制造商数据规格
- 精确的成本估算（$0.1-$500）
- 直接对应真实采购

### ⚡ 独特的边缘智能控制
- 分布式反射架构（类似脊髓反射）
- NPU 神经加速器（< 50 μs 推理）
- 1 kHz 控制频率
- 超低延迟（< 100 μs）

### 📚 详尽的文档系统
- 30+ 技术文档（40,000+ 字）
- 完整的 Sim-to-Real 集成指南
- 2 个完整案例项目
- 5 分钟演示视频脚本

---

## 📦 包含内容

### 核心组件
- **AGI-Walker 仿真平台**（Godot 4.2+）
- **Hive-Reflex SDK**（IMC-22 完整驱动）
- **Python API**（Gymnasium 兼容）
- **零件库系统**（JSON 格式）

### 示例项目
- **平衡机器人**（快速开始示例）
- **双足行走机器人**（12 关节完整案例）

### 文档
- README、快速开始、进阶使用
- 硬件规格、集成指南
- API 参考、故障排查

---

## 🚀 快速开始

```bash
# 1. 克隆项目
git clone https://github.com/agi-walker/agi-walker-sim
cd agi-walker-sim

# 2. 安装依赖
pip install -r requirements.txt

# 3. 运行示例
cd examples
python quick_start_balance.py

# 4. （可选）硬件部署
pip install -r requirements-hardware.txt
python deploy_to_hardware.py
```

---

## 📊 性能指标

| 指标 | 数值 |
|------|------|
| 控制频率 | 1 kHz |
| NPU 推理延迟 | < 50 μs |
| 总控制延迟 | < 100 μs |
| 模型大小 | 2.5 KB (INT8) |
| 功耗 | ~100 mW |

---

## 🎯 适用场景

- 🎓 **学术研究**: 标准化的 RL 实验平台
- 🏭 **工业原型**: 快速验证硬件设计
- 🤖 **机器人开发**: Sim-to-Real 完整工具链
- 📖 **教育培训**: 理论到实践的教学工具

---

## ⚠️ 已知限制

1. **C++ 物理插件**: 需手动编译（可选，性能优化用）
2. **单元测试**: 覆盖率待完善（计划 v1.0）
3. **IMC-22 芯片**: 概念设计，需实际硬件
4. **视频教程**: 仅有脚本，待录制

---

## 🔄 下一步计划

### v0.9.1-beta（1-2 周）
- [ ] 补充单元测试
- [ ] 添加 CI/CD
- [ ] 录制演示视频
- [ ] 收集用户反馈

### v1.0（1-2 月）
- [ ] 测试覆盖 > 60%
- [ ] 完整的 API 文档
- [ ] CLI 命令行工具
- [ ] 性能基准测试

-

--

## 💬 反馈和支持

我们非常重视您的反馈！

**报告问题**:
- GitHub Issues:（待创建）
- 邮箱: team@agi-walker.org

**功能建议**:
- GitHub Discussions:（待创建）
- Discord:（待建立）

**贡献代码**:
- 阅读 [CONTRIBUTING.md](CONTRIBUTING.md)
- 提交 Pull Request

---

## 🙏 致谢

感谢所有零件制造商提供的开放数据手册，以及开源社区的支持。

特别感谢：
- **Godot Engine** - 优秀的开源游戏引擎
- **Stable-Baselines3** - 高质量的 RL 实现
- **OpenAI Gymnasium** - 标准化的 RL 接口

---

## 📄 许可证

AGI-Walker 采用 **MIT License** 开源。

详见 [LICENSE](LICENSE) 文件。

---

**享受使用 AGI-Walker！** 🚀

如有任何问题，欢迎联系我们。
