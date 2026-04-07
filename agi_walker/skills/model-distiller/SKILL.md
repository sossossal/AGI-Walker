---
name: model-distiller
description: 将大规模教师模型的决策知识蒸馏到轻量级学生模型中，以实现边缘端的实时推理�?
category: 优化
emoji: ⚗️
requires:
  python_modules:
    - torch
    - numpy
---

# ⚗️ Model Distiller Skill

�?Skill 提供工业级的模型蒸馏能力，旨在将复杂的机器人决策模型（如 3B 参数�?Transformer）转化为适用于边缘嵌入式设备（如 Jetson Orin）的高性能轻量化网络�?

## 核心输入 (Inputs)

- `teacher_model_path`: 原始教师模型路径 (.pt)
- `student_config`: 学生模型架构配置 (dict)
- `training_dataset`: 离线经验数据�?(.h5)
- `temperature`: 蒸馏温度 (default: 2.0)

## 核心输出 (Outputs)

- `student_model_path`: 蒸馏后的轻量化模型路�?
- `distillation_report`: 性能对比与压缩率报告

## 使用示例

```bash
agi_walker skills run model-distiller train --teacher_model_path ./weights/teacher.pt --student_config student_tiny.json
```
