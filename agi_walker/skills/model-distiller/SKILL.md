---
name: model-distiller
description: "将大型教师模型的决策能力蒸馏到轻量学生模型中，用于边缘侧实时推理部署。"
category: 优化
emoji: "⚗️"
inputs:
  teacher_model_path:
    type: file_path
    description: 教师模型权重路径
  student_config:
    type: dict
    description: 学生模型结构配置
  dataset_path:
    type: file_path
    description: 蒸馏数据集路径
  output_path:
    type: file_path
    description: 输出学生模型路径
  temperature:
    type: number
    description: 蒸馏温度
    default: 2.0
    required: false
  alpha:
    type: number
    description: 软目标损失权重
    default: 0.5
    required: false
outputs:
  student_model_path:
    type: file_path
    description: 蒸馏后的学生模型文件
  distillation_report:
    type: dict
    description: 蒸馏结果摘要，包括压缩率和延迟收益
metadata:
  agi_walker:
    requires:
      python_modules:
        - torch
        - numpy
---

# Model Distiller Skill

将大模型策略压缩为适合边缘设备部署的小模型。

当前仓库中的核心入口位于 `scripts/distill_model.py`，主要函数为 `run_distillation(...)`。这个 skill 适合以下场景：

- 将教师策略模型压缩为轻量 MLP 学生模型
- 为 Jetson、工控机或其他边缘设备准备低延迟推理模型
- 在保持决策趋势的前提下降低模型体积和推理成本

## 主要能力

- 定义轻量学生模型 `StudentModel`
- 组合软目标损失与硬目标损失的蒸馏训练目标
- 输出蒸馏后的权重文件和简要收益报告

## 当前实现说明

- 教师模型加载与训练循环目前是简化实现，适合作为 workflow 中的占位能力或原型入口
- `run_distillation(...)` 会保存学生模型权重，并返回状态、压缩率和延迟收益等摘要信息
- 该 skill 当前没有稳定的包级导入包装器，通常由 workflow 或脚本直接调用

## 关键参数

- `teacher_model_path`: 教师模型权重路径
- `student_config`: 学生模型结构配置，常见字段包括 `input_dim`、`hidden_dim`、`output_dim`、`num_layers`
- `dataset_path`: 训练数据集路径
- `output_path`: 学生模型输出路径
- `temperature`: 蒸馏温度，默认 `2.0`
- `alpha`: 软目标损失权重，默认 `0.5`

## 调用约定

建议在 workflow 或上层脚本中调用 `run_distillation(...)`，并显式传入：

- 教师模型路径
- 学生模型配置
- 数据集路径
- 输出模型路径

## 结果

成功时通常返回一个字典，包含：

- `status`
- `student_model`
- `compression_ratio`
- `latency_reduction`
