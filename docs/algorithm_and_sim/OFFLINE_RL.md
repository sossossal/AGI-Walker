# Offline RL

更新日期：`2026-04-12`

本页说明 AGI-Walker 当前的离线强化学习模块。它已经有真实代码，但仍依赖额外三方库和可用环境，不属于默认主线路径。

## 1. 当前模块

文件：

- `agi_walker/core/api/learning/offline_rl.py`

主要类：

- `ExpertDataCollector`
- `OfflineRLTrainer`

当前算法实现重点是：

- CQL（Conservative Q-Learning）

## 2. 依赖

至少需要：

- `gymnasium`
- `numpy`
- `d3rlpy`

如果你要先验证导入链，可以直接运行：

```bash
python -m pytest tests/test_offline_rl.py -q
```

该测试在依赖缺失时会跳过，而不是强行失败。

## 3. 当前工作流

离线 RL 的基本路径是：

1. 从专家策略收集数据
2. 保存离线数据集
3. 转成 `MDPDataset`
4. 用 CQL 训练
5. 必要时再在线 fine-tune

`OfflineRLTrainer.train_offline()` 当前会在 `save_dir` 下写出：

- `training_run_manifest.json`

该 manifest 使用 `training_run` artifact v1，记录 `offline_dataset_training` 的算法、环境、输入数据摘要、训练步数、训练耗时、模型目录和硬件边界。该路径默认 `hardware_required=false`、`hardware_enabled=false`，不应访问真实硬件。

## 4. 推荐的起步方式

不要一开始就用仓库里的历史占位环境 ID。建议先从标准 Gym 环境验证工具链，例如：

```python
from agi_walker.core.api.learning.offline_rl import ExpertDataCollector

collector = ExpertDataCollector("CartPole-v1")
```

这也是当前测试里采用的更稳妥路径。

如果只验证 manifest 契约和离线训练写盘逻辑，可运行：

```bash
python -m pytest tests/test_training_contracts.py tests/test_offline_rl.py -q
```

## 5. 关于示例脚本

文件：

- `examples/offline_rl_demo.py`

需要注意：

- 它使用 `AGI-Walker/Walker2D-v0` 这类占位环境 ID
- 它假设已有专家模型文件

因此它更像训练模板，而不是默认开箱即用 demo。

## 6. 与在线 RL 的关系

仓库里另一条更完整的训练链路在：

- `agi_walker/core/controllers/rl_optimizer.py`

区别：

- `offline_rl.py` 重点是离线数据集 + CQL
- `rl_optimizer.py` 重点是 SB3 在线训练与 ONNX 导出

## 7. 常见坑

- `d3rlpy` 没装
- 环境 ID 不可用
- 没有专家策略文件
- 数据集字段格式不符合 `MDPDataset`

## 8. 何时使用离线 RL

适合：

- 先有专家数据或历史轨迹
- 想减少在线交互成本
- 想先在标准环境验证离线管线

不适合：

- 还没跑通基础仿真链路
- 还没有任何可用数据集

## 结论

AGI-Walker 当前已经有可用的离线 RL 模块，但它是专项训练能力，不是默认主线。最务实的方式是先用标准 Gym 环境验证工具链，再考虑迁移到自定义机器人环境。
