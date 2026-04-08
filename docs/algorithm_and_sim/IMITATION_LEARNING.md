# Imitation Learning

更新日期：`2026-04-08`

本页说明 AGI-Walker 当前的模仿学习模块。当前实现已经覆盖行为克隆和 GAIL，但仍依赖额外第三方库和可用环境。

## 1. 当前模块

文件：

- `agi_walker/core/api/learning/imitation_learning.py`

主要类：

- `ImitationLearner`
- `BehaviorCloning`
- `GAIL`

## 2. 当前支持的方法

### Behavior Cloning

特点：

- 最简单直接
- 训练快
- 依赖专家轨迹质量

### GAIL

特点：

- 对奖励设计要求更低
- 训练链更重
- 依赖更多包和更长时间

## 3. 依赖

常见依赖包括：

- `gymnasium`
- `stable-baselines3`
- `imitation`
- `numpy`

如果缺这些包，模块不会变成主线路径可运行状态。

## 4. 推荐起步方式

和离线 RL 一样，建议先用标准 Gym 环境验证管线，而不是直接套历史占位环境 ID。

也就是说，先证明：

- 专家策略可运行
- 轨迹可收集
- BC / GAIL 训练能起

再迁移到机器人自定义环境。

## 5. 关于示例脚本

文件：

- `examples/imitation_learning_demo.py`

需要注意：

- 它使用了 `AGI-Walker/Walker2D-v0` 风格的示例环境名
- 这更像示意模板，不是默认安装后立即可跑的 demo

## 6. 一个务实流程

1. 选一个真实可运行的 Gym 环境
2. 训练或加载专家策略
3. 用 `collect_expert_demos()` 采集数据
4. 先跑 BC
5. 需要更强表达时再跑 GAIL

## 7. 与仓库其他模块的关系

模仿学习模块更偏训练实验层，它与这些能力是并行关系：

- `offline_rl.py`
- `rl_optimizer.py`
- `GodotRobotEnv`

但它不是 workflow、CLI 或 Web 的默认主线。

## 8. 常见问题

- `imitation` 库未安装
- 环境 ID 不存在
- 专家策略本身效果不够好
- 轨迹格式和向量化环境不匹配

## 结论

AGI-Walker 当前的模仿学习模块已经具备 BC 和 GAIL 骨架，但它应被视为研究/训练能力。先用标准环境验证依赖和流程，再考虑接入自定义机器人环境。
