# AGI-Walker Capacity And Scale

更新日期：`2026-04-15`

本页给出当前客户交付面的容量与规模声明。它只声明当前已经进入交付与验收主链的部署拓扑，不承诺未经专项验证的吞吐、SLA、高可用或多租户能力。

## 声明原则

- 当前容量声明以已验证的部署拓扑和交付文档为依据，不以独立压测基准作为前提。
- 若客户需要固定吞吐、固定并发、长时稳定性、高可用或跨区域承诺，应单独走专项压测与扩展验收。
- 本页应与 `SUPPORT_MATRIX.md`、`KNOWN_LIMITATIONS.md` 和 `CUSTOMER_ACCEPTANCE_CHECKLIST.md` 一起使用，避免只看单页做超范围承诺。

## 当前声明范围

| 场景 | 当前声明 | 当前拓扑 | 当前适用范围 | 当前不声明能力 |
| --- | --- | --- | --- | --- |
| 单机评估版 | 支持 | 单机 Docker Compose，最小集为 `zenoh-router` + `web-panel`，默认 SQLite | 客户演示、UAT、单机控制面部署、基础 workflow 验收 | 高可用、集群数据库、多租户、跨区域 |
| 小规模团队 | 条件支持 | 单套共享 Compose 控制面，少量实施/操作用户共用一套 runtime | 交付验证、试运行、低并发 workflow 操作、支持团队联合验收 | 固定并发量承诺、24x7 SLA、托管式 HA |
| 分布式实验环境 | 条件支持 | Compose 控制面 + distributed profile + 外部 Godot/ROS2 扩展节点 | actor discovery、learner action loop、扩展链路集成演练 | 大规模 actor fleet、生产级多节点编排、互联网规模 |

## 非支持声明

- 当前不声明高可用部署。
- 当前不声明多租户隔离。
- 当前不声明 Helm / Kubernetes 集群规模能力。
- 当前不声明互联网规模或跨区域部署能力。
- 当前不声明托管式数据库集群与自动故障转移。

## 交付要求

- 客户交付前，应先把目标环境映射到“单机评估版”“小规模团队”或“分布式实验环境”之一。
- 若客户目标超出本页范围，应在 `customer_acceptance_bundle` 中补充专项说明和对应 live evidence。
- 若客户长期运行共享环境，应先确认 SQLite、备份恢复、回滚流程和 residual risk 是否满足现场要求。
