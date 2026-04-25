# AGI-Walker Known Limitations

更新日期：`2026-04-15`

本页汇总当前客户交付包需要显式声明的已知限制和不支持项。它不是变更日志，而是当前可支持边界的正式补充说明。

## 1. 部署与运行边界

- 当前唯一的一线客户部署路径是 Docker Compose。
- Helm / Kubernetes 当前不作为正式支持入口。
- 默认数据库是 SQLite，适合单机部署，不声明高可用数据库或多节点主备能力。
- 当前日志主入口仍是 `docker compose logs`，不内建集中日志平台。

## 2. 扩展链路边界

- distributed profile 仍依赖外部 Godot TCP 目标或等价环境准备，不属于零配置默认能力。
- Godot headless smoke 与 ROS2 bridge live smoke 都保持 opt-in，不进入默认 `not live` 门禁。
- ROS2 扩展当前以 Humble runtime 为基线；其他发行版未进入正式验收矩阵。
- distributed / ROS2 / Godot 的当前支持边界已进入 `customer_delivery_surface.extension_support_surface`；若客户目标超出该对象的条件支持范围，应先补专项验收，不要默认视为已支持。

## 3. 客户接口与操作边界

- MCP 当前只声明 `stdio` 作为产品化传输面。
- 浏览器当前默认支持基线是最新桌面 Chromium 内核浏览器；Firefox、Safari 和移动浏览器未进入正式验收矩阵。
- 源码启动模式 `python -m web_panel.server` 属于开发者或实施排障入口，不等同于默认客户部署面。

## 4. 安全与运维边界

- 当前不声明托管 secrets manager 集成。
- 当前不声明第三方 SIEM / 审计平台对接。
- 当前不声明 Kubernetes secret / vault 模板。
- `deployment-web-panel-distributed` 的 canonical `104` 条 findings 当前通过 `31` 条 active no-fix exceptions 进入 accepted residual risk，当前到期时间为 `2026-05-15`，在到期前应优先用真实上游修复替换。

## 5. 当前不支持项

- Helm / Kubernetes 生产发布面
- 多租户或集中式身份治理能力声明
- 托管式高可用数据库交付模板
- 移动端浏览器正式支持
- 未进入 support matrix 的环境组合默认支持
- 未附带 `distributed_runtime_live` / `ros2_bridge_live` / `godot_headless_live` 证据的扩展面默认支持

## 使用要求

- 交付前应把本页与 `SUPPORT_MATRIX.md`、`CAPACITY_AND_SCALE.md`、`CUSTOMER_ACCEPTANCE_CHECKLIST.md` 一起纳入 customer acceptance bundle。
- 若某条限制已被专项验收或被正式修复，应同步更新本页、`CURRENT_STATUS.md` 与发布证据。
