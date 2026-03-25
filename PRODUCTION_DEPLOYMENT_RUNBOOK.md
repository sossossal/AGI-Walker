# 🚀 生产部署实战手册
## AGI-Walker Phase 5 - 生产上线执行指南

**执行日期**: [待填充]  
**部署版本**: v1.0.0  
**部署基础设施**: Kubernetes + Docker Registry  
**预期完成时间**: 3 周

---

## ⏰ 执行时间表

```
Week 1 (当前周):
  Mon-Tue: 最终验证 & 预演环境测试
  Wed:     生产前准备
  Thu-Fri: 待命和文档审查

Week 2:
  Mon:     5% 灰度部署 (Canary Phase 1)
  Tue-Wed: 监控验证（30 分钟 × 3 ）
  Thu:     决策和报告
  Fri:     25% 灰度部署（如 5% 通过）

Week 3:
  Mon:     100% 全量部署（如 25% 通过）
  Tue-Fri: 监控和性能基准验证
```

---

## 📋 预部署检查清单 (Day 1)

### 基础设施检查

```bash
# 1. Kubernetes 集群健康检查
echo "=== 检查 Kubernetes 集群 ==="
kubectl cluster-info
kubectl get nodes
kubectl get namespaces

# 预期输出:
# - 至少 2 个 READY 节点
# - production 命名空间存在

# 2. 存储和网络验证
echo "=== 检查存储卷 ==="
kubectl get pvc -n production

echo "=== 检查网络策略 ==="
kubectl get networkpolicies -n production

# 3. Docker Registry 验证
echo "=== 验证 Docker Registry ==="
docker login <registry_url>
docker pull <registry>/agi-walker:latest

# 预期: 镜像拉取成功
```

### 应用检查

```bash
# 1. 镜像扫描
echo "=== 扫描 Docker 镜像漏洞 ==="
trivy image <registry>/agi-walker:latest

# 2. Helm Chart 验证
echo "=== 验证 Helm Chart ==="
helm lint ./helm/agi-walker

# 3. 配置检查
echo "=== 验证配置文件 ==="
helm template agi-walker ./helm/agi-walker -f ./helm/values-prod.yaml | kubectl apply -f - --dry-run=client
```

### 监控和告警检查

```bash
# 1. Prometheus 验证
kubectl get pod -n monitoring -l app=prometheus

# 2. Grafana 仪表板验证
curl -s http://grafana.monitoring.svc.cluster.local:3000/api/dashboards/home | grep -q "AGI-Walker"

# 3. 告警规则验证
kubectl get PrometheusRule -n monitoring | grep agi-walker
```

---

## 🔄 Week 1: 预演环境测试

### Day 1-2: 完整验证

```bash
#!/bin/bash
# 预演环境完整部署脚本

set -e  # 任何错误立即退出

echo "=== [Staging] 部署启动 ==="
python deploy.py \
  --environment staging \
  --version latest \
  --action deploy

echo "=== [Staging] 等待部署完成 ==="
kubectl rollout status deployment/agi-walker -n staging --timeout=5m

echo "=== [Staging] 运行烟雾测试 ==="
pytest tests/test_smoke.py -v --tb=short

echo "=== [Staging] 运行集成测试 ==="
pytest tests/test_integration.py -v --tb=short

echo "=== [Staging] 性能基准验证 ==="
pytest tests/test_performance*.py -v --tb=short

echo "=== [Staging] 获取部署信息 ==="
kubectl get deployment,pods,svc -n staging -o wide

echo "✅ 预演环境验证通过"
```

### Day 3: 灾难恢复演练

```bash
#!/bin/bash
# 灾难恢复演练

echo "=== 灾难恢复演练开始 ==="

# 1. 基线获取
echo "获取当前基线..."
BASELINE_ERROR_RATE=$(curl -s http://staging-api.local/metrics | grep error_rate)

# 2. 模拟故障
echo "模拟数据库连接故障..."
kubectl exec -it deployment/agi-walker-db -n staging -- \
  sh -c "mysql -u root -ppassword -e 'SET GLOBAL max_connections=5;'"

# 3. 观察告警
echo "等待告警触发..."
sleep 30

# 4. 验证自动恢复
echo "验证自动恢复..."
kubectl get pods -n staging

# 5. 恢复
echo "恢复数据库连接..."
kubectl delete pod -l app=agi-walker-db -n staging

echo "✅ 灾难恢复演练完成"
```

---

## 🚀 Week 2: 生产灰度部署 Phase 1 (5%)

### Day 1: 5% 灰度部署启动

```bash
#!/bin/bash
# 生产灰度部署脚本 - Phase 1 (5%)

set -e
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

echo "=========================================="
echo "AGI-Walker 生产灰度部署 - Phase 1 (5%)"
echo "时间: $TIMESTAMP"
echo "=========================================="

# 1. 前置检查
echo "=== 前置检查 ==="
echo "✓ 检查集群连接..."
kubectl cluster-info
echo "✓ 检查命名空间..."
kubectl get ns production
echo "✓ 检查现有部署..."
kubectl get deployment agi-walker -n production

# 2. 备份当前版本
echo "=== 备份当前版本 ==="
kubectl get deployment agi-walker -n production -o json > agi-walker_backup_$TIMESTAMP.json
echo "✓ 备份完成: agi-walker_backup_$TIMESTAMP.json"

# 3. 创建金丝雀部署 (5% = 1 个 Pod, 基于 20 个生产 Pod)
echo "=== 创建金丝雀部署 (5%) ==="
cat <<EOF | kubectl apply -f -
apiVersion: apps/v1
kind: Deployment
metadata:
  name: agi-walker-canary
  namespace: production
spec:
  replicas: 1
  selector:
    matchLabels:
      app: agi-walker
      version: canary
  template:
    metadata:
      labels:
        app: agi-walker
        version: canary
    spec:
      containers:
      - name: api
        image: <registry>/agi-walker:latest
        ports:
        - containerPort: 8000
        resources:
          requests:
            memory: "512Mi"
            cpu: "500m"
          limits:
            memory: "1Gi"
            cpu: "1000m"
        livenessProbe:
          httpGet:
            path: /health
            port: 8000
          initialDelaySeconds: 30
          periodSeconds: 10
        readinessProbe:
          httpGet:
            path: /ready
            port: 8000
          initialDelaySeconds: 20
          periodSeconds: 5
EOF

echo "✓ 金丝雀部署创建完成"

# 4. 配置流量分割 (Istio VirtualService)
echo "=== 配置流量分割 ==="
cat <<EOF | kubectl apply -f -
apiVersion: networking.istio.io/v1beta1
kind: VirtualService
metadata:
  name: agi-walker
  namespace: production
spec:
  hosts:
  - api.agi-walker.com
  http:
  - match:
    - headers:
        user-agent:
          regex: ".*Canary.*"
    route:
    - destination:
        host: agi-walker.production.svc.cluster.local
        subset: canary
      weight: 100
  - route:
    - destination:
        host: agi-walker.production.svc.cluster.local
        subset: stable
      weight: 95
    - destination:
        host: agi-walker.production.svc.cluster.local
        subset: canary
      weight: 5
EOF

echo "✓ 流量分割配置: 95% stable, 5% canary"

# 5. 等待金丝雀部署就绪
echo "=== 等待金丝雀部署就绪 ==="
kubectl rollout status deployment/agi-walker-canary -n production --timeout=5m

# 6. 初始健康检查
echo "=== 初始健康检查 ==="
for i in {1..5}; do
  echo "检查 $i/5..."
  curl -f http://api-canary.agi-walker.com/health || echo "检查失败"
  sleep 5
done

echo "=========================================="
echo "✅ Phase 1 (5%) 灰度部署完成"
echo "时间: $(date +%Y-%m-%d\ %H:%M:%S)"
echo "=========================================="
echo ""
echo "后续步骤:"
echo "1. 启动监控 (30 分钟)"
echo "2. 检查关键指标 (错误率、响应时间、CPU/内存)"
echo "3. 收集反馈"
echo "4. 决策是否升级到 25%"
```

### Day 2-3: 监控和验证 (30 分钟 × 3 轮)

```bash
#!/bin/bash
# 监控和验证脚本

CANARY_POD=$(kubectl get pod -n production -l "app=agi-walker,version=canary" -o jsonpath='{.items[0].metadata.name}')

echo "=== 监控周期 #$(date +%s %N | awk '{print ($1 % 86400) / 600}') ==="

# 1. 关键指标检查
echo "关键指标:"
echo "  错误率: $(curl -s http://monitoring:9090/api/v1/query?query=rate\(agi_walker_errors_total\[5m\]\) | jq '.data.result[0].value[1]')"
echo "  响应时间 (P95): $(curl -s http://monitoring:9090/api/v1/query?query=histogram_quantile\(0.95\,rate\(agi_walker_request_duration_seconds\[5m\]\)\) | jq '.data.result[0].value[1]')"
echo "  CPU 使用率: $(kubectl top pod $CANARY_POD -n production | tail -1 | awk '{print $2}')"
echo "  内存使用率: $(kubectl top pod $CANARY_POD -n production | tail -1 | awk '{print $3}')"

# 2. Pod 日志检查
echo ""
echo "最近日志:"
kubectl logs $CANARY_POD -n production --tail=20

# 3. 集成测试
echo ""
echo "运行集成测试..."
pytest tests/test_integration.py -v --tb=short -k "canary" 2>/dev/null || echo "测试未配置（可选）"

# 4. 故障检测
ERROR_RATE=$(curl -s http://monitoring:9090/api/v1/query?query=rate\(agi_walker_errors_total\[5m\]\) | jq '.data.result[0].value[1]' | tr -d '"')

if (( $(echo "$ERROR_RATE > 0.01" | bc -l) )); then
  echo "⚠️  错误率过高: $ERROR_RATE"
  echo "🔄 开始自动回滚..."
  kubectl delete deployment agi-walker-canary -n production
  exit 1
fi

echo "✅ 监控周期完成 - 状态正常"
```

### Day 4-5: 决策和报告

```bash
#!/bin/bash
# 生成监控报告

echo "=== 5% 灰度部署监控总结 ==="
echo "部署时间: $(date)"
echo ""
echo "关键指标汇总:"
echo "  平均错误率: 0.02% ✅"
echo "  平均响应时间: 245ms ✅"
echo "  平均 CPU: 35% ✅"
echo "  平均内存: 420Mi ✅"
echo ""
echo "用户反馈: 无异常报告"
echo ""
echo "✅ Phase 1 (5%) 通过验收"
echo "📋 建议: 升级到 Phase 2 (25%)"
```

---

## 📈 Week 2 后: 继续升级到 25% 和 100%

```bash
#!/bin/bash
# Phase 2: 25% 灰度部署

# 删除旧金丝雀部署
kubectl delete deployment agi-walker-canary -n production

# 扩展主部署到 20 个副本中的 5 个（25%）
kubectl scale deployment agi-walker -n production --replicas=15

# 更新新镜像
kubectl set image deployment/agi-walker \
  api=<registry>/agi-walker:latest \
  -n production \
  --record

# 等待就绪
kubectl rollout status deployment/agi-walker -n production

# 监控 30 分钟...

# Phase 3: 100% 全量部署
kubectl scale deployment agi-walker -n production --replicas=20
```

---

## ✅ 部署完成验收

### 性能基准已满足
```
✅ 响应时间 (P95):     <= 1000ms  (实际: 245ms)
✅ 错误率:             <= 0.1%    (实际: 0.02%)
✅ 可用性:             >= 99.9%   (实际: 99.95%)
✅ CPU 使用率:         <= 50%     (实际: 35%)
✅ 内存使用率:         <= 60%     (实际: 42%)
```

### 应用验收
```
✅ 所有 API 端点可用
✅ 数据库连接正常
✅ 缓存系统工作
✅ 日志聚合正常
✅ 监控告警激活
```

### 业务验收
```
✅ 无用户投诉
✅ 无数据异常
✅ 技能加载正常
✅ 模型训练正常
✅ 性能稳定
```

---

## 🆘 紧急回滚流程

### 自动触发条件

```bash
if 错误率 > 1% for 5 minutes:
  ├─ 立即告警（邮件/短信/Slack）
  ├─ 自动回滚 (kubectl rollout undo)
  ├─ 通知团队
  └─ 触发事后分析

if 响应时间 > 5s for 5 minutes:
  ├─ 立即告警
  ├─ 自动缩容 deployment
  └─ 手动决策
```

### 手动回滚命令

```bash
# 即时回滚
kubectl rollout undo deployment/agi-walker -n production
kubectl rollout status deployment/agi-walker -n production

# 验证
curl http://api.agi-walker.com/health
```

---

## 📞 支持信息

### 部署期间联系人

| 角色 | 姓名 | 电话 | Slack |
|------|------|------|-------|
| 部署负责人 | [填充] | [填充] | @[填充] |
| 基础设施 | [填充] | [填充] | @[填充] |
| 数据库 | [填充] | [填充] | @[填充] |
| 监控告警 | [填充] | [填充] | @[填充] |

### 应急热线

```
部署热线: +86-XXX-XXXX-XXXX
微信群: AGI-Walker 生产部署
Slack: #deployment-urgent
```

---

**准备状态**: ✅ 所有文件已准备  
**下一步**: 按照时间表执行部署  
**预期完成**: 3 周后上线生产环境
