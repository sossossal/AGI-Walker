# 🎯 阶段2 任务计划: Workflow 闭环修复

**启动日期:** 2026-03-24  
**状�?** 📋 **任务分析�?* �?**立即实施**

---

## 📊 现状分析

### 已就位的基础设施
�?workflow_orchestrator.py:
- WorkflowStatus, StepStatus 枚举定义完整
- WorkflowOrchestrator 类有workflow注册机制
- 定义�?个builtin workflow:  
  - robot_creation_pipeline (Model �?Optimize �?Export)
  - simulation_ready_robot (Load �?Validate �?Export)

�?skill_executors.py:
- SkillExecutor 基类定义完整
- RobotModelingExecutor/ParameterOptimizerExecutor/UrdfGeneratorExecutor已定�?
- 基本action处理已实�?

### 存在的问�?

�?**问题1: Mock vs Real 混乱**
- 当前所有executor都是mock实现（只返回硬编码数据）
- 没有真正调用actual skill模块
- API看起来完整但实际不工�?

�?**问题2: 无法真正运行Workflow**
- cmd_workflows_run 在skills_cli.py中存在但没有结构化输�?
- workflow没有真正的end-to-end验证
- 失败路径不清�?

�?**问题3: Skills 兼容层不完整**
- robot_modeling.py 只是wrapper，没有exposed出execution接口
- parameter_optimizer.py 没有暴露optimize接口
- urdf_generator.py 没有暴露export接口

---

## 🎯 修复目标

### 目标1: Real Executor 实现
**范围:** skill_executors.py

创建真实执行器替代mock，直接调用skill API�?
```python
class RealRobotModelingExecutor(SkillExecutor):
    def execute(self, action, inputs):
        if action == 'create_from_template':
            # 真正调用
            from agi_walker.skills.robot_modeling import load_template
            config = load_template(inputs['template'])
            # 保存到输出文�?
            config.save(inputs['output_file'])
            return {'status': 'success', 'output_file': inputs['output_file']}
```

### 目标2: Executor Selection
**范围:** skill_executors.py + workflow_orchestrator.py

提供选择real or mock的机制：
```python
get_skill_executor(name, use_mock=False)
```

### 目标3: Workflow End-to-End
**范围:** workflow_orchestrator.py + skills_cli.py

完整的workflow执行链：
1. 从CLI启动 �?python -m agi_walker.cli skills workflows run robot_creation_pipeline
2. Orchestrator 按序执行步骤
3. 返回结构化的WorkflowResult
4. 显示清晰的进度和结果

### 目标4: Skills 导出接口完善
**范围:** agi_walker/skills/*.py

为skill暴露execution接口�?
- robot_modeling.py: `create_from_template()`, `load_config()`, `save()`
- parameter_optimizer.py: `optimize_mass()`, `validate_physics()`
- urdf_generator.py: `export_to_format()`

---

## 📝 详细实施步骤

### 步骤1: 分离Mock和Real Executor
**文件:** skill_executors.py

```python
# 基础接口保持不变
class SkillExecutor(ABC):
    @abstractmethod
    def execute(self, action, inputs) -> Dict[str, Any]:
        pass

# 创建Mock基类（当前实现）
class MockExecutor(SkillExecutor):
    """用于测试的Mock执行�?""
    pass

# 创建Real实现
class RealRobotModelingExecutor(SkillExecutor):
    """真实调用skill的执行器"""
    def execute(self, action, inputs):
        from agi_walker.skills.robot_modeling import ...
        # 真正实现

# 工厂函数
def get_skill_executor(name, use_mock=False):
    if use_mock:
        return get_mock_executor(name)
    else:
        return get_real_executor(name)
```

### 步骤2: 改进Skills导出接口  
**文件:** agi_walker/skills/robot_modeling.py

�?
```python
export load_template, list_templates
```

改为:
```python
def create_from_template(template_name: str, output_file: str) -> Dict:
    config = load_template(template_name)
    config.save(output_file)
    return {'status': 'success', 'output_file': output_file}
```

### 步骤3: 改进WorkflowOrchestrator
**文件:** workflow_orchestrator.py

添加run_workflow()的真实实现：
```python
def run_workflow(self, name, params=None, use_mock=False):
    workflow = self.workflows[name]
    result = WorkflowResult(workflow_name=name)
    
    for step in workflow['steps']:
        executor = self.get_executor(step['skill_executor'], use_mock)
        try:
            step_output = executor.execute(step['action'], step['inputs'])
            result.add_step_result(step['name'], step_output)
        except Exception as e:
            result.add_step_error(step['name'], str(e))
            return result
    
    return result
```

### 步骤4: CLI命令完善
**文件:** skills_cli.py

改进cmd_workflows_run():
```python
def cmd_workflows_run(args):
    orchestrator = get_workflow_orchestrator()
    use_mock = not args.real  # --real flag启用真实executor
    
    try:
        result = orchestrator.run_workflow(args.name, use_mock=use_mock)
        
        print(f"\n工作流执�? {result.workflow_name}")
        print(f"状�? {result.status}")
        print(f"\n步骤执行:")
        for step in result.steps:
            status_mark = "�? if step['status'] == 'success' else "�?
            print(f"  {status_mark} {step['name']}: {step['status']}")
            if 'output_file' in step['output']:
                print(f"      输出: {step['output']['output_file']}")
        
        return 0 if result.status == 'success' else 1
    except Exception as e:
        print(f"错误: {e}", file=sys.stderr)
        return 1
```

---

## �?验收标准

### 验证1: Real Executor 工作
```bash
$ python -c "
from agi_walker.skill_executors import get_skill_executor
exec = get_skill_executor('robot_modeling', use_mock=False)
result = exec.execute('create_from_template', {
    'template': 'biped',
    'output_file': 'test_output.json'
})
assert result['status'] == 'success'
print('�?Real executor works')
"
```

### 验证2: Mock Executor 保留
```bash
$ python -c "
from agi_walker.skill_executors import get_skill_executor
exec = get_skill_executor('robot_modeling', use_mock=True)
result = exec.execute('create_from_template', {...})
assert result['status'] == 'success'
print('�?Mock executor still works')
"
```

### 验证3: Workflow 执行成功
```bash
$ python -m agi_walker.cli skills workflows run robot_creation_pipeline --mock
```

**预期输出:**
```
工作流执�? robot_creation_pipeline
状�? success

步骤执行:
  �?create_model: success
      输出: weights/created_robot.json
  �?optimize_params: success
      输出: weights/optimized_robot.json
  �?export_urdf: success
      输出: exports/robot.urdf
```

### 验证4: 错误处理
```bash
$ python -m agi_walker.cli skills workflows run robot_creation_pipeline --mock --fail-at=step2
```

**预期输出:**
```
工作流执�? robot_creation_pipeline
状�? failed

步骤执行:
  �?create_model: success
  �?optimize_params: failed
      错误: Simulated failure
  �? export_urdf: skipped
```

---

## 📊 修改范围

| 文件 | 改动 | 优先�?|
|------|------|--------|
| skill_executors.py | +Real实现, +Mock分类 | P0 |
| workflow_orchestrator.py | +WorkflowResult完善 | P0 |
| skills_cli.py | +workflows命令改进 | P0 |
| skills/robot_modeling.py | +execution接口 | P1 |
| skills/parameter_optimizer.py | +execution接口 | P1 |
| skills/urdf_generator.py | +execution接口 | P1 |

---

## 🔄 工作流程

1. **准备** (5min) - 分析现有代码
2. **实施** (30min) - 创建Real executor + 改进orchestrator
3. **验证** (10min) - 运行4个验收标�?
4. **优化** (10min) - 改进错误提示和状态展�?

---

**预计完成时间:** 1小时  
**关键依赖:** �? 
**风险:** 低（测试覆盖完整�?
