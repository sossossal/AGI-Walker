# 棣冨箚 闂冭埖顔? 娴犺濮熺拋鈥冲灊: Workflow 闂傤厾骞嗘穱顔碱槻

**閸氼垰濮╅弮銉︽埂:** 2026-03-24  
**閻樿埖鈧?** 棣冩惖 **娴犺濮熼崚鍡樼€芥稉?* 閳?**缁斿宓嗙€圭偞鏌?*

---

## 棣冩惓 閻滄壆濮搁崚鍡樼€?

### 瀹告彃姘ㄦ担宥囨畱閸╄櫣顢呯拋鐐煢
閴?workflow_orchestrator.py:
- WorkflowStatus, StepStatus 閺嬫矮濡囩€规矮绠熺€瑰本鏆?
- WorkflowOrchestrator 缁粯婀亀orkflow濞夈劌鍞介張鍝勫煑
- 鐎规矮绠熸禍?娑撶寵uiltin workflow:  
  - robot_creation_pipeline (Model 閳?Optimize 閳?Export)
  - simulation_ready_robot (Load 閳?Validate 閳?Export)

閴?skill_executors.py:
- SkillExecutor 閸╄櫣琚€规矮绠熺€瑰本鏆?
- RobotModelingExecutor/ParameterOptimizerExecutor/UrdfGeneratorExecutor瀹告彃鐣炬稊?
- 閸╃儤婀癮ction婢跺嫮鎮婂鎻掔杽閻?

### 鐎涙ê婀惃鍕６妫?

閴?**闂傤噣顣?: Mock vs Real 濞ｈ渹璐?*
- 瑜版挸澧犻幍鈧張濉瓁ecutor闁姤妲竚ock鐎圭偟骞囬敍鍫濆涧鏉╂柨娲栫涵顒傜椽閻焦鏆熼幑顕嗙礆
- 濞屸剝婀侀惇鐔割劀鐠嬪啰鏁ctual skill濡€虫健
- API閻鎹ｉ弶銉ョ暚閺佺繝绲剧€圭偤妾稉宥呬紣娴?

閴?**闂傤噣顣?: 閺冪姵纭堕惇鐔割劀鏉╂劘顢慦orkflow**
- cmd_workflows_run 閸︹暞kills_cli.py娑擃厼鐡ㄩ崷銊ょ稻濞屸剝婀佺紒鎾寸€崠鏍翻閸?
- workflow濞屸剝婀侀惇鐔割劀閻ㄥ垾nd-to-end妤犲矁鐦?
- 婢惰精瑙︾捄顖氱窞娑撳秵绔婚弲?

閴?**闂傤噣顣?: Skills 閸忕厧顔愮仦鍌欑瑝鐎瑰本鏆?*
- robot_modeling.py 閸欘亝妲竪rapper閿涘本鐥呴張濉瓁posed閸戠xecution閹恒儱褰?
- parameter_optimizer.py 濞屸剝婀侀弳鎾苟optimize閹恒儱褰?
- urdf_generator.py 濞屸剝婀侀弳鎾苟export閹恒儱褰?

---

## 棣冨箚 娣囶喖顦查惄顔界垼

### 閻╊喗鐖?: Real Executor 鐎圭偟骞?
**閼煎啫娲?** skill_executors.py

閸掓稑缂撻惇鐔风杽閹笛嗩攽閸ｃ劍娴涙禒顤硂ck閿涘瞼娲块幒銉ㄧ殶閻⑩暞kill API閿?
```python
class RealRobotModelingExecutor(SkillExecutor):
    def execute(self, action, inputs):
        if action == 'create_from_template':
            # 閻喐顒滅拫鍐暏
            from agi_walker.skills.robot_modeling import load_template
            config = load_template(inputs['template'])
            # 娣囨繂鐡ㄩ崚鎷岀翻閸戠儤鏋冩禒?
            config.save(inputs['output_file'])
            return {'status': 'success', 'output_file': inputs['output_file']}
```

### 閻╊喗鐖?: Executor Selection
**閼煎啫娲?** skill_executors.py + workflow_orchestrator.py

閹绘劒绶甸柅澶嬪real or mock閻ㄥ嫭婧€閸掕绱?
```python
get_skill_executor(name, use_mock=False)
```

### 閻╊喗鐖?: Workflow End-to-End
**閼煎啫娲?** workflow_orchestrator.py + skills_cli.py

鐎瑰本鏆ｉ惃鍓媜rkflow閹笛嗩攽闁炬拝绱?
1. 娴犲钉LI閸氼垰濮?閳?python -m agi_walker.cli skills workflows run robot_creation_pipeline
2. Orchestrator 閹稿绨幍褑顢戝銉╊€?
3. 鏉╂柨娲栫紒鎾寸€崠鏍畱WorkflowResult
4. 閺勫墽銇氬〒鍛珰閻ㄥ嫯绻樻惔锕€鎷扮紒鎾寸亯

### 閻╊喗鐖?: Skills 鐎电厧鍤幒銉ュ經鐎瑰苯鏉?
**閼煎啫娲?** agi_walker/skills/*.py

娑撶皧kill閺嗘挳婀秂xecution閹恒儱褰涢敍?
- robot_modeling.py: `create_from_template()`, `load_config()`, `save()`
- parameter_optimizer.py: `optimize_mass()`, `validate_physics()`
- urdf_generator.py: `export_to_format()`

---

## 棣冩憫 鐠囷妇绮忕€圭偞鏌﹀銉╊€?

### 濮濄儵顎?: 閸掑棛顬嘙ock閸滃eal Executor
**閺傚洣娆?** skill_executors.py

```python
# 閸╄櫣顢呴幒銉ュ經娣囨繃瀵旀稉宥呭綁
class SkillExecutor(ABC):
    @abstractmethod
    def execute(self, action, inputs) -> Dict[str, Any]:
        pass

# 閸掓稑缂揗ock閸╄櫣琚敍鍫濈秼閸撳秴鐤勯悳甯礆
class MockExecutor(SkillExecutor):
    """閻劋绨ù瀣槸閻ㄥ嚜ock閹笛嗩攽閸?""
    pass

# 閸掓稑缂揜eal鐎圭偟骞?
class RealRobotModelingExecutor(SkillExecutor):
    """閻喎鐤勭拫鍐暏skill閻ㄥ嫭澧界悰灞芥珤"""
    def execute(self, action, inputs):
        from agi_walker.skills.robot_modeling import ...
        # 閻喐顒滅€圭偟骞?

# 瀹搞儱宸堕崙鑺ユ殶
def get_skill_executor(name, use_mock=False):
    if use_mock:
        return get_mock_executor(name)
    else:
        return get_real_executor(name)
```

### 濮濄儵顎?: 閺€纭呯箻Skills鐎电厧鍤幒銉ュ經  
**閺傚洣娆?** agi_walker/skills/robot_modeling.py

娴?
```python
export load_template, list_templates
```

閺€閫涜礋:
```python
def create_from_template(template_name: str, output_file: str) -> Dict:
    config = load_template(template_name)
    config.save(output_file)
    return {'status': 'success', 'output_file': output_file}
```

### 濮濄儵顎?: 閺€纭呯箻WorkflowOrchestrator
**閺傚洣娆?** workflow_orchestrator.py

濞ｈ濮瀝un_workflow()閻ㄥ嫮婀＄€圭偛鐤勯悳甯窗
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

### 濮濄儵顎?: CLI閸涙垝鎶ょ€瑰苯鏉?
**閺傚洣娆?** skills_cli.py

閺€纭呯箻cmd_workflows_run():
```python
def cmd_workflows_run(args):
    orchestrator = get_workflow_orchestrator()
    use_mock = not args.real  # --real flag閸氼垳鏁ら惇鐔风杽executor
    
    try:
        result = orchestrator.run_workflow(args.name, use_mock=use_mock)
        
        print(f"\n瀹搞儰缍斿ù浣瑰⒔鐞? {result.workflow_name}")
        print(f"閻樿埖鈧? {result.status}")
        print(f"\n濮濄儵顎冮幍褑顢?")
        for step in result.steps:
            status_mark = "閴? if step['status'] == 'success' else "閴?
            print(f"  {status_mark} {step['name']}: {step['status']}")
            if 'output_file' in step['output']:
                print(f"      鏉堟挸鍤? {step['output']['output_file']}")
        
        return 0 if result.status == 'success' else 1
    except Exception as e:
        print(f"闁挎瑨顕? {e}", file=sys.stderr)
        return 1
```

---

## 閴?妤犲本鏁归弽鍥у櫙

### 妤犲矁鐦?: Real Executor 瀹搞儰缍?
```bash
$ python -c "
from agi_walker.skill_executors import get_skill_executor
exec = get_skill_executor('robot_modeling', use_mock=False)
result = exec.execute('create_from_template', {
    'template': 'biped',
    'output_file': 'test_output.json'
})
assert result['status'] == 'success'
print('閴?Real executor works')
"
```

### 妤犲矁鐦?: Mock Executor 娣囨繄鏆€
```bash
$ python -c "
from agi_walker.skill_executors import get_skill_executor
exec = get_skill_executor('robot_modeling', use_mock=True)
result = exec.execute('create_from_template', {...})
assert result['status'] == 'success'
print('閴?Mock executor still works')
"
```

### 妤犲矁鐦?: Workflow 閹笛嗩攽閹存劕濮?
```bash
$ python -m agi_walker.cli skills workflows run robot_creation_pipeline --mock
```

**妫板嫭婀℃潏鎾冲毉:**
```
瀹搞儰缍斿ù浣瑰⒔鐞? robot_creation_pipeline
閻樿埖鈧? success

濮濄儵顎冮幍褑顢?
  閴?create_model: success
      鏉堟挸鍤? weights/created_robot.json
  閴?optimize_params: success
      鏉堟挸鍤? weights/optimized_robot.json
  閴?export_urdf: success
      鏉堟挸鍤? exports/robot.urdf
```

### 妤犲矁鐦?: 闁挎瑨顕ゆ径鍕倞
```bash
$ python -m agi_walker.cli skills workflows run robot_creation_pipeline --mock --fail-at=step2
```

**妫板嫭婀℃潏鎾冲毉:**
```
瀹搞儰缍斿ù浣瑰⒔鐞? robot_creation_pipeline
閻樿埖鈧? failed

濮濄儵顎冮幍褑顢?
  閴?create_model: success
  閴?optimize_params: failed
      闁挎瑨顕? Simulated failure
  閳? export_urdf: skipped
```

---

## 棣冩惓 娣囶喗鏁奸懠鍐ㄦ纯

| 閺傚洣娆?| 閺€鐟板З | 娴兼ê鍘涚痪?|
|------|------|--------|
| skill_executors.py | +Real鐎圭偟骞? +Mock閸掑棛琚?| P0 |
| workflow_orchestrator.py | +WorkflowResult鐎瑰苯鏉?| P0 |
| skills_cli.py | +workflows閸涙垝鎶ら弨纭呯箻 | P0 |
| skills/robot_modeling.py | +execution閹恒儱褰?| P1 |
| skills/parameter_optimizer.py | +execution閹恒儱褰?| P1 |
| skills/urdf_generator.py | +execution閹恒儱褰?| P1 |

---

## 棣冩敡 瀹搞儰缍斿ù浣衡柤

1. **閸戝棗顦?* (5min) - 閸掑棙鐎介悳鐗堟箒娴狅絿鐖?
2. **鐎圭偞鏌?* (30min) - 閸掓稑缂揜eal executor + 閺€纭呯箻orchestrator
3. **妤犲矁鐦?* (10min) - 鏉╂劘顢?娑擃亪鐛欓弨鑸电垼閸?
4. **娴兼ê瀵?* (10min) - 閺€纭呯箻闁挎瑨顕ら幓鎰仛閸滃瞼濮搁幀浣哥潔缁€?

---

**妫板嫯顓哥€瑰本鍨氶弮鍫曟？:** 1鐏忓繑妞? 
**閸忔娊鏁笟婵婄:** 閺? 
**妞嬪酣娅?** 娴ｅ函绱欏ù瀣槸鐟曞棛娲婄€瑰本鏆ｉ敍?
