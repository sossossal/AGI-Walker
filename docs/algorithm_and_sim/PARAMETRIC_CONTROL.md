# 閸欏倹鏆熼崠鏍ㄦ簚閸ｃ劋姹夐幒褍鍩楃化鑽ょ埠

闁俺绻冪拫鍐╂殻閻椻晝鎮婇崣鍌涙殶閺夈儵妫块幒銉﹀付閸掕埖婧€閸ｃ劋姹夐敍宀冣偓宀勬姜閻╁瓨甯撮崣鎴︹偓浣稿З娴ｆ粌鎳℃禒銈冣偓?

## 閺嶇绺惧鍌氬悍

### 娴肩姷绮洪幒褍鍩?vs 閸欏倹鏆熼崠鏍ㄥ付閸?

**娴肩姷绮洪幒褍鍩?*:
```python
action = [0.5, -0.3, 0.8, ...]  # 閻╁瓨甯撮幐鍥х暰閸忓疇濡幍顓犵叐
env.step(action)
```

**閸欏倹鏆熼崠鏍ㄥ付閸?*:
```python
controller.set_physics_param('motor_power_multiplier', 1.5)
# 閹貉冨煑閸ｃ劍鐗撮幑顔煎棘閺佹媽鍤滈崝銊吀缁犳濮╂担?
result = controller.run_episode()
```

## 韫囶偊鈧喎绱戞慨?

### 閸╄櫣顢呴悽銊︾《

```python
from python_api.parametric_control import ParametricRobotController

# 閸掓稑缂撻幒褍鍩楅崳?
controller = ParametricRobotController()

# 鐠嬪啯鏆ｉ悽鍨簚閸旂喓宸?
controller.set_physics_param('motor_power_multiplier', 1.2)

# 鏉╂劘顢戦獮鑸电叀閻鏅ラ弸?
result = controller.run_episode()
print(f"婵傛牕濮? {result['total_reward']}")
```

### 閸欘垵鐨熼崣鍌涙殶

| 閸欏倹鏆?| 姒涙顓婚崐?| 閼煎啫娲?| 瑜板崬鎼?|
|------|--------|------|------|
| `motor_power_multiplier` | 1.0 | 0.5-2.0 | 閺堚偓婢堆勫閻?闁喎瀹?|
| `joint_stiffness` | 1.0 | 0.5-3.0 | 缁儳瀹?闂囧洩宕?|
| `joint_damping` | 0.5 | 0.1-1.0 | 缁嬪啿鐣鹃幀?閸濆秴绨查柅鐔峰 |
| `friction` | 0.9 | 0.1-1.5 | 閹解晜鎽?閼冲€熲偓?|
| `mass_multiplier` | 1.0 | 0.5-1.5 | 閹垱鈧?閹碘偓闂団偓閸旀稓鐓?|
| `gravity` | 9.81 | 0-20 | 闁插秴濮忛悳顖氼暔 |

## 娴ｈ法鏁ょ粈杞扮伐

### 缁€杞扮伐 1: 鐎圭偤鐛欐稉宥呮倱闁板秶鐤?

```python
# 妤傛ê濮涢悳鍥帳缂?
controller.set_physics_param('motor_power_multiplier', 1.5)
result = controller.run_episode()

# 妤傛绨挎惔锕傚帳缂?
controller.set_physics_param('joint_stiffness', 2.0)
controller.set_physics_param('joint_damping', 0.7)
result = controller.run_episode()
```

### 缁€杞扮伐 2: 閸欏倹鏆熼幍顐ｅ伎

```python
import numpy as np

# 閹殿偅寮块悽鍨簚閸旂喓宸?
for power in np.linspace(0.5, 2.0, 10):
    controller.set_physics_param('motor_power_multiplier', power)
    result = controller.run_episode()
    print(f"閸旂喓宸?{power:.2f}: 婵傛牕濮?{result['total_reward']:.2f}")
```

### 缁€杞扮伐 3: 閼奉亜濮╂导妯哄

```python
# 鐎规矮绠熼幖婊呭偍缁屾椽妫?
param_ranges = {
    'motor_power_multiplier': (0.8, 1.5),
    'joint_stiffness': (0.5, 2.0),
    'joint_damping': (0.3, 0.8)
}

# 閹兼粎鍌ㄩ張鈧导姗€鍘ょ純?
result = controller.find_optimal_params(param_ranges, n_trials=20)
print(f"閺堚偓娴兼ê寮弫? {result['best_params']}")
print(f"閺堚偓妤傛ê顨涢崝? {result['best_reward']}")
```

### 缁€杞扮伐 4: 娴溿倓绨板蹇氱殶閺?

```python
from python_api.parametric_control import InteractiveParameterTuner

tuner = InteractiveParameterTuner(controller)
tuner.interactive_tuning()

# 閸︺劋姘︽禍鎺撃佸蹇庤厬:
# > set motor_power_multiplier 1.3
# > test
# > show
# > quit
```

## 鐎瑰本鏆ｅ鏃傘仛

```bash
# 鏉╂劘顢戦幍鈧張澶屻仛娓?
python examples/parametric_control_demo.py

# 鏉╂劘顢戦悧鐟扮暰缁€杞扮伐
python examples/parametric_control_demo.py --demo 1  # 閸╄櫣顢呴幒褍鍩?
python examples/parametric_control_demo.py --demo 2  # 閸欏倹鏆熼幍顐ｅ伎
python examples/parametric_control_demo.py --demo 3  # 閼奉亜濮╂导妯哄

# 娴溿倓绨板Ο鈥崇础
python examples/parametric_control_demo.py --interactive
```

## 閸欏倹鏆熻ぐ鍗炴惙閸掑棙鐎?

### 閻㈠灚婧€閸旂喓宸?(motor_power_multiplier)

- **婢х偛濮?*: 閺囨潙宸遍惃鍕攳閸斻劌濮忛敍宀冨厴閻栴剙娼担鍡氬厴閼版鐝?
- **閸戝繐鐨?*: 閼哄倽鍏樻担鍡楀讲閼宠姤妫ゅ▔鏇炵暚閹存劒鎹㈤崝?

**瀵ら缚顔呴懠鍐ㄦ纯**: 0.8 - 1.5

### 閸忓疇濡崚姘 (joint_stiffness)

- **婢х偛濮?*: 閺囧绨跨涵顕嗙礉娴ｅ棗褰查懗浠嬫缚閼?
- **閸戝繐鐨?*: 閺囧瓨鐓嶉崪宀嬬礉娴ｅ棛绨挎惔锕傛娴?

**瀵ら缚顔呴懠鍐ㄦ纯**: 0.5 - 2.0

### 閸忓疇濡梼璇插嚬 (joint_damping)

- **婢х偛濮?*: 閺囧菙鐎规熬绱濇担鍡楁惙鎼存梹鍙?
- **閸戝繐鐨?*: 閸濆秴绨茶箛顐礉娴ｅ棗褰查懗浠嬫缚閼?

**瀵ら缚顔呴懠鍐ㄦ纯**: 0.3 - 0.8

### 鐠愩劑鍣洪崐宥嗘殶 (mass_multiplier)

- **婢х偛濮?*: 閺囧菙鐎规熬绱濇担鍡涙付鐟曚焦娲挎径褍濮忛惌?
- **閸戝繐鐨?*: 閺囧浼掑ú浼欑礉娴ｅ棛菙鐎规碍鈧囨娴?

**瀵ら缚顔呴懠鍐ㄦ纯**: 0.7 - 1.3

## 鐎圭偤妾惔鏃傛暏

### 閺堝搫娅掓禍楦款啎鐠侊繝鐛欑拠?

```python
# 濞村鐦潪濠氬櫤閸栨牞顔曠拋?
controller.set_physics_param('mass_multiplier', 0.7)
controller.set_physics_param('motor_power_multiplier', 0.9)
result = controller.run_episode()
```

### 閻滎垰顣ㄩ柅鍌氱安

```python
# 濡剝瀚欓張鍫㈡倖闁插秴濮?
controller.set_physics_param('gravity', 1.62)

# 濡剝瀚欐妯绘噰閹匡箑婀撮棃?
controller.set_physics_param('friction', 1.3)
```

### 閹嗗厴娴兼ê瀵?

```python
# 閼奉亜濮╅幍鎯у煂閺堚偓娴兼﹢鍘ょ純?
optimal = controller.find_optimal_params({
    'motor_power_multiplier': (0.8, 1.5),
    'joint_stiffness': (0.5, 2.0)
}, n_trials=30)
```

## 娑撳酣娴傛禒璺虹氨闂嗗棙鍨?

```python
from python_api.custom_parts import CustomMotor, CustomJoint

# 閸掓稑缂撶€规艾鍩楅悽鍨簚
motor = CustomMotor({'power': 750, 'gear_ratio': 80})

# 鐏忓棛鏁搁張鍝勫棘閺佺増妲х亸鍕煂閹貉冨煑閸?
power_multiplier = motor.params['power'] / 500.0  # 瑜版帊绔撮崠?
controller.set_physics_param('motor_power_multiplier', power_multiplier)

# 閸掓稑缂撶€规艾鍩楅崗瀹犲Ν
joint = CustomJoint({'stiffness': 6000})

# 閺勭姴鐨犻崚姘
stiffness_multiplier = joint.params['stiffness'] / 5000.0
controller.set_physics_param('joint_stiffness', stiffness_multiplier)
```

## 鐢瓕顫嗛梻顕€顣?

**Q: 婵″倷缍嶉柅澶嬪閸掓繂顫愰崣鍌涙殶閿?*
A: 娴犲酣绮拋銈呪偓鐓庣磻婵绱濋柅鎰嚋閸欏倹鏆熺拫鍐╂殻楠炴儼顫囩€电喎濂栭崫宥冣偓?

**Q: 閸欏倹鏆熸导妯哄闂団偓鐟曚礁顦挎稊鍜冪吹**
A: 10-20濞喡ょ槸妤犲矂鈧艾鐖剁搾鍐差檮閿涘本鐦″▎陇鐦宀€瀹?-10缁夋帇鈧?

**Q: 閸欘垯浜掓穱婵嗙摠閺堚偓娴兼﹢鍘ょ純顔兼偋閿?*
A: 閸欘垯浜掗敍灞煎▏閻?`result['best_params']` 娣囨繂鐡ㄦ稉绡擲ON閵?

**Q: 閸欏倹鏆熼崣妯哄娴兼氨鐝涢崡宕囨晸閺佸牆鎮ч敍?*
A: 閺勵垳娈戦敍灞肩瑓娑撯偓濞?`run_episode()` 閺冭泛姘ㄦ导姘▏閻劍鏌婇崣鍌涙殶閵?

## API 閸欏倽鈧?

### ParametricRobotController

```python
controller = ParametricRobotController(env_id='AGI-Walker/Walker2D-v0')

# 鐠佸墽鐤嗛崣鍌涙殶
controller.set_physics_param(param_name, value)

# 鏉╂劘顢戦崶鐐叉値
result = controller.run_episode(max_steps=1000, render=False)

# 閼奉亜濮╂导妯哄
optimal = controller.find_optimal_params(param_ranges, n_trials=10)
```

### InteractiveParameterTuner

```python
tuner = InteractiveParameterTuner(controller)

# 閺勫墽銇氳ぐ鎾冲閸欏倹鏆?
tuner.show_current_params()

# 鏉╂稑鍙嗘禍銈勭鞍濡€崇础
tuner.interactive_tuning()
```

## 娑撳绔村?

- [ ] 閹碘晛鐫嶉崚鏉挎磽鐡掕櫕婧€閸ｃ劋姹?
- [ ] 濞ｈ濮炵€圭偞妞傞崣顖濐潒閸?
- [ ] 閺€顖涘瘮婢舵氨娲伴弽鍥︾喘閸?
- [ ] 闂嗗棙鍨氬鍝勫鐎涳缚绡?

---

**娴兼ê濞嶉幀鑽ょ波**:
- 閴?閻╃顫囬敍姘崇殶閺佸澧块悶鍡楀棘閺佹媽鈧矂娼幎鍊熻杽閸斻劋缍?
- 閴?閻喎鐤勯敍姘棘閺佹澘顕惔鏂跨杽闂勫懘娴傛禒璺虹潣閹?
- 閴?閸欘垵袙闁插绱板〒鍛殶瑜板崬鎼烽張鍝勫煑
- 閴?妤傛ɑ鏅ラ敍姘冲殰閸斻劍鎮崇槐銏℃付娴兼﹢鍘ょ純?
