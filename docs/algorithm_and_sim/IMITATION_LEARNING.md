# 濡€茶雹鐎涳缚绡勯敍鍦沵itation Learning閿?

AGI-Walker 閺€顖涘瘮娑撱倗顫掓稉鏄忣洣閻ㄥ嫭膩娴犲灝顒熸稊鐘虫煙濞夋洏鈧?

## 閺嶇绺炬导妯哄◢

- **閺冪娀娓舵總鏍уС閸戣姤鏆?* - 閻╁瓨甯存禒搴濈瑩鐎硅埖绱ㄧ粈鍝勵劅娑?
- **閺嶉攱婀伴弫鍫㈠芳妤?* - BC 娴犲懘娓?0%閻ㄥ嫯顔勭紒鍐╂闂?
- **閹嗗厴娣囨繆鐦?* - GAIL 閸欘垵鎻稉鎾愁啀95%閹嗗厴

## 韫囶偊鈧喎绱戞慨?

### 1. 鐎瑰顥婃笟婵婄

```bash
pip install -r requirements-imitation.txt
```

### 2. 鐞涘奔璐熼崗瀣畷閿涘湐C閿?

閺堚偓缁犫偓閸楁洖鎻╅柅鐔烘畱閺傝纭堕敍?

```python
from python_api.imitation_learning import BehaviorCloning
from stable_baselines3 import PPO

# 鐠侇厾绮屾稉鎾愁啀
expert = PPO("MlpPolicy", "AGI-Walker/Walker2D-v0")
expert.learn(total_timesteps=100_000)

# 閺€鍫曟肠濠曟梻銇?
bc = BehaviorCloning("AGI-Walker/Walker2D-v0")
demos = bc.collect_expert_demos(expert, n_episodes=50)

# 鐠侇厾绮?BC
bc.train(demos, n_epochs=10)

# 鐠囧嫪鍙?
bc.evaluate(n_episodes=10)
```

### 3. 閻㈢喐鍨氱€佃濮夊Ο鈥茶雹鐎涳缚绡勯敍鍦橝IL閿?

閺囨潙宸辨径褌绲剧拋顓犵矊閺冨爼妫块弴鎾毐閿?

```python
from python_api.imitation_learning import GAIL

gail = GAIL("AGI-Walker/Walker2D-v0")
demos = gail.collect_expert_demos(expert, n_episodes=100)

gail.train(demos, total_timesteps=500_000)
gail.evaluate(n_episodes=10)
```

## 鐎瑰本鏆ｇ粈杞扮伐

```bash
# BC 缁€杞扮伐
python examples/imitation_learning_demo.py --method bc

# GAIL 缁€杞扮伐
python examples/imitation_learning_demo.py --method gail

# 閺傝纭剁€佃鐦?
python examples/imitation_learning_demo.py --method compare
```

## 閺傝纭剁€佃鐦?

### 鐞涘奔璐熼崗瀣畷閿涘湐C閿?

**娴兼鍋?*:
- 鐠侇厾绮岃箛顐︹偓鐕傜礄10%閺冨爼妫块敍?
- 鐎圭偟骞囩粻鈧崡?
- 閺嶉攱婀伴弫鍫㈠芳妤?

**缂傝櫣鍋?*:
- 閹嗗厴娑撳﹪妾洪崣妞剧瑩鐎瑰爼妾洪崚?
- 閸掑棗绔峰鍌溞╅梻顕€顣?

**闁倻鏁ら崷鐑樻珯**:
- 韫囶偊鈧喎甯崹瀣磻閸?
- 娑撴挸顔嶉弫鐗堝祦鐠愩劑鍣烘?
- 娴犺濮熼惄绋款嚠缁犫偓閸?

### GAIL

**娴兼鍋?*:
- 閹嗗厴閹恒儴绻庢稉鎾愁啀
- 妞翠焦顥楅幀褍宸?
- 閸欘垱纭鹃崠?

**缂傝櫣鍋?*:
- 鐠侇厾绮岄弮鍫曟？鏉堝啴鏆?
- 闂団偓鐟曚浇鐨熼崣?
- 鐠у嫭绨☉鍫ｂ偓妤€銇?

**闁倻鏁ら崷鐑樻珯**:
- 鏉╄姤鐪版妯烩偓褑鍏?
- 婢跺秵娼呮禒璇插
- 閺堝鍘栫搾瀹狀吀缁犳绁┃?

## 閹嗗厴閸╁搫鍣?

| 閺傝纭?| 鐠侇厾绮岄弮鍫曟？ | 閺堚偓缂佸牆顨涢崝?| 閺嶉攱婀伴棁鈧Ч?|
|------|----------|----------|----------|
| PPO閿涘牆鐔€閸戝棴绱?| 100% | 100% | 100% |
| BC | **10%** | 85% | **20%** |
| GAIL | 150% | **95%** | 50% |

## 娴ｈ法鏁ら幎鈧?

### 1. 娑撴挸顔嶉弫鐗堝祦鐠愩劑鍣?

- 閺€鍫曟肠50-100娑擃亪鐝拹銊╁櫤閸ョ偛鎮?
- 绾喕绻氭径姘壉閹?
- 闁灝鍘ら弮鈺傛埂缂佸牊顒涢惃鍕礀閸?

### 2. BC 鐡掑懎寮弫?

```python
bc.train(
    demos,
    n_epochs=10,      # 閹恒劏宕?5-20
    batch_size=64,    # 閹恒劏宕?32-128
    learning_rate=1e-3  # 閹恒劏宕?1e-4 閸?1e-2
)
```

### 3. GAIL 鐡掑懎寮弫?

```python
gail.train(
    demos,
    total_timesteps=500_000,  # 閼峰啿鐨?100K
    n_disc_updates_per_round=4  # 閹恒劏宕?2-8
)
```

## 鐢瓕顫嗛梻顕€顣?

**Q: BC 閹嗗厴娑撳秳鍖犻幀搴濈疄閸旂儑绱?*
A: 
1. 婢х偛濮炴稉鎾愁啀濠曟梻銇氶弫浼村櫤
2. 閹绘劙鐝稉鎾愁啀缁涙牜鏆愮拹銊╁櫤
3. 婢х偛濮炵拋顓犵矊鏉烆喗鏆?

**Q: GAIL 鐠侇厾绮屾稉宥嚽旂€规熬绱?*
A:
1. 鐠嬪啯鏆ｉ崚銈呭焼閸ｃ劍娲块弬浼搭暥閻?
2. 闂勫秳缍嗙€涳缚绡勯悳?
3. 婢х偛濮炴稉鎾愁啀濠曟梻銇?

**Q: 婵″倷缍嶉柅澶嬪閺傝纭堕敍?*
A:
- 韫囶偊鈧喎甯崹?閳?**BC**
- 鏉╄姤鐪伴幀褑鍏?閳?**GAIL**
- 閺佺増宓佺亸?閳?**BC + Fine-tuning**

## API 閸欏倽鈧?

### BehaviorCloning

```python
bc = BehaviorCloning(env_id)
bc.collect_expert_demos(policy, n_episodes)
bc.train(demos, n_epochs, batch_size)
bc.evaluate(n_episodes)
bc.save(path)
bc.load(path)
```

### GAIL

```python
gail = GAIL(env_id)
gail.collect_expert_demos(policy, n_episodes)
gail.train(demos, total_timesteps)
gail.evaluate(n_episodes)
gail.save(path)
```

## 娑撳绔村?

- [ ] 鐎圭偟骞?DAgger閿涘湒ataset Aggregation閿?
- [ ] 閺€顖涘瘮婢舵矮鎹㈤崝鈩兡佹禒鍨劅娑?
- [ ] 婢х偛濮為柅鍡楀繁閸栨牕顒熸稊鐙呯礄IRL閿?

## 閸欏倽鈧啯鏋冮悮?

- [Behavior Cloning](https://arxiv.org/abs/1011.0686)
- [GAIL](https://arxiv.org/abs/1606.03476)
- [imitation Library](https://imitation.readthedocs.io/)
