# 缁傝崵鍤庡鍝勫鐎涳缚绡?(Offline RL)

AGI-Walker 閻ㄥ嫮顬囩痪鍨繁閸栨牕顒熸稊鐘插閼宠棄鐔€娴?**d3rlpy** 鎼存搫绱濇担璺ㄦ暏 **CQL (Conservative Q-Learning)** 缁犳纭堕妴?

## 閺嶇绺炬导妯哄◢

- **閺嶉攱婀伴弫鍫㈠芳閹绘劕宕?5-10 閸?* - 閸忓懎鍨庨崚鈺冩暏瀹稿弶婀侀弫鐗堝祦
- **鐠侇厾绮岄弮鍫曟？閸戝繐鐨?50%+** - 閺冪娀娓舵径褔鍣洪崷銊у殠閹恒垻鍌?
- **闁倸鎮庨惇鐔风杽閺堝搫娅掓禍?* - 闁灝鍘ら崡閬嶆珦閻ㄥ嫰娈㈤張鐑樺赴缁?

## 韫囶偊鈧喎绱戞慨?

### 1. 鐎瑰顥婃笟婵婄

```bash
pip install -r requirements-offline-rl.txt
```

### 2. 閺€鍫曟肠娑撴挸顔嶉弫鐗堝祦

```python
from stable_baselines3 import PPO
from python_api.offline_rl import ExpertDataCollector

# 鐠侇厾绮屾稉鎾愁啀缁涙牜鏆愰敍鍫熷灗閸旂姾娴囧鍙夋箒濡€崇€烽敍?
expert_policy = PPO.load("weights/expert_policy.zip")

# 閺€鍫曟肠濠曟梻銇氶弫鐗堝祦
collector = ExpertDataCollector("AGI-Walker/Walker2D-v0")
dataset = collector.collect_from_policy(expert_policy, n_episodes=1000)

# 娣囨繂鐡ㄩ弫鐗堝祦闂?
collector.save_dataset(dataset, "expert_data.pkl")
```

### 3. 缁傝崵鍤庣拋顓犵矊

```python
from python_api.offline_rl import OfflineRLTrainer

# 閸掓稑缂撶拋顓犵矊閸?
trainer = OfflineRLTrainer(
    env_id="AGI-Walker/Walker2D-v0",
    algorithm="cql"
)

# 閸戝棗顦弫鐗堝祦闂?
mdp_dataset = trainer.prepare_dataset(dataset)

# 缁傝崵鍤庣拋顓犵矊
trainer.train_offline(mdp_dataset, n_steps=100000)
trainer.save("weights/offline_cql.pt")
```

### 4. 閸︺劎鍤?Fine-tuning

```python
import gymnasium as gym

env = gym.make("AGI-Walker/Walker2D-v0")
trainer.finetune_online(env, n_steps=10000)
trainer.save("weights/cql_finetuned.pt")
```

## 鐎瑰本鏆ｇ粈杞扮伐

鏉╂劘顢戠€瑰本鏆ｉ惃?Offline RL 濞翠胶鈻?

```bash
python examples/offline_rl_demo.py
```

## 閹嗗厴鐎佃鐦?

娴ｈ法鏁ょ粋鑽ゅ殠 RL vs. 娴肩姷绮洪崷銊у殠 RL (PPO/SAC):

| 閹稿洦鐖?| 閸︺劎鍤?RL | 缁傝崵鍤?RL | 閺€纭呯箻 |
|------|---------|---------|------|
| 閺嶉攱婀伴弫鍫㈠芳 | 100K steps | 10K steps | **10x 閳?* |
| 鐠侇厾绮岄弮鍫曟？ | 2 hours | 30 min | **4x 閳?* |
| 閺堚偓缂佸牊鈧嗗厴 | 100% | 95-100% | 閻╃缍?|

## 瀹搞儰缍斿ù浣衡柤

```
1. 娑撴挸顔嶉弫鐗堝祦閺€鍫曟肠 (Expert Data Collection)
   閳?
2. 缁傝崵鍤庣拋顓犵矊 (Offline Training with CQL)
   閳?
3. 閸︺劎鍤?Fine-tuning (Online Fine-tuning)
   閳?
4. 闁劎璁?(Deployment)
```

## API 閸欏倽鈧?

### ExpertDataCollector

閺€鍫曟肠妤傛宸濋柌蹇旂川缁€鐑樻殶閹诡喓鈧?

```python
collector = ExpertDataCollector(env_id, save_dir="offline_data")
dataset = collector.collect_from_policy(policy, n_episodes=1000)
```

### OfflineRLTrainer

CQL 缁傝崵鍤庣拋顓犵矊閸ｃ劊鈧?

```python
trainer = OfflineRLTrainer(
    env_id="AGI-Walker/Walker2D-v0",
    algorithm="cql",  # 閻╊喖澧犻弨顖涘瘮 "cql"
    actor_lr=3e-4,
    critic_lr=3e-4
)
```

**娑撴槒顩﹂弬瑙勭《**:
- `prepare_dataset(raw_dataset)` - 鏉烆剚宕查弫鐗堝祦閺嶇厧绱?
- `train_offline(dataset, n_steps)` - 缁傝崵鍤庣拋顓犵矊
- `finetune_online(env, n_steps)` - 閸︺劎鍤?fine-tuning
- `save(filepath)` / `load(filepath)` - 娣囨繂鐡?閸旂姾娴囧Ο鈥崇€?

## 濞夈劍鍓版禍瀣€?

1. **閺佺増宓佺拹銊╁櫤瀵板牓鍣哥憰?* - 绾喕绻氭稉鎾愁啀缁涙牜鏆愰幀褑鍏樼搾鍐差檮婵?
2. **閺佺増宓佹径姘壉閹?* - 閺€鍫曟肠娑撳秴鎮撻崷鐑樻珯閸滃瞼濮搁幀浣烘畱閺佺増宓?
3. **CQL 鐡掑懎寮弫?* - `conservative_weight` 閹貉冨煑娣囨繂鐣х粙瀣閿涘牓绮拋?5.0閿?
4. **Fine-tuning 韫囧懓顩﹂幀?* - 缁傝崵鍤庣拋顓犵矊閸氬酣鈧艾鐖堕棁鈧憰浣哥毌闁插繐婀痪?fine-tuning

## 閸欏倽鈧啯鏋冮悮?

- [Conservative Q-Learning](https://arxiv.org/abs/2006.04779) (Kumar et al., 2020)
- [d3rlpy Documentation](https://d3rlpy.readthedocs.io/)

## 娑撳绔村?

- [ ] 閺€顖涘瘮閺囨潙顦跨粋鑽ゅ殠 RL 缁犳纭?(TD3+BC, IQL)
- [ ] 閼奉亜濮╅崠鏍ㄦ殶閹诡喛宸濋柌蹇旑梾閺?
- [ ] 婢х偤鍣哄蹇旀殶閹诡噣娉﹂弴瀛樻煀
- [ ] 婢舵矮鎹㈤崝锛勵瀲缁?RL
