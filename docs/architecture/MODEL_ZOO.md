# 濡€崇€烽崣鎴濈閹稿洤宕?

## 妫板嫯顔勭紒鍐┠侀崹瀣氨

AGI-Walker 閹绘劒绶垫０鍕唲缂佸啰娈戝鍝勫鐎涳缚绡勫Ο鈥崇€?閸欘垳娲块幒銉ф暏娴滃氦鐦庢导鐗堝灗瀵邦喛鐨熼妴?

### 閸欘垳鏁ゅΟ鈥崇€?

| 娴犺濮?| 缁犳纭?| 鐠侇厾绮屽銉︽殶 | 閹存劕濮涢悳?| 娑撳娴?|
|------|------|---------|--------|------|
| 濡ゅ吋顫弨鈧悥?| PPO | 1M | 85% | [娑撳娴嘳(weights/stair_climbing/final_model.zip) |
| 瀹曞骸鐭曢崷鏉胯埌 | PPO | 1M | 78% | 鐠侇厾绮屾稉?|
| 閻椻晙缍嬮幎鎾冲絿 | SAC | 500K | 72% | 鐠侇厾绮屾稉?|

### 韫囶偊鈧喎绱戞慨?

#### 1. 娑撳娴囧Ο鈥崇€?
```bash
# 娑撳娴囧Δ鍏碱潽閺€鈧悥顒伳侀崹?
wget https://github.com/sossossal/AGI-Walker/releases/download/v4.2.0/stair_climbing_ppo.zip
```

#### 2. 閸旂姾娴囧Ο鈥崇€?
```python
from stable_baselines3 import PPO
import gymnasium as gym

# 閸旂姾娴囧Ο鈥崇€?
model = PPO.load("stair_climbing_ppo.zip")

# 閸掓稑缂撻悳顖氼暔
env = gym.make('StairClimbing-v0')

# 鏉╂劘顢?
obs, info = env.reset()
for _ in range(1000):
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(action)
    if terminated or truncated:
        break

print(f"Steps climbed: {info['steps_climbed']}/5")
```

#### 3. 瀵邦喛鐨熷Ο鈥崇€?
```python
# 缂佈呯敾鐠侇厾绮?
model.learn(total_timesteps=100000)
model.save("finetuned_model.zip")
```

### 鐠侇厾绮岄懛顏勭箒閻ㄥ嫭膩閸?

```bash
# 鐎瑰顥婃笟婵婄
pip install stable-baselines3

# 鐠侇厾绮屽Δ鍏碱潽閺€鈧悥?
python examples/tasks/stair_climbing/train.py --timesteps 1000000

# 鐠囧嫪鍙婂Ο鈥崇€?
python examples/tasks/stair_climbing/train.py --mode eval --model weights/stair_climbing/final_model.zip
```

### 閹嗗厴 Baseline

#### 濡ゅ吋顫弨鈧悥?(Stair Climbing)
- **缁犳纭?*: PPO
- **鐠侇厾绮屽銉︽殶**: 1,000,000
- **閹存劕濮涢悳?*: 85%
- **楠炲啿娼庢總鏍уС**: 12.5 鍗?2.3
- **鐠侇厾绮岄弮鍫曟？**: ~2 鐏忓繑妞?(NVIDIA A100)

**婵傛牕濮抽弴鑼殠**:
```
Episode 0-100:    楠炲啿娼庢總鏍уС -5.2
Episode 100-500:  楠炲啿娼庢總鏍уС 3.8
Episode 500-1000: 楠炲啿娼庢總鏍уС 12.5
```

### 鐠愶紕灏炲Ο鈥崇€?

濞嗐垼绻嬬拹锛勫盀妫板嫯顔勭紒鍐┠侀崹?

1. 鐠侇厾绮屽Ο鈥崇€?
2. 鐠囧嫪鍙婇幀褑鍏?(>70% 閹存劕濮涢悳?
3. 閹绘劒姘?Pull Request
4. 閸栧懎鎯?
   - 濡€崇€烽弬鍥︽ (.zip)
   - 鐠侇厾绮岀紒鐔活吀 (training_stats.json)
   - 鐠囧嫪鍙婄紒鎾寸亯

### 濡€崇€风拋绋垮讲

閹碘偓閺堝顣╃拋顓犵矊濡€崇€烽柌鍥╂暏 MIT 鐠佺褰茬拠?閸欘垵鍤滈悽鍙樺▏閻劊鈧椒鎱ㄩ弨鐟版嫲閸掑棗褰傞妴?
