# 瀵偓閸欐垼鈧懏瀵氶崡?

閺堫剚瀵氶崡妤€搴滈崝鈺佺磻閸欐垼鈧懎鎻╅柅鐔剁瑐閹?AGI-Walker 閻ㄥ嫬绱戦崣鎴欌偓?

## 閻滎垰顣ㄧ拋鍓х枂

### 1. 閸忓娈曟禒鎾崇氨
```bash
git clone https://github.com/sossossal/AGI-Walker.git
cd AGI-Walker
```

### 2. 鐎瑰顥婃笟婵婄
```bash
# 娴ｈ法鏁ゆ稉鈧柨顔肩暔鐟佸懓鍓奸張?
./install.sh  # Linux/Mac
# 閹?
install.bat   # Windows

# 閹存牗澧滈崝銊ョ暔鐟?
python -m venv venv
source venv/bin/activate  # Linux/Mac
# venv\Scripts\activate.bat  # Windows
pip install -r requirements.txt
```

### 3. 鏉╂劘顢戝ù瀣槸
```bash
python tests/test_integration.py
python tests/test_extended.py
```

## 娴狅絿鐖滅憴鍕瘱

### Python 妞嬪孩鐗?
闁潧鎯?PEP 8:
- 娴ｈ法鏁?4 缁岀儤鐗哥紓鈺勭箻
- 鐞涘矂鏆辨惔?閳?100 鐎涙顑?
- 娴ｈ法鏁?`black` 閺嶇厧绱￠崠?

### 缁鐎峰▔銊ㄐ?
閹碘偓閺堝鍙曢崗?API 韫囧懘銆忛張澶岃閸ㄥ鏁炵憴?
```python
from typing import List, Dict, Optional

def process_data(
    data: List[float],
    config: Optional[Dict[str, Any]] = None
) -> np.ndarray:
    """婢跺嫮鎮婇弫鐗堝祦"""
    ...
```

### 閺傚洦銆傜€涙顑佹稉?
娴ｈ法鏁?Google 妞嬪孩鐗?
```python
def train_model(env, algorithm="PPO"):
    """
    鐠侇厾绮屽鍝勫鐎涳缚绡勫Ο鈥崇€?
    
    Args:
        env: Gymnasium 閻滎垰顣?
        algorithm: 缁犳纭堕崥宥囆?
    
    Returns:
        鐠侇厾绮屾總鐣屾畱濡€崇€?
    
    Example:
        >>> model = train_model(env, "PPO")
    """
    ...
```

## 妞ゅ湱娲扮紒鎾寸€?

```
AGI-Walker/
閳规壕鏀㈤埞鈧?python_api/          # Python 閹恒儱褰?
閳?  閳规壕鏀㈤埞鈧?zenoh_interface.py
閳?  閳规壕鏀㈤埞鈧?task_editor.py
閳?  閳规柡鏀㈤埞鈧?mujoco_backend.py
閳规壕鏀㈤埞鈧?python_controller/   # 閹貉冨煑閸?
閳?  閳规壕鏀㈤埞鈧?evolution_manager.py
閳?  閳规柡鏀㈤埞鈧?rl_optimizer.py
閳规壕鏀㈤埞鈧?examples/            # 缁€杞扮伐娴狅絿鐖?
閳?  閳规柡鏀㈤埞鈧?tasks/          # 娴犺濮熼悳顖氼暔
閳规壕鏀㈤埞鈧?tests/              # 濞村鐦?
閳规壕鏀㈤埞鈧?docs/               # 閺傚洦銆?
閳规柡鏀㈤埞鈧?godot_project/      # Godot 娴犺法婀?
```

## 濞ｈ濮為弬棰佹崲閸?

### 1. 閸掓稑缂撻悳顖氼暔閺傚洣娆?
```python
# examples/tasks/my_task/env.py
import gymnasium as gym

class MyTaskEnv(gym.Env):
    def __init__(self):
        super().__init__()
        # 鐎规矮绠熺憴鍌涚ゴ閸滃苯濮╂担婊呪敄闂?
        ...
    
    def reset(self, seed=None, options=None):
        ...
    
    def step(self, action):
        ...
```

### 2. 濞夈劌鍞介悳顖氼暔
```python
gym.register(
    id='MyTask-v0',
    entry_point='examples.tasks.my_task.env:MyTaskEnv'
)
```

### 3. 濞ｈ濮炲ù瀣槸
```python
def test_my_task():
    env = gym.make('MyTask-v0')
    obs, info = env.reset()
    assert obs.shape == env.observation_space.shape
```

## 閹绘劒姘︽禒锝囩垳

### 1. 閸掓稑缂撻崚鍡樻暜
```bash
git checkout -b feature/my-feature
```

### 2. 缂傛牕鍟撴禒锝囩垳閸滃本绁寸拠?
```bash
# 鏉╂劘顢戝ù瀣槸
python tests/test_extended.py

# 閺嶇厧绱￠崠鏍﹀敩閻?
black python_api/ python_controller/
```

### 3. 閹绘劒姘?
```bash
git add .
git commit -m "feat: add my feature"
git push origin feature/my-feature
```

### 4. 閸掓稑缂?Pull Request
閸?GitHub 娑撳﹤鍨卞?PR,缁涘绶熺€光剝鐗抽妴?

## 鐠嬪啳鐦幎鈧?

### 1. 娴ｈ法鏁?Python 鐠嬪啳鐦崳?
```python
import pdb; pdb.set_trace()
```

### 2. Zenoh 鐠嬪啳鐦?
```bash
# 閸氼垳鏁ょ拠锔剧矎閺冦儱绻?
export RUST_LOG=debug
python your_script.py
```

### 3. 閺屻儳婀?Godot 閺冦儱绻?
```bash
# 閸?Godot 缂傛牞绶崳銊よ厬閺屻儳婀呮潏鎾冲毉闂堛垺婢?
```

## 鐢瓕顫嗛梻顕€顣?

### Q: Zenoh 鏉╃偞甯存径杈Е?
A: 濡偓閺屻儵妲婚悘顐㈩暰鐠佸墽鐤?绾喕绻氱粩顖氬經 7447 瀵偓閺€淇扁偓?

### Q: MuJoCo 鐎瑰顥婃径杈Е?
A: 绾喕绻氶張?C++ 缂傛牞鐦ч崳?(Linux: gcc, Windows: MSVC)閵?

### Q: 濞村鐦径杈Е?
A: 鏉╂劘顢?`pip install -r requirements.txt` 绾喕绻氭笟婵婄鐎瑰本鏆ｉ妴?

## 鐠у嫭绨柧鐐复

- [GitHub 娴犳挸绨盷(https://github.com/sossossal/AGI-Walker)
- [鐠愶紕灏為幐鍥у础](../CONTRIBUTING.md)
- [閹垛偓閺堫垰鈧搫濮熺拋鈥冲灊](../archive_and_reports/TECH_DEBT_PLAN.md)
