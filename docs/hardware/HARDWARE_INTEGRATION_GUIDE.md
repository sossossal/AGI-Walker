# 绾兛娆㈤梿鍡樺灇閹稿洤宕￠敍姘矤 AGI-Walker 娴犺法婀￠崚?IMC-22 闁劎璁?

## 棣冩惖 濮掑倽鍫?

閺堫剚瀵氶崡妞剧矙缂佸秴顩ф担鏇炵殺閸?AGI-Walker 娑擃叀顔勭紒鍐畱閹貉冨煑缁涙牜鏆愰柈銊ц閸掓澘鐔€娴?IMC-22 閼侯垳澧栭惃鍕埂鐎圭偟鈥栨禒韬测偓?

---

## 棣冩敡 Sim-to-Real 瀹搞儰缍斿ù?

```
1. 娴犺法婀＄拋顓犵矊 (AGI-Walker) 閳?2. 缁涙牜鏆愮€电厧鍤?(ONNX) 閳?3. 濡€崇€烽柌蹇撳 (INT8) 閳?4. 绾兛娆㈤柈銊ц (IMC-22)
```

### 鐎瑰本鏆ｅù浣衡柤閸?

```mermaid
graph LR
    A[AGI-Walker 娴犺法婀 --> B[鐠侇厾绮?PPO 缁涙牜鏆怾
    B --> C[鐎电厧鍤?ONNX 濡€崇€穄
    C --> D[闁插繐瀵叉稉?INT8]
    D --> E[閻懷冪秿閸?IMC-22]
    E --> F[閻喎鐤勯張鍝勬珤娴滅儤绁寸拠鏄?
    F --> G{閹嗗厴濠娐ゅ喕鐟曚焦鐪?}
    G -->|閸氼洟 H[鐠嬪啯鏆ｆ禒璺ㄦ埂閸欏倹鏆焆
    H --> A
    G -->|閺勭槃 I[闁劎璁茬€瑰本鍨歖
```

---

## 棣冩憹 缁楊兛绔村銉窗閻滎垰顣ㄩ崙鍡楊槵

### 1.1 Python 娓氭繆绂?

```bash
# 閸╄櫣顢呮禒璺ㄦ埂閻滎垰顣ㄩ敍鍫濆嚒閺堝绱?
pip install gymnasium numpy stable-baselines3

# 绾兛娆㈤柈銊ц瀹搞儱鍙块敍鍫熸煀婢х儑绱?
pip install python-can onnx torch
```

### 1.2 瀹搞儱鍙块柧鎯х暔鐟?

```bash
# RISC-V 瀹搞儱鍙块柧?
# Ubuntu/Debian:
sudo apt-get install gcc-riscv32-unknown-elf

# macOS:
brew install riscv-gnu-toolchain

# Windows:
# 娑撳娴囨０鍕椽鐠囨垹澧楅張? https://github.com/riscv/riscv-gnu-toolchain/releases
```

### 1.3 绾兛娆㈤崙鍡楊槵

| 缂佸嫪娆?| 閺佷即鍣?| 鐠囧瓨妲?|
|------|------|------|
| IMC-22 瀵偓閸欐垶婢?| N 娑?| N = 閸忓疇濡弫浼村櫤 |
| CAN 闁倿鍘ら崳?| 1 娑?| USB-CAN 閹?SPI-CAN |
| 閻㈠灚绨?| 1 娑?| 5V 娓氭稓鏁?|
| J-Link 鐠嬪啳鐦崳?| 1 娑?| 閿涘牆褰查柅澶涚礆閻劋绨拫鍐槸 |

---

## 棣冨笚 缁楊兛绨╁銉窗閸︺劋璞㈤惇鐔惰厬鐠侇厾绮岀粵鏍殣

### 2.1 娴ｈ法鏁?AGI-Walker 鐠侇厾绮?

```python
from godot_robot_env import GodotRobotEnv
from stable_baselines3 import PPO

# 閸掓稑缂撻悳顖氼暔
env = GodotRobotEnv(
    env_preset="earth",      # 娴ｈ法鏁ら崷鎵倖闁插秴濮?
    ground_material="concrete"  # 濞ｅ嘲鍤岄崷鐔锋勾闂?
)

# 鐠侇厾绮岀粵鏍殣
model = PPO("MultiInputPolicy", env, verbose=1)
model.learn(total_timesteps=500000)

# 娣囨繂鐡ㄥΟ鈥崇€?
model.save("walker_policy")
```

### 2.2 鐠囧嫪鍙婇幀褑鍏?

```python
# 濞村鐦拋顓犵矊婵傜晫娈戠粵鏍殣
obs, info = env.reset()
for _ in range(1000):
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(action)
    if terminated or truncated:
        obs, info = env.reset()
```

---

## 棣冩憶 缁楊兛绗佸銉窗鐎电厧鍤Ο鈥崇€?

### 3.1 閹绘劕褰囩粊鐐电病缂冩垹绮?

```python
import torch

# 閹绘劕褰囩粵鏍殣缂冩垹绮?
policy_net = model.policy.mlp_extractor

# 鐎电厧鍤稉?ONNX
dummy_input = torch.randn(1, env.observation_space.shape[0])
torch.onnx.export(
    policy_net,
    dummy_input,
    "walker_policy.onnx",
    input_names=['observation'],
    output_names=['action'],
    opset_version=11
)

print(f"濡€崇€峰鎻掝嚤閸? walker_policy.onnx")
```

### 3.2 濡€崇€烽柌蹇撳

```python
import onnx
from onnxruntime.quantization import quantize_dynamic, QuantType

# 閸斻劍鈧線鍣洪崠鏍﹁礋 INT8
quantize_dynamic(
    "walker_policy.onnx",
    "walker_policy_int8.onnx",
    weight_type=QuantType.QInt8
)

# 濡偓閺屻儲膩閸ㄥ銇囩亸?
import os
fp32_size = os.path.getsize("walker_policy.onnx") / 1024
int8_size = os.path.getsize("walker_policy_int8.onnx") / 1024
print(f"FP32: {fp32_size:.2f} KB")
print(f"INT8: {int8_size:.2f} KB (閸樺缂?{(1-int8_size/fp32_size)*100:.1f}%)")
```

---

## 棣冩暋 缁楊剙娲撳銉窗閻懷冪秿閸ヨ桨娆?

### 4.1 缂傛牞鐦?IMC-22 閸ヨ桨娆?

```bash
cd hive-reflex

# 婢跺秴鍩楀Ο鈥崇€烽弶鍐櫢閿涘牓娓剁憰浣芥祮閹诡澀璐?C 閺佹壆绮嶉敍?
python tools/onnx_to_c_array.py walker_policy_int8.onnx > reflex_weights.c

# 缂傛牞鐦ч崶杞版
make APP_SRCS=examples/example_reflex_node.c
```

### 4.2 閻懷冪秿閸掓壆鈥栨禒?

```bash
# 娴ｈ法鏁?OpenOCD 閻懷冪秿
make flash

# 閹存牗澧滈崝銊у劤瑜?
openocd -f interface/jlink.cfg -f target/riscv.cfg \
        -c "program build/hive_node.bin verify reset exit"
```

---

## 棣冩敳 缁楊兛绨插銉窗闁板秶鐤?CAN 缂冩垹绮?

### 5.1 缂冩垹绮堕幏鎾村ⅳ

```
娑撶粯甯?PC (Python)  閳劏鍟? CAN 闁倿鍘ら崳? 閳劏鍟? IMC-22 閼哄倻鍋?1 (ID=1)
                                   閳劏鍟? IMC-22 閼哄倻鍋?2 (ID=2)
                                   閳劏鍟? ...
                                   閳劏鍟? IMC-22 閼哄倻鍋?N (ID=N)
```

### 5.2 Python CAN 閹恒儱褰?

```python
import can

# 閸掓繂顫愰崠?CAN 閹崵鍤?
bus = can.interface.Bus(
    channel='can0',          # Linux: can0, Windows: PCAN_USBBUS1
    bustype='socketcan',     # Linux: socketcan, Windows: pcan
    bitrate=1000000          # 1 Mbps
)

# 閸欐垿鈧礁鎳℃禒銈呭煂閼哄倻鍋?1
def send_command(node_id, target_angle, compliance):
    # 鐏忓棜顫楁惔锕佹祮閹诡澀璐?int16 (閸楁洑缍? 0.01鎼?
    angle_int16 = int(target_angle * 100)
    
    msg = can.Message(
        arbitration_id=0x200 + node_id,
        data=[
            angle_int16 & 0xFF,
            (angle_int16 >> 8) & 0xFF,
            int(compliance * 255)
        ],
        is_extended_id=False
    )
    bus.send(msg)

# 缁€杞扮伐閿涙俺顔曠純顔垮Ν閻?1 閻╊喗鐖ｇ憴鎺戝娑?45鎼达讣绱濋弻鏃堛€庢惔?0.5
send_command(node_id=1, target_angle=45.0, compliance=0.5)
```

### 5.3 閹恒儲鏁归悩鑸碘偓浣稿冀妫?

```python
# 鐠囪褰囬懞鍌滃仯閻樿埖鈧?
msg = bus.recv(timeout=0.1)
if msg and msg.arbitration_id >= 0x100 and msg.arbitration_id < 0x200:
    node_id = msg.arbitration_id - 0x100
    angle = int.from_bytes(msg.data[0:2], 'little', signed=True) * 0.01
    current = int.from_bytes(msg.data[2:4], 'little') * 0.001  # mA
    print(f"閼哄倻鍋?{node_id}: 鐟欐帒瀹?{angle:.2f}鎺? 閻㈠灚绁?{current:.2f}A")
```

---

## 棣冃?缁楊剙鍙氬銉窗鐎圭偞妞傞幒褍鍩?

### 6.1 閹貉冨煑瀵邦亞骞?

```python
import time

def hardware_control_loop():
    """100 Hz 閹貉冨煑瀵邦亞骞?""
    
    while True:
        start_time = time.time()
        
        # 1. 鐠囪褰囬幍鈧張澶庡Ν閻愬湱濮搁幀?
        states = {}
        for node_id in range(1, 13):  # 閸嬪洩顔?12 娑擃亜鍙ч懞?
            msg = bus.recv(timeout=0.001)
            if msg:
                states[node_id] = parse_status(msg)
        
        # 2. 娴ｈ法鏁ょ粵鏍殣缂冩垹绮剁拋锛勭暬閸斻劋缍?
        # 閿涘牆婀?PC 缁旑垵绻嶇悰灞惧腹閻炲棴绱濋幋鏍纯閹恒儱婀?IMC-22 娑撳﹨绻嶇悰宀嬬礆
        observation = build_observation(states)
        action = model.predict(observation)[0]
        
        # 3. 閸欐垿鈧礁鎳℃禒銈呭煂閸氬嫯濡悙?
        for node_id, target in enumerate(action, start=1):
            send_command(node_id, target, compliance=0.5)
        
        # 4. 娣囨繃瀵?100 Hz
        elapsed = time.time() - start_time
        if elapsed < 0.01:
            time.sleep(0.01 - elapsed)

# 鏉╂劘顢戦幒褍鍩楀顏嗗箚
hardware_control_loop()
```

---

## 閳跨媴绗?鐢瓕顫嗛梻顕€顣介崪宀冪殶鐠?

### Q1: 娴犺法婀＄粵鏍殣閸︺劎婀＄€圭偟鈥栨禒鏈电瑐鐞涖劎骞囨稉宥勫尃閿?

**閸樼喎娲?*: Sim-to-Real Gap閿涘牅璞㈤惇鐔剁瑢閻滄澘鐤勫顔跨獩閿?

**鐟欙絽鍠呴弬瑙勵攳**:
1. **閸╃喖娈㈤張鍝勫鐠侇厾绮?*
   ```python
   from godot_robot_env import DomainRandomizationWrapper
   env = DomainRandomizationWrapper(GodotRobotEnv())
   ```

2. **鐠嬪啯鏆ｉ悧鈺冩倞閸欏倹鏆?*
   - 婢х偛濮炴禒璺ㄦ埂娑擃厾娈戦幗鈺傛憹閸?
   - 濞ｈ濮炴导鐘冲妳閸ｃ劌娅旀竟?
   - 濡剝瀚欓悽鍨簚瀵ゆ儼绻?

3. **閸︺劎鈥栨禒鏈电瑐瀵邦喛鐨?*
   - 閺€鍫曟肠閻喎鐤勯弫鐗堝祦
   - 娴ｈ法鏁ゆ潻浣盒╃€涳缚绡?

### Q2: CAN 闁矮淇婃稉宥嚽旂€规熬绱?

**濡偓閺屻儲绔婚崡?*:
- [ ] CAN 閹崵鍤庣紒鍫㈩伂閻㈢敻妯嗛敍?20鎯熼敍?
- [ ] 濞夈垻澹掗悳鍥ㄦЦ閸氾箑灏柊宥忕礄1 Mbps閿?
- [ ] 缁捐法绱楅梹鍨閿涘牆缂撶拋?< 5m閿?
- [ ] 閻㈠灚绨崷鐗堟Ц閸氾箑鍙￠崷?

### Q3: IMC-22 閹恒劎鎮婇柅鐔峰閹鳖澁绱?

**娴兼ê瀵查弬瑙勵攳**:
- 娴ｈ法鏁?INT8 闁插繐瀵查敍鍫濆嚒鐎瑰本鍨氶敍?
- 缁犫偓閸栨牗膩閸ㄥ绱欓崙蹇撶毌闂呮劘妫岀仦鍌︾礆
- 閹绘劙鐝幒褍鍩楁０鎴犲芳閸?NPU 閸愬懘鍎存潻鎰攽

---

## 棣冩惓 閹嗗厴鐎佃鐦?

| 閹稿洦鐖?| 娴犺法婀?(AGI-Walker) | 閻喎鐤勭涵顑挎 (IMC-22) |
|------|------------------|------------------|
| 閹貉冨煑妫版垹宸?| 60 Hz (Godot) | 1000 Hz |
| 瀵ゆ儼绻?| 16 ms | < 0.1 ms |
| 娴肩姵鍔呴崳銊ユ珨婢?| 閸欘垶鍘ょ純?| 閻喎鐤勯崳顏勶紣 |
| 閸旂喕鈧?| - | ~6 W (12 閼哄倻鍋? |

---

## 棣冨箚 閺堚偓娴ｅ啿鐤勭捄?

1. **濞撴劘绻樺蹇涘劥缂?*
   - 閸忓牆婀崡鏇氶嚋閸忓疇濡稉濠冪ゴ鐠?
   - 闁劖顒炴晶鐐插閼哄倻鍋ｉ弫浼村櫤
   - 閺堚偓閸氬孩绁寸拠鏇炵暚閺佸瓨婧€閸ｃ劋姹?

2. **鐎瑰鍙忛幒顏呮煢**
   - 鐠佸墽鐤嗛崝娑氱叐闂勬劕鍩?
   - 鐎圭偟骞囩槐褎鈧儱浠犲?
   - 鏉烆垰鎯庨崝銊ユ嫲鏉烆垰浠犲?

3. **閺佺増宓佺拋鏉跨秿**
   - 鐠佹澘缍嶉幍鈧張澶夌炊閹扮喎娅掗弫鐗堝祦
   - 閻劋绨崚鍡樼€介崪灞炬暭鏉?

4. **閻楀牊婀伴幒褍鍩?*
   - 娣囨繂鐡ㄥВ蹇庨嚋閻楀牊婀伴惃鍕祼娴?
   - 鐠佹澘缍嶉柈銊ц闁板秶鐤?

---

## 棣冩憥 閸欏倽鈧啳绁┃?

- [IMC-22 绾兛娆㈢憴鍕壐](../hardware/HARDWARE_SPEC.md)
- [Hive-Reflex SDK 閹稿洤宕(../hive-reflex/SDK_GUIDE.md)
- [AGI-Walker 闂嗘湹娆㈡惔鎻?../hardware/PARTS_LIBRARY_GUIDE.md)

---

**閺傚洦銆傞悧鍫熸拱**: 1.0  
**閺堚偓閸氬孩娲块弬?*: 2026-01-16  
**缂佸瓨濮㈤懓?*: AGI-Walker Team
