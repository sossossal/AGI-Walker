# AGI-Walker AI濡€崇€烽梿鍡樺灇韫囶偊鈧喐瀵氶崡?

閺堫剚瀵氶崡妤€搴滈崝鈺傚亶韫囶偊鈧喎鐣幋鎬塈濡€崇€烽惃鍕暔鐟佸懎鎷伴柊宥囩枂閵?

---

## 棣冩畬 韫囶偊鈧喎绱戞慨瀣剁礄5閸掑棝鎸撻敍?

### 濮濄儵顎?: 鐎瑰顥奜llama

**Windows**:
```powershell
winget install Ollama.Ollama
```

**閹存牔绗呮潪钘夌暔鐟佸懎娅?*: https://ollama.com/download

### 濮濄儵顎?: 娑撳娴嘇I濡€崇€?

```bash
ollama pull phi3:mini
```

缁涘绶熸稉瀣祰鐎瑰本鍨氶敍鍫㈠2.3GB閿?

### 濮濄儵顎?: 鐎瑰顥奝ython娓氭繆绂?

```bash
cd python_controller
pip install ollama
```

### 濮濄儵顎?: 濞村鐦疉I濡€崇€?

```bash
python ai_model.py
```

鎼存棁顕氶惇瀣煂:
```
閴?濡€崇€?phi3:mini 瀹告彃濮炴潪?
閹笛嗩攽閹恒劎鎮?..

妫板嫭绁撮崝銊ょ稊:
{
  "motors": {
    "hip_left": -2.3,
    "hip_right": 1.8
  },
  "confidence": 0.92
}
```

### 濮濄儵顎?: 鏉╂劘顢慉I閹貉冨煑閸?

1. **閸氼垰濮〨odot娴犺法婀?*閿涘牊瀵淔5閿?
2. **鏉╂劘顢慉I閹貉冨煑閸?*:
   ```bash
   python ai_controller.py --duration 60
   ```

---

## 棣冩惖 鐠囷妇绮忕€瑰顥婂銉╊€?

### 閺傝顢岮: Ollama閿涘牊甯归懡鎰剁礆

#### 1. 鐎瑰顥奜llama

**Windows PowerShell**:
```powershell
# 閺傝纭?: 娴ｈ法鏁inget
winget install Ollama.Ollama

# 閺傝纭?: 娴ｈ法鏁hocolatey
choco install ollama

# 閺傝纭?: 閹靛濮╂稉瀣祰
# 鐠佸潡妫?https://ollama.com/download
```

鐎瑰顥婇崥搴礉Ollama娴兼俺鍤滈崝銊ユ儙閸斻劋璐熼崥搴″酱閺堝秴濮熼妴?

#### 2. 妤犲矁鐦夌€瑰顥?

```bash
ollama --version
```

鎼存棁顕氶弰鍓с仛閻楀牊婀伴崣鍑ょ礉婵? `ollama version 0.1.x`

#### 3. 娑撳娴囧Ο鈥崇€?

**Phi-3-mini**閿涘牊甯归懡鎰剁礆:
```bash
ollama pull phi3:mini
```

**閸忔湹绮柅澶愩€?*:
```bash
ollama pull gemma:2b      # 閺囨潙鐨弴鏉戞彥
ollama pull qwen2:3b      # 娑擃厽鏋冩导妯哄
```

#### 4. 濞村鐦Ο鈥崇€?

```bash
ollama run phi3:mini
```

鏉堟挸鍙? `娴ｇ姴銈介敍灞肩矙缂佸秳绔存稉瀣╃稑閼奉亜绻乣

婵″倹鐏夊Ο鈥崇€峰锝呯埗閸濆秴绨查敍宀冾嚛閺勫骸鐣ㄧ憗鍛灇閸旂噦绱?

#### 5. 鐎瑰顥奝ython鐎广垺鍩涚粩?

```bash
pip install ollama
```

---

### 閺傝顢岯: llama.cpp閿涘牓鐝痪褏鏁ら幋鍑ょ礆

#### 1. 缂傛牞鐦lama.cpp

**Windows (娴ｈ法鏁Make)**:
```powershell
git clone https://github.com/ggerganov/llama.cpp
cd llama.cpp

# 缂傛牞鐦?
cmake -B build
cmake --build build --config Release
```

**閸氼垳鏁PU閸旂娀鈧?* (婵″倹鐏夐張濉廣IDIA GPU):
```powershell
cmake -B build -DLLAMA_CUDA=ON
cmake --build build --config Release
```

#### 2. 娑撳娴囧Ο鈥崇€?

娴犲订uggingFace娑撳娴嘒GUF閺嶇厧绱?

```bash
# 娴ｈ法鏁uggingface-cli
pip install huggingface-hub

huggingface-cli download \
  microsoft/Phi-3-mini-4k-instruct-gguf \
  Phi-3-mini-4k-instruct-q4.gguf \
  --local-dir ./models
```

**閹存牗澧滈崝銊ょ瑓鏉?*:
https://huggingface.co/microsoft/Phi-3-mini-4k-instruct-gguf

#### 3. 濞村鐦Ο鈥崇€?

```bash
.\build\bin\Release\main.exe \
  -m weights/Phi-3-mini-4k-instruct-q4.gguf \
  -p "Hello, introduce yourself" \
  -n 50
```

#### 4. 鐎瑰顥奝ython缂佹垵鐣?

```bash
pip install llama-cpp-python
```

**GPU閻楀牊婀?*:
```bash
CMAKE_ARGS="-DLLAMA_CUDA=ON" pip install llama-cpp-python
```

---

## 棣冃?濞村鐦稉搴ㄧ崣鐠?

### 濞村鐦?: 濡€崇€烽崫宥呯安闁喎瀹?

```bash
python -c "
from ai_model import create_ai_model
import time

ai = create_ai_model()
dummy = {
    'sensors': {
        'imu': {'orient': [5, -3, 0]},
        'joints': {
            'hip_left': {'angle': 10},
            'hip_right': {'angle': -8}
        }
    },
    'torso_height': 1.45
}

# 濞村鐦?0濞?
times = []
for _ in range(10):
    t0 = time.time()
    ai.predict(dummy)
    times.append(time.time() - t0)

print(f'楠炲啿娼? {sum(times)/len(times)*1000:.1f}ms')
"
```

**閻╊喗鐖?*: < 100ms

### 濞村鐦?: JSON閺嶇厧绱?

```bash
python ai_model.py
```

濡偓閺屻儴绶崙鐑樻Ц閸氾缚璐熼張澶嬫櫏JSON閵?

### 濞村鐦?: 闂嗗棙鍨氶幒褍鍩?

```bash
# 1. 閸氼垰濮〨odot (閸欙缚绔存稉顏嗙矒缁?
# 2. 鏉╂劘顢慉I閹貉冨煑閸?
python ai_controller.py --duration 30
```

**妤犲本鏁归弽鍥у櫙**:
- 閴?閺冪姾绻涢幒銉╂晩鐠?
- 閴?閹貉冨煑妫版垹宸?> 20Hz
- 閴?閺冪嚒SON鐟欙絾鐎介柨娆掝嚖
- 閴?閺堝搫娅掓禍杞扮箽閹镐胶鐝粩?

---

## 棣冩礈閿?閺佸懘娈伴幒鎺楁珟

### 闂傤噣顣?: Ollama閺堝秴濮熼張顏勬儙閸?

**閻ュ洨濮?*:
```
閴?閺冪姵纭舵潻鐐村复閸掔櫂llama閺堝秴濮?
```

**鐟欙絽鍠?*:
```bash
# Windows: 閸︺劌绱戞慨瀣綅閸楁洘鎮崇槐?Ollama"楠炶泛鎯庨崝?

# 閹存牕鎳℃禒銈堫攽閸氼垰濮?
ollama serve
```

### 闂傤噣顣?: 濡€崇€锋稉瀣祰婢惰精瑙?

**鐟欙絽鍠?*:
```bash
# 娴ｈ法鏁ら崶钘夊敶闂€婊冨剼閿涘牆顩ч弸婊堟付鐟曚緤绱?
export OLLAMA_HOST=https://ollama.mirror.cn

# 閹存牗澧滈崝銊ょ瑓鏉炶棄鎮楃€电厧鍙?
ollama create phi3:mini -f Modelfile
```

### 闂傤噣顣?: 閹恒劎鎮婃径顏呭弮

**閻ュ洨濮?*: 閹恒劎鎮?> 200ms

**鐟欙絽鍠呴弬瑙勵攳**:
1. 娴ｈ法鏁PU閸旂娀鈧?
2. 娴ｈ法鏁ら弴鏉戠毈閻ㄥ嫭膩閸ㄥ绱檊emma:2b閿?
3. 閸戝繐鐨痐num_predict`閸欏倹鏆?
4. 閸楀洨楠囩涵顑挎

### 闂傤噣顣?: JSON閺嶇厧绱￠柨娆掝嚖

**閻ュ洨濮?*: 
```
閴?JSON鐟欙絾鐎介柨娆掝嚖
```

**娑撳瓨妞傜憴锝呭枀**:
AI濡€崇€锋导姘崇箲閸ョ偤绮拋銈呯暔閸忋劌濮╂担婊愮礄鐟欐帒瀹?閿?

**闂€鎸庢埂鐟欙絽鍠?*:
1. 娴ｈ法鏁rammar缁撅附娼敍鍧檒ama.cpp閿?
2. 娴兼ê瀵睵rompt
3. 婢х偛濮為崥搴☆槱閻炲棔鎱ㄦ径?

---

## 閳挎瑱绗?闁板秶鐤嗘导妯哄

### 闂勫秳缍嗗鎯扮箿

閸?`ai_model.py` 娑擃叀鐨熼弫?
```python
options={
    'temperature': 0.05,  # 闂勫秳缍嗗〒鈺佸
    'num_predict': 30,    # 閸戝繐鐨悽鐔稿灇token閺?
    'top_p': 0.85
}
```

### 閹绘劙鐝粙鍐茬暰閹?

```python
options={
    'temperature': 0.01,  # 閺嬩椒缍嗗〒鈺佸
    'top_k': 10,          # 闂勬劕鍩楅柌鍥ㄧ壉閼煎啫娲?
    'repeat_penalty': 1.1
}
```

---

## 棣冩惓 閹嗗厴閸╁搫鍣?

| 绾兛娆㈤柊宥囩枂 | 濡€崇€?| 閹恒劎鎮婇柅鐔峰 | 閹貉冨煑妫版垹宸?|
|---------|------|---------|---------|
| i7-12700 | Phi-3 Q4 | 40-60ms | 25-30Hz 閴?|
| i5-10400 | Phi-3 Q4 | 80-120ms | 12-18Hz 閳跨媴绗?|
| RTX 4060 | Phi-3 Q4 | 15-25ms | 40-50Hz 棣冩暉 |

---

## 棣冨笚 娑撳绔村?

鐎瑰本鍨氱€瑰顥婇崥?

1. **鐠嬪啩绱璓rompt** - 閺€纭呯箻 `_build_prompt()` 閺傝纭?
2. **閺€鍫曟肠閺佺増宓?* - 鐠佹澘缍嶉幋鎰閻ㄥ嫭甯堕崚鎯板缓鏉?
3. **闂嗗棙鍨歅ID** - 缂佹挸鎮嶱ID閹貉冨煑閸?
4. **濞ｈ濮?0B娴兼ê瀵查崳?* - 鐎圭偟骞囩粵鏍殣娴兼ê瀵?

---

> 棣冩寱 **閹绘劗銇?*: 缁楊兛绔村▎陇绻嶇悰灞藉讲閼充粙娓剁憰浣风娴滄稒妞傞梻纾嬵唨濡€崇€?閻戭叀闊?閿涘苯鎮楃紒顓熷腹閻炲棔绱伴弴鏉戞彥閵?
