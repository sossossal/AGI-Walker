# AGI-Walker Godot闂嗗棙鍨氶幐鍥у础

閺堫剚瀵氶崡妤勵嚛閺勫骸顩ф担鏇炵殺GUI闁板秶鐤嗛崳銊ㄧ箾閹恒儱鍩孏odot娴犺法婀″鏇熸惛閿涘苯鐤勯悳鎵埂鐎圭偟娈戦張鍝勬珤娴滆桨璞㈤惇鐔粹偓?

---

## 棣冨箚 閺嬭埖鐎鍌濐潔

```
GUI闁板秶鐤嗛崳?(Python/Tkinter)
        閳?
  godot_client.py (TCP Client)
        閳?(TCP Socket)
Godot TCP Server (GDScript)
        閳?
  Godot閻椻晝鎮婂鏇熸惛/閺堝搫娅掓禍鍝勬簚閺?
```

---

##  瀹告彃鐣幋鎰畱缂佸嫪娆?

### 1. Python闁矮淇婄€广垺鍩涚粩?

**閺傚洣娆?*: `python_api/godot_client.py`

**閸旂喕鍏?*:
- TCP鏉╃偞甯寸粻锛勬倞
- 閸涙垝鎶ら崣鎴︹偓渚婄礄閸氼垰濮?閸嬫粍顒?閸欏倹鏆熼弴瀛樻煀閿?
- 閺佺増宓侀幒銉︽暪閿涘牆绱撳銉ユ倵閸欐壆鍤庣粙瀣剁礆
- 閸ョ偠鐨熼張鍝勫煑

**娴ｈ法鏁ょ粈杞扮伐**:
```python
from python_api.godot_client import GodotSimulationClient

# 閸掓稑缂撶€广垺鍩涚粩?
client = GodotSimulationClient(host="127.0.0.1", port=9999)

# 鐠佸墽鐤嗛弫鐗堝祦閸ョ偠鐨?
client.set_data_callback(lambda data: print(data))

# 鏉╃偞甯?
if client.connect():
    # 閸氼垰濮╂禒璺ㄦ埂
    robot_config = {'parts': [...], 'connections': [...]}
    client.start_simulation(robot_config)
    
    # 鐎圭偞妞傞弴瀛樻煀閸欏倹鏆?
    client.update_parameters({'motor_power': 1.2})
    
    # 閸嬫粍顒?
    client.stop_simulation()
    client.disconnect()
```

### 2. GUI闂嗗棙鍨氶敍鍫ュ劥閸掑棴绱?

**閺傚洣娆?*: `tools/robot_configurator_gui.py`

**瀹稿弶鍧婇崝?*:
- GodotSimulationClient鐎电厧鍙?
- 鏉╃偞甯碪I閹貉傛閿涘牆婀撮崸鈧?缁旑垰褰涙潏鎾冲弳閿?
- 鏉╃偞甯撮悩鑸碘偓浣规▔缁€?

**瀵板懎鐣幋?*: 鐏忓棛骞囬張澶屾畱FeedbackPanel鐎瑰苯鍙忛弨褰掆偓鐘辫礋Godot闂嗗棙鍨氶悧鍫熸拱

---

## 棣冩暋 Godot缁旑垰鐤勯悳鎷岊洣濮?

### TCP閺堝秴濮熼崳?(GDScript)

**閺傚洣娆?*: `godot_project/scripts/TCPSimulationServer.gd`

```gdscript
extends Node

var server = TCP_Server.new()
var clients = []
var port = 9999

func _ready():
    server.listen(port)
    print("娴犺法婀￠張宥呭閸ｃ劌鎯庨崝銊ょ艾缁旑垰褰? ", port)

func _process(delta):
    # 閹恒儱褰堟潻鐐村复
    if server.is_connection_available():
        var client = server.take_connection()
        clients.append(client)
        print("鐎广垺鍩涚粩顖氬嚒鏉╃偞甯?)
    
    # 婢跺嫮鎮婂☉鍫熶紖
    for client in clients:
        if client.get_available_bytes() > 0:
            handle_message(client)

func handle_message(client):
    # 鐠囪褰囬梹鍨閸撳秶绱?
    var length_bytes = client.get_data(4)
    if length_bytes[0] != OK:
        return
    
    var length = bytes_to_var(length_bytes[1])
    
    # 鐠囪褰嘕SON閺佺増宓?
    var data_bytes = client.get_data(length)
    if data_bytes[0] != OK:
        return
    
    var json_str = data_bytes[1].get_string_from_utf8()
    var data = JSON.parse(json_str).result
    
    match data.command:
        "start_sim":
            start_simulation(data.data)
        "stop_sim":
            stop_simulation()
        "update_params":
            update_parameters(data.data)
        "load_robot":
            load_robot_config(data.data)

func start_simulation(config):
    print("閸氼垰濮╂禒璺ㄦ埂: ", config)
    # TODO: 閸旂姾娴囬張鍝勬珤娴滄椽鍘ょ純?
    # TODO: 瀵偓婵澧块悶鍡樐侀幏?
    pass

func send_feedback(client, data):
    var json_str = JSON.print(data)
    var json_bytes = json_str.to_utf8()
    
    var length = json_bytes.size()
    var length_bytes = var_to_bytes(length)
    
    client.put_data(length_bytes)
    client.put_data(json_bytes)
```

---

## 棣冩惖 闂嗗棙鍨氬銉╊€?

### 濮濄儵顎?: 鐎瑰苯鏉絇ython GUI閿涘牆鍑＄€瑰本鍨?0%閿?

- [x] 閸掓稑缂揼odot_client.py
- [x] 濞ｈ濮炴潻鐐村复UI
- [ ] 鐎瑰苯鍙忛柌宥呭晸FeedbackPanel
- [ ] 闂嗗棙鍨氶崣鍌涙殶閸氬本顒?
- [ ] 濞ｈ濮為柨娆掝嚖婢跺嫮鎮?

### 濮濄儵顎?: 鐎圭偟骞嘒odot閺堝秴濮熼崳?

1. 閸掓稑缂?`godot_project/scripts/TCPSimulationServer.gd`
2. 鐏忓棗鍙惧ǎ璇插閸掗瀵岄崷鐑樻珯娴ｆ粈璐熼懛顏勫З閸旂姾娴囬懞鍌滃仯
3. 鐎圭偟骞囧☉鍫熶紖婢跺嫮鎮婇柅鏄忕帆
4. 濞ｈ濮炴禒璺ㄦ埂閺佺増宓侀崣宥夘洯

### 濮濄儵顎?: 濞村鐦潻鐐村复

1. 閸氼垰濮〨odot妞ゅ湱娲伴敍鍦盋P閺堝秴濮熼崳銊ㄥ殰閸斻劏绻嶇悰宀嬬礆
2. 閸氼垰濮〨UI闁板秶鐤嗛崳?
3. 閻愮懓鍤?鏉╃偞甯碐odot"
4. 妤犲矁鐦夋潻鐐村复閻樿埖鈧?

### 濮濄儵顎?: 鐎圭偟骞囨禒璺ㄦ埂閸旂喕鍏?

1. 閸︹剠odot娑擃厼鐤勯悳鐗堟簚閸ｃ劋姹夐崝鐘烘祰
2. 閸欏倹鏆熺€圭偞妞傞弴瀛樻煀
3. 閻樿埖鈧焦鏆熼幑顔兼礀娴?
4. GUI閺勫墽銇氱€圭偞妞傞弫鐗堝祦

---

## 棣冩敳 鏉╃偞甯村ù瀣槸

### 閺傝纭?: 娴ｈ法鏁ゅΟ鈩冨珯閺堝秴濮熼崳?

```bash
# 缂佸牏顏?: 閸氼垰濮╁Ο鈩冨珯Godot閺堝秴濮熼崳?
cd d:\閺傛澘缂撻弬鍥︽婢剁AGI-Walker
python -c "from python_api.godot_client import MockGodotServer; import time; s=MockGodotServer(); s.start(); time.sleep(999)"

# 缂佸牏顏?: 閸氼垰濮〨UI
python tools\robot_configurator_gui.py
```

### 閺傝纭?: 娴ｈ法鏁ら惇鐔风杽Godot

1. 閹垫挸绱慓odot妞ゅ湱娲? `godot_project/project.godot`
2. 濞ｈ濮濼CPSimulationServer.gd閸掓澘婧€閺?
3. 鏉╂劘顢慓odot妞ゅ湱娲?
4. 閸氼垰濮〨UI楠炴儼绻涢幒?

---

## 棣冩惓 閺佺増宓侀崡蹇氼唴

### Python 閳?Godot (閸涙垝鎶?

```json
{
  "command": "start_sim",
  "data": {
    "robot": {
      "parts": [{"id": "motor_1", "type": "motor"}],
      "connections": [{"from": "motor_1", "to": "ctrl_1"}]
    },
    "physics": {
      "gravity": 9.81,
      "timestep": 0.01
    }
  },
  "timestamp": 1234567890.123
}
```

### Godot 閳?Python (閸欏秹顩?

```json
{
  "type": "simulation_data",
  "position": 0.5,
  "velocity": 0.3,
  "battery": 85.0,
  "joint_angles": [0.1, 0.2, 0.3, 0.4],
  "timestamp": 1234567890.456
}
```

---

## 棣冃?閸楁洖鍘撳ù瀣槸

### 濞村鐦痝odot_client.py

```bash
# 鏉╂劘顢戦崘鍛枂濞村鐦?
cd d:\閺傛澘缂撻弬鍥︽婢剁AGI-Walker
python python_api\godot_client.py
```

### 濞村鐦疓UI鏉╃偞甯?

1. 閸氼垰濮〨UI
2. 閸︺劏绻涢幒銉╂桨閺夎儻绶崗?`127.0.0.1:9999`
3. 閻愮懓鍤?鏉╃偞甯碐odot"
4. 閺屻儳婀呴悩鑸碘偓浣瑰瘹缁€鍝勬珤

---

## 閳跨媴绗?鐢瓕顫嗛梻顕€顣?

### Q: 鏉╃偞甯存径杈Е閹簼绠為崝鐑囩吹
A: 濡偓閺屻儻绱?
1. Godot閺勵垰鎯佹潻鎰攽
2. TCP閺堝秴濮熼崳銊︽Ц閸氾箑鎯庨崝?
3. 缁旑垰褰?999閺勵垰鎯佺悮顐㈠窗閻?
4. 闂冭尙浼€婢ф瑨顔曠純?

### Q: 閺佺増宓佹稉宥嗘纯閺傚府绱?
A: 濡偓閺屻儻绱?
1. 閸ョ偠鐨熼崙鑺ユ殶閺勵垰鎯佸锝団€樼拋鍓х枂
2. Godot閺勵垰鎯佸锝呮躬閸欐垿鈧焦鏆熼幑?
3. 缂冩垹绮跺鎯扮箿

### Q: GUI閸椻剝顒撮敍?
A: 閹碘偓閺堝缍夌紒婊勬惙娴ｆ粓鍏橀崷銊ユ倵閸欐壆鍤庣粙瀣剁礉娑撳秴绨茬拠銉ュ幢濮濇眹鈧倹顥呴弻銉窗
1. 閺勵垰鎯侀張澶婄磽鐢憡婀幑鏇″箯
2. 閺佺増宓侀崶鐐剁殶閺勵垰鎯佹担璺ㄦ暏娴滃摲after()`閺傝纭?

---

## 棣冩畬 娑撳绔村?

1. **鐎瑰本鍨欶eedbackPanel閺€褰掆偓?*
   - 鐏忓棙澧嶉張澶嬆侀幏鐔告殶閹诡喗娴涢幑顫礋Godot閺佺増宓?
   - 濞ｈ濮炴潻鐐村复閻樿埖鈧胶娲冮幒?
   - 鐎圭偟骞囬崣鍌涙殶鐎圭偞妞傞崥灞绢劄

2. **鐎圭偟骞嘒odot閺堝秴濮熼崳?*
   - 閸掓稑缂撶€瑰本鏆ｉ惃鍑綜P閺堝秴濮熼崳?
   - 閺堝搫娅掓禍娲帳缂冾喖濮炴潪?
   - 閻椻晝鎮婇崣鍌涙殶閸斻劍鈧浇鐨熼弫?

3. **濞ｈ濮炲ù瀣槸**
   - 閸楁洖鍘撳ù瀣槸
   - 闂嗗棙鍨氬ù瀣槸
   - 閹嗗厴濞村鐦?

---

**閻樿埖鈧?*: 棣冪厸 鏉╂稖顢戞稉?(60%)  
**娑撳绔撮惄顔界垼**: 鐎瑰本鍨欶eedbackPanel闁插秴鍟撻獮鑸电ゴ鐠囨洖鐔€閺堫剝绻涢幒?
