# 閾忔碍瀚欓悳顖氼暔閺嬪嫬缂撴稉搴ｆ晸閹存劖瀵氶崡?(Environment Generation Guide)

閺堫剚鏋冨锝呮礀缁涙柨鍙ф禍?婵″倷缍嶉崚娑樼紦閾忔碍瀚欓悳顖氼暔"娴犮儱寮?閺勵垰鎯侀棁鈧憰浣哄箚婢у啰鏁撻幋鎰侀崹?閻ㄥ嫰妫舵０妯糕偓?

## 1. 閾忔碍瀚欓悳顖氼暔閸掓稑缂撻惃鍕瑏缁夊秹鈧柨绶?

闁藉牆顕?AGI-Walker 妞ゅ湱娲伴敍灞惧灉娴狀剚甯归懡鎰簰娑撳绗佺粔宥嗙€铏瑰箚婢у啰娈戠捄顖氱窞閿涘奔绮犻弰鎾冲煂闂呮拝绱?

### 鐠侯垰绶?A: 閹靛濮╅幖顓炵紦 (Manual Setup)
**闁倸鎮庨梼鑸殿唽**: 閸掓繃婀℃宀冪槈閵嗕胶澹掔€规艾婧€閺咁垱绁寸拠?(婵?閸欎即妯佸ù瀣槸")閵?
*   **閺傝纭?*: 娴ｈ法鏁?Godot 缂傛牞绶崳顭掔礉閹锋牗瀚?`CSGBox3D` 閹碱厼缂撻崷浼存桨閵嗕礁顣炬竟浣告嫲闂呮粎顣查悧鈹库偓?
*   **娴兼鍋?*: 缁墽鈥橀幒褍鍩楅敍宀€鐣濋崡鏇犳纯鐟欏倶鈧?
*   **閹稿洤宕?*: 鐠囧嘲寮懓?[SCENE_SETUP_GUIDE.md](../godot_project/SCENE_SETUP_GUIDE.md)閵?

### 鐠侯垰绶?B: 缁嬪绨崠鏍晸閹?(Procedural Generation - **閹恒劏宕?*)
**闁倸鎮庨梼鑸殿唽**: 瀵搫瀵茬€涳缚绡勭拋顓犵矊 (RL Training)閵?
*   **閺傝纭?*: 缂傛牕鍟?Godot 閼存碍婀?(`.gd`) 閹?Python 閼存碍婀伴敍灞藉焺閻劎鐣诲▔?(婵?Perlin Noise, Wave Function Collapse) 閼奉亜濮╅悽鐔稿灇閺冪娀妾洪崣妯哄閻?terrain閵?
*   **娴兼鍋?*: 閼宠棄顧勯悽鐔稿灇鏉╂垳绠弮鐘绘閻ㄥ嫯顔勭紒鍐ㄦ簚閺咁垽绱濋梼鍙夘剾閺堝搫娅掓禍?濮濇槒顔囩涵顒冨剹"閸︽澘娴?(Overfitting)閵?

### 鐠侯垰绶?C: AI 濡€崇€烽悽鐔稿灇 (GenAI / Model-based)
**闁倸鎮庨梼鑸殿唽**: 妤傛楠?Sim2Real 鐟欏棜顫庣拋顓犵矊閵?
*   **閺傝纭?*: 閹恒儱鍙?Stable Diffusion (閻㈢喐鍨氱痪鍦倞/閼冲本娅? 閹?Shap-E/Point-E (閻㈢喐鍨?D鐠у嫪楠?閵?
*   **閸ョ偟鐡熼幃銊ф畱閹绘劙妫?*: **閻╊喖澧犻梼鑸殿唽娑撳秹娓剁憰?*韫囧懏甯?閻滎垰顣ㄩ悽鐔稿灇濡€崇€?閵嗗倸顕禍搴″蓟鐡掕櫕婧€閸ｃ劋姹夐惃鍕箥閸斻劍甯堕崚鎯邦唲缂佸喛绱濋崙鐘辩秿缂佹挻鐎惃鍕柤鎼村繐瀵查悽鐔稿灇 (鐠侯垰绶?B) 濮ｆ棁顫嬬憴澶屾晸閹存劗娈?AI 濡€崇€烽弴鎾櫢鐟曚椒绗栭弴鎾彯閺佸牄鈧?

---

## 2. 閺傝顢嶇拠锕佇掗敍姘扁柤鎼村繐瀵查悽鐔稿灇 (PCG)

閹存垳婊戝楦款唴閸?Godot 娑擃厺濞囬悽?**HeightMapShape3D** 閹?**GridMap** 閺夈儱鐤勯悳鏉垮З閹礁婀磋ぐ顫偓?

### 閺傝顢嶆稉鈧敍姘剁彯缁嬪娴橀崷鏉胯埌 (HeightMap)
闁倻鏁ゆ禍搴⒛侀幏鐔煎櫣婢舵牞鎹ｆ导蹇撴勾闂堫潿鈧?

**Godot 鐎圭偟骞囨导顏冨敩閻?(`Main.gd`):**

```gdscript
extends Node3D

func generate_terrain(size: int = 100):
    var height_map = HeightMapShape3D.new()
    height_map.map_width = size
    height_map.map_depth = size
    
    var data = PackedFloat32Array()
    var noise = FastNoiseLite.new() # 娴ｈ法鏁?Godot 閸愬懐鐤嗛崳顏勶紣
    
    for y in range(size):
        for x in range(size):
            # 閻㈢喐鍨氶梾蹇旀簚妤傛ê瀹?
            var h = noise.get_noise_2d(x, y) * 2.0 
            data.append(h)
    
    height_map.map_data = data
    
    # 閸掓稑缂撶喊鐗堟寬娴?
    var collider = CollisionShape3D.new()
    collider.shape = height_map
    $Ground.add_child(collider)
```

### 閺傝顢嶆禍宀嬬窗缂冩垶鐗搁崷鏉挎禈 (GridMap)
闁倻鏁ゆ禍搴⒛侀幏鐔风厔鐢倻骞嗘晶鍐︹偓浣广偧濮婎垬鈧浇铔嬪濞库偓?

1.  **閸掓湹缍旈崶鎯ф健闂?(MeshLibrary)**: 閸掓湹缍?楠炲啿婀?閵?閺傛粌娼?閵?閸欎即妯?閵?婢ф瑥顥?缁涘顣╅崚鏈垫閵?
2.  **娴狅絿鐖滈悽鐔稿灇**:

```gdscript
extends GridMap

func generate_dungeon():
    clear()
    for x in range(20):
        for z in range(20):
            if randf() > 0.2:
                set_cell_item(Vector3i(x, 0, z), 0) # 0閸欓攱妲搁獮鍐叉勾
            else:
                set_cell_item(Vector3i(x, 0, z), 1) # 1閸欓攱妲搁梾婊咁暡閻?
```

---

## 3. Python 婢舵牠鍎撮悽鐔稿灇閹恒儱褰?

婵″倹鐏夐幃銊ョ瑖閺堟稑婀?Python 缁旑垱甯堕崚鍓佸箚婢у啰鏁撻幋?(娓氬顩х紒鎾虫値 `terrain_mapper.py`)閿涘苯褰叉禒銉ョ暰娑斿鈧矮淇婇崡蹇氼唴閿?

1.  **Python 缁?*: 閻㈢喐鍨氭稉鈧稉?`N x N` 閻ㄥ嫰鐝粙瀣叐闂?(numpy array)閵?
2.  **闁矮淇?*: 闁俺绻?TCP 閸欐垿鈧?`update_terrain` 閹稿洣鎶ら崣濠勭叐闂冨灚鏆熼幑顔煎煂 Godot閵?
3.  **Godot 缁?*: 閹恒儲鏁归弫鐗堝祦楠炶埖娲块弬?`HeightMapShape3D`閵?

### 缁€杞扮伐閹稿洣鎶ょ紒鎾寸€?
```json
{
    "type": "update_terrain",
    "width": 32,
    "depth": 32,
    "data": [0.0, 0.1, 0.2, ...] // 鐏炴洖閽╅惃鍕彯鎼达附鏆熺紒?
}
```

---

## 4. 閹崵绮ㄦ稉搴＄紦鐠?

1.  **瑜版挸澧?*: 缂佈呯敾娴ｈ法鏁ら幍瀣З閹碱厼缂撻惃鍕暆閸楁洖婧€閺?(`scenes/main.tscn`) 鐠烘垿鈧艾鐔€绾偓 RL閵?
2.  **娑擃厽婀?*: 鐎圭偟骞囨稉鈧稉顏嗙暆閸楁洜娈?Godot 閼存碍婀伴敍灞剧槨濞?Reset 閺冨爼娈㈤張鍝勫閸︿即娼伴崸鈥冲閹存牗鍧婇崝鐘绘閺堢儤鏌熼崸妤呮绾板秶澧块妴?
3.  **妤傛楠?*: 閸欘亝婀侀崷銊╂付鐟曚焦鐎妯款潒鐟欏鈧偐婀℃惔锔芥 (濮ｆ柨顩х拋顓犵矊缁绢垵顫嬬憴澶婎嚤閼?閿涘本澧犻棁鈧憰浣解偓鍐閹恒儱鍙?GenAI 濡€崇€烽悽鐔稿灇閻滎垰顣ㄧ挧鍕獓閵?
