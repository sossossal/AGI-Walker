# AGI-Walker 娴犺法婀￠悳顖氼暔娴ｈ法鏁ら幐鍥у础 (Simulation Guide)

閺堫剚瀵氶崡妤€鐨㈢敮顔煎И閹劌鎻╅柅鐔剁瑐閹?AGI-Walker 閻ㄥ嫪琚辩粔宥勮雹閻喐膩瀵骏绱?*閸欘垵顫嬮崠?3D 娴犺法婀?(Godot)** 閸?**韫囶偊鈧喐鏆熺€涳缚璞㈤惇?(Python)**閵?

---

## 棣冩礈閿?1. 閻滎垰顣ㄩ崙鍡楊槵

### 1.1 鐎瑰顥?Python 娓氭繆绂?
绾喕绻氬鎻掔暔鐟佸懘銆嶉惄顔藉闂団偓閻?Python 鎼存搫绱?
```bash
pip install -r requirements.txt
```

### 1.2 鐎瑰顥?Godot 瀵洘鎼?(閻劋绨?3D 娴犺法婀?
AGI-Walker 娴ｈ法鏁?**Godot Engine 4.x** 鏉╂稖顢戦悧鈺冩倞濞撳弶鐓嬮崪灞艰雹閻喆鈧?
1.  娑撳娴?Godot 4.x: [Godot 鐎规缍塢(https://godotengine.org/download)
2.  鐏?Godot 閸欘垱澧界悰灞炬瀮娴犳儼鐭惧鍕潑閸旂姴鍩岄悳顖氼暔閸欐﹢鍣洪敍灞惧灗鐠侀绗呴崗鎯扮熅瀵板嫨鈧?
3.  **鐎电厧鍙嗘い鍦窗**: 閹垫挸绱?Godot閿涘瞼鍋ｉ崙?"Import"閿涘矂鈧瀚?`AGI-Walker/godot_project/project.godot` 閺傚洣娆㈤妴?

---

## 棣冩灱閿?2. 濡€崇础娑撯偓閿涙艾褰茬憴鍡楀 3D 娴犺法婀?(Godot)
**闁倻鏁ら崷鐑樻珯**: 瀵搫瀵茬€涳缚绡勭拋顓犵矊 (RL)閵嗕浇顫嬬憴澶嬫殶閹诡噣鍣伴梿鍡愨偓浣圭川缁€楦款潎鐎电喆鈧?

濮濄倖膩瀵繋绗呴敍瀛瓂thon 閼存碍婀伴崗鍛秼"婢堆嗗壋"閿涘瓘odot 閸忓懎缍?闊偂缍?閸?娑撴牜鏅?閿涘奔琚遍懓鍛粹偓姘崇箖 TCP 缂冩垹绮堕柅姘繆閵?

### 棣冩礈閿?閸忔娊鏁柊宥囩枂閿涙艾鎯庨悽銊ユ勾瑜般垻鏁撻幋?(妫ｆ牗顐兼潻鎰攽韫囧懎浠?
娑撹桨绨℃担璺ㄢ柤鎼村繐瀵查崷鏉胯埌閻㈢喐鍨?(PCG) 閻㈢喐鏅ラ敍灞惧亶闂団偓鐟曚礁婀?Godot 缂傛牞绶崳銊よ厬**閹靛濮╅幙宥勭稊娑撯偓濞?*閿?

> [!IMPORTANT]
> 濮濄倖顒炴銈咁嚠娴滃骸婀?RL 鐠侇厾绮屾稉顓″箯瀵?*闂呭繑婧€閸︽澘鑸?*閼峰啿鍙ч柌宥堫洣閿涗礁顩ч弸婊€绗夐崑姘剧礉閺堝搫娅掓禍鍝勭殺閸欘亣鍏橀崷銊╃帛鐠併倕閽╅崷棰佺瑐鐞涘矁铔嬮妴?

1.  **閹垫挸绱戞稉璇叉簚閺?*: 閸?Godot 閺傚洣娆㈢化鑽ょ埠娑擃厼寮婚崙?`res://scenes/main.tscn`閵?
2.  **濞ｈ濮為悽鐔稿灇閸ｃ劏濡悙?*: 
    *   閸欐娊鏁悙鐟板毊閺嶇濡悙?`Main` -> `Add Child Node`閵?
    *   閹兼粎鍌ㄩ獮鍫曗偓澶嬪 `Node3D`閵?
    *   **闁插秴鎳￠崥?*: 鐏忓棙鏌婇懞鍌滃仯闁插秴鎳￠崥宥勮礋 `TerrainGenerator` (韫囧懘銆忕€瑰苯鍙忛崠褰掑帳閿涘苯灏崚鍡椼亣鐏忓繐鍟?閵?
3.  **闂勫嫬濮為懘姘拱**:
    *   鐏忓棜鍓奸張?`scripts/environment/procedural_terrain.gd` 娴犲孩鏋冩禒鍫曟桨閺夋寧瀚嬮幏钘夊煂閸掓艾鍨卞铏规畱 `TerrainGenerator` 閼哄倻鍋ｆ稉濞库偓?
4.  **娣囨繂鐡ㄩ崷鐑樻珯**: 閹?`Ctrl+S` 娣囨繂鐡ㄩ妴?

---

### 閸氼垰濮╁銉╊€?
1.  **閸氼垰濮?Godot**:
    *   閸?Godot 缂傛牞绶崳銊よ厬閹垫挸绱戞い鍦窗閵?
    *   閻愮懓鍤崣鍏呯瑐鐟欐帞娈?**Run (閹绢厽鏂侀崶鐐垼)** 閸氼垰濮╂禒璺ㄦ埂閺堝秴濮熼崳銊ｂ偓?
    *   *濮濄倖妞?Godot 缁愭褰涙惔鏃€妯夌粈?Waiting for connection..."*

2.  **鏉╂劘顢?Python 閹貉冨煑閼存碍婀?*:
    *   閹垫挸绱戠紒鍫㈩伂閿涘矁绻嶇悰?RL 鐠侇厾绮岄懘姘拱閿?
        ```bash
        python examples/my_first_robot_train.py
        ```
    *   閹存牞鈧懓绻嶇悰?Sim2Real 閸掑棙鐎介懘姘拱閿?
        ```bash
        python python_controller/sim2real_analyzer.py
        ```

3.  **鐟欏倸鐧傛潻鎰攽**:
    *   Python 缁旑垯绱伴弰鍓с仛鐠侇厾绮岄弮銉ョ箶/閹貉冨煑閺冦儱绻旈妴?
    *   Godot 缁旑垯绱伴弰鍓с仛閺堝搫娅掓禍铏规畱鐎圭偞妞傞崝銊ょ稊閵?

---

## 閳?3. 濡€崇础娴滃矉绱拌箛顐︹偓鐔告殶鐎涳缚璞㈤惇?(Python)
**闁倻鏁ら崷鐑樻珯**: 閻椻晝鎮婇崣鍌涙殶妤犲矁鐦夐妴涔€CO 閹存劖婀伴崚鍡樼€介妴浣哥唨閺堫剝绻嶉崝銊ヮ劅濠曟梻銇氶妴?

濮濄倖膩瀵繋绗夐棁鈧憰?Godot閿涘瞼鍑?Python 鏉╂劘顢戦敍宀勨偓鐔峰閺嬩礁鎻╅敍宀勨偓鍌氭値韫囶偊鈧喖鐛欑拠浣稿棘閺佹澘鎮庨悶鍡樷偓褋鈧?

### 鏉╂劘顢戠粈杞扮伐
鏉╂劘顢戦崜宥堢箻 1 缁磭娈戦崣鍌涙殶鐠嬪啯鏆ｅ鏃傘仛閿?
```bash
python examples/walk_1m_demo.py
```
*   鐠囥儴鍓奸張顑跨窗鐎佃鐦?姒涙顓?閵?妤傛ê濮涢悳?閵?鏉╁洭鍣?缁涘绗夐崥宀勫帳缂冾喕绗呴惃鍕簚閸ｃ劋姹夌悰銊у箛閵?
*   鏉╂劘顢戠紒鎾存将閸氬簼绱伴悽鐔稿灇鏉炪劏鎶楅崶?`robot_forward_1m_demo.png`閵?

---

## 棣冾樆 4. 濡剝瀚欓張鍝勬珤娴滄椽鍘ょ純顔荤瑢閸掑洦宕?

### 4.1 閸掑洦宕查張鍝勬珤娴滆櫣琚崹?
AGI-Walker 閸愬懐鐤嗘禍鍡曠瑏缁夊秵鐖ｉ崙鍡樼€崹瀣剁窗`Biped` (閸欏矁鍐?, `Quadruped` (閸ユ稖鍐?, `Wheeled` (鏉烆喖绱?閵?

閸︺劋鍞惍浣疯厬閸欘垯浜掗柅姘崇箖 `robot_models.base_robot` 閸掓稑缂撴稉宥呮倱鐎圭偘绶ラ敍?

```python
from robot_models.base_robot import create_robot

# 閸掓稑缂撻崶娑滃喕閺堝搫娅掓禍?
robot = create_robot("quadruped")
```

### 4.2 娣囶喗鏁奸悧鈺冩倞閸欏倹鏆?(Parametric Tuning)
鐟曚椒鎱ㄩ弨瑙勬簚閸ｃ劋姹夐惃鍕⒖閻炲棗鐫橀幀褝绱欐俊鍌濆窛闁插繈鈧胶鏁搁張鍝勫繁鎼达讣绱氶敍灞藉讲娴犮儳娲块幒銉х椽鏉堟垿鍘ょ純顔芥瀮娴犺埖鍨ㄩ崷銊ゅ敩閻椒鑵戦崝銊︹偓浣界殶閺佹番鈧?

**閺傝纭?A: 缂傛牞绶?JSON 闁板秶鐤嗛弬鍥︽**
娴ｅ秳绨?`robot_weights/biped/config.json` (鏉╂劘顢戞稉鈧▎锛勩仛娓氬鍓奸張顒€鎮楁导姘冲殰閸斻劎鏁撻幋?閵?

**閺傝纭?B: 娴狅絿鐖滄稉顓炲З閹浇鐨熼弫?*
閸?RL 閻滎垰顣ㄩ幋?Sim2Real 闁板秶鐤嗘稉顓ㄧ窗
```python
from python_api.godot_robot_env.gym_env import RandomizerConfig

# 鐠嬪啯鏆ｉ崝銊ュ鐎涳箑寮弫?
config = RandomizerConfig(
    mass_range=(1.0, 1.2),       # 鐠愩劑鍣洪梾蹇旀簚閸栨牞瀵栭崶?
    friction_range=(0.8, 1.0),   # 閸︿即娼伴幗鈺傛憹缁粯鏆?
    motor_strength_range=(0.9, 1.0) # 閻㈠灚婧€閸嬨儱鎮嶆惔?
)
```

---

## 閴?鐢瓕顫嗛梻顕€顣?

**Q: 鏉╃偞甯?Godot 婢惰精瑙?(Connection Refused)?**
*   濡偓閺?Godot 閺勵垰鎯佸鑼仯閸?閹绢厽鏂?楠炶埖顒滈崷銊ㄧ箥鐞涘被鈧?
*   姒涙顓荤粩顖氬經娑?`9090`閿涘矁顕涵顔荤箽闂冭尙浼€婢ф瑦婀幏锔藉焻閵?

**Q: 閺堝搫娅掓禍杞扮閸斻劋绗夐崝?**
*   濡偓閺?`walk_1m_demo.py` 鏉堟挸鍤敍灞藉讲閼宠姤妲搁崝鐔哄芳闁插秹鍣哄В鏃囩箖娴ｅ骸顕遍懛瀛樻￥濞夋洟鈹嶉崝銊ｂ偓?
*   閸?Godot 娑擃叏绱濆Λ鈧弻銉︽Ц閸氾箑娲滄稉?`motor_strength` 鐠佸墽鐤嗘潻鍥︾秵閵?

**Q: 婵″倷缍嶉崝鐘烩偓鐔活唲缂?**
*   閸?RL 鐠侇厾绮屾稉顓ㄧ礉閸欘垯浜掓担璺ㄦ暏 `CloudSimInterface` (鐠囷箒顫?`python_controller/cloud_sim.py`) 閺夈儱鑻熺悰灞芥儙閸斻劌顦挎稉顏呮￥婢?(Headless) Godot 鐎圭偘绶ラ妴?

---

## 棣冩懙 5. 鏉╂粎鈻奸崣顖濐潒閸?GUI (Remote Dashboard)

婵″倹鐏夐幃銊ョ瑖閺堟稑婀?Python 缁嬪绨稉顓犳纯閹恒儳婀呴崚?Godot 閻ㄥ嫭瑕嗛弻鎾舵暰闂堫澁绱濋崣顖欎簰閸氼垳鏁ょ憴鍡涱暥濞翠礁濮涢懗濮愨偓?

### 5.1 闁板秶鐤?Godot
1.  閸︺劌婧€閺咁垯鑵戦幍鎯у煂 `Camera3D` 閼哄倻鍋ｉ妴?
2.  闂勫嫬濮為懘姘拱 `scripts/camera_streamer.gd`閵?
3.  娣囨繂鐡ㄩ崷鐑樻珯閵?

### 5.2 閸氼垰濮╂禒顏囥€冮惄?
鏉╂劘顢戞禒銉ょ瑓閸涙垝鎶ら崥顖氬З Python 閸欘垵顫嬮崠鏍櫕闂堫澁绱?
```bash
python examples/dashboard_demo.py
```
*   鐠囥儳鈻兼惔蹇庣窗閼奉亜濮╃亸婵婄槸鏉╃偞甯?Godot 閻ㄥ嫮顏崣?`9998`閵?
*   鐎瑰啩濞囬悽?TCP 娴肩姾绶?JPEG 閸樺缂夊ù渚婄礉瀵ゆ儼绻滈弸浣风秵閵?

---

## 棣冃?6. 濡€虫健閸栨牗婧€閸ｃ劋姹夐弸鍕紦 (Modular Builder)

閹劌褰叉禒銉ュ剼缂佸嫯顥婃稊鎰扮彯娑撯偓閺嶅嚖绱濇担璺ㄦ暏閻喎鐤勯惃鍕暩閺堟椽娴傛禒璁圭礄婵?Unitree/Tesla 鐟欏嫭鐗搁敍澶嬫降閺嬪嫬缂撻懛顏勭暰娑斿婧€閸ｃ劋姹夐妴?

### 6.1 闁瀚ㄩ梿鏈垫娑撳海鏁撻幋鎰板帳缂?
娴ｈ法鏁ら幋鎴滄粦閹绘劒绶甸惃鍕€楦垮壖閺堫剨绱濋懛顏勫З鐠侊紕鐣?BOM 閹存劖婀伴崪灞锯偓濠氬櫢闁插骏绱?

```bash
python examples/custom_parts_demo.py
```
*   濮濄倕鎳℃禒銈勭窗娴?`python_api/parts_library.json` 鐠囪褰囬梿鏈垫閺佺増宓侀妴?
*   閻㈢喐鍨?`custom_robot_config.json` 闁板秶鐤嗛弬鍥︽閵?

### 6.2 閸旂姾娴囬懛顏勭暰娑斿婧€閸ｃ劋姹?
閸︺劍鍋嶉惃鍕敩閻椒鑵戦敍灞煎▏閻劏顕氶柊宥囩枂閺傚洣娆㈤崚婵嗩潗閸栨牔璞㈤惇鐕傜窗

```python
from python_api.godot_robot_env.gym_env import GodotRobotEnv

# 閸旂姾娴囬崚姘灠閻㈢喐鍨氶惃鍕帳缂?
env = GodotRobotEnv(
    robot_config_path="custom_robot_config.json"
)
```

閺囨潙顦跨拠锔剧矎娣団剝浼呴敍宀冾嚞閸欏倿妲勬稉鎾绘，閹靛鍞介敍姝擬ODULAR_ROBOT_BUILDER.md](../hardware/MODULAR_ROBOT_BUILDER.md)閵?
