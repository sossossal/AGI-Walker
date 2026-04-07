# 棣冃?濡€虫健閸栨牗婧€閸ｃ劋姹夐弸鍕紦閹稿洤宕?(Modular Robot Builder)

AGI-Walker 閻滄澘鍑￠弨顖涘瘮閸╄桨绨惇鐔风杽闂嗗爼鍎存禒鑸垫殶閹诡噯绱橠ata-Driven Parts閿涘娈戦張鍝勬珤娴滅儤鐎鐑樼ウ缁嬪鈧倹鍋嶉崣顖欎簰閸嶅繒绮嶇憗鍛暩閼存垳绔撮弽鍑ょ礉闁瀚ㄩ悽鍨簚閵嗕胶鏁稿Ч鐘叉嫲娴肩姵鍔呴崳銊︽降"閹疯壈顥?閹劎娈戦張鍝勬珤娴滄亽鈧?

## 1. 闂嗘湹娆㈡惔?(Parts Library)

閺嶇绺鹃弫鐗堝祦鎼存挷缍呮禍?`python_api/parts_library.json`閵嗗倻娲伴崜宥嗘暪瑜版洑绨℃禒銉ょ瑓娑撶粯绁︾涵顑挎鐟欏嫭鐗搁敍?

### 棣冾瀶 閹笛嗩攽閸?(Actuators)
*   **Unitree Go-M8010 Style**: 妤傛ê濮╅幀浣告磽鐡掑啿鍙ч懞鍌滄暩閺?(23.7Nm, 30rad/s)閵?
*   **Tesla Optimus Style Hip**: 闁插秴鐎风悰灞炬Е姒昏儻鐤嗛崗瀹犲Ν (200Nm, 8rad/s)閵?
*   **Tesla Optimus Style Knee**: 鐡掑懘鍣搁崹瀣暀閸忓疇濡?(300Nm)閵?
*   **SG90 Servo**: 瀵邦喖鐎烽懜鍨簚閵?

### 棣冩啑閿?娴肩姵鍔呴崳?(Sensors)
*   **MPU-6050**: 濞戝牐鍨傜痪?IMU閵?
*   **Xsens MTi**: 瀹搞儰绗熺痪?IMU閵?
*   **VLP-16**: 16缁炬寧绺洪崗澶愭祫鏉堜勘鈧?
*   **RealSense D435**: 濞ｅ崬瀹抽惄鍛婃簚閵?

### 棣冩敱 閻㈠灚鐫?(Batteries)
*   **LiPo 4S 5000mAh**: 閼割亝膩閻㈠灚鐫?(0.5kg)閵?
*   **Tesla Module 2kWh**: 閸斻劌濮忛悽鍨潨閸?(12kg)閵?

---

## 2. 婵″倷缍嶉弸鍕紦閼奉亜鐣炬稊澶嬫簚閸ｃ劋姹?

閹存垳婊戦幓鎰返娴滃棔绔存稉?Python API `PartsManager` 閺夈儴绶熼崝鈺傜€鎭掆偓?

### 濮濄儵顎?1: 鐎电厧鍙嗗銉ュ徔
```python
from python_api.parts_manager import PartsManager
from robot_models.base_robot import RobotConfig, LinkConfig, JointConfig

# 閸掓繂顫愰崠鏍吀閻炲棗娅?
pm = PartsManager()
```

### 濮濄儵顎?2: 闁瀚ㄩ梿鏈垫娑撳氦顓哥粻妗濷M
```python
# 闁瀚ㄩ梿鏈垫 ID
motor_id = "go_m8010"
battery_id = "lipo_4s_5000mah"

# 閼奉亜濮╃拋锛勭暬閹鍣搁崪灞锯偓璁崇幆
parts_list = [motor_id] * 12 + [battery_id]
bom = pm.calculate_bom(parts_list)

print(f"BOM Cost: ${bom['total_cost_usd']}")
print(f"Total Mass: {bom['total_weight_kg']} kg")
```

### 濮濄儵顎?3: 閻㈢喐鍨氶張鍝勬珤娴滄椽鍘ょ純?
閸掆晝鏁ら梿鏈垫閻ㄥ嫬寮弫甯礄婵?`max_torque_nm`閿涘娼垫繅顐㈠帠 `JointConfig`閿涘本妫ら棁鈧幍瀣З閺屻儴銆冮妴?

```python
motor = pm.get_part(motor_id)

joint = JointConfig(
    name="hip_joint",
    type="hinge",
    max_torque=motor.specs["max_torque_nm"], # 閼奉亜濮╁鏇犳暏
    max_speed=motor.specs["max_speed_rad_s"]
)
```

### 鐎瑰本鏆ｇ粈杞扮伐
鐠囩柉绻嶇悰灞剧川缁€楦垮壖閺堫剚鐓￠惇瀣暚閺佸瓨绁︾粙瀣剁窗
```bash
python examples/custom_parts_demo.py
```

鏉╂劘顢戦崥搴濈窗閻㈢喐鍨?`custom_robot_config.json`閿涘本鍋嶉崣顖欎簰閸╄桨绨銈嗘瀮娴犺泛濮炴潪鎴掕雹閻喓骞嗘晶鍐︹偓?

---

## 3. 閹碘晛鐫嶉梿鏈垫鎼?

閹劌褰ч棁鈧紓鏍帆 `python_api/parts_library.json` 閸楀啿褰插ǎ璇插閺備即娴傛禒韬测偓?
閺嶇厧绱℃俊鍌欑瑓閿?

```json
"my_new_motor": {
  "name": "Super Motor X",
  "weight_kg": 0.8,
  "max_torque_nm": 50.0,
  "cost_usd": 199
}
```
缁崵绮烘导姘冲殰閸斻劏顕伴崣鏍ㄦ煀濞ｈ濮為惃鍕祩娴犺翰鈧?
