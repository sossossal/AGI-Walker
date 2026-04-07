# OpenNeuro 闁矮淇婂鍡樼仸闂嗗棙鍨氶幐鍥у础

## 濮掑倽鍫?

AGI-Walker 閻滄澘鍑￠梿鍡樺灇 **OpenNeuro** 闁矮淇婂鍡樼仸,閺€顖涘瘮:
*   閴?**Zenoh** 鏉炲鍣虹痪?Pub/Sub 闁矮淇?
*   閴?**ROS 2** 閻㈢喐鈧胶閮寸紒鐔兼肠閹?
*   閴?**TCP 閸氭垵鎮楅崗鐓庮啇** (娣囨繃瀵旈悳鐗堟箒 Godot 閹恒儱褰?

---

## 鐎瑰顥婃笟婵婄

### 1. Zenoh Python SDK

```bash
pip install eclipse-zenoh
```

### 2. ROS 2 (閸欘垶鈧?閻劋绨?ROS 2 闂嗗棙鍨?

```bash
# Ubuntu 22.04/24.04
sudo apt update
sudo apt install ros-jazzy-desktop
sudo apt install ros-jazzy-rclpy ros-jazzy-sensor-msgs

# 闁板秶鐤嗛悳顖氼暔
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 韫囶偊鈧喎绱戞慨?

### 濡€崇础 1: 缁?Zenoh 闁矮淇?

```python
from python_api.zenoh_interface import ZenohInterface

# 閸掓稑缂撻幒銉ュ經
zenoh = ZenohInterface()

# 閸欐垵绔烽崨鎴掓姢
zenoh.declare_publisher("rt/robot/cmd")
zenoh.publish("rt/robot/cmd", {"joint_0": 1.5})

# 鐠併垽妲勯悩鑸碘偓?
def on_state(data):
    print(f"State: {data}")

zenoh.declare_subscriber("rt/robot/state", on_state)
```

### 濡€崇础 2: TCP-Zenoh 濡椼儲甯?(娣囨繃瀵?Godot 閸忕厧顔?

```bash
# 缂佸牏顏?1: 閸氼垰濮╁銉﹀复閸?
python python_api/tcp_zenoh_bridge.py

# 缂佸牏顏?2: 閸氼垰濮?Godot (閻滅増婀佸ù浣衡柤娑撳秴褰?
# Godot 娴兼俺绻涢幒銉ュ煂 TCP 缁旑垰褰?9090

# 缂佸牏顏?3: 闁俺绻?Zenoh 閸欐垿鈧礁鎳℃禒?
python -c "
from python_api.zenoh_interface import ZenohInterface
z = ZenohInterface()
z.publish('rt/python/cmd', {'type': 'reset'})
"
```

### 濡€崇础 3: ROS 2 閼哄倻鍋?

```bash
# 缂佸牏顏?1: 閸氼垰濮?ROS 2 閼哄倻鍋?
python python_api/ros2_robot_node.py

# 缂佸牏顏?2: 閺屻儳婀?Topic
ros2 topic list
# 鏉堟挸鍤?
#   /robot/joint_states
#   /robot/joint_commands

# 缂佸牏顏?3: 閸欐垿鈧礁鎳℃禒?
ros2 topic pub /robot/joint_commands std_msgs/msg/Float64MultiArray \
  "data: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]"

# 缂佸牏顏?4: 閺屻儳婀呴悩鑸碘偓?
ros2 topic echo /robot/joint_states
```

---

## 閺嬭埖鐎拠瀛樻

### 闁矮淇婇幏鎾村ⅳ

```
閳瑰备鏀㈤埞鈧埞鈧埞鈧埞鈧埞鈧埞鈧埞鈧埞鈧埞鈧埞鈧埞鈧埞鈧埞鈧埞鈧埞鈧埞鈧埞?
閳? Python 閹貉冨煑缁?  閳?
閳? (RL / 鐟欏嫬鍨?     閳?
閳规柡鏀㈤埞鈧埞鈧埞鈧埞鈧埞鈧埞鈧埞鈧埞顑芥敘閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳?
         閳?
    Zenoh Pub/Sub
         閳?
    閳瑰备鏀㈤埞鈧埞鈧埞鈧埞绮规敘閳光偓閳光偓閳光偓閳?
    閳?        閳?
閳瑰备鏀㈤埞鈧埞鈧埢灏栨敘閳光偓閳光偓閳?閳瑰备鏀㈤埞鈧埢灏栨敘閳光偓閳光偓閳光偓閳光偓閳光偓閳?
閳?Godot 閳?閳?ROS 2   閳?
閳?(娴犺法婀? 閳?閳?(瀹搞儱鍙?  閳?
閳规柡鏀㈤埞鈧埞鈧埞鈧埞鈧埞鈧埞鈧埞?閳规柡鏀㈤埞鈧埞鈧埞鈧埞鈧埞鈧埞鈧埞鈧埞鈧埞?
```

### 閺佺増宓佸ù?

**閸涙垝鎶ゅù?* (Python 閳?Godot):
```
Python RL 缁涙牜鏆?
    閳?
zenoh.publish("rt/python/cmd", {...})
    閳?
TCP-Zenoh 濡椼儲甯撮崳?
    閳?
TCP Socket 閳?Godot
```

**閻樿埖鈧焦绁?* (Godot 閳?Python):
```
Godot 娴肩姵鍔呴崳?
    閳?
TCP Socket 閳?濡椼儲甯撮崳?
    閳?
zenoh.publish("rt/godot/state", {...})
    閳?
Python 鐠併垽妲勯懓?
```

---

## 鐢瓕顫嗛梻顕€顣?

### Q: Zenoh 鏉╃偞甯存径杈Е?
**A**: 濡偓閺屻儵妲婚悘顐㈩暰鐠佸墽鐤?绾喕绻氱粩顖氬經 7447 (Zenoh 姒涙顓? 閺堫亣顫﹂崡鐘垫暏閵?

### Q: ROS 2 閼哄倻鍋ｉ弮鐘崇《閸氼垰濮?
**A**: 绾喛顓诲鎻掔暔鐟?ROS 2 楠?source 閻滎垰顣?
```bash
source /opt/ros/jazzy/setup.bash
```

### Q: Godot 鏉╃偞甯寸搾鍛?
**A**: 绾喕绻?TCP-Zenoh 濡椼儲甯撮崳銊ュ嚒閸氼垰濮?
```bash
python python_api/tcp_zenoh_bridge.py
```

### Q: 婵″倷缍嶉弻銉ф箙 Zenoh 濞翠線鍣?
**A**: 娴ｈ法鏁?Zenoh 閼奉亜鐢惃鍕磧閹貉冧紣閸?
```bash
# 鐎瑰顥?Zenoh CLI
cargo install zenoh --features=zenoh/unstable

# 閻╂垶甯堕幍鈧張澶嬬Х閹?
zenoh scout
```

---

## 閹嗗厴閹稿洦鐖?

| 閹稿洦鐖?| TCP (閺? | Zenoh (閺? |
|------|---------|-----------|
| 瀵ゆ儼绻?| ~5-10ms | ~1-2ms |
| 閸氱偛鎮欓柌?| ~10MB/s | ~100MB/s |
| CPU 閸楃姷鏁?| 娑?| 娴?|
| 閸愬懎鐡ㄩ崡鐘垫暏 | 娑?| 娴?|

---

## 娑撳绔村?

1.  **妤犲矁鐦夊鎯扮箿**: 鏉╂劘顢?`examples/zenoh_ros2_demo.py`
2.  **RViz 閸欘垵顫嬮崠?*: 閸氼垰濮?ROS 2 閼哄倻鍋ｉ崥?娴ｈ法鏁?RViz 閺屻儳婀呴張鍝勬珤娴滆櫣濮搁幀?
3.  **绾兛娆㈤柈銊ц**: 閸欏倽鈧?OpenNeuro 閺傚洦銆傞柈銊ц閸掓壆婀＄€圭偟鈥栨禒?

---

**閺囧瓨鏌婇弮銉︽埂**: 2026-01-21  
**閻楀牊婀?*: v4.1.0-alpha
