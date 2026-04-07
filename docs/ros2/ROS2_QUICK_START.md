# AGI-Walker ROS 2 闂嗗棙鍨氳箛顐︹偓鐔峰弳闂傘劍瀵氶崡?

閺堫剚瀵氶崡妤€搴滈崝鈺傚亶韫囶偊鈧喎绱戞慨瀣╁▏閻⑺婫I-Walker閻ㄥ嚧OS 2闂嗗棙鍨氶崝鐔诲厴閵?

---

## 棣冩惖 閸撳秵褰侀弶鈥叉

### 缁崵绮虹憰浣圭湴
- **閹垮秳缍旂化鑽ょ埠**: Ubuntu 22.04 LTS閿涘牊甯归懡鎰剁礆
- **ROS 2**: Humble Hawksbill
- **Python**: 3.10+
- **Godot**: 4.x閿涘牆褰查柅澶涚礉閻劋绨?D娴犺法婀￠敍?

### 鐎瑰顥奟OS 2

```bash
# 鐠佸墽鐤唋ocale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 濞ｈ濮濺OS 2 apt濠?
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 鐎瑰顥奟OS 2
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions
```

---

## 棣冩畬 韫囶偊鈧喎绱戞慨?

### 濮濄儵顎?: 閸忓娈曟禒鎾崇氨

```bash
git clone https://github.com/sossossal/AGI-Walker.git
cd AGI-Walker
```

### 濮濄儵顎?: 鐎瑰顥夾GI-Walker娓氭繆绂?

```bash
pip install -r requirements.txt
pip install matplotlib pillow  # GUI娓氭繆绂?
```

### 濮濄儵顎?: 缂傛牞鐦OS 2 Packages

```bash
cd hardware/ros2_ws

# Source ROS 2閻滎垰顣?
source /opt/ros/humble/setup.bash

# 缂傛牞鐦?
colcon build

# Source瀹搞儰缍旂粚娲？
source install/setup.bash
```

### 濮濄儵顎?: 閸氼垰濮╁銉﹀复閼哄倻鍋?

```bash
# 閺傜懓绱?: 娴ｈ法鏁aunch閺傚洣娆㈤敍鍫熷腹閼芥劧绱?
ros2 launch agi_walker_ros2 agi_walker.launch.py

# 閺傜懓绱?: 閻╁瓨甯存潻鎰攽閼哄倻鍋?
ros2 run agi_walker_ros2 bridge_node
```

---

## 棣冃?濞村鐦崝鐔诲厴

### 1. 閺屻儳婀匱opics

```bash
# 閸掓鍤幍鈧張濉紀pics
ros2 topic list

# 閺屻儳婀呴崗瀹犲Ν閻樿埖鈧?
ros2 topic echo /joint_states

# 閺屻儳婀呴張鍝勬珤娴滆櫣濮搁幀?
ros2 topic echo /robot_state
```

### 2. 鐠嬪啰鏁ervices

```bash
# 閸氼垰濮╂禒璺ㄦ埂
ros2 service call /start_simulation std_srvs/srv/Trigger

# 閸嬫粍顒涙禒璺ㄦ埂
ros2 service call /stop_simulation std_srvs/srv/Trigger
```

### 3. 閸欐垿鈧礁鎳℃禒?

```bash
# 閸欐垿鈧線鈧喎瀹抽崨鎴掓姢
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}" \
  --once

# 閹存牞鈧懏瀵旂紒顓炲絺闁?
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.3}}"
```

### 4. 閺屻儳婀呴崪灞兼叏閺€鐟板棘閺?

```bash
# 閸掓鍤幍鈧張澶婂棘閺?
ros2 param list /agi_walker_bridge

# 閺屻儳婀呴崣鍌涙殶閸?
ros2 param get /agi_walker_bridge motor_power_multiplier

# 娣囶喗鏁奸崣鍌涙殶
ros2 param set /agi_walker_bridge motor_power_multiplier 1.5

# 娴犲孩鏋冩禒璺哄鏉炶棄寮弫?
ros2 param load /agi_walker_bridge src/agi_walker_ros2/config/params.yaml
```

---

## 棣冨箖 鐎瑰本鏆ｆ担璺ㄦ暏濞翠胶鈻?

### 閸︾儤娅?: 缁绢垱膩閹风噦绱欓弮鐕漮dot閿?

```bash
# 缂佸牏顏?: 閸氼垰濮╁銉﹀复閼哄倻鍋?
cd AGI-Walker/hardware/ros2_ws
source install/setup.bash
ros2 launch agi_walker_ros2 agi_walker.launch.py

# 缂佸牏顏?: 閸欐垿鈧礁鎳℃禒?
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```

濡椼儲甯撮懞鍌滃仯娴兼氨鐡戝鍖瀘dot鏉╃偞甯撮敍灞肩稻娴犲秴褰查幒銉︽暪ROS閸涙垝鎶ら妴?

### 閸︾儤娅?: 娑撳定odot娴犺法婀￠梿鍡樺灇

```bash
# 缂佸牏顏?: 閸氼垰濮〨odot娴犺法婀￠敍鍫濇躬AGI-Walker閻╊喖缍嶉敍?
cd AGI-Walker/godot_project
godot --headless  # 閹存牞鈧懐鏁UI閸氼垰濮?

# 缂佸牏顏?: 閸氼垰濮㏑OS 2濡椼儲甯?
cd AGI-Walker/hardware/ros2_ws
source install/setup.bash
ros2 launch agi_walker_ros2 agi_walker.launch.py

# 缂佸牏顏?: 閸氼垰濮╂禒璺ㄦ埂
ros2 service call /start_simulation std_srvs/srv/Trigger

# 缂佸牏顏?: 閺屻儳婀呯€圭偞妞傞弫鐗堝祦
ros2 topic echo /joint_states
```

### 閸︾儤娅?: 娴ｈ法鏁ython閼存碍婀伴幒褍鍩?

閸掓稑缂撻弬鍥︽ `test_control.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
import time

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.start_client = self.create_client(Trigger, '/start_simulation')
        
    def start_sim(self):
        req = Trigger.Request()
        future = self.start_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
        
    def move(self, linear_x, angular_z, duration=1.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_pub.publish(msg)
            time.sleep(0.1)

def main():
    rclpy.init()
    controller = RobotController()
    
    # 閸氼垰濮╂禒璺ㄦ埂
    controller.get_logger().info('Starting simulation...')
    result = controller.start_sim()
    controller.get_logger().info(f'Result: {result.message}')
    
    # 閸撳秷绻?
    controller.get_logger().info('Moving forward...')
    controller.move(0.5, 0.0, 2.0)
    
    # 鏉烆剙鎮?
    controller.get_logger().info('Turning...')
    controller.move(0.0, 0.5, 2.0)
    
    controller.get_logger().info('Done!')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

鏉╂劘顢?
```bash
chmod +x test_control.py
python3 test_control.py
```

---

## 棣冩暋 閸欏倹鏆熼柊宥囩枂

缂傛牞绶?`hardware/hardware/hardware/ros2_ws/src/agi_walker_ros2/config/params.yaml`:

```yaml
/agi_walker_bridge:
  ros__parameters:
    # 娣囶喗鏁糋odot鏉╃偞甯?
    godot_host: "192.168.1.100"  # 鏉╂粎鈻糋odot閺堝秴濮熼崳?
    godot_port: 9999
    
    # 鐠嬪啯鏆ｉ崣鎴濈妫版垹宸?
    joint_state_rate: 100.0  # 閹绘劙鐝崚?00Hz
    
    # 鐠嬪啯鏆ｉ幒褍鍩楅崣鍌涙殶
    motor_power_multiplier: 1.5
    joint_stiffness: 2.0
```

闁插秵鏌婇崥顖氬З閼哄倻鍋ｆ禒銉ョ安閻劍娲块弨骞库偓?

---

## 棣冩偘 閺佸懘娈伴幒鎺撶叀

### 闂傤噣顣?: 閺冪姵纭堕幍鎯у煂package

**闁挎瑨顕?*: `Package 'agi_walker_ros2' not found`

**鐟欙絽鍠?*:
```bash
cd hardware/ros2_ws
colcon build
source install/setup.bash
```

### 闂傤噣顣?: 鏉╃偞甯碐odot婢惰精瑙?

**闁挎瑨顕?*: `Failed to connect to Godot`

**濡偓閺?*:
1. Godot閺勵垰鎯佹潻鎰攽閿?
2. TCP閺堝秴濮熼崳銊︽Ц閸氾箑鎯庨崝顭掔礄缁旑垰褰?999閿涘绱?
3. 闂冭尙浼€婢ф瑦妲搁崥锕傛▎濮濄垼绻涢幒銉吹

**濞村鐦潻鐐村复**:
```bash
telnet 127.0.0.1 9999
```

### 闂傤噣顣?: 閼奉亜鐣炬稊澶嬬Х閹垱婀幍鎯у煂

**闁挎瑨顕?*: `ModuleNotFoundError: No module named 'agi_walker_msgs'`

**鐟欙絽鍠?*:
```bash
# 绾喕绻氱紓鏍槯娴滃棙绉烽幁鐥痑ckage
cd hardware/ros2_ws
colcon build --packages-select agi_walker_msgs
source install/setup.bash
```

### 闂傤噣顣?: Python鐠侯垰绶為梻顕€顣?

**闁挎瑨顕?*: `Cannot import godot_client`

**鐟欙絽鍠?*:
```bash
# 绾喕绻欰GI-Walker閸︹墥ython鐠侯垰绶炴稉?
export PYTHONPATH=$PYTHONPATH:/path/to/AGI-Walker
```

---

## 棣冩惓 閸欘垵顫嬮崠?

### 娴ｈ法鏁qt閺屻儳婀呴弫鐗堝祦

```bash
# 鐎瑰顥妑qt瀹搞儱鍙?
sudo apt install ros-humble-rqt ros-humble-rqt-common-plugins

# 閸氼垰濮﹔qt
rqt
```

閸︹暜qt娑?
- Plugins 閳?Topics 閳?Topic Monitor - 閺屻儳婀呴幍鈧張濉紀pics
- Plugins 閳?Visualization 閳?Plot - 缂佹ê鍩楅弫鐗堝祦閺囪尙鍤?
- Plugins 閳?Services 閳?Service Caller - 鐠嬪啰鏁ら張宥呭

### 娴ｈ法鏁lotjuggler

```bash
# 鐎瑰顥?
sudo apt install ros-humble-plotjuggler-ros

# 閸氼垰濮?
ros2 run plotjuggler plotjuggler
```

---

## 棣冨箚 娑撳绔村?

1. **鐏忔繆鐦疪Viz閸欘垵顫嬮崠?* (闂団偓鐟曚箑F缁崵绮洪敍瀛環ase 2)
2. **闂嗗棙鍨歁oveIt鏉╂劕濮╃憴鍕灊** (閺堫亝娼甸崝鐔诲厴)
3. **閸掓稑缂撻懛顏勭暰娑斿甯堕崚璺烘珤**
4. **鏉╃偞甯撮惇鐔风杽绾兛娆?*

---

## 棣冩憥 閺囨潙顦跨挧鍕爱

- [ROS 2鐎规ɑ鏌熼弬鍥ㄣ€俔(https://docs.ros.org/en/humble/)
- [AGI-Walker妞ゅ湱娲版稉濠氥€塢(https://github.com/sossossal/AGI-Walker)
- [ROS 2闂嗗棙鍨氱拋鎹愵吀閺傚洦銆俔(../docs/ROS2_INTEGRATION_DESIGN.md)

---

**闁洤鍩岄梻顕€顣介敍?* 鐠囧嘲婀狦itHub娑撳﹥褰佹禍顥痵sue閿?
