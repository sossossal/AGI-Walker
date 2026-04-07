# AGI-Walker 鐠侊紕鐣婚張楦款潒鐟?(CV) 閺佺増宓侀悽鐔稿灇鐎圭偟骞囬弬瑙勵攳

**閺冦儲婀?*: 2026-01-18  
**閻楀牊婀?*: 1.0  
**閻樿埖鈧?*: 鐎圭偞鏌﹂弬瑙勵攳

---

## 棣冩惖 閹笛嗩攽閹芥顩?

### 瑜版挸澧犻悩鑸碘偓?
- 閴?閺佹澘鈧吋鏆熼幑顔炬晸閹? 92% (鐎瑰本鍨?
- 閳跨媴绗?鐟欏棜顫庨弫鐗堝祦閻㈢喐鍨? 60% (闂団偓鐟曚礁鐤勯悳?

### 閻╊喗鐖?
鐎圭偟骞囩€瑰本鏆ｉ惃鍑淰鐠侇厾绮岄弫鐗堝祦閻㈢喐鍨氶懗钘夊閿涘苯瀵橀幏顒婄窗
- RGB閸ユ儳鍎?
- 濞ｅ崬瀹抽崶?
- 閸掑棗澹婇幒鈺冪垳
- 閸忔娊鏁悙瑙勭垼濞?
- 鏉堝湱鏅鍡樼垼濞?

### 妫板嫭婀＄€瑰本鍨氭惔?
60% 閳?**95%**

---

## 棣冨箚 閺傝顢嶅鍌濐潔

### 娑撳顫掔€圭偟骞囬弬瑙勵攳

| 閺傝顢?| 婢跺秵娼呮惔?| 閺佸牊鐏?| 閺冨爼妫?| 閹恒劏宕樻惔?|
|------|-------|------|------|--------|
| 閺傝顢?: Godot闂嗗棙鍨?| 妤?| 娴兼顫?| 5-7婢?| 鐚告劏鐡欑尭鎰ㄧ摍鐚?|
| 閺傝顢?: PyBullet闂嗗棙鍨?| 娑?| 閼诡垰銈?| 3-4婢?| 鐚告劏鐡欑尭鎰ㄧ摍 |
| 閺傝顢?: 缁犫偓閸栨牗瑕嗛弻?| 娴?| 閸╄櫣顢?| 1-2婢?| 鐚告劏鐡欑尭?|

**閹恒劏宕?*: 閺傝顢? (Godot闂嗗棙鍨? - 閺堚偓鐎瑰本鏆ｉ惃鍕掗崘铏煙濡?

---

## 閺傝顢?: Godot闂嗗棙鍨?(閹恒劏宕?

### 濮掑倽鍫?
閸掆晝鏁ら悳鐗堟箒閻ㄥ嚕odot妞ゅ湱娲伴敍宀勨偓姘崇箖Python-Godot闁矮淇婇悽鐔稿灇鐟欏棜顫庨弫鐗堝祦

### 閺嬭埖鐎?

```
Python閹貉冨煑閸?                Godot濞撳弶鐓嬪鏇熸惛
    閳?                          閳?
    閳规壕鏀? 閸欐垿鈧焦婧€閸ｃ劋姹夐悩鑸碘偓?閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓> 閳?
    閳?                          閳规壕鏀? 閺囧瓨鏌婇張鍝勬珤娴滃搫协閹?
    閳?                          閳规壕鏀? 濞撳弶鐓嬮崷鐑樻珯
    閳?  <閳光偓閳光偓閳光偓閳光偓 鏉╂柨娲栭崶鎯у剼閺佺増宓?<閳光偓閳光偓閳光偓閳光偓 閳规壕鏀? 閹规洝骞忛惄鍛婃簚鐟欏棗娴?
    閳?                          閳规壕鏀? 閻㈢喐鍨氬ǎ鍗炲閸?
    閳?                          閳规柡鏀? 鐠囶厺绠熼崚鍡楀
    閳?
    閳规柡鏀? 娣囨繂鐡ㄩ崶鎯у剼 + 閺嶅洦鏁?
```

### 鐎圭偟骞囧銉╊€?

#### 濮濄儵顎?: Godot缁旑垰鐤勯悳?(2-3婢?

**1.1 閸掓稑缂撻惄鍛婃簚缁崵绮?*

```gdscript
# godot_project/scripts/VisionDataGenerator.gd
extends Node

var cameras = []
var capture_resolution = Vector2(640, 480)

func _ready():
    setup_cameras()

func setup_cameras():
    # 缁楊兛绗佹禍铏剐為惄鍛婃簚
    var third_person_cam = create_camera(
        Vector3(2, 1.5, 2),  # 娴ｅ秶鐤?
        Vector3(-30, -45, 0)  # 閺冨娴?
    )
    cameras.append(third_person_cam)
    
    # 缁楊兛绔存禍铏剐為惄鍛婃簚 (閺堝搫娅掓禍楦款潒鐟?
    var first_person_cam = create_camera(
        Vector3(0, 0.3, 0.2),  # 閻╃顕張鍝勬珤娴?
        Vector3(0, 0, 0)
    )
    cameras.append(first_person_cam)
    
    # 娣囶垵顫嬮惄鍛婃簚
    var top_down_cam = create_camera(
        Vector3(0, 5, 0),
        Vector3(-90, 0, 0)
    )
    cameras.append(top_down_cam)

func create_camera(pos: Vector3, rot: Vector3) -> Camera:
    var camera = Camera.new()
    camera.transform.origin = pos
    camera.rotation_degrees = rot
    return camera

func capture_all_views() -> Dictionary:
    var images = {}
    
    for i in range(cameras.size()):
        var cam = cameras[i]
        
        # RGB閸ユ儳鍎?
        images["rgb_" + str(i)] = capture_rgb(cam)
        
        # 濞ｅ崬瀹抽崶?
        images["depth_" + str(i)] = capture_depth(cam)
        
        # 閸掑棗澹婇崶?
        images["segmentation_" + str(i)] = capture_segmentation(cam)
    
    return images

func capture_rgb(camera: Camera) -> Image:
    var viewport = get_viewport()
    viewport.set_clear_mode(Viewport.CLEAR_MODE_ONLY_NEXT_FRAME)
    
    # 濞撳弶鐓嬫稉鈧敮?
    yield(get_tree(), "idle_frame")
    
    # 閹规洝骞忛崶鎯у剼
    var image = viewport.get_texture().get_data()
    image.flip_y()
    
    return image

func capture_depth(camera: Camera) -> Image:
    # 閸掑洦宕查崚鐗堢箒鎼达附瑕嗛弻鎾茨佸?
    var shader = preload("res://shaders/depth_shader.shader")
    # ... 鐎圭偟骞囧ǎ鍗炲濞撳弶鐓?
    pass

func capture_segmentation(camera: Camera) -> Image:
    # 鐠囶厺绠熼崚鍡楀濞撳弶鐓?
    # 濮ｅ繋閲滅€电钖勭猾璇插焼閻劋绗夐崥宀勵杹閼?
    pass
```

**1.2 閸掓稑缂撳ǎ鍗炲閻偓閼规彃娅?*

```glsl
// godot_project/shaders/depth_shader.shader
shader_type spatial;

varying float depth;

void vertex() {
    vec4 world_pos = WORLD_MATRIX * vec4(VERTEX, 1.0);
    vec4 view_pos = VIEW_MATRIX * world_pos;
    depth = -view_pos.z;
}

void fragment() {
    # 濞ｅ崬瀹宠ぐ鎺嶇閸栨牕鍩?-1
    float normalized_depth = depth / 10.0; # 10m閺堚偓婢堆勭箒鎼?
    ALBEDO = vec3(normalized_depth);
}
```

**1.3 TCP闁矮淇婇張宥呭閸?*

```gdscript
# godot_project/scripts/TCPVisionServer.gd
extends Node

var server = TCP_Server.new()
var connection = null
var port = 9999

func _ready():
    server.listen(port)
    print("Vision server listening on port ", port)

func _process(delta):
    # 閹恒儱褰堟潻鐐村复
    if server.is_connection_available():
        connection = server.take_connection()
        print("Client connected")
    
    # 婢跺嫮鎮婄拠閿嬬湴
    if connection and connection.get_available_bytes() > 0:
        var request = connection.get_utf8_string(connection.get_available_bytes())
        handle_request(request)

func handle_request(request: String):
    var data = JSON.parse(request).result
    
    match data.command:
        "capture":
            var images = $VisionDataGenerator.capture_all_views()
            send_images(images)
        
        "update_robot":
            update_robot_state(data.state)
        
        "set_camera":
            set_camera_params(data.camera_id, data.params)

func send_images(images: Dictionary):
    var response = {
        "status": "success",
        "images": {}
    }
    
    for key in images:
        var image = images[key]
        # 鏉烆剚宕叉稉绡礱se64閹存牔绻氱€涙ê鍩屾稉瀛樻閺傚洣娆?
        response.images[key] = image_to_base64(image)
    
    connection.put_data(JSON.print(response).to_utf8())

func image_to_base64(image: Image) -> String:
    var buffer = image.save_png_to_buffer()
    return Marshalls.raw_to_base64(buffer)
```

#### 濮濄儵顎?: Python缁旑垰鐤勯悳?(2-3婢?

**2.1 Godot闁矮淇婄€广垺鍩涚粩?*

```python
# python_api/godot_vision_client.py
"""
Godot鐟欏棜顫庨弫鐗堝祦闁插洭娉︾€广垺鍩涚粩?
"""

import socket
import json
import base64
import numpy as np
from PIL import Image
import io
from typing import Dict, List, Optional


class GodotVisionClient:
    """Godot鐟欏棜顫庨弫鐗堝祦鐎广垺鍩涚粩?""
    
    def __init__(self, host: str = 'localhost', port: int = 9999):
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
    
    def connect(self):
        """鏉╃偞甯撮崚鐧巓dot閺堝秴濮熼崳?""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            self.connected = True
            print(f"Connected to Godot server at {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """閺傤厼绱戞潻鐐村复"""
        if self.socket:
            self.socket.close()
            self.connected = False
    
    def send_command(self, command: Dict) -> Dict:
        """閸欐垿鈧礁鎳℃禒銈呰嫙閹恒儲鏁归崫宥呯安"""
        if not self.connected:
            raise ConnectionError("Not connected to Godot server")
        
        # 閸欐垿鈧?
        message = json.dumps(command).encode('utf-8')
        self.socket.sendall(message)
        
        # 閹恒儲鏁?
        response_data = b""
        while True:
            chunk = self.socket.recv(4096)
            if not chunk:
                break
            response_data += chunk
            
            # 鐏忔繆鐦憴锝嗙€絁SON閿涘牏鐣濋崠鏍閿?
            try:
                response = json.loads(response_data.decode('utf-8'))
                return response
            except:
                continue
        
        return {}
    
    def update_robot_state(self, position: List[float], 
                          orientation: List[float],
                          joint_angles: List[float]):
        """閺囧瓨鏌婇張鍝勬珤娴滆櫣濮搁幀?""
        command = {
            'command': 'update_robot',
            'state': {
                'position': position,
                'orientation': orientation,
                'joint_angles': joint_angles
            }
        }
        
        return self.send_command(command)
    
    def capture_images(self) -> Dict[str, np.ndarray]:
        """閹规洝骞忛幍鈧張澶庮潒鐟欐帞娈戦崶鎯у剼"""
        command = {'command': 'capture'}
        
        response = self.send_command(command)
        
        if response.get('status') != 'success':
            raise RuntimeError("Image capture failed")
        
        # 鐟欙絿鐖滈崶鎯у剼
        images = {}
        for key, base64_data in response['images'].items():
            image_bytes = base64.b64decode(base64_data)
            image = Image.open(io.BytesIO(image_bytes))
            images[key] = np.array(image)
        
        return images
    
    def set_camera_params(self, camera_id: int, fov: float = 70.0,
                         position: Optional[List[float]] = None):
        """鐠佸墽鐤嗛惄鍛婃簚閸欏倹鏆?""
        command = {
            'command': 'set_camera',
            'camera_id': camera_id,
            'params': {
                'fov': fov,
                'position': position or [0, 0, 0]
            }
        }
        
        return self.send_command(command)
```

**2.2 CV閺佺増宓侀悽鐔稿灇閸?*

```python
# python_api/cv_data_generator.py
"""
鐠侊紕鐣婚張楦款潒鐟欏顔勭紒鍐╂殶閹诡喚鏁撻幋鎰珤
"""

import numpy as np
import cv2
from pathlib import Path
from typing import Dict, List, Tuple
import json
from tqdm import tqdm

from python_api.godot_vision_client import GodotVisionClient
from python_api.data_recorder import DataRecorder


class CVDataGenerator:
    """CV鐠侇厾绮岄弫鐗堝祦閻㈢喐鍨氶崳?""
    
    def __init__(self, output_dir: str = "data/cv_dataset"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Godot鐎广垺鍩涚粩?
        self.godot_client = GodotVisionClient()
        
        # 閺佺増宓佺紒鐔活吀
        self.num_frames = 0
    
    def connect_to_godot(self) -> bool:
        """鏉╃偞甯撮崚鐧巓dot"""
        return self.godot_client.connect()
    
    def generate_episode_images(self, episode_id: int, 
                               trajectory: List[Dict],
                               save_interval: int = 10):
        """
        娑撹桨绔存稉鐚爌isode閻㈢喐鍨氶崶鎯у剼
        
        閸欏倹鏆?
            episode_id: Episode ID
            trajectory: 鏉炪劏鎶楅弫鐗堝祦 (閸栧懎鎯堝В蹇旑劄閻ㄥ嫮濮搁幀?
            save_interval: 娣囨繂鐡ㄩ梻鎾
        """
        episode_dir = self.output_dir / f"episode_{episode_id:06d}"
        episode_dir.mkdir(exist_ok=True)
        
        # 閸掓稑缂撶€涙劗娲拌ぐ?
        (episode_dir / "rgb").mkdir(exist_ok=True)
        (episode_dir / "depth").mkdir(exist_ok=True)
        (episode_dir / "segmentation").mkdir(exist_ok=True)
        
        annotations = []
        
        for step_id, state in enumerate(tqdm(trajectory, desc=f"Episode {episode_id}")):
            if step_id % save_interval != 0:
                continue
            
            # 閺囧瓨鏌奊odot娑擃厾娈戦張鍝勬珤娴滆櫣濮搁幀?
            self.godot_client.update_robot_state(
                position=state.get('position', [0, 0, 0]),
                orientation=state.get('orientation', [0, 0, 0]),
                joint_angles=state.get('joint_angles', [0]*6)
            )
            
            # 閹规洝骞忛崶鎯у剼
            images = self.godot_client.capture_images()
            
            # 娣囨繂鐡ㄩ崶鎯у剼
            for view_id, (key, image) in enumerate(images.items()):
                if 'rgb' in key:
                    filename = episode_dir / "rgb" / f"frame_{step_id:06d}_view_{view_id}.png"
                    cv2.imwrite(str(filename), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
                
                elif 'depth' in key:
                    filename = episode_dir / "depth" / f"frame_{step_id:06d}_view_{view_id}.png"
                    # 濞ｅ崬瀹抽崶鍙ョ箽鐎涙ü璐?6娴?
                    depth_16bit = (image * 65535).astype(np.uint16)
                    cv2.imwrite(str(filename), depth_16bit)
                
                elif 'segmentation' in key:
                    filename = episode_dir / "segmentation" / f"frame_{step_id:06d}_view_{view_id}.png"
                    cv2.imwrite(str(filename), image)
            
            # 閻㈢喐鍨氶弽鍥ㄦ暈
            annotation = self.generate_annotations(state, images)
            annotation['frame_id'] = step_id
            annotations.append(annotation)
            
            self.num_frames += 1
        
        # 娣囨繂鐡ㄩ弽鍥ㄦ暈閺傚洣娆?
        with open(episode_dir / "annotations.json", 'w') as f:
            json.dump(annotations, f, indent=2)
    
    def generate_annotations(self, state: Dict, images: Dict) -> Dict:
        """
        閻㈢喐鍨氶弽鍥ㄦ暈娣団剝浼?
        
        閸栧懏瀚?
        - 閺堝搫娅掓禍鍝勑幀?
        - 閸忔娊鏁悙閫涚秴缂?
        - 鏉堝湱鏅?
        - 鐠囶厺绠熼弽鍥╊劮
        """
        annotation = {
            'robot_state': state,
            'keypoints': self.detect_keypoints(images),
            'bounding_boxes': self.detect_bounding_boxes(images),
            'semantic_labels': self.extract_semantic_labels(images)
        }
        
        return annotation
    
    def detect_keypoints(self, images: Dict) -> List[Dict]:
        """濡偓濞村鍙ч柨顔惧仯閿涘牆鍙ч懞鍌欑秴缂冾噯绱?""
        # 鏉╂瑩鍣锋惔鏃囶嚉娴犲定odot閼惧嘲褰?D閸忓疇濡担宥囩枂
        # 楠炶埖濮囪ぐ鍗炲煂2D閸ユ儳鍎氶獮鎶芥桨
        keypoints = [
            {'name': 'hip', 'position_2d': [320, 240, 1.0]},  # [x, y, visibility]
            {'name': 'knee', 'position_2d': [340, 300, 1.0]},
            # ... 閺囨潙顦块崗瀹犲Ν
        ]
        return keypoints
    
    def detect_bounding_boxes(self, images: Dict) -> List[Dict]:
        """濡偓濞村绔熼悾灞绢攱"""
        # 閺堝搫娅掓禍铏规畱鏉堝湱鏅?
        boxes = [
            {
                'class': 'robot',
                'bbox': [100, 150, 500, 400],  # [x, y, w, h]
                'confidence': 1.0
            }
        ]
        return boxes
    
    def extract_semantic_labels(self, images: Dict) -> Dict:
        """閹绘劕褰囩拠顓濈疅閺嶅洨顒?""
        # 娴犲骸鍨庨崜鎻掓禈娑擃厽褰侀崣?
        labels = {
            'robot': 1,
            'ground': 2,
            'obstacle': 3,
            'background': 0
        }
        return labels
    
    def batch_generate(self, num_episodes: int, 
                      episode_length: int = 100,
                      save_interval: int = 5):
        """
        閹靛綊鍣洪悽鐔稿灇CV閺佺増宓侀梿?
        
        閸欏倹鏆?
            num_episodes: Episode閺佷即鍣?
            episode_length: 濮ｅ繋閲渆pisode閻ㄥ嫰鏆辨惔?
            save_interval: 閸ユ儳鍎氭穱婵嗙摠闂傛挳娈?
        """
        if not self.connect_to_godot():
            print("Failed to connect to Godot. Make sure Godot is running.")
            return
        
        print(f"Generating CV dataset: {num_episodes} episodes")
        
        for ep_id in range(num_episodes):
            # 鏉╂瑩鍣锋惔鏃囶嚉鏉╂劘顢戞禒璺ㄦ埂閼惧嘲褰囨潪銊ㄦ姉
            # 缁犫偓閸栨牜銇氭笟瀣剁窗娴ｈ法鏁ら梾蹇旀簚鏉炪劏鎶?
            trajectory = self.generate_random_trajectory(episode_length)
            
            # 閻㈢喐鍨氶崶鎯у剼
            self.generate_episode_images(ep_id, trajectory, save_interval)
        
        print(f"\nGeneration complete!")
        print(f"Total frames: {self.num_frames}")
        print(f"Output directory: {self.output_dir}")
    
    def generate_random_trajectory(self, length: int) -> List[Dict]:
        """閻㈢喐鍨氶梾蹇旀簚鏉炪劏鎶楅敍鍫㈡暏娴滃孩绁寸拠鏇礆"""
        trajectory = []
        
        for i in range(length):
            state = {
                'position': [i * 0.01, 0, 0],  # 閸撳秷绻?
                'orientation': [0, 0, 0],
                'joint_angles': [0] * 6
            }
            trajectory.append(state)
        
        return trajectory
```

#### 濮濄儵顎?: 閺佺増宓侀梿鍡樼壐瀵?(1婢?

**COCO閺嶇厧绱￠弨顖涘瘮**

```python
# python_api/cv_dataset_converter.py
"""
鏉烆剚宕睠V閺佺増宓侀梿鍡曡礋閺嶅洤鍣弽鐓庣础
"""

import json
from pathlib import Path


class COCOConverter:
    """鏉烆剚宕叉稉绡奜CO閺嶇厧绱?""
    
    def __init__(self, dataset_dir: str):
        self.dataset_dir = Path(dataset_dir)
    
    def convert(self, output_file: str):
        """鏉烆剚宕查弫缈犻嚋閺佺増宓侀梿?""
        coco_data = {
            'images': [],
            'annotations': [],
            'categories': [
                {'id': 1, 'name': 'robot'},
                {'id': 2, 'name': 'ground'},
                {'id': 3, 'name': 'obstacle'}
            ]
        }
        
        annotation_id = 0
        
        # 闁秴宸婚幍鈧張濉璸isodes
        for episode_dir in self.dataset_dir.glob("episode_*"):
            annotations_file = episode_dir / "annotations.json"
            
            if not annotations_file.exists():
                continue
            
            with open(annotations_file, 'r') as f:
                annotations = json.load(f)
            
            for ann in annotations:
                frame_id = ann['frame_id']
                
                # 濞ｈ濮為崶鎯у剼娣団剝浼?
                image_info = {
                    'id': len(coco_data['images']),
                    'file_name': f"{episode_dir.name}/rgb/frame_{frame_id:06d}_view_0.png",
                    'width': 640,
                    'height': 480
                }
                coco_data['images'].append(image_info)
                
                # 濞ｈ濮為弽鍥ㄦ暈
                for bbox in ann['bounding_boxes']:
                    coco_ann = {
                        'id': annotation_id,
                        'image_id': image_info['id'],
                        'category_id': 1,  # robot
                        'bbox': bbox['bbox'],
                        'area': bbox['bbox'][2] * bbox['bbox'][3],
                        'iscrowd': 0
                    }
                    coco_data['annotations'].append(coco_ann)
                    annotation_id += 1
                
                # 濞ｈ濮為崗鎶芥暛閻?
                for kp in ann['keypoints']:
                    coco_kp = {
                        'id': annotation_id,
                        'image_id': image_info['id'],
                        'category_id': 1,
                        'keypoints': kp['position_2d'] * 6,  # COCO閺嶇厧绱?
                        'num_keypoints': 6
                    }
                    coco_data['annotations'].append(coco_kp)
                    annotation_id += 1
        
        # 娣囨繂鐡?
        with open(output_file, 'w') as f:
            json.dump(coco_data, f, indent=2)
        
        print(f"COCO dataset saved to: {output_file}")
```

---

## 閺傝顢?: PyBullet闂嗗棙鍨?(婢跺洭鈧?

### 濮掑倽鍫?
娴ｈ法鏁yBullet閻ㄥ嫭瑕嗛弻鎾冲閼崇晫鏁撻幋鎰禈閸?

### 娴兼ê濞?
- 鐎瑰苯鍙廝ython鐎圭偟骞?
- 閺囨潙顔愰弰鎾绘肠閹?
- 娑撳秹娓剁憰涓無dot鏉╂劘顢?

---

## 閺傝顢?: 缁犫偓閸栨牗瑕嗛弻?(韫囶偊鈧喐鏌熷?

### 濮掑倽鍫?
娴ｈ法鏁atplotlib閹存潤ygame鏉╂稖顢?D濞撳弶鐓?

### 闁倻鏁ら崷鐑樻珯
- 韫囶偊鈧喎甯崹?
- 娑撳秹娓剁憰浣烘埂鐎圭偞鍔?
- 2D娴犺濮?

---

## 棣冩惓 鐎佃鐦幀鑽ょ波

| 閻楄鈧?| Godot | PyBullet | 缁犫偓閸栨牗瑕嗛弻?|
|------|-------|----------|----------|
| 閸ユ儳鍎氱拹銊╁櫤 | 鐚告劏鐡欑尭鎰ㄧ摍鐚?| 鐚告劏鐡欑尭鎰ㄧ摍 | 鐚告劏鐡?|
| 鐎圭偟骞囬梾鎯у | 妤?| 娑?| 娴?|
| 瀵偓閸欐垶妞傞梻?| 5-7婢?| 3-4婢?| 1-2婢?|
| 閹恒劏宕樻惔?| 鐚告劏鐡欑尭鎰ㄧ摍鐚?| 鐚告劏鐡欑尭鎰ㄧ摍 | 鐚告劏鐡欑尭?|

---

## 棣冨箚 閹恒劏宕樼€圭偞鏌︾捄顖滃殠

### Phase 1: 閸╄櫣顢呯€圭偟骞?(3-4婢?
1. Godot TCP閺堝秴濮熼崳?
2. Python鐎广垺鍩涚粩?
3. 閸╄櫣顢呴崶鎯у剼閹规洝骞?

### Phase 2: 婢х偛宸遍崝鐔诲厴 (2-3婢?
4. 濞ｅ崬瀹抽崶鐐閺?
5. 鐠囶厺绠熼崚鍡楀
6. 婢舵氨娴夐張楦款潒鐟?

### Phase 3: 闂嗗棙鍨氭导妯哄 (1-2婢?
7. 閹靛綊鍣洪悽鐔稿灇闂嗗棙鍨?
8. 閺佺増宓侀梿鍡樼壐瀵繗娴嗛幑?
9. 閹嗗厴娴兼ê瀵?

**閹粯妞傞梻?*: 6-9婢?

---

## 閴?鐎圭偞鏌﹂崥搴㈡櫏閺?

**閺佺増宓佺猾璇茬€?*:
- RGB閸ユ儳鍎?(640x480 閹存牗娲挎?
- 濞ｅ崬瀹抽崶?(16-bit)
- 鐠囶厺绠熼崚鍡楀閸?
- 閸忔娊鏁悙瑙勭垼濞?
- 鏉堝湱鏅鍡樼垼濞?
