# AGI-Walker 3D閸欘垵顫嬮崠鏍ь杻瀵儤鏌熷?

**閻楀牊婀?*: 1.0  
**閺冦儲婀?*: 2026-01-18  
**閻╊喗鐖?*: 娑撶儤澧嶉張澶愭祩娴犺埖鍧婇崝?D濡€崇€烽敍宀€绮烘稉鈧珿UI閸滃瓘odot閻ㄥ嫯顫嬬憴澶夌秼妤?

---

## 棣冩惖 濮掑倽鍫?

### 瑜版挸澧犻悩鑸碘偓?

**GUI闁板秶鐤嗛崳?*:
- 閴?2D閻晛鑸扮悰銊с仛闂嗘湹娆?
- 閴?閺傚洦婀伴弽鍥╊劮閺勫墽銇氶崥宥囆?
- 閳跨媴绗?缂傚搫鐨憴鍡氼潕缂佸棜濡?

**Godot娴犺法婀?*:
- 閴?閸╄櫣顢呴悧鈺冩倞绾扮増鎸掓担?
- 閳跨媴绗?缁犫偓閸楁洖鑸伴悩璁圭礄缁斿鏌熸担鎾扁偓浣烘倖娴ｆ搫绱?
- 閳跨媴绗?缂傚搫鐨惇鐔风杽婢舵牞顫?

### 閻╊喗鐖?

閸掓稑缂撶紒鐔剁閻?D閸欘垵顫嬮崠鏍兇缂佺噦绱?
- 棣冨腹 GUI娑擃厽妯夌粈?D闂嗘湹娆㈢紓鈺冩殣閸?妫板嫯顫?
- 棣冨箖 Godot娑擃厺濞囬悽銊嚊缂?D濡€崇€?
- 棣冩敡 娣囨繃瀵旂憴鍡氼潕娑撯偓閼峰瓨鈧?
- 棣冩憹 35+闂嗘湹娆㈤惃鍕暚閺佸瓨膩閸ㄥ绨?

---

## 棣冨箚 鐎圭偞鏌﹂弬瑙勵攳

### 閺傝顢岮: 濞撴劘绻樺蹇撶杽閻滃府绱欓幒銊ㄥ礃閿涘鐡?

#### 闂冭埖顔?: 閸╄櫣顢?D鎼存搫绱?-2閸涱煉绱?
閸掓稑缂撶粻鈧崠鏍畱3D濡€崇€烽悽銊ょ艾韫囶偊鈧喎鐤勯悳?

#### 闂冭埖顔?: GUI 3D妫板嫯顫嶉敍?-2閸涱煉绱?
閸︹剠UI娑擃參娉﹂幋?D濞撳弶鐓?

#### 闂冭埖顔?: Godot鐠囷妇绮忓Ο鈥崇€烽敍?-3閸涱煉绱?
閸︹剠odot娑擃厺濞囬悽銊╃彯鐠愩劑鍣哄Ο鈥崇€?

#### 闂冭埖顔?: 娴兼ê瀵查崪宀€绮烘稉鈧敍?閸涱煉绱?
缂佺喍绔撮弶鎰窛閸滃矂顥撻弽?

**閹粯妞傞梻?*: 5-8閸?

### 閺傝顢岯: 韫囶偊鈧喎甯崹瀣剁礄閺堚偓鐏忓繐褰茬悰宀嬬礆

娴犲懍璐熼崗鎶芥暛闂嗘湹娆㈤敍?-10娑擃亷绱氶崚娑樼紦濡€崇€?
**閹粯妞傞梻?*: 1-2閸?

---

## 棣冩礈閿?閹垛偓閺堫垰鐤勯悳?

### 1. 3D濡€崇€烽弽鐓庣础

#### GUI妫板嫯顫?
**閺嶇厧绱?*: PNG/WebP缂傗晝鏆愰崶鎾呯礄妫板嫭瑕嗛弻鎿勭礆
- 娴ｈ法鏁lender妫板嫭瑕嗛弻鎾虫倗鐟欐帒瀹崇憴鍡楁禈
- 鏉炲鍣虹痪褝绱濋崝鐘烘祰韫囶偊鈧?
- 闁倸鎮嶵kinter閺勫墽銇?

**婢跺洭鈧?*: Matplotlib 3D閿涘牆鐤勯弮鑸佃閺屾搫绱?
```python
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
```

#### Godot娴犺法婀?
**閺嶇厧绱?*: GLTF 2.0 / FBX
- 閺€顖涘瘮閻椻晝鎮婄喊鐗堟寬娴?
- 閺夋劘宸濋崪宀€姹楅悶?
- 娴兼ê瀵查惃鍕樋鏉堢懓鑸伴弫?

### 2. 闂嗘湹娆?D濡€崇€风憴鍕瘱

#### 濡€崇€风憰浣圭湴
```yaml
闂嗘湹娆㈢猾璇插焼: [閻㈠灚婧€, 娴肩姵鍔呴崳? 閹貉冨煑閸? 閸忓疇濡? 閻㈠灚鐫淽

濮ｅ繋閲滃Ο鈥崇€?
  - 閺嶇厧绱? GLTF 2.0
  - 婢舵俺绔熻ぐ銏℃殶: <5000 (娴ｅ孩膩)
  - 鐏忓搫顕? 鐎圭偤妾悧鈺冩倞鐏忓搫顕敍鍫㈣儗閿?
  - 閸樼喓鍋? 鐠愩劌绺鹃幋鏍х暔鐟佸懐鍋?
  - 閺夋劘宸? PBR閺夋劘宸?
  - LOD: 閸欘垶鈧绱欐妯硅厬娴ｅ簼绗佹稉顏嗛獓閸掝偓绱?
```

#### 缁€杞扮伐閿涙ynamixel XL430閻㈠灚婧€
```
濡€崇€烽弬鍥︽: weights/motors/dynamixel_xl430.gltf
鐏忓搫顕? 28.5mm 鑴?46.5mm 鑴?34mm
婢舵俺绔熻ぐ? 2,847
閺夋劘宸? 闁叉垵鐫?婵夋垶鏋?
绾扮増鎸掓担? 缁犫偓閸栨牜鐝涢弬閫涚秼
```

---

## 棣冩惢 GUI 3D妫板嫯顫嶇€圭偟骞?

### 闁銆?: 妫板嫭瑕嗛弻鎾剁級閻ｃ儱娴橀敍鍫熷腹閼芥劧绱氱尭?

**娴兼ê濞?*:
- 韫囶偊鈧喎濮炴潪?
- 娴ｅ钉PU濞戝牐鈧?
- 閺勬挷绨€圭偟骞?

**鐎圭偟骞?*:
```python
# tools/render_part_thumbnails.py

import bpy  # Blender Python API
from pathlib import Path

def render_part_thumbnail(model_path, output_path, angle=45):
    """
    娴ｈ法鏁lender濞撳弶鐓嬮梿鏈垫缂傗晝鏆愰崶?
    
    Args:
        model_path: 3D濡€崇€风捄顖氱窞
        output_path: 鏉堟挸鍤崶鍓у鐠侯垰绶?
        angle: 鐟欏棜顫楅敍鍫濆閿?
    """
    # 鐎电厧鍙嗗Ο鈥崇€?
    bpy.ops.import_scene.gltf(filepath=model_path)
    
    # 鐠佸墽鐤嗛惄鍛婃簚
    camera = bpy.data.objects['Camera']
    camera.location = (2, -2, 2)
    camera.rotation_euler = (math.radians(60), 0, math.radians(45))
    
    # 鐠佸墽鐤嗛崗澶屽弾
    light = bpy.data.lights.new(name="Key Light", type='SUN')
    light.energy = 5.0
    
    # 濞撳弶鐓嬬拋鍓х枂
    bpy.context.scene.render.resolution_x = 256
    bpy.context.scene.render.resolution_y = 256
    bpy.context.scene.render.image_settings.file_format = 'PNG'
    
    # 濞撳弶鐓?
    bpy.context.scene.render.filepath = output_path
    bpy.ops.render.render(write_still=True)

# 閹靛綊鍣哄〒鍙夌厠閹碘偓閺堝娴傛禒?
for part in parts_library:
    render_part_thumbnail(
        f"weights/{part.category}/{part.model}.gltf",
        f"assets/thumbnails/{part.id}.png"
    )
```

**閸︹剠UI娑擃厺濞囬悽?*:
```python
# tools/robot_configurator_gui.py

from PIL import Image, ImageTk

class PartNode:
    def __init__(self, canvas, part_id, part_data, x, y):
        # 閸旂姾娴?D缂傗晝鏆愰崶?
        thumbnail_path = f"assets/thumbnails/{part_id}.png"
        if Path(thumbnail_path).exists():
            img = Image.open(thumbnail_path)
            img = img.resize((60, 60), Image.LANCZOS)
            self.photo = ImageTk.PhotoImage(img)
            
            # 閺勫墽銇氶崶鍓у閼板矂娼惌鈺佽埌
            self.image_obj = canvas.create_image(
                x + 30, y + 30,
                image=self.photo,
                tags=('part', part_id)
            )
        else:
            # 闂勫秶楠囬崚?D閻晛鑸?
            self.rect = canvas.create_rectangle(...)
```

### 闁銆?: Matplotlib 3D鐎圭偞妞傚〒鍙夌厠

**娴兼ê濞?*:
- 閸欘垯姘︽禍鎺撴鏉?
- 閸斻劍鈧焦娲块弬?

**閸旓絽濞?*:
- 鏉堝啯鍙?
- 閸楃姷鏁PU

**鐎圭偟骞?*:
```python
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def create_3d_preview_widget(parent, part_data):
    """閸掓稑缂?D妫板嫯顫嶇粣妤€褰?""
    fig = plt.Figure(figsize=(3, 3))
    ax = fig.add_subplot(111, projection='3d')
    
    # 缂佹ê鍩楃粻鈧崠鏍侀崹瀣剁礄缁斿鏌熸担鎾躲仛娓氬绱?
    vertices = np.array([...])  # 娴犲孩膩閸ㄥ鏋冩禒璺哄鏉?
    faces = [...]
    
    collection = Poly3DCollection(faces, alpha=0.8)
    ax.add_collection3d(collection)
    
    # 瀹撳苯鍙嗛崚鐧焝inter
    canvas = FigureCanvasTkAgg(fig, parent)
    canvas.get_tk_widget().pack()
```

---

## 棣冨箖 Godot 3D濡€崇€烽梿鍡樺灇

### 1. 濡€崇€风€电厧鍙?

**閻╊喖缍嶇紒鎾寸€?*:
```
godot_project/
閳规壕鏀㈤埞鈧?assets/
閳?  閳规壕鏀㈤埞鈧?weights/
閳?  閳?  閳规壕鏀㈤埞鈧?motors/
閳?  閳?  閳?  閳规壕鏀㈤埞鈧?dynamixel_xl430.gltf
閳?  閳?  閳?  閳规壕鏀㈤埞鈧?dynamixel_ax12.gltf
閳?  閳?  閳?  閳规柡鏀㈤埞鈧?...
閳?  閳?  閳规壕鏀㈤埞鈧?sensors/
閳?  閳?  閳?  閳规壕鏀㈤埞鈧?mpu6050.gltf
閳?  閳?  閳?  閳规柡鏀㈤埞鈧?...
閳?  閳?  閳规柡鏀㈤埞鈧?controllers/
閳?  閳?      閳规柡鏀㈤埞鈧?raspberry_pi4.gltf
閳?  閳规柡鏀㈤埞鈧?materials/
閳?      閳规壕鏀㈤埞鈧?metal_brushed.tres
閳?      閳规柡鏀㈤埞鈧?plastic_black.tres
閳规柡鏀㈤埞鈧?scripts/
    閳规柡鏀㈤埞鈧?part_loader.gd
```

### 2. 閸斻劍鈧礁濮炴潪鍊熷壖閺?

**閺傚洣娆?*: `godot_project/scripts/part_loader.gd`

```gdscript
extends Node3D
class_name PartLoader

# 闂嗘湹娆㈠Ο鈥崇€风紓鎾崇摠
var model_cache := {}

# 閸旂姾娴囬梿鏈垫3D濡€崇€?
func load_part(part_id: String, part_type: String) -> Node3D:
    var model_path = "res://assets/weights/%s/%s.gltf" % [part_type, part_id]
    
    # 濡偓閺屻儳绱︾€?
    if model_cache.has(model_path):
        return model_cache[model_path].duplicate()
    
    # 閸旂姾娴囧Ο鈥崇€?
    if ResourceLoader.exists(model_path):
        var scene = load(model_path)
        var instance = scene.instantiate()
        model_cache[model_path] = instance
        return instance.duplicate()
    else:
        # 闂勫秶楠囬崚鎵暆閸楁洖鑸伴悩?
        return create_simple_shape(part_type)

# 缁犫偓閸楁洖鑸伴悩鏈电稊娑撴椽妾风痪褎鏌熷?
func create_simple_shape(part_type: String) -> Node3D:
    var shape = MeshInstance3D.new()
    
    match part_type:
        "motor":
            shape.mesh = CylinderMesh.new()
            shape.mesh.height = 0.05
            shape.mesh.radius = 0.02
        "sensor":
            shape.mesh = BoxMesh.new()
            shape.mesh.size = Vector3(0.02, 0.02, 0.01)
        "controller":
            shape.mesh = BoxMesh.new()
            shape.mesh.size = Vector3(0.08, 0.06, 0.02)
        _:
            shape.mesh = SphereMesh.new()
    
    return shape

# 濞ｈ濮為悧鈺冩倞绾扮増鎸掓担?
func add_collision_shape(part: Node3D, collision_data: Dictionary):
    var collision = CollisionShape3D.new()
    
    match collision_data.get("type", "box"):
        "box":
            var shape = BoxShape3D.new()
            shape.size = Vector3(
                collision_data.get("width", 0.05),
                collision_data.get("height", 0.05),
                collision_data.get("depth", 0.05)
            )
            collision.shape = shape
        "sphere":
            var shape = SphereShape3D.new()
            shape.radius = collision_data.get("radius", 0.025)
            collision.shape = shape
        "cylinder":
            var shape = CylinderShape3D.new()
            shape.height = collision_data.get("height", 0.05)
            shape.radius = collision_data.get("radius", 0.02)
            collision.shape = shape
    
    part.add_child(collision)
```

### 3. TCP閺堝秴濮熼崳銊╂肠閹?

閺囧瓨鏌奰TCPSimulationServer.gd`娴犮儲鏁幐?D濡€崇€烽敍?

```gdscript
func handle_load_robot(data):
    var parts = data.get("parts", [])
    
    for part_info in parts:
        # 閸旂姾娴?D濡€崇€?
        var part_node = part_loader.load_part(
            part_info.part_id,
            part_info.part_type
        )
        
        # 鐠佸墽鐤嗘担宥囩枂
        part_node.position = Vector3(
            part_info.position[0],
            part_info.position[1],
            part_info.position[2]
        )
        
        # 濞ｈ濮炵喊鐗堟寬娴?
        if part_info.has("collision"):
            part_loader.add_collision_shape(
                part_node,
                part_info.collision
            )
        
        # 濞ｈ濮為崚鏉挎簚閺?
        robot_container.add_child(part_node)
```

---

## 棣冨腹 鐟欏棜顫庢稉鈧懛瀛樷偓褏鐡ラ悾?

### 1. 缂佺喍绔撮弶鎰窛缁崵绮?

**Godot閺夋劘宸?* (`.tres`閺傚洣娆?:
```gdscript
# assets/materials/motor_body.tres
[resource]
type = "StandardMaterial3D"
albedo_color = Color(0.2, 0.2, 0.2, 1.0)  # 濞ｈ京浼?
metallic = 0.8
roughness = 0.3
```

**Blender濞撳弶鐓嬬拋鍓х枂** (缂傗晝鏆愰崶?:
- 閻╃鎮撻惃鍕杹閼规彃鈧?
- 閻╅晲鎶€閻ㄥ嫬鍘滈悡褑顫楁惔?
- 娑撯偓閼峰娈戦懗灞炬珯

### 2. 妞嬪孩鐗搁幐鍥у础

閹碘偓閺堝膩閸ㄥ浼掑顏庣窗
- **闁板秷澹?*: 瀹搞儰绗熼悘鑸偓渚€绮﹂妴渚€鍣剧仦鐐跺
- **缂佸棜濡?*: 娑擃厾鐡戠紒鍡氬Ν閿涘牓娼悡褏澧栫痪褝绱?
- **濮ｆ柧绶?*: 缁墽鈥橀惃鍕⒖閻炲棗鏄傜€?
- **閺嶅洤鍣憴鍡氼潡**: 45鎼达妇鐡戠捄婵婎潒閸?

### 3. 閸ョ偓鐖ｇ憴鍕瘱

GUI缂傗晝鏆愰崶鎾呯窗
```
鐏忓搫顕? 256鑴?56 (濞撳弶鐓? 閳?64鑴?4 (閺勫墽銇?
閼冲本娅? 闁繑妲戦幋鏍ㄧガ閻?
閸忓鍙? 3閻愮懓鍘滈悡褝绱欐稉璇插帨+鐞涖儱鍘?鏉烆喖绮ㄩ崗澶涚礆
鐟欏棜顫? 45鎼达缚鍒婄憴?
```

---

## 棣冩憹 3D鐠у嫪楠囬崚娑樼紦瀹搞儰缍斿ù?

### 瀹搞儱鍙块柧?

1. **Blender 3.6+** - 娑撴槒顩﹀鐑樐佸銉ュ徔
2. **MeshLab** - 缁犫偓閸栨牕鎷版导妯哄
3. **gltf-pipeline** - 閺嶇厧绱℃潪顒佸床閸滃苯甯囩紓?

### 閸掓稑缂撳ù浣衡柤

```bash
# 1. 閸λ檒ender娑擃厼缂撳Ο?
blender --background --python create_motor_model.py

# 2. 鐎电厧鍤崚鐧嶭TF
# (閸λ檒ender娑? File 閳?Export 閳?glTF 2.0)

# 3. 娴兼ê瀵插Ο鈥崇€?
gltf-pipeline -i input.gltf -o output.gltf -d

# 4. 濞撳弶鐓嬬紓鈺冩殣閸?
blender --background --python render_thumbnail.py -- model.gltf

# 5. 婢跺秴鍩楅崚浼淬€嶉惄?
cp output.gltf godot_project/assets/weights/motors/
cp thumbnail.png assets/thumbnails/
```

### 閹靛綊鍣烘径鍕倞閼存碍婀?

**閺傚洣娆?*: `tools/batch_create_models.sh`

```bash
#!/bin/bash
# 閹靛綊鍣洪崚娑樼紦閹碘偓閺堝娴傛禒鍓佹畱3D濡€崇€?

PARTS_CSV="parts_library/parts_specs.csv"

while IFS=, read -r id type model; do
    echo "Processing: $id ($model)"
    
    # 1. 閸掓稑缂撳Ο鈥崇€烽敍鍫濐洤閺嬫粍膩閺夊灝鐡ㄩ崷顭掔礆
    if [ -f "blender_templates/${type}_template.blend" ]; then
        blender --background \
            blender_templates/${type}_template.blend \
            --python scripts/customize_model.py \
            -- --part-id "$id" --model "$model"
    fi
    
    # 2. 濞撳弶鐓嬬紓鈺冩殣閸?
    blender --background \
        --python scripts/render_thumbnail.py \
        -- --input "output/${id}.gltf" \
           --output "assets/thumbnails/${id}.png"
    
done < "$PARTS_CSV"
```

---

## 棣冩畬 鐎圭偞鏌﹀銉╊€?

### Phase 1: 閸戝棗顦銉ょ稊閿?-5婢垛晪绱?

- [ ] 鐎瑰顥夿lender閸滃苯浼愰崗?
- [ ] 閸掓稑缂撳Ο鈥崇€风憴鍕瘱閺傚洦銆?
- [ ] 瀵よ櫣鐝汢lender濡剝婢?
- [ ] 鐠佸墽鐤嗛弶鎰窛鎼?

### Phase 2: 閺嶇绺鹃梿鏈垫瀵ょ儤膩閿?-2閸涱煉绱?

娴兼ê鍘涚痪褔鐝惃鍕祩娴犺绱?0娑擃亷绱?
- [ ] Dynamixel XL430
- [ ] Dynamixel AX-12
- [ ] MPU6050
- [ ] Raspberry Pi 4
- [ ] 閸╄櫣顢呴崗瀹犲Ν鑴?
- [ ] 閻㈠灚鐫滆劤2
- [ ] 缂傛牜鐖滈崳顭?

### Phase 3: GUI闂嗗棙鍨氶敍?閸涱煉绱?

- [ ] 鐎圭偟骞囩紓鈺冩殣閸ユ儳濮炴潪?
- [ ] 閺囧瓨鏌奝artNode缁?
- [ ] 濞ｈ濮?D妫板嫯顫嶇粣妤€褰涢敍鍫濆讲闁绱?
- [ ] 濞村鐦幀褑鍏?

### Phase 4: Godot闂嗗棙鍨氶敍?-2閸涱煉绱?

- [ ] 鐎电厧鍙嗛幍鈧張濉哃TF濡€崇€?
- [ ] 鐎圭偟骞噋art_loader.gd
- [ ] 閺囧瓨鏌奣CP閺堝秴濮熼崳?
- [ ] 闁板秶鐤嗛悧鈺冩倞绾扮増鎸掓担?
- [ ] 濞村鐦〒鍙夌厠閹嗗厴

### Phase 5: 閹碘晛鐫嶉崜鈺€缍戦梿鏈垫閿?-2閸涱煉绱?

- [ ] 鐎瑰本鍨氶幍鈧張?5+闂嗘湹娆?
- [ ] 娴兼ê瀵插Ο鈥崇€?
- [ ] 缂佺喍绔寸憴鍡氼潕妞嬪孩鐗?

### Phase 6: 娴兼ê瀵查崪灞界暚閸犲嫸绱?閸涱煉绱?

- [ ] 閹嗗厴娴兼ê瀵?
- [ ] LOD缁崵绮洪敍鍫濆讲闁绱?
- [ ] 閺傚洦銆傞弴瀛樻煀
- [ ] 閻劍鍩涘ù瀣槸

**閹粯妞傞梻?*: 5-8閸?

---

## 棣冩惓 鐠у嫭绨棁鈧Ч?

### 鐎涙ê鍋嶇粚娲？

```
娴兼壆鐣?
- 濮ｅ繋閲淕LTF濡€崇€? 100KB - 2MB
- 濮ｅ繋閲淧NG缂傗晝鏆愰崶? 10-50KB
- 35娑擃亪娴傛禒鑸碘偓鏄忣吀: ~50-100MB
```

### 閹嗗厴瑜板崬鎼?

**GUI**:
- 缂傗晝鏆愰崶鐐煙瀵? 閸戠姳绠弮鐘插閸?
- 鐎圭偞妞?D: +20-50% CPU

**Godot**:
- 娴ｅ孩膩閻楀牊婀? 閸欘垱甯撮崣?
- 妤傛ɑ膩閻楀牊婀? 闂団偓鐟曚俯OD娴兼ê瀵?

---

## 棣冩寱 韫囶偊鈧喎鐤勯悳鐗堟煙濡楀牞绱欓幒銊ㄥ礃閿?

婵″倹鐏夐棁鈧憰浣告彥闁喓婀呴崚鐗堟櫏閺嬫粣绱濆楦款唴閿?

### 閺堚偓鐏忓繐褰茬悰灞奸獓閸濅緤绱?閸涱煉绱?

1. **闁瀚?娑擃亙鍞悰銊︹偓褔娴傛禒?*:
   - Dynamixel XL430閿涘牏鏁搁張鐚寸礆
   - MPU6050閿涘牅绱堕幇鐔锋珤閿?
   - Raspberry Pi 4閿涘牊甯堕崚璺烘珤閿?
   - 閺冨娴嗛崗瀹犲Ν
   - 閻㈠灚鐫滈崠?

2. **娴ｈ法鏁ょ粻鈧崠鏍侀崹?*:
   - 閸╄櫣顢呴崙鐘辩秿娴ｆ挾绮嶉崥?
   - 閸楁洑绔撮弶鎰窛
   - 閺冪姴顦查弶鍌滄睏閻?

3. **GUI妫板嫭瑕嗛弻鎾剁級閻ｃ儱娴?*:
   - 韫囶偊鈧喎婀狟lender娑擃厽瑕嗛弻?
   - 閻╁瓨甯撮弴鎸庡床閻滅増婀侀惌鈺佽埌

4. **Godot閸╄櫣顢呰ぐ銏㈠Ц**:
   - 閸忓牏鏁ょ粙瀣碍閸栨牕鍤戞担鏇氱秼
   - 閸氬海鐢婚弴鎸庡床娑撻缚顕涚紒鍡樐侀崹?

---

## 棣冨箚 妫板嫭婀￠弫鍫熺亯

### GUI閺佸牊鐏?
```
娑斿澧? 閽冩繆澹婇惌鈺佽埌 + 閺傚洦婀伴弽鍥╊劮
娑斿鎮? 閻喎鐤?D闂嗘湹娆㈢紓鈺冩殣閸?+ 閺傚洦婀伴弽鍥╊劮
```

### Godot閺佸牊鐏?
```
娑斿澧? 缁犫偓閸楁洜鐝涢弬閫涚秼/閻炲啩缍?
娑斿鎮? 鐠囷妇绮?D濡€崇€?+ 閻喎鐤勬径鏍潎
```

### 閻劍鍩涙担鎾荤崣
- 閴?閺囧娲跨憴鍌滄畱闂嗘湹娆㈢拠鍡楀焼
- 閴?閺囩繝绗撴稉姘辨畱鐟欏棜顫庨弫鍫熺亯
- 閴?GUI閸滃奔璞㈤惇鐔烘畱娑撯偓閼峰瓨鈧?
- 閴?閺囨潙顔愰弰鎾舵倞鐟欙絾婧€閸ｃ劋姹夌紒鎾寸€?

---

## 閳跨媴绗?濞夈劍鍓版禍瀣€?

### 1. 閻楀牊娼堥梻顕€顣?
- 娴ｈ法鏁ら崢鐔峰灡濡€崇€烽幋鏍х磻濠ф劘绁┃?
- 闁灝鍘ら惄瀛樺复婢跺秴鍩楅崯鍡曠瑹濡€崇€?
- 濞夈劍妲戝Ο鈥崇€烽弶銉︾爱

### 2. 閹嗗厴閼板啳妾?
- 娣囨繃瀵旀担搴☆樋鏉堢懓鑸伴弫?
- 娴ｈ法鏁ょ痪鍦倞閼板矂娼崙鐘辩秿缂佸棜濡?
- 鐎圭偟骞嘗OD缁崵绮洪敍鍫濆讲闁绱?

### 3. 缂佸瓨濮㈤幋鎰拱
- 閺備即娴傛禒鍫曟付鐟曚焦鍧婇崝鐘衬侀崹?
- 閺囧瓨鏌婂Ο鈥崇€烽棁鈧憰浣告倱濮濐檷UI閸滃瓘odot

---

## 棣冩憥 閸欏倽鈧啳绁┃?

### 鐎涳缚绡勭挧鍕爱
- [Blender鐎规ɑ鏌熼弬鍥ㄣ€俔(https://docs.blender.org/)
- [GLTF 2.0鐟欏嫯瀵朷(https://www.khronos.org/gltf/)
- [Godot 3D閺佹瑧鈻糫(https://docs.godotengine.org/en/stable/tutorials/3d/)

### 濡€崇€风挧鍕爱
- [Sketchfab](https://sketchfab.com/) - 3D濡€崇€锋惔?
- [TurboSquid](https://www.turbosquid.com/) - 娑撴挷绗熷Ο鈥崇€?
- [OpenRoboticsAssets](https://fuel.gazebosim.org/) - 閺堝搫娅掓禍娲祩娴?

---

**瀵ら缚顔?*: 閸忓牏鏁?*韫囶偊鈧喎鐤勯悳鐗堟煙濡?*閿?閸涱煉绱氶崚娑樼紦5娑擃亝鐗宠箛鍐祩娴犲墎娈?D濡€崇€烽敍宀勭崣鐠囦焦鏅ラ弸婊冩倵閸愬秴鍠呯€规碍妲搁崥锕€鍙忛棃銏″腹鏉╂稏鈧?

**娑撳绔村?*: 
1. 绾喛顓婚弰顖氭儊鐟曚礁鐤勯弬鑺ヮ劃閺傝顢?
2. 閸愬啿鐣鹃柌鍥╂暏鐎瑰本鏆ｉ悧鍫ｇ箷閺勵垰鎻╅柅鐔哄
3. 閹存垵褰叉禒銉ョ磻婵鍨卞绡塴ender閼存碍婀伴崪灞灸侀弶?
