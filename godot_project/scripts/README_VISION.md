# Godot视觉数据服务器
# Vision Data Server for Godot

此目录包含用于生成视觉训练数据的Godot脚本

## 文件说明

### 需要在Godot项目中创建的脚本:

1. **VisionDataGenerator.gd** - 相机系统和图像捕获
   - 位置: `godot_project/scripts/VisionDataGenerator.gd`
   - 功能: 管理多个相机，捕获RGB/深度/分割图

2. **TCPVisionServer.gd** - TCP通信服务器
   - 位置: `godot_project/scripts/TCPVisionServer.gd`
   - 功能: 接收Python命令，返回图像数据

3. **depth_shader.shader** - 深度渲染着色器
   - 位置: `godot_project/shaders/depth_shader.shader`
   - 功能: 渲染深度图

## 实施步骤

### 步骤1: 创建VisionDataGenerator.gd

```gdscript
# godot_project/scripts/VisionDataGenerator.gd
extends Node

var cameras = []
var capture_resolution = Vector2(640, 480)

func _ready():
    setup_cameras()

func setup_cameras():
    # 第三人称相机
    var third_person_cam = Camera.new()
    third_person_cam.transform.origin = Vector3(2, 1.5, 2)
    third_person_cam.rotation_degrees = Vector3(-30, -45, 0)
    add_child(third_person_cam)
    cameras.append(third_person_cam)
    
    # 可以添加更多相机...

func capture_all_views() -> Dictionary:
    var images = {}
    
    for i in range(cameras.size()):
        var cam = cameras[i]
        images["rgb_" + str(i)] = capture_rgb(cam)
        # images["depth_" + str(i)] = capture_depth(cam)
        # images["segmentation_" + str(i)] = capture_segmentation(cam)
    
    return images

func capture_rgb(camera: Camera) -> Image:
    var viewport = get_viewport()
    yield(get_tree(), "idle_frame")
    var image = viewport.get_texture().get_data()
    image.flip_y()
    return image
```

### 步骤2: 创建TCPVisionServer.gd

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
    if server.is_connection_available():
        connection = server.take_connection()
        print("Client connected")
    
    if connection and connection.get_available_bytes() > 0:
        var request = connection.get_utf8_string(connection.get_available_bytes())
        handle_request(request)

func handle_request(request: String):
    var data = JSON.parse(request).result
    
    match data.command:
        "ping":
            send_response({"status": "success", "message": "pong"})
        
        "capture":
            var images = $VisionDataGenerator.capture_all_views()
            send_images(images)
        
        "update_robot":
            update_robot_state(data.state)
            send_response({"status": "success"})

func send_response(response: Dictionary):
    var json = JSON.print(response)
    connection.put_data((json + "\n").to_utf8())

func send_images(images: Dictionary):
    var response = {
        "status": "success",
        "images": {}
    }
    
    for key in images:
        var image = images[key]
        response.images[key] = image_to_base64(image)
    
    send_response(response)

func image_to_base64(image: Image) -> String:
    var buffer = image.save_png_to_buffer()
    return Marshalls.raw_to_base64(buffer)

func update_robot_state(state: Dictionary):
    # 更新机器人位置和姿态
    var robot = get_node("/root/Main/Robot")  # 根据实际路径修改
    if robot:
        robot.translation = Vector3(state.position[0], state.position[1], state.position[2])
        # 更新旋转和关节角度...
```

### 步骤3: 在Godot场景中设置

1. 打开主场景
2. 添加一个新节点，附加 `TCPVisionServer.gd`
3. 在TCP Server节点下添加子节点，附加 `VisionDataGenerator.gd`
4. 运行场景，服务器应该开始监听端口9999

## 测试

### 从Python端测试:

```bash
cd D:\新建文件夹\AGI-Walker
python python_api\godot_vision_client.py
```

## 故障排除

### 问题: 连接失败

- 检查Godot项目是否正在运行
- 确认TCP服务器已启动（控制台应显示 "Vision server listening..."）
- 检查防火墙设置

### 问题: 图像为空

- 确认相机已正确设置
- 检查场景中是否有内容可渲染
- 查看Godot控制台的错误信息

### 问题: 性能慢

- 减少图像分辨率
- 增加保存间隔 (save_interval)
- 使用更少的相机

## 注意事项

1. Godot必须先启动并运行，Python客户端才能连接
2. TCP端口9999必须可用
3. 图像质量和性能需要平衡
4. Base64编码会增加数据传输大小

## 完整代码

完整的Godot脚本代码请参考 `CV_IMPLEMENTATION_PLAN.md`
