extends Camera3D
## 视觉传感器节点
## 用于机器人视觉输入，支持障碍物检测等CV任务

# 配置
@export var capture_resolution := Vector2i(320, 240) # 捕获分辨率
@export var fov_degrees := 60.0 # 视野角度
@export var near_plane := 0.1 # 近裁剪面
@export var far_plane := 10.0 # 远裁剪面

# SubViewport用于离屏渲染
var viewport: SubViewport = null
var is_ready := false

# 最后捕获的帧
var last_frame: Image = null
var last_capture_time := 0.0

# 统计
var captures_count := 0


func _ready():
	_setup_camera()
	_setup_viewport()
	print("📷 视觉传感器初始化完成")


func _setup_camera():
	"""配置相机参数"""
	fov = fov_degrees
	near = near_plane
	far = far_plane


func _setup_viewport():
	"""设置离屏渲染视口"""
	viewport = SubViewport.new()
	viewport.size = capture_resolution
	viewport.render_target_update_mode = SubViewport.UPDATE_WHEN_VISIBLE
	viewport.transparent_bg = false
	
	# 添加到场景
	add_child(viewport)
	
	# 克隆相机到视口
	var cam = Camera3D.new()
	cam.fov = fov_degrees
	cam.near = near_plane
	cam.far = far_plane
	viewport.add_child(cam)
	
	is_ready = true


func capture_frame() -> Image:
	"""捕获当前帧"""
	if not is_ready:
		return null
	
	# 等待渲染完成
	await RenderingServer.frame_post_draw
	
	# 获取纹理
	var texture = viewport.get_texture()
	if texture:
		last_frame = texture.get_image()
		last_capture_time = Time.get_ticks_msec() / 1000.0
		captures_count += 1
		return last_frame
	
	return null


func capture_frame_base64() -> String:
	"""捕获当前帧并编码为Base64"""
	var image = await capture_frame()
	if image == null:
		return ""
	
	# 转换为PNG并编码
	var png_data = image.save_png_to_buffer()
	return Marshalls.raw_to_base64(png_data)


func get_vision_data() -> Dictionary:
	"""获取视觉传感器数据（同步版本，使用缓存帧）"""
	return {
		"frame_id": captures_count,
		"resolution": [capture_resolution.x, capture_resolution.y],
		"fov": fov_degrees,
		"has_frame": last_frame != null,
		"last_capture_time": last_capture_time
	}


func get_sensor_info() -> Dictionary:
	"""获取传感器配置信息"""
	return {
		"type": "camera",
		"resolution": [capture_resolution.x, capture_resolution.y],
		"fov_degrees": fov_degrees,
		"near_plane": near_plane,
		"far_plane": far_plane,
		"is_ready": is_ready,
		"captures_count": captures_count
	}


# 简单的边缘检测（使用着色器或后处理）
# 注意：完整CV处理应在Python端进行
func detect_obstacles_simple() -> Array:
	"""简化版障碍物检测
	
	基于深度缓冲的简单检测，返回可能的障碍物区域
	完整处理请使用Python视觉模块
	"""
	# TODO: 实现基于深度的简单障碍物检测
	# 这里返回占位数据
	return []
