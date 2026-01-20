extends Camera3D

## 摄像机流媒体发送器
## 挂载到 Camera3D 节点上
## 负责将渲染画面发送给 Python GUI

const STREAM_PORT = 9998
var server: TCPServer
var connection: StreamPeerTCP

# 发送频率控制
var target_fps = 20
var time_accum = 0.0
var frame_interval = 1.0 / target_fps

func _ready():
	server = TCPServer.new()
	if server.listen(STREAM_PORT) == OK:
		print("📷 视频流服务器已启动: 端口 %d" % STREAM_PORT)
	else:
		push_error("视频流服务器启动失败!")

func _process(delta):
	# 接受连接
	if server.is_connection_available():
		connection = server.take_connection()
		print("📷 视频客户端已连接!")
		
	# 发送图像 (限制帧率)
	time_accum += delta
	if time_accum >= frame_interval:
		time_accum = 0.0
		if connection and connection.get_status() == StreamPeerTCP.STATUS_CONNECTED:
			_capture_and_send()

func _capture_and_send():
	# 获取视口图像
	var viewport = get_viewport()
	# 等待这一帧渲染完成（这可能会稍微降低FPS，但必须保证图像是最新的）
	# await RenderingServer.frame_post_draw # Godot 4 中通常直接 get_texture get_image 即可
	
	var tex = viewport.get_texture()
	var img = tex.get_image()
	
	# 缩放以减少带宽 (可选)
	img.resize(640, 360)
	
	# 压缩为 JPEG
	var buffer = img.save_jpg_to_buffer(0.75) # 质量 0.75
	
	# 发送协议: [4字节长度][JPEG数据]
	var size_bytes = PackedByteArray()
	size_bytes.resize(4)
	size_bytes.encode_u32(0, buffer.size())
	
	connection.put_data(size_bytes)
	connection.put_data(buffer)
