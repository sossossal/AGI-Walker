extends Node
## TCP服务器 - 负责与Python控制端通信
## 发送传感器数据，接收电机控制指令

# 服务器配置
const PORT = 9999
const HOST = "127.0.0.1"

# TCP相关
var server: TCPServer
var peer: StreamPeerTCP
var buffer := PackedByteArray()

# 状态标志
var has_client := false

# 引用机器人节点
@onready var robot = get_node("/root/Main/Robot")
# 引用地形生成器 (可选)
@onready var terrain_generator = get_node_or_null("/root/Main/TerrainGenerator")

signal client_connected
signal client_disconnected
signal command_received(command: Dictionary)


func _ready():
	server = TCPServer.new()
	var err = server.listen(PORT, HOST)
	
	if err != OK:
		push_error("TCP服务器启动失败!")
		return
	
	print("✅ TCP服务器已启动: %s:%d" % [HOST, PORT])
	print("等待Python客户端连接...")


func _process(_delta):
	# 检查新连接
	if server.is_connection_available():
		peer = server.take_connection()
		has_client = true
		print("✅ 客户端已连接: %s" % peer.get_connected_host())
		client_connected.emit()
	
	# 处理已连接的客户端
	if peer and peer.get_status() == StreamPeerTCP.STATUS_CONNECTED:
		# 发送传感器数据
		_send_sensor_data()
		
		# 接收控制指令
		_receive_commands()
	elif peer and peer.get_status() != StreamPeerTCP.STATUS_CONNECTED:
		if has_client:
			print("⚠️ 客户端断开连接")
			has_client = false
			client_disconnected.emit()
		peer = null


func _send_sensor_data():
	"""发送传感器数据到客户端"""
	if not robot:
		return
	
	var sensor_data = robot.get_sensor_data()
	var json_str = JSON.stringify(sensor_data) + "\n"
	
	var bytes = json_str.to_utf8_buffer()
	peer.put_data(bytes)


func _receive_commands():
	"""接收并处理控制指令"""
	var available = peer.get_available_bytes()
	if available > 0:
		var data = peer.get_data(available)
		if data[0] == OK:
			buffer.append_array(data[1])
			_process_buffer()


func _process_buffer():
	"""处理缓冲区中的命令"""
	var text = buffer.get_string_from_utf8()
	var lines = text.split("\n")
	
	# 处理完整的JSON行
	for i in range(lines.size() - 1):
		var line = lines[i].strip_edges()
		if line.is_empty():
			continue
		
		var json = JSON.new()
		var parse_result = json.parse(line)
		
		if parse_result == OK:
			var command = json.data
			_apply_command(command)
			command_received.emit(command)
		else:
			push_warning("JSON解析失败: %s" % line)
	
	# 保留未完成的数据
	buffer = lines[-1].to_utf8_buffer()


func _apply_command(command: Dictionary):
	"""应用控制指令到机器人"""
	if not robot:
		return
	
	if command.has("type"):
		match command.type:
			"reset":
				# 处理重置逻辑，包括地形更新
				if command.has("terrain_seed") and terrain_generator:
					var seed_val = int(command["terrain_seed"])
					terrain_generator.generate(seed_val)
				
				# 如果有 sim_params 也可以在这里应用(通过 robot script 或 env controller)
				pass
				
			"update_terrain":
				if terrain_generator:
					var seed_val = int(command.get("seed", 0))
					var roughness = float(command.get("roughness", 1.0))
					terrain_generator.generate(seed_val, roughness)
			
			"action":
				# 电机控制
				robot.apply_motor_commands(command)
	else:
		# 兼容旧协议 (直接是 motor map)
		robot.apply_motor_commands(command)


func _exit_tree():
	"""清理资源"""
	if peer:
		peer.disconnect_from_host()
	if server:
		server.stop()
