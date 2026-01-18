extends Node
"""
Godot TCP仿真服务器

监听来自Python GUI的连接和命令：
- 启动/停止仿真
- 加载机器人配置
- 动态更新参数
- 回传仿真数据
"""

var server := TCPServer.new()
var clients := []
var port := 9999
var is_simulating := false

# 当前机器人配置
var robot_config := {}

# 模拟数据
var sim_time := 0.0
var position := 0.0
var velocity := 0.1
var battery := 100.0

func _ready():
	var err = server.listen(port)
	if err == OK:
		print("✓ TCP仿真服务器启动于端口: ", port)
	else:
		push_error("✗ 无法启动服务器: ", err)

func _process(delta):
	# 接受新连接
	if server.is_connection_available():
		var client_stream = server.take_connection()
		clients.append({
			"stream": client_stream,
			"buffer": PackedByteArray()
		})
		print("✓ 客户端已连接 (总计: ", clients.size(), ")")
	
	# 处理现有客户端
	var i = 0
	while i < clients.size():
		var client = clients[i]
		var stream = client.stream
		
		if stream.get_status() != StreamPeerTCP.STATUS_CONNECTED:
			print("✗ 客户端断开连接")
			clients.remove_at(i)
			continue
		
		# 读取数据
		var available = stream.get_available_bytes()
		if available > 0:
			var data = stream.get_data(available)
			if data[0] == OK:
				client.buffer.append_array(data[1])
				_try_process_message(client)
		
		i += 1
	
	# 仿真更新
	if is_simulating:
		_update_simulation(delta)
		_send_feedback_to_all()

func _try_process_message(client):
	"""尝试处理完整消息"""
	if client.buffer.size() < 4:
		return # 等待长度前缀
	
	# 读取长度（4字节，大端序）
	var length = (client.buffer[0] << 24) | (client.buffer[1] << 16) | \
	             (client.buffer[2] << 8) | client.buffer[3]
	
	if client.buffer.size() < 4 + length:
		return # 消息不完整
	
	# 提取消息
	var message_bytes = client.buffer.slice(4, 4 + length)
	client.buffer = client.buffer.slice(4 + length)
	
	# 解析JSON
	var json_str = message_bytes.get_string_from_utf8()
	var json = JSON.new()
	var parse_result = json.parse(json_str)
	
	if parse_result != OK:
		push_error("JSON解析失败: ", json_str)
		return
	
	var message = json.data
	print("收到命令: ", message.get("command", "unknown"))
	
	# 处理命信
	match message.get("command", ""):
		"start_sim":
			_handle_start_sim(message.get("data", {}))
		"stop_sim":
			_handle_stop_sim()
		"update_params":
			_handle_update_params(message.get("data", {}))
		"load_robot":
			_handle_load_robot(message.get("data", {}))

func _handle_start_sim(data):
	"""启动仿真"""
	robot_config = data.get("robot", {})
	print("启动仿真，机器人零件数: ", robot_config.get("parts", []).size())
	
	# 重置模拟数据
	sim_time = 0.0
	position = 0.0
	velocity = 0.1
	battery = 100.0
	
	is_simulating = true

func _handle_stop_sim():
	"""停止仿真"""
	print("停止仿真")
	is_simulating = false

func _handle_update_params(params):
	"""更新参数"""
	print("更新参数: ", params)
	# TODO: 应用参数到机器人

func _handle_load_robot(data):
	"""加载机器人配置"""
	robot_config = data
	print("加载机器人配置: ", data.get("parts", []).size(), " 个零件")

func _update_simulation(delta):
	"""更新仿真（示例）"""
	sim_time += delta
	
	# 简单的运动模拟
	position += velocity * delta
	velocity = 0.1 + sin(sim_time) * 0.05
	battery -= delta * 0.5
	battery = max(battery, 0.0)

func _send_feedback_to_all():
	"""向所有客户端发送反馈数据"""
	var feedback = {
		"type": "simulation_data",
		"position": position,
		"velocity": velocity,
		"battery": battery,
		"timestamp": Time.get_ticks_msec() / 1000.0
	}
	
	var json_str = JSON.stringify(feedback)
	var json_bytes = json_str.to_utf8_buffer()
	
	# 长度前缀（4字节，大端序）
	var length = json_bytes.size()
	var length_bytes = PackedByteArray([
		(length >> 24) & 0xFF,
		(length >> 16) & 0xFF,
		(length >> 8) & 0xFF,
		length & 0xFF
	])
	
	for client in clients:
		var stream = client.stream
		stream.put_data(length_bytes)
		stream.put_data(json_bytes)

func _exit_tree():
	"""清理"""
	for client in clients:
		client.stream.disconnect_from_host()
	server.stop()
	print("TCP服务器已关闭")
