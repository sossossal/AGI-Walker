"""
Web-Godot WebSocket 客户端 (Godot GDScript)

实现 Web-Godot 协议 v1.0 的 Godot 端 WebSocket 客户端，支持：
- 接收来自 Web 面板的命令
- 发送遥测数据和状态信息
- 自动重连机制
- 完整的错误处理

使用方法:
  var client = WebGodotClient.new()
  client.connect_to_server("ws://localhost:8000", "godot_session_1")
  client.push_telemetry(telemetry_data)
"""

extends Node

class_name WebGodotClient

# 连接配置
var base_url: String = "ws://localhost:8000"
var session_id: String = "godot_session_1"
var websocket: WebSocketPeer = null
var is_connected: bool = false
var reconnect_enabled: bool = true
var reconnect_delay: float = 2.0
var max_reconnect_attempts: int = 5
var current_reconnect_attempts: int = 0

# 消息追踪
var message_counter: int = 0
var pending_responses: Dictionary = {}  # message_id -> callback

# 事件回调
var on_connected: Callable = func(): pass
var on_disconnected: Callable = func(): pass
var on_simulation_start: Callable = func(physics: Dictionary): pass
var on_simulation_stop: Callable = func(): pass
var on_load_robot: Callable = func(config: Dictionary): pass
var on_update_params: Callable = func(params: Dictionary): pass
var on_error: Callable = func(error: String): pass

# 重连计时器
var reconnect_timer: Timer = null


func _ready() -> void:
	"""初始化 WebSocket 客户端"""
	websocket = WebSocketPeer.new()
	
	# 创建重连计时器
	reconnect_timer = Timer.new()
	add_child(reconnect_timer)
	reconnect_timer.timeout.connect(_on_reconnect_timer)
	
	# 启动处理循环
	set_process(true)


func connect_to_server(url: String, session: String) -> bool:
	"""连接到 WebSocket 服务器"""
	base_url = url
	session_id = session
	
	print("[WebGodot] 正在连接到 %s/ws/%s" % [base_url, session_id])
	
	var error = websocket.connect_to_url("%s/ws/%s" % [base_url, session_id])
	
	if error != OK:
		print("[WebGodot] 连接失败: ", error)
		_trigger_reconnect()
		return false
	
	current_reconnect_attempts = 0
	return true


func disconnect_from_server() -> void:
	"""断开连接"""
	if websocket:
		websocket.close()
	is_connected = false
	reconnect_enabled = false
	on_disconnected.call()


func _process(_delta: float) -> void:
	"""处理 WebSocket 消息"""
	if not websocket:
		return
	
	# 轮询 WebSocket
	websocket.poll()
	
	var state = websocket.get_ready_state()
	
	# 处理连接状态变化
	match state:
		WebSocketPeer.STATE_OPEN:
			if not is_connected:
				is_connected = true
				current_reconnect_attempts = 0
				print("[WebGodot] 已连接到服务器")
				on_connected.call()
		
		WebSocketPeer.STATE_CLOSED:
			if is_connected:
				is_connected = false
				print("[WebGodot] 连接已关闭")
				on_disconnected.call()
				_trigger_reconnect()
	
	# 处理接收的消息
	while websocket.get_ready_state() == WebSocketPeer.STATE_OPEN and websocket.get_available_packet_count() > 0:
		var packet = websocket.get_message()
		if packet != null:
			var json_str = packet.get_string_from_utf8()
			_handle_message(json_str)


func _handle_message(json_str: String) -> void:
	"""处理从服务器接收的消息"""
	var json = JSON.new()
	var error = json.parse(json_str)
	
	if error != OK:
		print("[WebGodot] JSON 解析错误: ", json_str)
		return
	
	var message = json.data as Dictionary
	var msg_type = message.get("type", "")
	var msg_id = message.get("id", "")
	var payload = message.get("payload", {})
	var status = message.get("status", "")
	
	# 检查是否是响应消息（有对应的待处理请求）
	if msg_id and msg_id in pending_responses:
		var callback = pending_responses[msg_id]
		pending_responses.erase(msg_id)
		callback.call(message)
		return
	
	# 处理命令消息
	match msg_type:
		"simulation.start":
			var physics = payload.get("physics", {})
			print("[WebGodot] 收到启动仿真命令, physics: ", physics)
			on_simulation_start.call(physics)
			_send_response(msg_id, "success", {"status": "simulation_started"})
		
		"simulation.stop":
			print("[WebGodot] 收到停止仿真命令")
			on_simulation_stop.call()
			_send_response(msg_id, "success", {"status": "simulation_stopped"})
		
		"config.load_robot":
			var robot_config = payload.get("robot_config", {})
			print("[WebGodot] 收到加载机器人配置命令")
			on_load_robot.call(robot_config)
			_send_response(msg_id, "success", {"status": "robot_config_loaded"})
		
		"params.update":
			var params = payload.get("params", {})
			print("[WebGodot] 收到参数更新命令: ", params)
			on_update_params.call(params)
			_send_response(msg_id, "success", {"status": "parameters_updated", "params": params})
		
		"ping":
			_send_response(msg_id, "success", {"timestamp": Time.get_ticks_msec() / 1000.0}, "pong")
		
		_:
			print("[WebGodot] 未知消息类型: ", msg_type)
			_send_response(msg_id, "error", {"error": "Unknown message type: %s" % msg_type})


func _send_response(msg_id: String, response_status: String, payload: Dictionary, response_type: String = "") -> void:
	"""发送响应消息"""
	if response_type.is_empty():
		response_type = "response"
	
	var response = {
		"type": response_type,
		"id": msg_id,
		"timestamp": Time.get_ticks_msec() / 1000.0,
		"payload": payload,
		"status": response_status
	}
	
	var json_str = JSON.stringify(response)
	_send_raw_message(json_str)


func _send_command(command_type: String, payload: Dictionary, callback: Callable = Callable()) -> String:
	"""发送命令并等待响应"""
	var msg_id = str(message_counter)
	message_counter += 1
	
	var message = {
		"type": command_type,
		"id": msg_id,
		"timestamp": Time.get_ticks_msec() / 1000.0,
		"payload": payload
	}
	
	if callback.is_valid():
		pending_responses[msg_id] = callback
	
	var json_str = JSON.stringify(message)
	_send_raw_message(json_str)
	
	return msg_id


func _send_push_message(push_type: String, payload: Dictionary) -> void:
	"""发送推送消息（不期望响应）"""
	var message = {
		"type": push_type,
		"id": str(message_counter),
		"timestamp": Time.get_ticks_msec() / 1000.0,
		"payload": payload,
		"status": "push"
	}
	
	message_counter += 1
	var json_str = JSON.stringify(message)
	_send_raw_message(json_str)


func _send_raw_message(json_str: String) -> void:
	"""直接发送 JSON 消息"""
	if not is_connected or websocket.get_ready_state() != WebSocketPeer.STATE_OPEN:
		print("[WebGodot] 未连接，无法发送消息")
		return
	
	websocket.send_text(json_str)


# === 公共 API ===

func start_simulation(physics_config: Dictionary = {}) -> String:
	"""启动仿真"""
	print("[WebGodot] 发送启动仿真命令")
	return _send_command("simulation.start", {
		"physics": physics_config
	})


func stop_simulation() -> String:
	"""停止仿真"""
	print("[WebGodot] 发送停止仿真命令")
	return _send_command("simulation.stop", {})


func load_robot_config(robot_config: Dictionary) -> String:
	"""加载机器人配置"""
	print("[WebGodot] 发送加载机器人配置命令")
	return _send_command("config.load_robot", {
		"robot_config": robot_config
	})


func update_parameters(params: Dictionary) -> String:
	"""更新参数"""
	print("[WebGodot] 发送参数更新命令")
	return _send_command("params.update", {
		"params": params
	})


func push_telemetry(telemetry_data: Dictionary) -> void:
	"""推送遥测数据流"""
	_send_push_message("telemetry.update", {
		"data": telemetry_data
	})


func push_simulation_status(status: String, details: Dictionary = {}) -> void:
	"""推送仿真状态更新"""
	_send_push_message("simulation.status", {
		"status": status,
		"details": details,
		"timestamp": Time.get_ticks_msec() / 1000.0
	})


func push_error(error_message: String, error_type: String = "runtime") -> void:
	"""推送错误通知"""
	_send_push_message("simulation.error", {
		"error": error_message,
		"error_type": error_type,
		"timestamp": Time.get_ticks_msec() / 1000.0
	})


func ping() -> String:
	"""发送 ping 请求"""
	return _send_command("ping", {})


# === 重连机制 ===

func _trigger_reconnect() -> void:
	"""触发重连"""
	if not reconnect_enabled or current_reconnect_attempts >= max_reconnect_attempts:
		print("[WebGodot] 已达最大重连次数，停止重连")
		return
	
	current_reconnect_attempts += 1
	print("[WebGodot] 将在 %.1f 秒后重新连接 (尝试 %d/%d)" % [
		reconnect_delay,
		current_reconnect_attempts,
		max_reconnect_attempts
	])
	
	reconnect_timer.wait_time = reconnect_delay
	reconnect_timer.start()


func _on_reconnect_timer() -> void:
	"""重连计时器回调"""
	reconnect_timer.stop()
	print("[WebGodot] 尝试重新连接...")
	
	var success = connect_to_server(base_url, session_id)
	if not success and current_reconnect_attempts < max_reconnect_attempts:
		_trigger_reconnect()


func get_connection_status() -> Dictionary:
	"""获取连接状态"""
	return {
		"connected": is_connected,
		"url": "%s/ws/%s" % [base_url, session_id],
		"reconnect_attempts": current_reconnect_attempts,
		"pending_responses": pending_responses.size()
	}


func _exit_tree() -> void:
	"""清理资源"""
	if reconnect_timer:
		reconnect_timer.queue_free()
	if websocket:
		websocket.close()
