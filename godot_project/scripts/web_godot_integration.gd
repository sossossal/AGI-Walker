"""
Web-Godot 集成示例场景脚本

演示如何在 Godot 场景中集成 WebGodotClient，
处理来自 Web 面板的命令，并推送遥测数据。

使用方法:
  1. 在 Godot 场景中创建一个 Node
  2. 附加此脚本
  3. 在 _ready() 中初始化客户端
  4. 在 _process() 中更新遥测数据
"""

extends Node

class_name WebGodotIntegration

# WebSocket 客户端
var web_client: WebGodotClient = null

# 仿真状态
var is_simulating: bool = false
var simulation_time: float = 0.0
var target_speed: float = 1.0
var robot_position: Vector3 = Vector3.ZERO
var robot_velocity: Vector3 = Vector3.ZERO

# 遥测发送频率
var telemetry_interval: float = 0.05  # 20 Hz
var telemetry_accumulator: float = 0.0

# 物理配置
var gravity: float = 9.81
var timestep: float = 0.01

# 机器人引用
var robot: Node3D = null


func _ready() -> void:
	"""初始化 WebSocket 连接"""
	print("[集成] 初始化 Web-Godot 集成...")
	
	# 创建 WebSocket 客户端
	web_client = WebGodotClient.new()
	add_child(web_client)
	
	# 注册事件回调
	web_client.on_connected = func():
		print("[集成] 已连接到 Web 服务器")
	
	web_client.on_disconnected = func():
		print("[集成] 已断开连接")
		_on_disconnected()
	
	web_client.on_simulation_start = func(physics: Dictionary):
		_on_start_simulation(physics)
	
	web_client.on_simulation_stop = func():
		_on_stop_simulation()
	
	web_client.on_load_robot = func(config: Dictionary):
		_on_load_robot(config)
	
	web_client.on_update_params = func(params: Dictionary):
		_on_update_params(params)
	
	web_client.on_error = func(error: String):
		print("[集成] 错误: ", error)
	
	# 连接到服务器
	web_client.connect_to_server("ws://localhost:8000", "godot_scene_1")
	
	# 查找机器人节点（假设存在）
	robot = get_node_or_null("../Robot")


func _process(delta: float) -> void:
	"""处理每一帧"""
	if not is_simulating:
		return
	
	simulation_time += delta
	
	# 更新机器人物理（简化示例）
	_update_robot_physics(delta)
	
	# 累加遥测间隔
	telemetry_accumulator += delta
	if telemetry_accumulator >= telemetry_interval:
		telemetry_accumulator -= telemetry_interval
		_send_telemetry()


func _update_robot_physics(delta: float) -> void:
	"""更新机器人物理模拟（简化示例）"""
	# 这里应该调用实际的物理引擎
	# 示例: 简单的速度更新
	
	var acceleration = Vector3(target_speed * 5.0 * cos(simulation_time * 2.0), 0, 0)
	robot_velocity += acceleration * delta
	robot_velocity *= 0.95  # 阻尼
	
	robot_position += robot_velocity * delta
	
	# 更新机器人位置
	if robot:
		robot.position = robot_position


func _send_telemetry() -> void:
	"""发送遥测数据到 Web 面板"""
	if not web_client or not web_client.is_connected:
		return
	
	var telemetry_data = {
		"simulation_time": simulation_time,
		"position": {
			"x": robot_position.x,
			"y": robot_position.y,
			"z": robot_position.z
		},
		"velocity": {
			"x": robot_velocity.x,
			"y": robot_velocity.y,
			"z": robot_velocity.z
		},
		"sensors": {
			"speed": robot_velocity.length(),
			"height": robot_position.y
		},
		"simulation_status": {
			"running": is_simulating,
			"fps": Engine.get_frames_per_second(),
			"target_speed": target_speed
		}
	}
	
	web_client.push_telemetry(telemetry_data)


# === 命令处理函数 ===

func _on_start_simulation(physics: Dictionary) -> void:
	"""处理启动仿真命令"""
	print("[集成] 启动仿真, 物理配置: ", physics)
	
	gravity = physics.get("gravity", 9.81)
	timestep = physics.get("timestep", 0.01)
	
	is_simulating = true
	simulation_time = 0.0
	
	web_client.push_simulation_status("running", {
		"message": "Simulation started",
		"gravity": gravity,
		"timestep": timestep
	})


func _on_stop_simulation() -> void:
	"""处理停止仿真命令"""
	print("[集成] 停止仿真")
	
	is_simulating = false
	
	web_client.push_simulation_status("stopped", {
		"message": "Simulation stopped",
		"total_time": simulation_time
	})


func _on_load_robot(config: Dictionary) -> void:
	"""处理加载机器人配置命令"""
	print("[集成] 加载机器人配置: ", config)
	
	var parts = config.get("parts", [])
	var connections = config.get("connections", [])
	
	print("[集成] 机器人部分数: ", parts.size())
	print("[集成] 连接数: ", connections.size())
	
	# 这里应该创建或更新机器人模型
	# 示例: 创建简单的胶囊形机器人
	if robot == null:
		robot = Node3D.new()
		robot.name = "RobotModel"
		add_sibling(robot)
		
		var capsule = CapsuleMesh.new()
		var mesh_instance = MeshInstance3D.new()
		mesh_instance.mesh = capsule
		robot.add_child(mesh_instance)
	
	web_client.push_simulation_status("robot_loaded", {
		"message": "Robot configuration loaded",
		"parts_count": parts.size()
	})


func _on_update_params(params: Dictionary) -> void:
	"""处理参数更新命令"""
	print("[集成] 更新参数: ", params)
	
	if params.has("target_speed"):
		target_speed = params["target_speed"]
		print("[集成] 目标速度更新为: ", target_speed)
	
	if params.has("gravity"):
		gravity = params["gravity"]
		print("[集成] 重力加速度更新为: ", gravity)
	
	web_client.push_simulation_status("params_updated", {
		"message": "Parameters updated",
		"params": params
	})


func _on_disconnected() -> void:
	"""处理断开连接"""
	if is_simulating:
		print("[集成] 连接断开，停止仿真")
		is_simulating = false


func get_status() -> Dictionary:
	"""获取集成状态"""
	var web_status = {} if not web_client else web_client.get_connection_status()
	
	return {
		"web_connected": web_client.is_connected if web_client else false,
		"simulating": is_simulating,
		"simulation_time": simulation_time,
		"robot_position": [robot_position.x, robot_position.y, robot_position.z],
		"target_speed": target_speed,
		"web_status": web_status
	}


func _exit_tree() -> void:
	"""清理资源"""
	if web_client:
		web_client.disconnect_from_server()
