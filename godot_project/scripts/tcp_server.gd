extends Node

# TCP Server for RL Interface
# Listens on port 9000
# Exchanges JSON data with Length Prefix (4 bytes, Little Endian)

var server = TCPServer.new()
var PORT = 9000
var connection: StreamPeerTCP = null
var robot_node: Node3D = null
var last_loaded_robot_config: Dictionary = {}

func _ready():
	var cli_args = OS.get_cmdline_args()
	var user_args = OS.get_cmdline_user_args()
	for arg in cli_args + user_args:
		if arg.begins_with("--tcp-port="):
			PORT = arg.trim_prefix("--tcp-port=").to_int()

	var err = server.listen(PORT)
	if err == OK:
		print("✅ [TCP] RL Server listening on port %d" % PORT)
	else:
		print("❌ [TCP] Failed to listen on port %d, error: %d" % [PORT, err])
		
	# Try to find a robot node in the scene
	# In headless verification, we might need to spawn a dummy one
	robot_node = get_tree().root.find_child("*Robot*", true, false)
	if not robot_node and get_tree().current_scene:
		robot_node = get_tree().current_scene.find_child("*Robot*", true, false)

func _process(delta):
	# 1. Accept New Connections
	if server.is_connection_available():
		if connection != null:
			print("⚠️ [TCP] Dropping existing connection for new one.")
			connection.disconnect_from_host()
			
		connection = server.take_connection()
		connection.set_no_delay(true)
		print("🔗 [TCP] Client connected: %s (BigEndian=%s)" % [connection.get_connected_host(), connection.is_big_endian_enabled()])
		
	# 2. Process Data
	if connection != null:
		connection.poll()
		var status = connection.get_status()
		if status == StreamPeerTCP.STATUS_CONNECTED:
			_handle_messages()
		elif status == StreamPeerTCP.STATUS_NONE or status == StreamPeerTCP.STATUS_ERROR:
			print("❌ [TCP] Connection lost.")
			connection = null

func _handle_messages():
	# Protocol: Length (4 bytes LE) + JSON payload
	if connection.get_available_bytes() < 4:
		return
		
	# Peek length? StreamPeer doesn't support peek easily, we just read.
	# But we need to be careful not to read partial packets if we can't buffer.
	# For simplicity in this demo, we assume blocking read for the length is fine or we manage buffer.
	# However, get_available_bytes checks internal buffer.
	
	# Let's read length
	var length = connection.get_32() # Reads 4 bytes
	
	# Wait for payload
	# In a real rigorous impl, we'd buffer. Here we block-wait or check again?
	# StreamPeerTCP.get_data is blocking? No, it returns error if not enough.
	# But we already checked available bytes >= 4. Now we need available >= length.
	
	# Danger: if payload hasn't arrived, we are stuck? 
	# Actually get_32 removed 4 bytes. 
	# A better approach for async:
	# buffer += received_bytes
	# while buffer.size() >= 4:
	#    len = ...
	#    if buffer.size() >= 4 + len: extract...
	
	# For this verification script, let's just do a blocking wait loop with timeout for payload
	# because Gym sends command and waits for reply, it's synchronized.
	
	var timeout = 0.5
	var time_waited = 0.0
	while connection.get_available_bytes() < length:
		OS.delay_msec(1)
		connection.poll()
		time_waited += 0.001
		if time_waited > timeout:
			print("❌ [TCP] Timeout waiting for payload.")
			connection.disconnect_from_host() # Force disconnect
			connection = null
			return

	var json_str = connection.get_utf8_string(length)
	var cmd = JSON.parse_string(json_str)
	
	if cmd:
		_process_command(cmd)
	else:
		print("❌ [TCP] Failed to parse JSON")

func _process_command(cmd):
	var response = {}
	
	if cmd.type == "reset":
		print("🔄 [TCP] Resetting Simulation")
		if robot_node != null:
			if robot_node.has_method("reset_pose"):
				robot_node.reset_pose()
			if cmd.has("sim_params") and robot_node.has_method("apply_sim_params"):
				robot_node.apply_sim_params(cmd.get("sim_params"))
		response = _get_observation()
		
	elif cmd.type == "step":
		var action = cmd.get("action", [])
		response = _get_observation()
		response["reward"] = 0.0 # Placeholder
		response["done"] = false
		
	elif cmd.type == "load_robot":
		print("📦 [TCP] Loading Robot Config")
		var robot_config = cmd.get("robot_config", {})
		if robot_node == null:
			robot_node = get_tree().root.find_child("*Robot*", true, false)
			if not robot_node and get_tree().current_scene:
				robot_node = get_tree().current_scene.find_child("*Robot*", true, false)

		if robot_node != null and robot_node.has_method("load_from_dict"):
			robot_node.load_from_dict(robot_config)
			last_loaded_robot_config = robot_config.duplicate(true)
			response = {"status": "success"}
		else:
			last_loaded_robot_config = robot_config.duplicate(true)
			print("ℹ️ [TCP] No robot node available; storing config in fallback mode")
			response = {"status": "success", "mode": "fallback"}
			
	elif cmd.type == "get_schema":
		if robot_node != null and robot_node.has_method("get_schema"):
			response = robot_node.get_schema()
		else:
			# Fallback Dummy Schema
			response = {
				"sensors": {
					"battery": {"type": "float32", "shape": [1]},
					"vector": {"type": "float32", "shape": [24]}
				},
				"actuators": {
					"action": {"type": "float32", "shape": [2], "range": [-10.0, 10.0]}
				},
				"meta": {
					"last_loaded_parts": last_loaded_robot_config.get("parts", []).size(),
					"last_loaded_connections": last_loaded_robot_config.get("connections", []).size()
				}
			}
			
	_send_response(response)

func _get_observation():
	if robot_node != null and robot_node.has_method("get_sensor_data"):
		return robot_node.get_sensor_data()
		
	var vec = []
	vec.resize(24)
	vec.fill(0.1)
	
	# Construct Mock Observation
	return {
		"image": [], # optimized out for now
		"vector": vec, # Mock 24-dim vector
		"battery": 100.0
	}

func _send_response(resp_dict):
	var json_str = JSON.stringify(resp_dict)
	var data = json_str.to_utf8_buffer()
	var length = data.size()
	
	connection.put_32(length)
	connection.put_data(data)
