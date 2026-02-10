extends Node3D

# RL Server Runner for Headless Verification
# Usage: godot --headless run_rl_server.tscn

# var tcp_server_script = preload("res://scripts/tcp_server.gd") # Preload causing issues?

func _ready():
	print("\n🚀 Starting RL Server (Headless Mode)...")
	
	var script = load("res://scripts/tcp_server.gd")
	if not script:
		print("❌ Failed to load script: res://scripts/tcp_server.gd")
		return
		
	var server_node = script.new()
	if not server_node:
		print("❌ Failed to instantiate server node!")
		return
		
	add_child(server_node)
	print("   ✅ Server node instantiated.")

func _process(delta):
	# Keep alive
	pass
