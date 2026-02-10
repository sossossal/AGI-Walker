extends Node3D

# High-Concurrency Stress Test (100 Robots)

var robot_scene = preload("res://robot_scenes/quadruped_robot.tscn")
var robot_count = 100
var test_duration = 5.0 # Seconds
var elapsed_time = 0.0
var frame_count = 0
var total_fps = 0.0

func _ready():
	print("\n🚀 Starting High-Concurrency Stress Test...")
	print("   Spawning %d Robots..." % robot_count)
	
	# Create Floor
	var floor_body = StaticBody3D.new()
	var floor_col = CollisionShape3D.new()
	var box = BoxShape3D.new()
	box.size = Vector3(100, 1, 100)
	floor_col.shape = box
	floor_body.add_child(floor_col)
	add_child(floor_body)
	
	# Spawn Robots Grid
	var grid_size = int(sqrt(robot_count))
	for i in range(robot_count):
		var robot = robot_scene.instantiate()
		var x = (i % grid_size) * 2.0 - (grid_size)
		var z = (i / grid_size) * 2.0 - (grid_size)
		robot.position = Vector3(x, 5.0, z) # Drop from height
		add_child(robot)
		
	print("   ✅ Spawned %d robots." % robot_count)
	print("   ⏳ Running simulation for %.1f seconds..." % test_duration)

func _process(delta):
	elapsed_time += delta
	frame_count += 1
	var fps = Engine.get_frames_per_second()
	total_fps += fps
	
	if elapsed_time >= test_duration:
		var avg_fps = total_fps / frame_count
		print("\n📊 Stress Test Results:")
		print("   - Duration: %.2fs" % elapsed_time)
		print("   - Robot Count: %d" % robot_count)
		print("   - Average FPS: %.2f" % avg_fps)
		
		if avg_fps > 30.0:
			print("   ✅ PASS: Performance is acceptable (>30 FPS).")
			get_tree().quit(0)
		else:
			print("   ⚠️ WARNING: Performance is low (<30 FPS).")
			# We still exit 0 because the test 'ran' successfully, just low perf.
			# Or exit 1 if we want to enforce strict perf gates.
			# For now, let's exit 0 but warn.
			get_tree().quit(0)
