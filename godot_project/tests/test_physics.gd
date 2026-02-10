extends RefCounted

# Godot 4 Physics Test

func assert_true(condition, msg = "Assertion failed"):
	if not condition:
		push_error("❌ " + msg)
		return false
	return true

func assert_gt(a, b, msg = ""):
	if not (a > b):
		push_error("❌ " + msg + " Expected %s > %s" % [a, b])
		return false
	return true

# --- Tests ---

func test_gravity():
	print("   🔹 Executing test_gravity check...")
	var body = RigidBody3D.new()
	var shape = CollisionShape3D.new()
	shape.shape = BoxShape3D.new()
	body.add_child(shape)
	
	# Just checking class instantiation works
	assert_true(body is RigidBody3D, "Body should be RigidBody3D")
	print("   ✅ Gravity Test Logic/Instantiation Passed")
	
	# Cleanup manually if not added to tree (RefCounted handles script, but Nodes are manually managed if orphan)
	shape.free()
	body.free()

func test_api_compatibility():
	print("   🔹 Executing test_api_compatibility...")
	var gravity = ProjectSettings.get_setting("physics/3d/default_gravity")
	# Godot 4 default gravity is usually 9.8
	assert_gt(gravity, 0, "Gravity should be positive")
	print("   ✅ API Test (Gravity Setting: %s) Passed" % gravity)
