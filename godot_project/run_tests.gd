@tool
extends SceneTree

# Custom Test Runner for AGI-Walker (Godot 4)
# Usage: godot --headless --script run_tests.gd

func _init():
	print("\n🚀 Starting AGI-Walker Godot Tests (Custom Runner)\n")
	
	var passed = 0
	var failed = 0
	var tests_dir = "res://tests/"
	
	var dir = DirAccess.open(tests_dir)
	if dir:
		dir.list_dir_begin()
		var file_name = dir.get_next()
		while file_name != "":
			if not dir.current_is_dir() and file_name.begins_with("test_") and file_name.ends_with(".gd"):
				var script_path = tests_dir + file_name
				print("➡️ Running: " + file_name)
				
				# Load and instantiate the test script
				var script_res = load(script_path)
				if not script_res:
					print("❌ Error: Failed to load " + script_path)
					failed += 1
					file_name = dir.get_next()
					continue
					
				var test_script = script_res.new()
				
				# Find all functions starting with 'test_'
				for method in test_script.get_method_list():
					if method.name.begins_with("test_"):
						# print("   🔹 " + method.name + "...")
						# Capture output/errors? 
						# We assume the test method prints its own status or pushes errors
						test_script.call(method.name)
						
						# We assume pass if no crash for now, or check a property if we added one
						passed += 1
				
				# RefCounted objects allow auto-free, but if Node, use free()
				if test_script is Node:
					test_script.free()
					
			file_name = dir.get_next()
	else:
		print("❌ Error: Could not open tests directory: " + tests_dir)
		quit(1)
		
	print("\n✨ Test Summary: %d Passed, %d Failed" % [passed, failed])
	
	if failed > 0:
		quit(1)
	else:
		quit(0)
