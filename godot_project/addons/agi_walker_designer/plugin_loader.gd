@tool
extends EditorPlugin

var dock

func _enter_tree():
	# Load the dock scene and instance it
	dock = preload("res://addons/agi_walker_designer/designer_dock.tscn").instantiate()
	
	# Add the loaded scene to the docks
	add_control_to_dock(DOCK_SLOT_LEFT_UL, dock)

func _exit_tree():
	# Clean-up of the plugin goes here
	remove_control_from_docks(dock)
	dock.free()
