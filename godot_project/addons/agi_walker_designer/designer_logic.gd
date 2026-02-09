@tool
extends Control

@onready var name_input = $VBoxContainer/GridContainer/NameInput
@onready var type_option = $VBoxContainer/GridContainer/TypeOption
@onready var scenario_option = $VBoxContainer/GridContainer/ScenarioOption
@onready var height_spin = $VBoxContainer/GridContainer/HeightSpin

# Custom Fieldds
@onready var mass_spin = $VBoxContainer/GridCustom/MassSpin
@onready var mat_option = $VBoxContainer/GridCustom/MatOption
@onready var torque_spin = $VBoxContainer/GridCustom/TorqueSpin
@onready var friction_spin = $VBoxContainer/GridCustom/FrictionSpin

@onready var status_label = $VBoxContainer/StatusLabel

var python_path = "python"
var script_path = "D:/新建文件夹/AGI-Walker/quick_design.py"

func _on_generate_btn_pressed():
	status_label.text = "[color=yellow]Generating...[/color]"
	
	var r_name = name_input.text
	var r_type = "biped" if type_option.selected == 0 else "quadruped"
	var r_scenario = ""
	if scenario_option.selected == 0: r_scenario = "eco"
	elif scenario_option.selected == 1: r_scenario = "performance"
	else: r_scenario = "custom"
	
	var r_height = str(height_spin.value)
	
	var args = [
		script_path,
		"--non-interactive",
		"--name", r_name,
		"--type", r_type,
		"--scenario", r_scenario,
		"--height", r_height
	]
	
	# Optional Overrides
	if mass_spin.value > 0:
		args.append("--mass")
		args.append(str(mass_spin.value))
		
	if torque_spin.value > 0:
		args.append("--torque")
		args.append(str(torque_spin.value))
		
	if friction_spin.value >= 0:
		args.append("--friction")
		args.append(str(friction_spin.value))
		
	# Material
	var mat_idx = mat_option.selected
	if mat_idx == 1: args.append_array(["--material", "aluminum"])
	elif mat_idx == 2: args.append_array(["--material", "carbon_fiber"])
	elif mat_idx == 3: args.append_array(["--material", "plastic"])
	
	status_label.text += "\nRunning: " + str(args)
	
	var output = []
	var exit_code = OS.execute(python_path, args, output, true)
	
	if exit_code == 0:
		status_label.text = "[color=green]Success![/color]\n"
		status_label.text += str(output[0])
		# Trigger re-import
		EditorInterface.get_resource_filesystem().scan()
	else:
		status_label.text = "[color=red]Failed (Code " + str(exit_code) + ")[/color]\n"
		status_label.text += str(output)
