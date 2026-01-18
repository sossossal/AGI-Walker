# environment_ui.gd
# 环境参数控制UI
extends Control

@onready var env_controller = get_node("/root/EnvironmentController")
@onready var material_lib = get_node("/root/GroundMaterialLibrary")

# UI 控件引用
@onready var gravity_slider: HSlider = $Panel/VBox/GravitySlider
@onready var gravity_label: Label = $Panel/VBox/GravityLabel
@onready var air_density_slider: HSlider = $Panel/VBox/AirDensitySlider
@onready var air_density_label: Label = $Panel/VBox/AirDensityLabel
@onready var temperature_slider: HSlider = $Panel/VBox/TemperatureSlider
@onready var temperature_label: Label = $Panel/VBox/TemperatureLabel
@onready var preset_option: OptionButton = $Panel/VBox/PresetOption
@onready var material_option: OptionButton = $Panel/VBox/MaterialOption
@onready var info_label: RichTextLabel = $Panel/VBox/InfoLabel

var ground_body: StaticBody3D

func _ready():
	_setup_ui()
	_connect_signals()
	_update_display()

func _setup_ui():
	# 设置滑块范围
	gravity_slider.min_value = 0.0
	gravity_slider.max_value = 30.0
	gravity_slider.value = 9.81
	
	air_density_slider.min_value = 0.0
	air_density_slider.max_value = 3.0
	air_density_slider.value = 1.225
	air_density_slider.step = 0.01
	
	temperature_slider.min_value = -150.0
	temperature_slider.max_value = 150.0
	temperature_slider.value = 25.0
	
	# 设置环境预设下拉菜单
	preset_option.clear()
	preset_option.add_item("地球", 0)
	preset_option.add_item("月球", 1)
	preset_option.add_item("火星", 2)
	preset_option.add_item("木星", 3)
	preset_option.add_item("自定义", 4)
	
	# 设置材质下拉菜单
	material_option.clear()
	if material_lib:
		var materials = material_lib.list_materials()
		for i in range(materials.size()):
			var mat_name = materials[i]
			material_option.add_item(mat_name.capitalize(), i)

func _connect_signals():
	gravity_slider.value_changed.connect(_on_gravity_changed)
	air_density_slider.value_changed.connect(_on_air_density_changed)
	temperature_slider.value_changed.connect(_on_temperature_changed)
	preset_option.item_selected.connect(_on_preset_selected)
	material_option.item_selected.connect(_on_material_selected)
	
	if env_controller:
		env_controller.environment_changed.connect(_on_environment_changed)

func _on_gravity_changed(value: float):
	if env_controller:
		env_controller.set_gravity(value)
	_update_display()

func _on_air_density_changed(value: float):
	if env_controller:
		env_controller.set_air_density(value)
	_update_display()

func _on_temperature_changed(value: float):
	if env_controller:
		env_controller.set_temperature(value)
	_update_display()

func _on_preset_selected(index: int):
	var presets = ["earth", "moon", "mars", "jupiter", "custom"]
	if env_controller and index < presets.size():
		env_controller.load_preset(presets[index])
		_sync_sliders_with_controller()

func _on_material_selected(index: int):
	if material_lib and ground_body:
		var materials = material_lib.list_materials()
		if index < materials.size():
			material_lib.apply_material(ground_body, materials[index])

func _on_environment_changed(param_name: String, new_value: float):
	_update_display()

func _sync_sliders_with_controller():
	if env_controller:
		gravity_slider.value = env_controller.gravity
		air_density_slider.value = env_controller.air_density
		temperature_slider.value = env_controller.temperature

func _update_display():
	if env_controller:
		gravity_label.text = "重力: %.2f m/s²" % env_controller.gravity
		air_density_label.text = "空气密度: %.3f kg/m³" % env_controller.air_density
		temperature_label.text = "温度: %.1f °C" % env_controller.temperature
		
		# 更新信息面板
		var info_text = "[b]当前环境状态[/b]\n\n"
		info_text += "重力: [color=cyan]%.2f m/s²[/color]\n" % env_controller.gravity
		info_text += "空气: [color=cyan]%.3f kg/m³[/color]\n" % env_controller.air_density
		info_text += "温度: [color=cyan]%.1f °C[/color]\n" % env_controller.temperature
		info_text += "风速: [color=cyan]%.2f m/s[/color]\n" % env_controller.wind_velocity.length()
		
		# 显示与地球的对比
		var gravity_percent = (env_controller.gravity / 9.81) * 100.0
		info_text += "\n相对地球: [color=yellow]%.1f%%[/color] 重力" % gravity_percent
		
		info_label.text = info_text

func set_ground_body(body: StaticBody3D):
	ground_body = body

## 键盘快捷键
func _input(event):
	if event is InputEventKey and event.pressed:
		match event.keycode:
			KEY_F1:
				visible = not visible
