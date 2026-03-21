# agent_panel.gd — Godot 编辑器插件：Agent 侧边栏
# 在 Godot 编辑器内嵌入 Godot Studio Agent 对话面板
@tool
extends EditorPlugin

const PANEL_NAME = "Agent"
var panel: Control

func _enter_tree() -> void:
	panel = _build_panel()
	add_control_to_dock(DOCK_SLOT_RIGHT_UR, panel)
	print("🎮 Godot Studio Agent 插件已加载")

func _exit_tree() -> void:
	if panel:
		remove_control_from_docks(panel)
		panel.queue_free()

func _build_panel() -> Control:
	var vbox = VBoxContainer.new()
	vbox.name = "GodotStudioAgent"

	# 标题
	var title = Label.new()
	title.text = "🎮 Studio Agent"
	title.add_theme_font_size_override("font_size", 14)
	vbox.add_child(title)

	# 输入框
	var input = TextEdit.new()
	input.placeholder_text = "描述你需要什么…"
	input.custom_minimum_size = Vector2(0, 80)
	input.name = "Input"
	vbox.add_child(input)

	# 发送按钮
	var btn = Button.new()
	btn.text = "▶ 发送到 Agent"
	btn.pressed.connect(func(): _send_command(input.text))
	vbox.add_child(btn)

	# API 地址
	var api_label = Label.new()
	api_label.text = "API: http://localhost:8000"
	api_label.modulate = Color(0.6, 0.6, 0.6)
	api_label.add_theme_font_size_override("font_size", 11)
	vbox.add_child(api_label)

	# 结果输出
	var result = RichTextLabel.new()
	result.name = "Result"
	result.bbcode_enabled = true
	result.custom_minimum_size = Vector2(0, 200)
	result.size_flags_vertical = Control.SIZE_EXPAND_FILL
	vbox.add_child(result)

	# 快速按钮组
	var hbox = HBoxContainer.new()
	for label_cmd in [["💻 移动", "生成 2D 玩家移动脚本"], ["💾 存档", "生成存档系统"], ["🤖 AI", "为敌人创建巡逻追击 AI"]]:
		var qbtn = Button.new()
		qbtn.text = label_cmd[0]
		var cmd = label_cmd[1]
		qbtn.pressed.connect(func():
			input.text = cmd
			_send_command(cmd)
		)
		hbox.add_child(qbtn)
	vbox.add_child(hbox)

	# Skills 库分隔线
	var sep = HSeparator.new()
	vbox.add_child(sep)

	var skill_title = Label.new()
	skill_title.text = "📚 Godot Skills 图谱"
	skill_title.add_theme_font_size_override("font_size", 12)
	vbox.add_child(skill_title)

	var skill_hbox = HBoxContainer.new()
	var skill_dropdown = OptionButton.new()
	skill_dropdown.name = "SkillDropdown"
	skill_dropdown.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	skill_hbox.add_child(skill_dropdown)

	var apply_btn = Button.new()
	apply_btn.text = "⚡ 生成预设"
	apply_btn.pressed.connect(func(): _apply_skill(skill_dropdown))
	skill_hbox.add_child(apply_btn)
	vbox.add_child(skill_hbox)

	_fetch_skills_list()

	return vbox

func _send_command(command: String) -> void:
	if command.strip_edges().is_empty():
		return
	var result_label = panel.find_child("Result")
	if result_label:
		result_label.text = "[color=yellow]⏳ 正在请求 Agent...[/color]"

	var http = HTTPRequest.new()
	panel.add_child(http)
	http.request_completed.connect(func(result, code, headers, body):
		var text = body.get_string_from_utf8()
		var json = JSON.parse_string(text)
		if json and result_label:
			var msg = json.get("message", "")
			var code_data = json.get("data", {}).get("code", "")
			var script_name = json.get("data", {}).get("script_name", "generated_script.gd")
			var display = "[color=cyan]%s[/color]" % msg
			if code_data:
				# 1. 尝试保存并重新加载为资源
				var save_path = "res://" + script_name
				var file = FileAccess.open(save_path, FileAccess.WRITE)
				if file:
					file.store_string(code_data)
					file.close()
					display += "\n[color=yellow]💾 已保存至 " + save_path + "[/color]"
					
					# 必须强制扫描文件系统让 ResourceLoader 能找到新文件
					EditorInterface.get_resource_filesystem().scan()
					var script_res = load(save_path)
					
					# 2. 尝试挂载到选中的节点
					var selection = EditorInterface.get_selection().get_selected_nodes()
					if selection.size() > 0:
						var target = selection[0]
						if target.get_script() == null or target.get_script().resource_path != save_path:
							target.set_script(script_res)
							display += "\n[color=green]🔗 已挂载至: " + target.name + "[/color]"
					else:
						display += "\n[color=gray](未选中节点，代码已保存但不挂载)[/color]"

				display += "\n[color=gray]--- 代码预览 ---[/color]\n" + code_data.substr(0, 200) + ("..." if code_data.length() > 200 else "")
			result_label.text = display
		elif result_label:
			result_label.text = "[color=red]请求失败[/color]"
		http.queue_free()
	)

	var body = JSON.stringify({"command": command})
	http.request(
		"http://localhost:8000/execute",
		["Content-Type: application/json"],
		HTTPClient.METHOD_POST, body
	)

func _fetch_skills_list() -> void:
	var http = HTTPRequest.new()
	panel.add_child(http)
	http.request_completed.connect(func(result, code, headers, body):
		var dropdown = panel.find_child("SkillDropdown")
		if code == 200 and dropdown:
			var text = body.get_string_from_utf8()
			var json = JSON.parse_string(text)
			dropdown.clear()
			if json and json.get("status") == "success":
				for skill in json.get("skills", []):
					dropdown.add_item(skill.get("name") + " (" + skill.get("id") + ")")
					dropdown.set_item_metadata(dropdown.get_item_count() - 1, skill.get("id"))
			else:
				dropdown.add_item("加载技能失败")
		http.queue_free()
	)
	http.request("http://localhost:8000/api/godot_skills/list")

func _apply_skill(dropdown: OptionButton) -> void:
	if dropdown.get_item_count() == 0: return
	var idx = dropdown.selected
	var skill_id = dropdown.get_item_metadata(idx)
	if not skill_id: return

	var result_label = panel.find_child("Result")
	if result_label:
		result_label.text = "[color=yellow]⏳ 正在提取技能图谱并拼装场景...[/color]"

	var http = HTTPRequest.new()
	panel.add_child(http)
	http.request_completed.connect(func(result, code, headers, body):
		if code == 200:
			var text = body.get_string_from_utf8()
			var json = JSON.parse_string(text)
			if json and json.get("status") == "success":
				_build_skill_scene(json.get("data", {}), result_label)
			elif result_label:
				result_label.text = "[color=red]技能提取失败[/color]"
		elif result_label:
			result_label.text = "[color=red]请求失败，状态码: %s[/color]" % code
		http.queue_free()
	)
	var body = JSON.stringify({"skill_id": skill_id})
	http.request(
		"http://localhost:8000/api/godot_skills/apply",
		["Content-Type: application/json"],
		HTTPClient.METHOD_POST, body
	)

func _build_skill_scene(data: Dictionary, result_label: RichTextLabel) -> void:
	var scene_setup = data.get("scene_setup", {})
	var script_name = data.get("script_name", "")
	var script_code = data.get("script_code", "")
	var display = "[color=cyan]✨ 技能生成完成: %s[/color]\n" % data.get("name", "")

	var root_node = null
	if not scene_setup.is_empty():
		root_node = _create_node_recursive(scene_setup)

	if root_node:
		var selection = EditorInterface.get_selection().get_selected_nodes()
		var parent = get_tree().edited_scene_root
		if selection.size() > 0:
			parent = selection[0]
		
		if parent:
			parent.add_child(root_node)
			_set_owner_recursive(root_node, get_tree().edited_scene_root)
			display += "[color=green]✅ 已在场景树中编排节点拓扑[/color]\n"
		else:
			display += "[color=red]❌ 找不到正在编辑的场景根节点[/color]\n"

	if script_code and root_node:
		var save_path = "res://" + script_name
		var file = FileAccess.open(save_path, FileAccess.WRITE)
		if file:
			file.store_string(script_code)
			file.close()
			EditorInterface.get_resource_filesystem().scan()
			var script_res = load(save_path)
			root_node.set_script(script_res)
			display += "[color=yellow]💾 注入并绑定专属机制脚本: %s[/color]" % script_name

	result_label.text = display

func _create_node_recursive(setup: Dictionary) -> Node:
	var type_str = setup.get("type", "Node")
	var node = ClassDB.instantiate(type_str)
	if node == null:
		node = Node.new()
	node.name = setup.get("name", type_str)
	
	var props = setup.get("properties", {})
	for k in props:
		var v = props[k]
		if typeof(v) == TYPE_DICTIONARY and v.has("x") and v.has("y"):
			if v.has("z"):
				node.set(k, Vector3(v.x, v.y, v.z))
			else:
				node.set(k, Vector2(v.x, v.y))
		else:
			node.set(k, v)

	var children = setup.get("children", [])
	for child_def in children:
		var child_node = _create_node_recursive(child_def)
		node.add_child(child_node)

	return node

func _set_owner_recursive(node: Node, owner_node: Node) -> void:
	if node != owner_node:
		node.owner = owner_node
	for child in node.get_children():
		_set_owner_recursive(child, owner_node)
