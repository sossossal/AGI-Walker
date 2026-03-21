@tool
extends SceneTree

func _init():
    print("Testing Godot Skill Headless Execution...")
    
    # 1. 模拟收到 "/api/godot_skills/apply" 的响应 (ai_patrol_enemy)
    var setup_data = {
      "type": "CharacterBody2D",
      "name": "PatrolEnemy",
      "children": [
        {"type": "CollisionShape2D", "name": "BodyShape"},
        {"type": "NavigationAgent2D", "name": "NavAgent"},
        {
          "type": "Area2D",
          "name": "VisionArea",
          "children": [{"type": "CollisionShape2D", "name": "VisionShape"}]
        }
      ]
    }
    var script_name = "test_ai_patrol_enemy.gd"
    var script_code = "extends CharacterBody2D\npass\n"

    # 2. 模拟正在编辑的根节点
    var root = Node2D.new()
    root.name = "TestLevel"
    
    # 3. 提取 Node Builder Logic (copied from agent_panel.gd)
    var enemy_node = _create_node_recursive(setup_data)
    root.add_child(enemy_node)
    _set_owner_recursive(enemy_node, root)
    
    # 4. Save Script
    var save_path = "res://" + script_name
    var file = FileAccess.open(save_path, FileAccess.WRITE)
    if file:
        file.store_string(script_code)
        file.close()
        
    print("----- Generated Tree -----")
    root.print_tree_pretty()
    
    # 5. Save Scene to verify it's persistent
    var packed = PackedScene.new()
    packed.pack(root)
    ResourceSaver.save(packed, "res://test_level_with_skill.tscn")
    print("Scene saved to res://test_level_with_skill.tscn")
    
    quit()

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
