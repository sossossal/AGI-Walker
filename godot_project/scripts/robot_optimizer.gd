extends Node
## 机器人物理参数优化器
## 自动应用优化的物理参数到机器人

@export var auto_optimize := true # 启动时自动优化

@onready var robot = get_node("/root/Main/Robot")
@onready var ground = get_node_or_null("/root/Main/Ground")


func _ready():
	if auto_optimize:
		await get_tree().process_frame # 等待一帧确保节点已加载
		optimize_all()


func optimize_all():
	"""优化所有物理参数"""
	print("\n🔧 开始优化物理参数...")
	
	# 打印配置摘要
	PhysicsConfig.print_config_summary()
	
	# 优化地面
	if ground:
		optimize_ground()
	else:
		push_warning("⚠️ 未找到Ground节点")
	
	# 优化机器人
	if robot and robot.is_scene_ready:
		optimize_robot()
	else:
		push_warning("⚠️ 机器人场景未就绪，跳过优化")
	
	print("✅ 物理参数优化完成\n")


func optimize_ground():
	"""优化地面物理材质"""
	PhysicsConfig.apply_to_ground(ground)


func optimize_robot():
	"""优化机器人参数"""
	# 躯干
	if robot.torso:
		PhysicsConfig.apply_to_torso(robot.torso)
	
	# 左腿
	if robot.left_leg:
		PhysicsConfig.apply_to_leg(robot.left_leg)
	
	# 右腿
	if robot.right_leg:
		PhysicsConfig.apply_to_leg(robot.right_leg)
	
	# 左髋关节
	if robot.hip_left:
		PhysicsConfig.apply_to_hip_joint(robot.hip_left)
	
	# 右髋关节
	if robot.hip_right:
		PhysicsConfig.apply_to_hip_joint(robot.hip_right)


func reset_to_defaults():
	"""重置到默认参数（不使用优化配置）"""
	print("⚠️ 重置到默认参数")
	# 这里可以添加重置逻辑
