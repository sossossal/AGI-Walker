extends Node
## 物理参数配置管理器
## 集中管理所有物理相关的参数，方便调优

# ================== 地面材质参数 ==================
class_name PhysicsConfig

# 地面摩擦力
const GROUND_FRICTION = 0.8 # 0.0-1.0，越高越不易滑动
const GROUND_BOUNCE = 0.0 # 0.0-1.0，弹性系数

# ================== 机器人刚体参数 ==================

# 躯干参数
# ================== 机器刚体参数 ==================

# 躯干参数
const TORSO_MASS = 10.0 # kg
const TORSO_LINEAR_DAMP = 0.1
const TORSO_ANGULAR_DAMP = 0.5

# 腿部参数
const LEG_MASS = 3.0 # kg
const LEG_LINEAR_DAMP = 0.2
const LEG_ANGULAR_DAMP = 0.5

# 脚底摩擦力
const FOOT_FRICTION = 0.9
const FOOT_BOUNCE = 0.0

# ================== 关节参数 ==================

# 髋关节限位（度）
const HIP_LIMIT_LOWER = -45.0
const HIP_LIMIT_UPPER = 90.0

# 髋关节软限位
const HIP_LIMIT_SOFTNESS = 0.9
const HIP_LIMIT_BIAS = 0.3

# 电机参数
const MOTOR_MAX_IMPULSE = 500.0
const MOTOR_SPEED_MULTIPLIER = 5.0

# 关节阻尼
const JOINT_DAMPING = 0.5

# ================== 重力参数 ==================
const GRAVITY = 9.8

# ================== 仿真精度参数 ==================
const PHYSICS_FPS = 60
const SOLVER_ITERATIONS = 8

# ================== 动态参数覆盖 ==================
static var overrides = {}

static func set_param(key: String, value):
	overrides[key] = value
	print("  -> Set Physics Overrides: %s = %s" % [key, value])

static func _get(key: String, default_value):
	return overrides.get(key, default_value)

# ================== 工具函数 ==================

static func create_ground_material() -> PhysicsMaterial:
	"""创建地面物理材质"""
	var material = PhysicsMaterial.new()
	material.friction = _get("GROUND_FRICTION", GROUND_FRICTION)
	material.bounce = _get("GROUND_BOUNCE", GROUND_BOUNCE)
	return material


static func create_foot_material() -> PhysicsMaterial:
	"""创建脚底物理材质"""
	var material = PhysicsMaterial.new()
	material.friction = _get("FOOT_FRICTION", FOOT_FRICTION)
	material.bounce = _get("FOOT_BOUNCE", FOOT_BOUNCE)
	return material


static func apply_to_ground(ground: StaticBody3D):
	"""应用物理材质到地面"""
	ground.physics_material_override = create_ground_material()
	print("✅ 地面物理材质已应用")


static func apply_to_torso(torso: RigidBody3D):
	"""应用参数到躯干"""
	torso.mass = _get("TORSO_MASS", TORSO_MASS)
	torso.linear_damp = _get("TORSO_LINEAR_DAMP", TORSO_LINEAR_DAMP)
	torso.angular_damp = _get("TORSO_ANGULAR_DAMP", TORSO_ANGULAR_DAMP)
	print("✅ 躯干参数已应用: 质量=%.1fkg" % torso.mass)


static func apply_to_leg(leg: RigidBody3D):
	"""应用参数到腿部"""
	leg.mass = _get("LEG_MASS", LEG_MASS)
	leg.linear_damp = _get("LEG_LINEAR_DAMP", LEG_LINEAR_DAMP)
	leg.angular_damp = _get("LEG_ANGULAR_DAMP", LEG_ANGULAR_DAMP)
	leg.physics_material_override = create_foot_material()
	print("✅ 腿部参数已应用: 质量=%.1fkg" % leg.mass)


static func apply_to_hip_joint(joint: HingeJoint3D):
	"""应用参数到髋关节"""
	# 限位
	joint.set_param(HingeJoint3D.PARAM_LIMIT_LOWER, deg_to_rad(_get("HIP_LIMIT_LOWER", HIP_LIMIT_LOWER)))
	joint.set_param(HingeJoint3D.PARAM_LIMIT_UPPER, deg_to_rad(_get("HIP_LIMIT_UPPER", HIP_LIMIT_UPPER)))
	joint.set_flag(HingeJoint3D.FLAG_USE_LIMIT, true)
	
	# 软限位
	joint.set_param(HingeJoint3D.PARAM_LIMIT_SOFTNESS, _get("HIP_LIMIT_SOFTNESS", HIP_LIMIT_SOFTNESS))
	joint.set_param(HingeJoint3D.PARAM_LIMIT_BIAS, _get("HIP_LIMIT_BIAS", HIP_LIMIT_BIAS))
	
	# 电机
	joint.set_flag(HingeJoint3D.FLAG_ENABLE_MOTOR, true)
	joint.set_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE, _get("MOTOR_MAX_IMPULSE", MOTOR_MAX_IMPULSE))
	
	# 阻尼
	joint.set_param(HingeJoint3D.PARAM_ANGULAR_LIMIT_DAMPING, _get("JOINT_DAMPING", JOINT_DAMPING))
	
	print("✅ 关节参数已应用: 扭矩=%.0fN·m" % _get("MOTOR_MAX_IMPULSE", MOTOR_MAX_IMPULSE))


static func print_config_summary():
	"""打印配置摘要"""
	print("\n" + "=" * 60)
	print("📋 物理参数配置摘要 (含动态覆盖)")
	print("=" * 60)
	print("地面: 摩擦=%.2f" % _get("GROUND_FRICTION", GROUND_FRICTION))
	print("躯干: 质量=%.1fkg" % _get("TORSO_MASS", TORSO_MASS))
	print("腿部: 质量=%.1fkg" % _get("LEG_MASS", LEG_MASS))
	print("关节: 扭矩=%.0fN·m" % _get("MOTOR_MAX_IMPULSE", MOTOR_MAX_IMPULSE))
	print("=" * 60 + "\n")
