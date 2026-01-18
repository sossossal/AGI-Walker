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
const TORSO_MASS = 10.0 # kg
const TORSO_LINEAR_DAMP = 0.1 # 线性阻尼（空气阻力）
const TORSO_ANGULAR_DAMP = 0.5 # 角阻尼（旋转阻力）

# 腿部参数
const LEG_MASS = 3.0 # kg（增加质量提高稳定性）
const LEG_LINEAR_DAMP = 0.2
const LEG_ANGULAR_DAMP = 0.5

# 脚底摩擦力
const FOOT_FRICTION = 0.9 # 高摩擦防滑
const FOOT_BOUNCE = 0.0

# ================== 关节参数 ==================

# 髋关节限位（度）
const HIP_LIMIT_LOWER = -45.0
const HIP_LIMIT_UPPER = 90.0

# 髋关节软限位（使限位更柔和，避免突然停止）
const HIP_LIMIT_SOFTNESS = 0.9 # 0.0-1.0
const HIP_LIMIT_BIAS = 0.3 # 限位恢复速度

# 电机参数
const MOTOR_MAX_IMPULSE = 500.0 # 最大扭矩（N·m）
const MOTOR_SPEED_MULTIPLIER = 5.0 # PD控制器增益

# 关节阻尼（抑制震荡）
const JOINT_DAMPING = 0.5 # 0.0-1.0

# ================== 重力参数 ==================
const GRAVITY = 9.8 # m/s²（地球标准重力）

# ================== 仿真精度参数 ==================
const PHYSICS_FPS = 60 # 物理帧率（推荐60-120）
const SOLVER_ITERATIONS = 8 # 求解器迭代次数（越高越精确但越慢）


# ================== 工具函数 ==================

static func create_ground_material() -> PhysicsMaterial:
	"""创建地面物理材质"""
	var material = PhysicsMaterial.new()
	material.friction = GROUND_FRICTION
	material.bounce = GROUND_BOUNCE
	return material


static func create_foot_material() -> PhysicsMaterial:
	"""创建脚底物理材质"""
	var material = PhysicsMaterial.new()
	material.friction = FOOT_FRICTION
	material.bounce = FOOT_BOUNCE
	return material


static func apply_to_ground(ground: StaticBody3D):
	"""应用物理材质到地面"""
	ground.physics_material_override = create_ground_material()
	print("✅ 地面物理材质已应用: 摩擦力=%.2f" % GROUND_FRICTION)


static func apply_to_torso(torso: RigidBody3D):
	"""应用参数到躯干"""
	torso.mass = TORSO_MASS
	torso.linear_damp = TORSO_LINEAR_DAMP
	torso.angular_damp = TORSO_ANGULAR_DAMP
	print("✅ 躯干参数已应用: 质量=%.1fkg, 线性阻尼=%.2f" % [TORSO_MASS, TORSO_LINEAR_DAMP])


static func apply_to_leg(leg: RigidBody3D):
	"""应用参数到腿部"""
	leg.mass = LEG_MASS
	leg.linear_damp = LEG_LINEAR_DAMP
	leg.angular_damp = LEG_ANGULAR_DAMP
	leg.physics_material_override = create_foot_material()
	print("✅ 腿部参数已应用: 质量=%.1fkg, 摩擦力=%.2f" % [LEG_MASS, FOOT_FRICTION])


static func apply_to_hip_joint(joint: HingeJoint3D):
	"""应用参数到髋关节"""
	# 限位
	joint.set_param(HingeJoint3D.PARAM_LIMIT_LOWER, deg_to_rad(HIP_LIMIT_LOWER))
	joint.set_param(HingeJoint3D.PARAM_LIMIT_UPPER, deg_to_rad(HIP_LIMIT_UPPER))
	joint.set_flag(HingeJoint3D.FLAG_USE_LIMIT, true)
	
	# 软限位
	joint.set_param(HingeJoint3D.PARAM_LIMIT_SOFTNESS, HIP_LIMIT_SOFTNESS)
	joint.set_param(HingeJoint3D.PARAM_LIMIT_BIAS, HIP_LIMIT_BIAS)
	
	# 电机
	joint.set_flag(HingeJoint3D.FLAG_ENABLE_MOTOR, true)
	joint.set_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE, MOTOR_MAX_IMPULSE)
	
	# 阻尼
	joint.set_param(HingeJoint3D.PARAM_ANGULAR_LIMIT_DAMPING, JOINT_DAMPING)
	
	print("✅ 关节参数已应用: 限位=[%.0f°, %.0f°], 扭矩=%.0fN·m" %
		[HIP_LIMIT_LOWER, HIP_LIMIT_UPPER, MOTOR_MAX_IMPULSE])


static func print_config_summary():
	"""打印配置摘要"""
	print("\n" + "=" * 60)
	print("📋 物理参数配置摘要")
	print("=" * 60)
	print("地面: 摩擦=%.2f, 弹性=%.2f" % [GROUND_FRICTION, GROUND_BOUNCE])
	print("躯干: 质量=%.1fkg, 线性阻尼=%.2f, 角阻尼=%.2f" % [TORSO_MASS, TORSO_LINEAR_DAMP, TORSO_ANGULAR_DAMP])
	print("腿部: 质量=%.1fkg, 摩擦=%.2f" % [LEG_MASS, FOOT_FRICTION])
	print("关节: 限位=[%.0f°, %.0f°], 扭矩=%.0fN·m" % [HIP_LIMIT_LOWER, HIP_LIMIT_UPPER, MOTOR_MAX_IMPULSE])
	print("重力: %.2f m/s²" % GRAVITY)
	print("=" * 60 + "\n")
