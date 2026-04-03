extends Node3D

@onready var hip_left: HingeJoint3D = $HipLeft
@onready var hip_right: HingeJoint3D = $HipRight
@onready var torso: RigidBody3D = $Torso

var time_passed: float = 0.0
var rl_mode: bool = false
var target_vel_left: float = 0.0
var target_vel_right: float = 0.0
var last_loaded_robot_config: Dictionary = {}

func _ready():
	if hip_left and hip_right:
		hip_left.set_flag(HingeJoint3D.FLAG_ENABLE_MOTOR, true)
		hip_right.set_flag(HingeJoint3D.FLAG_ENABLE_MOTOR, true)
		hip_left.set_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE, 200.0)
		hip_right.set_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE, 200.0)

func _physics_process(delta):
	if not rl_mode:
		time_passed += delta
		target_vel_left = sin(time_passed * 4.0) * 3.0
		target_vel_right = -target_vel_left
	
	if hip_left and hip_right:
		hip_left.set_param(HingeJoint3D.PARAM_MOTOR_TARGET_VELOCITY, target_vel_left)
		hip_right.set_param(HingeJoint3D.PARAM_MOTOR_TARGET_VELOCITY, target_vel_right)

func reset_pose():
	rl_mode = true
	time_passed = 0.0
	target_vel_left = 0.0
	target_vel_right = 0.0
	if torso:
		torso.linear_velocity = Vector3.ZERO
		torso.angular_velocity = Vector3.ZERO

func apply_motor_commands(command: Dictionary):
	rl_mode = true
	if command.has("motors"):
		var motors = command["motors"]
		if motors.has("hip_left"): target_vel_left = motors["hip_left"]
		if motors.has("hip_right"): target_vel_right = motors["hip_right"]

func get_sensor_data() -> Dictionary:
	var euler = Vector3.ZERO
	var height = 1.5
	if torso:
		euler = torso.global_transform.basis.get_euler()
		height = torso.global_position.y
		
	return {
		"timestamp": Time.get_ticks_msec() / 1000.0,
		"torso_height": height,
		"sensors": {
			"imu": {
				"orient": [rad_to_deg(euler.x), rad_to_deg(euler.y), rad_to_deg(euler.z)]
			}
		}
	}


func get_schema() -> Dictionary:
	return {
		"sensors": {
			"torso_height": {"type": "float32", "shape": [1]},
			"imu_orient": {"type": "float32", "shape": [3]}
		},
		"actuators": {
			"hip_velocity": {"type": "float32", "shape": [2], "range": [-10.0, 10.0]}
		},
		"meta": {
			"last_loaded_parts": last_loaded_robot_config.get("parts", []).size(),
			"last_loaded_connections": last_loaded_robot_config.get("connections", []).size()
		}
	}


func load_from_dict(config: Dictionary) -> void:
	last_loaded_robot_config = config.duplicate(true)
	rl_mode = true
