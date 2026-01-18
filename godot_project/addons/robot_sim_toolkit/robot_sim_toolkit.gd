## robot_sim_toolkit.gd
## 机器人仿真工具包插件主脚本
@tool
extends EditorPlugin

const PLUGIN_NAME = "Robot Simulation Toolkit"

func _enter_tree():
	print("✅ ", PLUGIN_NAME, " 已加载")

func _exit_tree():
	print("🔌 ", PLUGIN_NAME, " 已卸载")
