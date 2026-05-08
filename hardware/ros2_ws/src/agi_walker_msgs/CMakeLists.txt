cmake_minimum_required(VERSION 3.8)
project(agi_walker_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Part.msg"
  "msg/Connection.msg"
  "msg/RobotState.msg"
  "msg/InstructionStep.msg"
  "msg/InstructionSet.msg"
  "msg/SimulatedCircuit.msg"
  "msg/HardwareFault.msg"
  "msg/HardwareRecoveryAction.msg"
  "msg/HardwareRecoveryStatus.msg"
  "msg/BehaviorCommand.msg"
  "msg/NavigationGoal.msg"
  "msg/PerceptionSnapshot.msg"
  "srv/LoadRobot.srv"
  "srv/ApplyInstructionSet.srv"
  "srv/ConfigureSimulatedCircuit.srv"
  "srv/HardwareRecovery.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
