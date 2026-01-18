# RobotState.msg - 机器人整体状态

std_msgs/Header header
geometry_msgs/Pose pose          # 机器人位姿
geometry_msgs/Twist twist        # 速度 (线速度和角速度)
float64 battery_level            # 电池电量 (0-100)
float64 cpu_usage                # CPU使用率 (0-100)
float64 temperature              # 温度 (摄氏度)
string status                    # 状态: IDLE, RUNNING, PAUSED, ERROR
string[] warnings                # 警告信息列表
