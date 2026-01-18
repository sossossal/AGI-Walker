# Part.msg - 机器人零件定义

string part_id          # 零件唯一ID
string part_type        # 零件类型: motor, sensor, controller, joint, battery
string model            # 型号名称
geometry_msgs/Point position  # 位置 (x, y, z)
float64[] parameters    # 零件参数数组
