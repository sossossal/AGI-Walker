# Recovery status snapshot suitable for operator and automation surfaces.
std_msgs/Header header
string status
agi_walker_msgs/HardwareFault[] faults
agi_walker_msgs/HardwareRecoveryAction[] actions
string summary_json
