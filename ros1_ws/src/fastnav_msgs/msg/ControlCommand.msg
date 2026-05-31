# FastNav internal control command.
# This message is published by fastnav_planner
# and consumed by fastnav_control.

std_msgs/Header header

# Command type
uint8 COMMAND_POSITION = 0
uint8 COMMAND_VELOCITY = 1
uint8 COMMAND_HOVER    = 2
uint8 COMMAND_LAND     = 3
uint8 COMMAND_IDLE     = 4

uint8 command_type

# Position setpoint in local frame
geometry_msgs/Point position

# Velocity setpoint in local frame
geometry_msgs/Vector3 velocity

# Yaw angle for position control
float64 yaw

# Yaw rate for velocity control
float64 yaw_rate

# Whether this command is valid
bool enable