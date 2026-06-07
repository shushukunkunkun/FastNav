# FastNav internal control command.
# This message is usually published by traj_utils/minco_traj_server
# and consumed by fastnav_control. Planner should publish trajectory, not direct PX4 setpoint.

std_msgs/Header header

# Command type
uint8 COMMAND_POSITION = 0
uint8 COMMAND_VELOCITY = 1
uint8 COMMAND_HOVER    = 2
uint8 COMMAND_LAND     = 3
uint8 COMMAND_IDLE     = 4
uint8 COMMAND_TRAJECTORY = 5

uint8 command_type

# Trajectory id, used to reject stale sampled commands if needed.
uint32 trajectory_id

# Position setpoint in local frame
geometry_msgs/Point position

# Velocity setpoint in local frame
geometry_msgs/Vector3 velocity

# Acceleration feed-forward in local frame, sampled from $a(t)$.
geometry_msgs/Vector3 acceleration

# Jerk feed-forward in local frame, sampled from $j(t)$; current PX4 raw local setpoint does not consume jerk directly.
geometry_msgs/Vector3 jerk

# Yaw angle for position control
float64 yaw

# Yaw rate for velocity control
float64 yaw_rate

# Whether this command is valid
bool enable
