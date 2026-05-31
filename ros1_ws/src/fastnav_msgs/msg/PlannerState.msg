# FastNav planner state.
# This message is published by fastnav_planner
# for debugging and visualization.

std_msgs/Header header

uint8 STATE_IDLE        = 0
uint8 STATE_TAKEOFF     = 1
uint8 STATE_NAVIGATE    = 2
uint8 STATE_AVOID       = 3
uint8 STATE_HOVER       = 4
uint8 STATE_GOAL_REACHED = 5
uint8 STATE_EMERGENCY   = 6

uint8 state
string state_name

geometry_msgs/Point current_goal

bool goal_reached
bool has_obstacle