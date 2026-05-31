# FastNav obstacle information.
# This message is published by fastnav_perception
# and consumed by fastnav_planner.

std_msgs/Header header

bool has_obstacle

float64 min_distance
float64 front_distance
float64 left_distance
float64 right_distance
float64 rear_distance

geometry_msgs/Point nearest_point

uint32 obstacle_point_count