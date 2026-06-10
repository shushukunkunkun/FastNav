std_msgs/Header header

uint32 traj_id
string fsm_state
bool success
string failure_reason

float64 total_ms
float64 local_target_ms
float64 guide_astar_ms
float64 frontend_astar_ms
float64 reference_ms
float64 shortcut_ms
float64 corridor_ms
float64 minco_ms
float64 fine_check_ms
float64 publish_ms

int32 astar_nodes
int32 corridor_num
int32 minco_retry_count
float64 clearance_used
