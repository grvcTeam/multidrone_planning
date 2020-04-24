## define a drone action

# types of action
uint8 TYPE_TAKEOFF=1
uint8 TYPE_LAND=2
uint8 TYPE_GOTOWAYPOINT=3
uint8 TYPE_SHOOTING=4

std_msgs/Header header
uint8 action_type
int32 delay_since_event     # Delay (in seconds) since the start event is received to start the shooting action
string action_id            # identifies this shooting action (or the next one if this is a navigation action before shooting action)
string action_sequence_id   # identifies this shooting action sequence (or the next one if this is a navigation action before shooting action)
string mission_id           # identifies the corresponding mission (or the next one if this is a navigation action before shooting action)
string start_event
ShootingAction shooting_action
geometry_msgs/PointStamped[] path
geometry_msgs/Quaternion final_yaw_if_gotowaypoint
