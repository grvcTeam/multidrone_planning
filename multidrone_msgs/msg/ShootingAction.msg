uint8 RT_MODE_VIRTUAL_TRAJ = 0
uint8 RT_MODE_VIRTUAL_PATH = 1
uint8 RT_MODE_ACTUAL_TARGET = 2

string start_event          # identifies the event of this shooting action
string action_id            # identifies this shooting action
string action_sequence_id   # identifies this shooting action sequence
string mission_id           # identifies the corresponding mission

std_msgs/Duration duration  # Expected duration in seconds of the shooting action
int32 estimated_start_time  # In seconds refered to the absolute origin of time (parameter of the Mission Controller launcher, "2019-01-01T00:00:00Z" by default).
int32 delay_since_event     # Delay in seconds of the shooting action start since the start event is received.
float32 length              # Expected length in meters of the reference target trajectory

TargetType target_type
geometry_msgs/PointStamped[] rt_trajectory  # list (trajectory) of reference target (RT) waypoints to be tracked during shooting
geometry_msgs/Vector3 rt_displacement       # Start vectorial displacement of the center of drone formation relative to the RT position.
float32 formation_speed                     # Speed of the formation (-1 if not defined)
uint8 rt_mode                               # RT MODE: VIRTUAL_TRAJECTORY (default), VIRTUAL_PATH or ACTUAL_TARGET
uint8 rt_id                                 # RT identifier (default: 0) used only when rt_mode is > 0

uint8 ndrones   # Number of drones participating in this shooting action

ShootingRole[] shooting_roles   # The size of the list is equal to ndrones, the ShootingAction has one ShootingRole per drone.
