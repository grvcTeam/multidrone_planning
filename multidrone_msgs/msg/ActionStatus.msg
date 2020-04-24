# define a simple action status

uint8 AS_IDLE = 0
uint8 AS_WAIT_FOR_EVENT = 1
uint8 AS_TAKING_OFF = 2
uint8 AS_LANDING = 3
uint8 AS_GOING_TO_START_POSE = 4
uint8 AS_GOING_HOME = 5
uint8 AS_RUNNING = 6
uint8 AS_EMERGENCY = 7
uint8 AS_WAIT_FOR_SAFE_TO_GO = 8
uint8 AS_WAIT_FOR_GET_READY = 9

std_msgs/Header header
string mission_id
string SAS_id
string action_id
uint8 status
string message
