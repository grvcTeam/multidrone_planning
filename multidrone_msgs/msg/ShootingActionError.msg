# planner error for each shooting action reply

uint8 ERROR_TYPE_NONE = 0
uint8 ERROR_TYPE_BAD_ACTION = 1
uint8 ERROR_TYPE_BAD_ARGUMENT = 2
uint8 ERROR_TYPE_INCOMPATIBLE = 3
uint8 ERROR_TYPE_OTHER = 4

string action_id
uint8 type
string message
