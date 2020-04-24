uint8 ERROR_TYPE_SAFETY = 1
uint8 ERROR_TYPE_PLANNER = 2

uint8 type
string message

# If planning error, give the combination of mission_ids and sas_roles that failed:
SASRole[] sas_roles