uint8 STATUS_ENROLLED = 0
uint8 STATUS_PLANNING = 1
uint8 STATUS_SUPERVISING = 2
uint8 STATUS_REJECTED = 3
uint8 STATUS_VALIDATED = 4
uint8 STATUS_READY = 5
uint8 STATUS_RUNNING = 6
uint8 STATUS_ABORTED = 7
uint8 STATUS_ENDED = 8

string mission_id
uint8 status
string[] selected_sas_ids
MissionError[] error_list