#No camera feedback so these values are the ones written into SBUS in BMMCC units
std_msgs/Header header
uint8 trigger_record
uint8 iris
uint8 focus
uint8 auto_focus
uint8 zoom
uint8 iso
uint8 shutter_angle
uint8 white_balance
uint8 audio_levels
uint8 frame_rate
uint8 codec

#cameraTXstatus
bool SBUS_write_error

#cameraControlstatus
bool manual_ctrl

#RC radio rx status
bool rcstatus_connected
bool rcstatus_failsafe

#TOREMOVE? move to a yaml under common/multidrone_configs/config?
#Intrinsic BMMCC Parameters
float32 focalLength = 14.0 #mm
float32 pixelSizeX = 0.0065 #mm
float32 pixelSizeY = 0.0065 #mm
int64 principalPointX = 960
int64 principalPointY = 540

float32 sensorWidth
float32 sensorHeight