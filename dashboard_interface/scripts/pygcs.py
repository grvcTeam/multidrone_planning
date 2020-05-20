#!/usr/bin/env python
from multidrone_msgs.msg import TargetStateArray, SystemStatus, DroneActionStatus, GimbalStatus
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from uav_abstraction_layer.msg import State
import rospkg
import rospy
import time
from os import system
from math import atan2

def ualState(state):
    if state == State.UNINITIALIZED:
        state_as_string = "UNINITIALIZED"
    elif state == State.LANDED_DISARMED:
        state_as_string = "LANDED_DISARMED"
    elif state == State.LANDED_ARMED:
        state_as_string = "LANDED_ARMED"
    elif state == State.TAKING_OFF:
        state_as_string = "TAKING_OFF"
    elif state == State.FLYING_AUTO:
        state_as_string = "FLYING_AUTO"
    elif state == State.FLYING_MANUAL:
        state_as_string = "FLYING_MANUAL"
    elif state == State.LANDING:
        state_as_string = "LANDING"
    return state_as_string

def actionStatus(status):
    if status == DroneActionStatus.AS_IDLE:
        status_as_string = "IDLE"
    elif status == DroneActionStatus.AS_WAIT_FOR_EVENT:
        status_as_string = "WAIT_FOR_EVENT"
    elif status == DroneActionStatus.AS_TAKING_OFF:
        status_as_string = "TAKING_OFF"
    elif status == DroneActionStatus.AS_LANDING:
        status_as_string = "LANDING"
    elif status == DroneActionStatus.AS_GOING_TO_START_POSE:
        status_as_string = "GOING_TO_START_POSE"
    elif status == DroneActionStatus.AS_GOING_HOME:
        status_as_string = "GOING_HOME"
    elif status == DroneActionStatus.AS_RUNNING:
        status_as_string = "RUNNING"
    elif status == DroneActionStatus.AS_EMERGENCY:
        status_as_string = "EMERGENCY"
    elif status == DroneActionStatus.AS_WAIT_FOR_SAFE_TO_GO:
        status_as_string = "AS_WAIT_FOR_SAFE_TO_GO"
    elif status == DroneActionStatus.AS_WAIT_FOR_GET_READY:
        status_as_string = "AS_WAIT_FOR_GET_READY"
    return status_as_string

class DroneStatus:
    def __init__(self,id):
        self.id = id
        self.gps_fix = 0
        self.n_satellites = 0
        self.gps_cov = 0.0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_z = 0.0
        self.orientation = 0.0
        self.gimbal_yaw = 0.0
        self.px4_mode = ""
        self.ual_state = ""
        self.mission_id = ""
        self.sas_id = ""
        self.action_id = ""
        self.action_status = ""

        rospy.Subscriber("/drone_" + str(id) + "/ual/pose", PoseStamped, self.dronePoseCallback)
        rospy.Subscriber("/drone_" + str(id) + "/ual/state", State, self.droneStateCallback)
        rospy.Subscriber("/drone_" + str(id) + "/mavros/global_position/global", NavSatFix, self.globalCallback)
        rospy.Subscriber("/drone_" + str(id) + "/gimbal/status", GimbalStatus, self.gimbalStatusCallback)

    def dronePoseCallback(self,data):
        self.pose_x = data.pose.position.x
        self.pose_y = data.pose.position.y
        self.pose_z = data.pose.position.z
        
        siny_cosp = 2.0 * (data.pose.orientation.w * data.pose.orientation.z + data.pose.orientation.x * data.pose.orientation.y)
        cosy_cosp = 1.0 - 2.0 * (data.pose.orientation.y * data.pose.orientation.y + data.pose.orientation.z * data.pose.orientation.z)
        self.orientation = atan2(siny_cosp, cosy_cosp) * 180.0/3.141592654

    def droneStateCallback(self,data):
        self.ual_state = ualState(data.state)

    def globalCallback(self,data):
        self.gps_cov = data.position_covariance[0]

    def gimbalStatusCallback(self,data):
        self.gimbal_yaw = data.yaw

class TargetStatus:
    def __init__(self,id):
        self.id = id
        self.gps_fix = 0
        self.n_satellites = 0
        self.gps_cov = 0.0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_z = 0.0
        self.time_now = 0.0
        self.time_prev = 0.0
        self.num_msgs_received = 0
        self.frequency = 0.0

        rospy.Subscriber("/target_" + str(id) + "/mavros/global_position/raw/fix", NavSatFix, self.globalCallback)
        rospy.Timer(rospy.Duration(3),self.frequencyTimerCallback)

    def globalCallback(self,data):
        self.gps_cov = data.position_covariance[0]
        self.num_msgs_received += 1
    
    def frequencyTimerCallback(self,event):
        self.time_now = rospy.Time.now().to_nsec()/1e9
        dt = self.time_now - self.time_prev
        self.time_prev = self.time_now

        self.frequency = self.num_msgs_received/dt

        self.num_msgs_received = 0

class MultidroneStatus:
    def __init__(self):
        self.targets = {}
        self.drones = {}

        rospy.Subscriber("/targets_pose", TargetStateArray, self.targetPoseCallback)
        rospy.Subscriber("/diagnostics", DiagnosticArray, self.diagnosticsCallback)
        rospy.Subscriber("/mission_controller/system_status", SystemStatus, self.systemStatusCallback)

    def targetPoseCallback(self,data):
        for target in data.targets:
            if target.target_id > 0:
                if not( target.target_id in self.targets ):
                    self.targets[target.target_id] = TargetStatus(target.target_id)
                self.targets[target.target_id].pose_x = target.pose.pose.position.x
                self.targets[target.target_id].pose_y = target.pose.pose.position.y
                self.targets[target.target_id].pose_z = target.pose.pose.position.z

    def diagnosticsCallback(self,data):

        fix_type = 0
        found_fix = False
        n_satellites = 0
        px4_mode = ""
        for element in data.status:
            if element.name.find("GPS") >= 0:
                for value in element.values:
                    if "Fix type" in value.key:
                        fix_type = int(value.value)
                        found_fix = True
                    if "Satellites visible" in value.key:
                        n_satellites = int(value.value)
            if element.name.find("Heartbeat") >= 0:
                for value in element.values:
                    if "Mode" in value.key:
                        px4_mode = value.value
        
        if found_fix:
            drone_id_pos = data.status[0].name.find("drone_")
            if drone_id_pos >= 0:
                drone_id = int( data.status[0].name[drone_id_pos+6] )
                if not( drone_id in self.drones ):
                    self.drones[drone_id] = DroneStatus(drone_id)
                self.drones[drone_id].gps_fix = fix_type
                self.drones[drone_id].n_satellites = n_satellites
                self.drones[drone_id].px4_mode = px4_mode
            else:
                target_id_pos = data.status[0].name.find("target_")
                if target_id_pos >= 0:
                    target_id = int( data.status[0].name[target_id_pos+7] )
                    if not( target_id in self.targets ):
                        self.targets[target_id] = TargetStatus(target_id)
                    self.targets[target_id].gps_fix = fix_type
                    self.targets[target_id].n_satellites = n_satellites

    def systemStatusCallback(self,data):
        for drone in data.drones:
            if not( drone.drone_id in self.drones ):
                self.drones[drone.drone_id] = DroneStatus(drone.drone_id)
            self.drones[drone.drone_id].mission_id = drone.mission_id
            self.drones[drone.drone_id].sas_id = drone.SAS_id
            self.drones[drone.drone_id].action_id = drone.action_id
            self.drones[drone.drone_id].action_status = actionStatus(drone.status)

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def print_info(system_status):
    system("clear")
    print bcolors.OKBLUE + "====================================" + bcolors.ENDC
    print bcolors.OKBLUE + "===== " + bcolors.ENDC + bcolors.BOLD + "MULTIDRONE system status" + bcolors.ENDC + bcolors.OKBLUE + " =====\n" + bcolors.ENDC
    print "          --- Drones  ---       "
    for id, drone in system_status.drones.items():
        print bcolors.BOLD + "\nDrone " + str(drone.id) + bcolors.ENDC
        print "  Action status: " + drone.action_status
        print "  Action ID: " + drone.action_id[-5:]
        print "  PX4 mode: " + drone.px4_mode
        print "  UAL state: " + drone.ual_state
        print "  GPS Fix/nSat/Cov: " + bcolors.BOLD + (bcolors.FAIL if drone.gps_fix<=4 else (bcolors.OKGREEN if drone.gps_fix==6 else bcolors.WARNING) ) + str(drone.gps_fix) + bcolors.ENDC + " / " + str(drone.n_satellites) + " / {:.4f}".format(drone.gps_cov)
        print "  Pose: {:.2f}  {:.2f}  {:.2f}  {:.2f}".format(drone.pose_x,drone.pose_y,drone.pose_z,drone.orientation)
        print "  Gimbal yaw: {:.2f}".format(drone.gimbal_yaw)
    
    print "\n          --- Targets ---"
    for id, target in system_status.targets.items():
        print bcolors.BOLD + "Target " + str(target.id) + bcolors.ENDC
        print "  GPS Fix/nSat/Cov: " + bcolors.BOLD + (bcolors.FAIL if target.gps_fix<=4 else (bcolors.OKGREEN if target.gps_fix==6 else bcolors.WARNING) ) + str(target.gps_fix) + bcolors.ENDC + " / " + str(target.n_satellites) + " / {:.4f}".format(target.gps_cov)
        print "  Pose: {:.2f} {:.2f} {:.2f}".format(target.pose_x,target.pose_y,target.pose_z)
        print "  Frequency: {:.2f} Hz".format(target.frequency)
    
    print bcolors.OKBLUE + "====================================" + bcolors.ENDC

if __name__ == "__main__":
    rospy.init_node('pygcs', anonymous=True)

    system_status = MultidroneStatus()

    while not rospy.is_shutdown():   
        print_info(system_status)
        time.sleep(0.1)
