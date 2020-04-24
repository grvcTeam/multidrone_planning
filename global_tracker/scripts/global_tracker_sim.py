#!/usr/bin/env python
import rospkg
import rospy
from multidrone_msgs.msg import TargetStateArray, TargetState
from nav_msgs.msg import Odometry
import time

target_state = TargetState()

def target_simulation_cb(msg):
    global target_state
    target_state.target_id = 1
    target_state.pose = msg.pose
    target_state.velocity = msg.twist


if __name__ == "__main__":
    rospy.init_node('tracker_simulation', anonymous=True)
    rospy.Subscriber("/drc_vehicle_xp900/odometry",Odometry,target_simulation_cb, queue_size=1)
    target_pub = rospy.Publisher("/targets_pose",TargetStateArray,queue_size=10)
    target_state_array = TargetStateArray()

    while not rospy.is_shutdown():
        target_state_array.targets.append(target_state)
        target_pub.publish(target_state_array)
        target_state_array.targets = []
        time.sleep(0.1)