#!/usr/bin/env python
import rospkg
import rospy
from multidrone_msgs.msg import ExecuteAction, ExecuteGoal, ExecuteFeedback, ExecuteResult, DroneAction, ShootingAction
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped
from uav_abstraction_layer.msg import State
import actionlib
import time

class dummyDrone:

    def __init__(self,drone_id):
        self.id = drone_id
        self.battery_pub = rospy.Publisher('drone_' + str(drone_id) + '/mavros/battery',BatteryState,queue_size=1)
        self.pose_pub = rospy.Publisher('drone_' + str(drone_id) + '/ual/pose',PoseStamped,queue_size=1)
        self.state_pub = rospy.Publisher('drone_' + str(drone_id) + '/ual/state',State,queue_size=1)
        self._as = actionlib.SimpleActionServer('drone_' + str(drone_id) + '/action_server', ExecuteAction, execute_cb=self.execute_callback, auto_start = False)
        self._as.start()
        self._result = ExecuteResult()
        self.state_msg = State()
        self.state_msg.state = State.LANDED_ARMED

    def publish(self):
        battery_msg = BatteryState()
        battery_msg.percentage = 1.0

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = self.id
        pose_msg.pose.position.y = self.id
        pose_msg.pose.position.z = 0

        self.battery_pub.publish(battery_msg)
        self.pose_pub.publish(pose_msg)
        self.state_pub.publish(self.state_msg)

    def execute_callback(self,goal):
        if goal.action_goal.action_type == DroneAction.TYPE_SHOOTING:
            wait_time = goal.action_goal.shooting_action.duration.data.secs + goal.action_goal.shooting_action.duration.data.nsecs/1e9
        else:
            wait_time = 4.0

        rospy.loginfo( 'Drone ' + str(self.id) + ': ' + self.get_action_str(goal.action_goal) + ' (' + str(wait_time) + ' seconds)' )
        time.sleep(wait_time)

        if goal.action_goal.action_type == DroneAction.TYPE_TAKEOFF:
            self.state_msg.state = State.FLYING_AUTO
        elif goal.action_goal.action_type == DroneAction.TYPE_LAND:
            self.state_msg.state = State.LANDED_ARMED
        
        self._result.goal_achieved = True
        self._as.set_succeeded(self._result)

    def get_action_str(self,action_goal):
        action_str = ''
        action_type = action_goal.action_type

        if action_type == DroneAction.TYPE_TAKEOFF:
            action_str = 'Taking off'
            self.state_msg.state = State.TAKING_OFF
        elif action_type == DroneAction.TYPE_LAND:
            action_str = 'Landing'
            self.state_msg.state = State.LANDING
        elif action_type == DroneAction.TYPE_GOTOWAYPOINT:
            action_str = 'Going to waypoint'
        elif action_type == DroneAction.TYPE_SHOOTING:
            shooting_type_str = [
                'STATIC',
                'FLY_THROUGH',
                'ESTABLISH',
                'CHASE',
                'LEAD',
                'FLYBY',
                'LATERAL',
                'ELEVATOR',
                'ORBIT'
            ]
            action_str = 'Shooting - ' + shooting_type_str[action_goal.shooting_action.shooting_roles[0].shooting_type.type]

        return action_str

if __name__ == "__main__":
    rospy.init_node('dummy_drones')
    drone_ids = rospy.get_param('~drone_ids',[1])
    dummy_drones = []

    for id in drone_ids:
        dummy_drones.append(dummyDrone(id))

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        for drone in dummy_drones:
            drone.publish()
        rate.sleep()