#include <multidrone_msgs/Event.h>

#include <string>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <multidrone_msgs/DirectorEvent.h>


//////////////////////////////// DEPRECATED. EVENT MANAGER WAS ABSORBED BY THE MISSION CONTROLLER ////////////////////////////////
//////////////////////////////// DEPRECATED. EVENT MANAGER WAS ABSORBED BY THE MISSION CONTROLLER ////////////////////////////////
//////////////////////////////// DEPRECATED. EVENT MANAGER WAS ABSORBED BY THE MISSION CONTROLLER ////////////////////////////////


std::string event_received_id;
ros::Time event_received_time_stamp;
bool event_received = false;


bool directorEventCallBack(multidrone_msgs::DirectorEvent::Request &req, multidrone_msgs::DirectorEvent::Request &res) {
  event_received_id = req.event_id;
  event_received_time_stamp = ros::Time::now();
  event_received = true;
  ROS_INFO("event %s received from the Director", event_received_id.c_str());
  return true;
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "event_manager_node");
  
  ros::NodeHandle nh;
  nh = ros::NodeHandle();


  ros::Publisher event_pub;
  event_pub =nh.advertise<multidrone_msgs::Event>("mission_controller/event", 1);
  ros::ServiceServer event_service;
  event_service = nh.advertiseService("mission_controller/director_event", directorEventCallBack);
  multidrone_msgs::Event event_msg;


  ros::Rate loop_rate(10); //[Hz]
  while (ros::ok) {


    if (event_received) {
      event_msg.event_id =event_received_id;
      event_msg.header.stamp = event_received_time_stamp;
      event_pub.publish(event_msg);
      event_received = false;
    }


    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
