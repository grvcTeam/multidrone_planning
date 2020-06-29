/**
 * MULTIDRONE Project:
 *    Prototyping functions to test the overall system specifications.
 *
 * Oboard scheduler node.
 * 
 */

#ifndef ONBOARD_SCHEDULER_H
#define ONBOARD_SCHEDULER_H

#include <ros/ros.h>
#include <thread>
#include <chrono>
#include <time.h>
#include <multidrone_msgs/DroneAction.h>
#include <multidrone_msgs/Event.h>
#include <multidrone_msgs/DroneTelemetry.h>
#include <tf/transform_datatypes.h>
#include <sstream>
#include <vector>
#include <string>
#include <multidrone_msgs/PushDroneAction.h>
#include <multidrone_msgs/ActionStatus.h>
#include <std_srvs/Empty.h>
#include <multidrone_msgs/ExecuteAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <boost/thread.hpp>
#include <multidrone_msgs/SupervisorAlarm.h>
#include <multidrone_planning/path_planner.h>
#include <sensor_msgs/BatteryState.h>
#include <multidrone_msgs/GetSemanticMap.h>
#include <fstream>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>
#include <atomic>
#include <uav_abstraction_layer/State.h>

typedef actionlib::SimpleActionClient<multidrone_msgs::ExecuteAction> Drone_action_client;

class OnBoardScheduler
{

public:
  OnBoardScheduler();
  ~OnBoardScheduler();

  void loop();
private:
    
  /// Callbacks
  void eventReceived(const multidrone_msgs::Event::ConstPtr& event);
  void droneTelemetryCallback(const sensor_msgs::BatteryState::ConstPtr& drone_status);
  void dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  bool droneactionlistServiceCallback(multidrone_msgs::PushDroneAction::Request &req, multidrone_msgs::PushDroneAction::Response &res);
  bool emergencyCallback(multidrone_msgs::SupervisorAlarm::Request &req, multidrone_msgs::SupervisorAlarm::Response &res);
  void updatingActionsStatus();
  void emergencyAlarms();
  void goToEmergencySite(float x, float y, float z, bool abort = false, bool emergency = false);
  bool checkTime(float max_dur_sec, ros::Time begin);
  bool checkDistance(float total_distance, float &dist_sum);
  bool safeToGoCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool abortServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  void activeCallback();
  void feedbackCallback(const multidrone_msgs::ExecuteFeedbackConstPtr& feedback);
  void checkCommunication(time_t last_time, double delay_allowed);
  void ualStateCallback(const uav_abstraction_layer::State::ConstPtr &_msg);
  void kmlUpdateCallback(const ros::TimerEvent&);

  /// Node handlers
  ros::NodeHandle nh_; 
	ros::NodeHandle pnh_;

  /// Subscribers
  ros::Subscriber event_sub_; 
  ros::Subscriber drone_telemetry_sub_;
  ros::Subscriber drone_action_sub_;
  ros::Subscriber drone_pose_sub_;
  ros::Subscriber ual_state_sub_;
  /// Publishers
  ros::Publisher action_status_pub_;

  /// Services
  ros::ServiceServer action_list_service_;
  ros::ServiceServer emergency_service_;
  ros::ServiceServer safe_to_go_service_;
  ros::ServiceServer abort_service_;

  ros::ServiceClient KML_client_;

  // Timer for KML update when drone idle:
  ros::Timer timer_;
  
  // execute action client
  actionlib::SimpleActionClient<multidrone_msgs::ExecuteAction>* action_client_;
  actionlib::SimpleActionClient<multidrone_msgs::ExecuteAction>* action_client_alfonso_experiment_;

  /// drone actions queues
  std::vector<multidrone_msgs::DroneAction> push_drone_action_;

  /// utility members
  std::string event_ = "NULL";
  bool event_received_ = false;

  double event_timestamp_;
  int drone_id_;
  typedef actionlib::SimpleActionClient<multidrone_msgs::ExecuteAction> client;
  std::atomic<bool> new_drone_action_list_ = {false};
  geometry_msgs::PoseStamped drone_pose_;
  geometry_msgs::PoseStamped drone_pose_previous_;

  bool first_event_received_ = false;

  // threads
  std::thread status_thread_;
  std::thread emergency_thread_;
  std::thread loop_thread_;

  // action status
  std::atomic<int> action_status_ = {multidrone_msgs::ActionStatus::AS_IDLE};
  std::string action_id_;
  std::string SAS_id_;
  std::string mission_id_;

  // loop thread
  bool emergency_ = false;
  bool low_battery_ = false;

  multidrone::PathPlanner path_planner_;

  int print_new_KML_rosinfo_ = 0;  // Attribute to avoid printing the same status message of the KML many times.
  int delay_countdown_ = 0;

  bool safe_to_go_flag_ = false;
  ros::Time last_event_received_time_;
  geometry_msgs::PoseStamped home_pose_;
  bool home_pose_saved_ = false;
  bool abort_service_received_ = false;
  bool drone_action_actived_ = false;
  time_t time_feedback_;
  uav_abstraction_layer::State state_;
  float max_time_landing_takeoff_ = 30.0;
  bool nan_inf_executor_ = false;
  int cont_nan_inf_executor_ = 0;
  std::string previous_id_NaN_inf_ = "";
  const float BATTERY_LIMIT = 0.1;
  const int MAX_NUMBER_NAN = 2;
};

#endif // ONBOARD_SCHEDULER_H