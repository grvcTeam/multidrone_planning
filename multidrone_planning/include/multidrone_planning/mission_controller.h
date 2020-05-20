/**
 * MULTIDRONE Project:
 *
 * Mission controller.
 * 
 */

#ifndef MISSION_CONTROLLER_H
#define MISSION_CONTROLLER_H

#include <sstream>
#include <thread>
#include <atomic>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geographic_msgs/GeoPoint.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>

#include <sensor_msgs/BatteryState.h>
#include <multidrone_msgs/ActionStatus.h>
#include <multidrone_msgs/MissionStatus.h>
#include <multidrone_msgs/DroneActionStatus.h>
#include <multidrone_msgs/TargetStateArray.h>
#include <multidrone_msgs/SupervisorAlarm.h>
#include <multidrone_msgs/GetSemanticMap.h>
#include <multidrone_msgs/PostSemanticAnnotations.h>
#include <multidrone_msgs/Event.h>
#include <multidrone_msgs/DirectorEvent.h>
#include <multidrone_msgs/SendXML.h>
#include <multidrone_msgs/ValidateMission.h>
#include <multidrone_msgs/SelectRole.h>
#include <multidrone_msgs/DroneAction.h>
#include <multidrone_msgs/MultidronePlan.h>
#include <multidrone_msgs/Clear.h>
#include <multidrone_planning/high_level_planner.h>


namespace multidrone {

struct DroneState {
  multidrone_msgs::ActionStatus action_status;
  geometry_msgs::Pose pose;
  double battery_remaining;
  bool drone_pose_defined = false;
  bool drone_alarmed = false;
};

struct EventStruct {
  std::string event_id;
  std::string message_if_parse_error;
  std::string element_uuid_if_parse_error;
  int planned_start_time;
  int planned_duration;
};

struct MissionStruct {
  std::map< std::string, std::vector<multidrone_msgs::ShootingAction> > sequence_role_and_shooting_actions; // key: SAS_role; value: list of SA
  std::map< std::string, std::vector<std::string> > KML_strings_for_each_SA;    // key: SAS_role; value: KML string for each SA
  std::map< std::string, std::vector<std::string> > SASs_id;                    // key: SAS_role; value: list of SAS_id
  multidrone_msgs::MissionStatus mission_status;                                // Contains: mission_id, mission_status (enumerated) and error_list.
  std::string ref_event_id;
  std::string mission_role;
  std::string message_if_parse_error;
  std::string element_uuid_if_parse_error;
};

struct MapKey {                                   // Struct used in the keys of the maps validated_plan_ and selected_plan_.
  std::string mission_role;
  std::map< std::string, std::string > sas_roles; // key: mission_id (std::string); value: SAS_role (std::string)

  bool operator<(const MapKey& other) const {     // This struct will be used as keys of maps, so it's necessary to define the operator "<" for the map sorting.
    return mapKey2String(*this) < mapKey2String(other);
  }

  inline std::string mapKey2String(const MapKey& map_key) const {
    std::string output = map_key.mission_role;
    for (auto& s: map_key.sas_roles) {
      output += s.first + s.second;
    }
    return output;
  }
};


/// Mission_Controller class that works as interface

class MissionController {

public:
  MissionController();
  ~MissionController();

private:
  ros::NodeHandle n_;
  ros::NodeHandle pnh_;


  // Callbacks
  void publishersCallBack(const ros::TimerEvent&);
  void targetCallback(const multidrone_msgs::TargetStateArray::ConstPtr &_msg);
  void actionStatusCallback(const multidrone_msgs::ActionStatus::ConstPtr &_msg, const int _drone_id);
  void dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg, const int _drone_id);
  bool eventEnrolmentXMLServiceCallback(multidrone_msgs::SendXML::Request &_req, multidrone_msgs::SendXML::Response &_res);
  bool missionEnrolmentXMLServiceCallback(multidrone_msgs::SendXML::Request &_req, multidrone_msgs::SendXML::Response &_res);
  bool validateMissionServiceCallback(multidrone_msgs::ValidateMission::Request &_req, multidrone_msgs::ValidateMission::Response &_res);
  bool selectRoleServiceCallback(multidrone_msgs::SelectRole::Request &_req, multidrone_msgs::SelectRole::Response &_res);
  bool clearServiceCallback(multidrone_msgs::Clear::Request &_req, multidrone_msgs::Clear::Response &_res);
  bool alarmCallback(multidrone_msgs::SupervisorAlarm::Request &_req, multidrone_msgs::SupervisorAlarm::Response &_res);
  bool directorEventCallBack(multidrone_msgs::DirectorEvent::Request &_req, multidrone_msgs::DirectorEvent::Response &_res);
  void eventReceived(const multidrone_msgs::Event::ConstPtr& _event);
  void droneBateryCallback(const sensor_msgs::BatteryState::ConstPtr &_battery_status, const int _drone_id);
  void validationThread(const std::string& _stem_event_id, bool _calculate_all_combinations_or_use_selected_roles);
  void pushDroneActionsThread(const multidrone_msgs::MultidronePlan& _multidrone_plan);
  void replanningThread(void);
  bool abortServiceCallback(std_srvs::Trigger::Request &_req, std_srvs::Trigger::Response &_res);

  // Subscribers
  ros::Subscriber target_sub_;
  std::map<int, ros::Subscriber> action_status_sub_;
  std::map<int, ros::Subscriber> drone_pose_sub_;
  std::map<int, ros::Subscriber> drone_telemetry_sub_;
  ros::Subscriber event_sub_;
  ros::Subscriber dynamic_annotations_sub_;

  // Services
  ros::ServiceServer event_XML_srv_;
  ros::ServiceServer mission_XML_srv_;
  ros::ServiceServer validate_mission_srv_;
  ros::ServiceServer select_role_srv_;
  ros::ServiceServer clear_srv_;
  ros::ServiceServer supervisor_alarm_srv_;
  ros::ServiceServer event_service_;
  ros::ServiceServer abort_service_;

  // Clients
  ros::ServiceClient KML_client_;
  ros::ServiceClient post_kml_client_;
  ros::ServiceClient get_kml_client_;
  std::map<int, ros::ServiceClient> push_drone_actions_client_;
  ros::ServiceClient supervisor_check_client_;
  std::map<int, ros::ServiceClient> abort_service_client_;
  std::map<int, ros::ServiceClient> alarm_service_scheduler_;


  // Publishers
  ros::Publisher event_pub_;
  ros::Publisher system_status_pub_;

  // Timer for publishers
  ros::Timer timer_;

  // Threads
  std::thread spin_thread_;
  std::thread validation_thread_;
  std::thread push_drone_actions_thread_;
  std::thread replanning_thread_;

  std::atomic<bool> validation_running_ = {false};
  std::atomic<bool> stop_current_validation_ = {false};
  std::atomic<bool> replanning_running_ = {false};
  std::atomic<bool> stop_current_replanning_ = {false};
  std::atomic<bool> push_drone_actions_running_ = {false};


  // Other mission controller class attributes:
  std::string stem_event_id_;   // Once an event is introduced, all the next events must have the same stem event id or else won't be enrolled.
  std::string time_zone_;       // Once an event introduced has time zone, all the next events must have the same time zone or else won't be enrolled.
  std::string absolute_origin_of_time_string_;    // Received by ros parameter. If too back in time there will be errors in the trajectories because of time resolution in header.stamp.sec of the PointStamped. If in the future (negative times) there will be errors.
  std::map<std::string,EventStruct>   events_;    // Map which keys are event's id (string) and values are its EventStruct. Built in enrolment time.
  std::map<std::string,MissionStruct> missions_;  // Map which keys are missions's id (string) and values are its MissionStruct. Built in enrolment time.
  std::vector<int> drones_id_;                    // drones [id_1,...,id_N] will be received with rosparam

  // (lat,lon,alt)=(0,0,0) if no origin of coordinate parameters given. That point is in nowhere (someplace in the Atlantic Ocean).
  // When that happens (pre-production), the first waypoint found in the first mission enrolled will be considered the geographic origin of coordinate.
  geographic_msgs::GeoPoint origin_coordinates_geo_;
  geographic_msgs::GeoPoint initial_origin_coordinates_geo_;

  HighLevelPlanner high_level_planner_;

  // Director event id (request of the director event service) that take off the drones and get them ready when receibed:
  std::string get_ready_event_;

  std::map< MapKey, std::vector<multidrone_msgs::ShootingAction> > validated_shooting_action_lists_;
  std::map< MapKey, multidrone_msgs::MultidronePlan > validated_plan_;
  MapKey selected_roles_;
  multidrone_msgs::MultidronePlan selected_plan_;

  std::map<int, DroneState> drone_status_list_;

  std::map<std::string, ros::Time> running_missions_;  // map to check the end of the missions (key: mission_id of running missions, value: last time mission was running)
  std::map<int, ros::Time> drones_last_pose_time_;     // map to check if there are drone poses not defined (key: drone id, value: last time drone gived pose)

  // Event manager:
  std::atomic<bool> event_received_ = {false};
  std::string event_received_id_;
  ros::Time event_received_time_stamp_;


  geometry_msgs::Point target_pos_msg_;
  geometry_msgs::Point vehicle_pos_msg_;

}; // MissionController class

} // namespace multidrone

#endif // MISSION_CONTROLLER_H