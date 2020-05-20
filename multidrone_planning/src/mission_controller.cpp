/**
 * MULTIDRONE Project:
 *
 * Mission controller.
 * 
 */

#include <multidrone_planning/mission_controller.h>

#include <fstream>
#include <functional>
#include <ros/package.h>

#include <multidrone_msgs/PushDroneAction.h>
#include <multidrone_msgs/SafetyCheck.h>
#include <multidrone_msgs/SystemStatus.h>
#include <multidrone_msgs/MissionError.h>
#include <multidrone_msgs/DroneActionStatus.h>
#include <multidrone_planning/multidrone_xml_parser.h>

#define END_MISSION_THRESHOLD 2
#define DRONE_POSE_NOT_DEFINED_THRESHOLD 10

namespace multidrone {


/** Constructor
 */
MissionController::MissionController() {

  pnh_ = ros::NodeHandle("~");

  // Read parameters
  pnh_.getParam("drones", drones_id_);   // drones [id_1,...,id_N]
  pnh_.getParam("origin_latitude", initial_origin_coordinates_geo_.latitude);
  pnh_.getParam("origin_longitude", initial_origin_coordinates_geo_.longitude);
  pnh_.getParam("origin_altitude", initial_origin_coordinates_geo_.altitude);
  pnh_.param<std::string>("absolute_origin_of_time_string", absolute_origin_of_time_string_, "2019-01-01T00:00:00Z");
  pnh_.param<std::string>("get_ready_event", get_ready_event_, "GET_READY");

  origin_coordinates_geo_ = initial_origin_coordinates_geo_;

  // Subscribers
  target_sub_ = n_.subscribe("targets_pose", 10, &MissionController::targetCallback, this);
  event_sub_ = n_.subscribe<multidrone_msgs::Event>("mission_controller/event", 10, &MissionController::eventReceived, this);

  for (int i = 0; i < drones_id_.size(); i++) {
    action_status_sub_[drones_id_[i]] = n_.subscribe<multidrone_msgs::ActionStatus>("drone_" + std::to_string(drones_id_[i]) + "/action_status", 10, std::bind(&MissionController::actionStatusCallback, this, std::placeholders::_1, drones_id_[i])); // Change for each drone ID
    drone_pose_sub_[drones_id_[i]] = n_.subscribe<geometry_msgs::PoseStamped>("drone_" + std::to_string(drones_id_[i]) + "/ual/pose", 10, std::bind(&MissionController::dronePoseCallback, this, std::placeholders::_1, drones_id_[i]));               // Change for each drone ID
    drone_telemetry_sub_[drones_id_[i]] = n_.subscribe<sensor_msgs::BatteryState>("drone_"+ std::to_string(drones_id_[i]) +"/mavros/battery", 10,std::bind(&MissionController::droneBateryCallback, this, std::placeholders::_1, drones_id_[i]));

    // Push drone action client. The client is the mission controller, the server is the scheduler.
    push_drone_actions_client_[drones_id_[i]] = n_.serviceClient<multidrone_msgs::PushDroneAction>("drone_" + std::to_string(drones_id_[i]) + "/push_drone_action");

    abort_service_client_[drones_id_[i]] = n_.serviceClient<std_srvs::Trigger>("drone_" + std::to_string(drones_id_[i]) + "/abort");
    alarm_service_scheduler_[drones_id_[i]] = n_.serviceClient<multidrone_msgs::SupervisorAlarm>("drone_" + std::to_string(drones_id_[i]) + "/emergency");

  }

  // Publishers
  event_pub_ = n_.advertise<multidrone_msgs::Event>("mission_controller/event", 1);
  system_status_pub_ = n_.advertise<multidrone_msgs::SystemStatus>("mission_controller/system_status", 1);

  // Service clients
  post_kml_client_ = n_.serviceClient<multidrone_msgs::PostSemanticAnnotations>("post_semantic_annotations");
  get_kml_client_ = n_.serviceClient<multidrone_msgs::GetSemanticMap>("get_semantic_map");
  supervisor_check_client_ = n_.serviceClient<multidrone_msgs::SafetyCheck>("supervisor/safety_check");


  // Advertised services
  event_service_ = n_.advertiseService("mission_controller/director_event", &MissionController::directorEventCallBack, this);
  event_XML_srv_ = n_.advertiseService("mission_controller/send_event_xml", &MissionController::eventEnrolmentXMLServiceCallback, this);
  mission_XML_srv_ = n_.advertiseService("mission_controller/send_mission_xml", &MissionController::missionEnrolmentXMLServiceCallback, this);
  validate_mission_srv_ = n_.advertiseService("mission_controller/validate", &MissionController::validateMissionServiceCallback, this);
  select_role_srv_ = n_.advertiseService("mission_controller/select_role", &MissionController::selectRoleServiceCallback, this);
  clear_srv_ = n_.advertiseService("mission_controller/clear", &MissionController::clearServiceCallback, this);
  supervisor_alarm_srv_ = n_.advertiseService("execution/alarm", &MissionController::alarmCallback, this);
  abort_service_ = n_.advertiseService("mission_controller/abort", &MissionController::abortServiceCallback, this);

  // connecting to the drones
  // Service TODO: The mission controller should try to connect to the onboard scheduler and provide information about it, instead blocking


  for (int i = 0; i < drones_id_.size(); i++) {
    if ( !ros::service::waitForService("drone_" + std::to_string(drones_id_[i]) + "/push_drone_action",5) ) {
      ROS_ERROR("Mission Controller: Service drone_%d/push_drone_action is not available.", drones_id_[i]);
    } else {
      ROS_INFO("Mission Controller: Service drone_%d/push_drone_action is ready.", drones_id_[i]);
    }
    if ( !ros::service::waitForService("drone_" + std::to_string(drones_id_[i]) + "/abort",5) ) {
      ROS_ERROR("Mission Controller: Service drone_%d/abort is not available.", drones_id_[i]);
    } else {
      ROS_INFO("Mission Controller: Service drone_%d/abort is ready.", drones_id_[i]);
    }
  }

  //timer callback for system status
  timer_ = n_.createTimer(ros::Duration(1), &MissionController::publishersCallBack,this); // 1 Hz


  ROS_INFO("Mission Controller running!");

  // Make communications spin!
  spin_thread_ = std::thread([this]() {
    ros::MultiThreadedSpinner spinner(2); // Use 2 threads
    spinner.spin();
  });

} // end MissionController constructor



// Brief Destructor
MissionController::~MissionController() {
  if(validation_thread_.joinable()) validation_thread_.join();
  if(push_drone_actions_thread_.joinable()) push_drone_actions_thread_.join();
  if(replanning_thread_.joinable()) replanning_thread_.join();
  if(spin_thread_.joinable()) spin_thread_.join();
} // end ~MissionController destructor



/**\ Brief abort service callback
*/
bool MissionController::abortServiceCallback(std_srvs::Trigger::Request &_req, std_srvs::Trigger::Response &_res){

  std_srvs::Trigger srv;
  std::vector<bool> list_of_errors;
  for (std::map<int, ros::ServiceClient>::iterator it=abort_service_client_.begin(); it!=abort_service_client_.end(); ++it) {
    if (it->second.call(srv)) {
      list_of_errors.push_back(false);
    } else {
      ROS_ERROR("Mission Controller: Failed to call Scheduler %d abort service", it->first);
      list_of_errors.push_back(true);
    }
  }

  // Set as aborted the mission status of the ready or running missions:
  for (std::map<std::string,MissionStruct>::iterator it = missions_.begin(); it != missions_.end(); it++) {
    if (it->second.mission_status.status == multidrone_msgs::MissionStatus::STATUS_READY || it->second.mission_status.status == multidrone_msgs::MissionStatus::STATUS_RUNNING) {
      it->second.mission_status.status = multidrone_msgs::MissionStatus::STATUS_ABORTED;
    }
  }

  // check success:
  _res.success = true;
  _res.message = "Abort drones successful.";
  for(int i=0; i<list_of_errors.size(); ++i) {
    if(list_of_errors[i]) {
      _res.success = false;
      _res.message = "Couldn't abort all drones.";
      break;
    }
  }
  return true;
} // end abortServiceCallback



/** Publishers callback
 *  MC periodically reports the status of the system, and when a director event is received, sends it through its topic once.
 */
void MissionController::publishersCallBack(const ros::TimerEvent&) {
  
  // Check if any mission has finished:
  std::vector<std::string> missions_to_delete_from_running_missions;
  for (std::map<std::string,ros::Time>::iterator it = running_missions_.begin(); it != running_missions_.end(); it++) {
    if (missions_[it->first].mission_status.status == multidrone_msgs::MissionStatus::STATUS_ABORTED) continue;
    int time_from_last_reception = ros::Time::now().sec - it->second.sec;
    if (time_from_last_reception > END_MISSION_THRESHOLD) {
      missions_[it->first].mission_status.status = multidrone_msgs::MissionStatus::STATUS_ENDED;
      missions_to_delete_from_running_missions.push_back(it->first);
    }
  }
  for (int i=0; i<missions_to_delete_from_running_missions.size(); i++) {
    running_missions_.erase(missions_to_delete_from_running_missions[i]);
  }

  // Check if any drone has been too much time without giving its pose:
  for (std::map<int,ros::Time>::iterator it = drones_last_pose_time_.begin(); it != drones_last_pose_time_.end(); it++) {
    int time_from_last_reception = ros::Time::now().sec - it->second.sec;
    if (time_from_last_reception > DRONE_POSE_NOT_DEFINED_THRESHOLD) {
      drone_status_list_[it->first].drone_pose_defined = false;
    }
  }

  multidrone_msgs::SystemStatus system_status_to_publish;

  // filling the event states
  for (std::map<std::string,EventStruct>::iterator it = events_.begin(); it != events_.end(); it++) {
    system_status_to_publish.events.push_back(it->second.event_id);
  }
  //filling the mission status
  for (std::map<std::string,MissionStruct>::iterator it = missions_.begin(); it != missions_.end(); it++) {
    it->second.mission_status.mission_id = it->first;
    system_status_to_publish.missions.push_back(it->second.mission_status);
  }
  // filling drone action status
  for (int i : drones_id_) {
    multidrone_msgs::DroneActionStatus current_drone_action_status;
    current_drone_action_status.drone_id  =  i;
    current_drone_action_status.mission_id = drone_status_list_[i].action_status.mission_id;
    current_drone_action_status.SAS_id    =  drone_status_list_[i].action_status.SAS_id;
    current_drone_action_status.action_id =  drone_status_list_[i].action_status.action_id;
    current_drone_action_status.status    =  drone_status_list_[i].action_status.status;
    current_drone_action_status.message   =  drone_status_list_[i].action_status.message;
    system_status_to_publish.drones.push_back(current_drone_action_status);
  }

  system_status_pub_.publish( system_status_to_publish );


  if (event_received_) {
    event_received_ = false;
    multidrone_msgs::Event event_msg;
    event_msg.event_id = event_received_id_;
    event_msg.header.stamp = event_received_time_stamp_;
    event_pub_.publish(event_msg);
  }

} // end publishersCallBack



/** Event enrolment callback
 *  MC receives the event's XML from the Dashboard
 */
bool MissionController::eventEnrolmentXMLServiceCallback(multidrone_msgs::SendXML::Request &_req, multidrone_msgs::SendXML::Response &_res) {

  ROS_INFO("Mission Controller: Event enrolment service received.");

  // Parse the XML string to get the event structure.
  EventStruct new_event = multidrone::XMLparser::event_XML_parser(_req.xml, events_, time_zone_, stem_event_id_, absolute_origin_of_time_string_);

  if (new_event.event_id.size()==0) { // New event with parse error.
    _res.success = false;
    _res.message = new_event.message_if_parse_error;
    _res.element_uuid = new_event.element_uuid_if_parse_error;
    ROS_WARN("Mission Controller: Event not enrolled. %s", _res.message.c_str());

  } else {  // New event is valid
    _res.success = true;

    if (events_.count(new_event.event_id)==0) {   // event not enrolled yet
      _res.message = "Event successfully enrolled.";
    } else {                                      // event already enrolled
      _res.message = "Event already enrolled, overwriting.";
    }

    ROS_INFO("Mission Controller: %s", _res.message.c_str());

    events_[new_event.event_id] = new_event;
  }

  return true;

} // end eventEnrolmentXMLServiceCallback



/** Mission enrolment callback
*  MC receives the mission's XML from the Dashboard, and if its corresponding leaf event has been already sent, the mission is enrolled.
*/
bool MissionController::missionEnrolmentXMLServiceCallback(multidrone_msgs::SendXML::Request &_req, multidrone_msgs::SendXML::Response &_res) {

  ROS_INFO("Mission Controller: Mission enrolment service received.");

  // Parse the XML string to get the mission structure.
  MissionStruct new_mission = multidrone::XMLparser::mission_XML_parser(_req.xml, events_, origin_coordinates_geo_);

  // mission_XML_parser may have changed the origin_coordinates_geo_ if this one was empty (0,0,0). In this case the server parameters will be updated:
  if (origin_coordinates_geo_.latitude != initial_origin_coordinates_geo_.latitude || origin_coordinates_geo_.longitude != initial_origin_coordinates_geo_.longitude || origin_coordinates_geo_.altitude != initial_origin_coordinates_geo_.altitude) {
    pnh_.setParam("origin_latitude", origin_coordinates_geo_.latitude);
    pnh_.setParam("origin_longitude", origin_coordinates_geo_.longitude);
    pnh_.setParam("origin_altitude", origin_coordinates_geo_.altitude);
  }

  if (new_mission.ref_event_id.size()==0) { // New mission with parse error.
    _res.success = false;
    _res.message = new_mission.message_if_parse_error;
    _res.element_uuid = new_mission.element_uuid_if_parse_error;
    ROS_WARN("Mission Controller: Mission not enrolled. %s", _res.message.c_str());

  } else { // New mission is valid
    _res.success = true;

    if (missions_.count(new_mission.mission_status.mission_id)==0) { // if mission is not enrolled

      // Check if there is any other mission in this leaf event with the same mission_role:
      for (std::map<std::string,MissionStruct>::iterator it_mis = missions_.begin(); it_mis != missions_.end(); it_mis++) {
        if (it_mis->second.ref_event_id == new_mission.ref_event_id && it_mis->second.mission_role == new_mission.mission_role) {
          _res.success = false;
          _res.message = "Mission not enrolled because there is another mission with the same mission_role in this leaf event.";
          _res.element_uuid = new_mission.mission_status.mission_id;
          ROS_WARN("Mission Controller: %s", _res.message.c_str());
          return true;
        }
      }

      _res.message = "Mission successfully enrolled." + new_mission.message_if_parse_error;

    } else { //if mission is already enrolled
      _res.message = "Mission already enrolled, overwriting." + new_mission.message_if_parse_error;
    }

    ROS_INFO("Mission Controller: %s", _res.message.c_str());

    missions_[new_mission.mission_status.mission_id] = new_mission;
    missions_[new_mission.mission_status.mission_id].mission_status.status = multidrone_msgs::MissionStatus::STATUS_ENROLLED;
  }

  return true;

} // end missionEnrolmentXMLServiceCallback



/** Validation thread
 */
void MissionController::validationThread(const std::string& _stem_event_id, bool _calculate_all_combinations_or_use_selected_roles) {
  // In the end it's seems that the stem_event_id won't be required for the validation.

  validation_running_ = true;

  int trigger_time_of_replanning = -1;

  if (_calculate_all_combinations_or_use_selected_roles == false) {
    ROS_INFO("Mission Controller: Validation service received. Validation thread calculating and printing all the combinations of mission_role, mission_id and SAS_role possible:");
  } else {
    ROS_INFO("Mission Controller: Validation service received. Validation thread will only use the combination of roles selected previously for replanning.");
    if (events_.count(event_received_id_)==1) {
      trigger_time_of_replanning = events_[event_received_id_].planned_start_time + ros::Time::now().sec - event_received_time_stamp_.sec;
    } else {
      ROS_WARN("Mission Controller: Replanning when no director event has been sent.");
      trigger_time_of_replanning = 0;
    }
  }

  // Not using the original attribute to avoid race conditions during this thread:
  auto drone_status_list = drone_status_list_;

  // KML stuff:
  multidrone_msgs::GetSemanticMap kml_srv;
  if (get_kml_client_.call(kml_srv)) {
    if (!kml_srv.response.semantic_map.empty()) {
      ROS_INFO("Mission Controller: KML received.");
    } else {
      ROS_WARN("Mission Controller: KML received empty.");
    }
  } else {
    ROS_WARN("Mission Controller: KML service not available.");
  }
  std::string incoming_KML = kml_srv.response.semantic_map;

  // Start conditions of the drones for the High-Level Planner .getPlan()
  std::map< int,std::tuple<geometry_msgs::Point32,double,bool> > initial_state_of_drones; // map of tuples. The keys are the drone ids, the values a tuple with three elements each oen. The first in the tuple is the pose of the drone, the second its battery and the third a boolean that is false if the drone is in the ground and true if flying.
  for (int current_drone_id : drones_id_) {
    if ( drone_status_list[ current_drone_id ].drone_pose_defined == true && drone_status_list[ current_drone_id ].drone_alarmed == false ) {
      std::tuple<geometry_msgs::Point32,double,bool> new_tuple;
      std::get<0>(new_tuple).x = drone_status_list[ current_drone_id ].pose.position.x;
      std::get<0>(new_tuple).y = drone_status_list[ current_drone_id ].pose.position.y;
      std::get<0>(new_tuple).z = drone_status_list[ current_drone_id ].pose.position.z;
      std::get<1>(new_tuple) = drone_status_list[ current_drone_id ].battery_remaining;
      if (drone_status_list[ current_drone_id ].action_status.status != multidrone_msgs::ActionStatus::AS_IDLE && drone_status_list[ current_drone_id ].action_status.status != multidrone_msgs::ActionStatus::AS_WAIT_FOR_SAFE_TO_GO && drone_status_list[ current_drone_id ].action_status.status != multidrone_msgs::ActionStatus::AS_WAIT_FOR_GET_READY) {
        std::get<2>(new_tuple) = true;    // If the drone is not idle, or waiting for safe to go or get ready means that the drone is flying, so this is true.
      } else {
        std::get<2>(new_tuple) = false;
      }
      initial_state_of_drones[ current_drone_id ] = new_tuple;
    }
  }

  if (initial_state_of_drones.size()==0) {
    ROS_WARN("Mission Controller: No drone is running. Drones position unknown. Try again when drones are running.");
  } else {

    validated_plan_.clear();

    if (_calculate_all_combinations_or_use_selected_roles == false) {
      validated_shooting_action_lists_.clear();

      // Build "table_of_missions_and_SASs" which is a map which keys are all the mission_role possible and values another map which keys are all the mission_id (for that mission_role) and values all the SAS_struct of <SAS_role,vector of SA in that SAS,auxiliar bool variable> (for that mission_id):
      struct SAS_struct {
        std::vector<multidrone_msgs::ShootingAction> SA_list_in_SAS;
        std::string SAS_role = "-1";
        bool combinatory_focus = false;
      };
      std::map< std::string, std::map< std::string, std::vector< SAS_struct > > > table_of_missions_and_SASs;
      for (std::map<std::string,MissionStruct>::iterator it = missions_.begin(); it != missions_.end(); it++) {
        it->second.mission_status.status = multidrone_msgs::MissionStatus::STATUS_PLANNING; // mission status
        it->second.mission_status.error_list.clear();
        it->second.mission_status.selected_sas_ids.clear();
        for (std::map< std::string, std::vector<multidrone_msgs::ShootingAction> >::iterator it_2 = it->second.sequence_role_and_shooting_actions.begin(); it_2 != it->second.sequence_role_and_shooting_actions.end(); it_2++) {
          SAS_struct current_SA_struct;
          current_SA_struct.SA_list_in_SAS = it_2->second;
          current_SA_struct.SAS_role       = it_2->first;
          current_SA_struct.combinatory_focus = it_2==it->second.sequence_role_and_shooting_actions.begin() ? true : false;    // Initially, the focus of the combinatory will be at the first SAS_role of the mission_id.
          table_of_missions_and_SASs[it->second.mission_role][it->second.mission_status.mission_id].push_back(current_SA_struct);
        }
        SAS_struct empty_SA_struct;   // Empty SA struct, when combinatory focus here means that the mission_id is ignored from the combination.
        table_of_missions_and_SASs[it->second.mission_role][it->second.mission_status.mission_id].push_back(empty_SA_struct);
      }

      // Build "validated_shooting_action_lists_" which is all the possible mission-SAS roles combinatory with its shooting action lists:
      for (std::map< std::string, std::map< std::string, std::vector< SAS_struct > > >::iterator it_1 = table_of_missions_and_SASs.begin(); it_1 != table_of_missions_and_SASs.end(); it_1++) {
        std::string mission_role = it_1->first;
        bool keep_in_loop = true;
        while (keep_in_loop) {
          // Create new combination and its SA list:
          MapKey combination;
          combination.mission_role = mission_role;
          std::vector<multidrone_msgs::ShootingAction> SA_list_of_this_combination;
          for (std::map< std::string, std::vector< SAS_struct > >::iterator it_2 = it_1->second.begin(); it_2 != it_1->second.end(); it_2++) {
            std::string mission_id = it_2->first;
            for (int i=0; i<it_2->second.size(); i++) {
              if (it_2->second[i].combinatory_focus) {   // For each mission_id, consider only the SAS_role with combinatory_focus == true.
                if (it_2->second[i].SAS_role!="-1") {
                  std::string SAS_role = it_2->second[i].SAS_role;
                  combination.sas_roles[mission_id] = SAS_role;
                  SA_list_of_this_combination.insert( SA_list_of_this_combination.end(), it_2->second[i].SA_list_in_SAS.begin(), it_2->second[i].SA_list_in_SAS.end() );
                  break;
                }
              }
            }
          }
          if (combination.sas_roles.size()==0) break; // If empty combination break.

          // Insert new combination and SA list in "validated_shooting_action_lists_":
          validated_shooting_action_lists_[combination] = SA_list_of_this_combination;
          // Toggle the combinatory focus of the SASs:
          bool toggle_from_last_to_first = false;   // True if the previous mission_id changed from the last SAS_role to the first one.
          for (std::map< std::string, std::vector< SAS_struct > >::iterator it_2 = it_1->second.begin(); it_2 != it_1->second.end(); it_2++) {
            for (int i=0; i<it_2->second.size(); i++) {
              if(it_2->second[i].combinatory_focus) {
                it_2->second[i].combinatory_focus = false;
                if (i+1==it_2->second.size()) {   // Next SAS_role is the first one
                  it_2->second[0].combinatory_focus = true;
                  toggle_from_last_to_first = true;
                  break;
                } else {
                  it_2->second[i+1].combinatory_focus = true;
                  toggle_from_last_to_first = false;
                  break;
                }
              }
            }
            if (!toggle_from_last_to_first) { // Only keep iterating to the next mission_id if this one changed its combinatory focus from the last SAS_role to the first_one.
              break;
            }
          }
          if (toggle_from_last_to_first) {    // If the for loop through missions_id ended with toggle_from_last_to_first==true means that all the combinatory has been built, exit while loop.
            keep_in_loop = false;
          }
          ROS_INFO("%s", combination.mapKey2String(combination).c_str() );
        }
      }

      ROS_INFO("Mission Controller: Validation thread calling the High-Level Planner for all the combinations.");

    } else {
      // If _calculate_all_combinations_or_use_selected_roles == true then all the elements in validated_shooting_action_lists_ but the one of the selected roles will be cleared.

      std::vector<multidrone_msgs::ShootingAction> SA_list_of_the_selected_combination_of_roles = validated_shooting_action_lists_[selected_roles_];
      validated_shooting_action_lists_.clear();
      validated_shooting_action_lists_[selected_roles_] = SA_list_of_the_selected_combination_of_roles;

      for (std::map<std::string,MissionStruct>::iterator it = missions_.begin(); it != missions_.end(); it++) {
        it->second.mission_status.status = multidrone_msgs::MissionStatus::STATUS_PLANNING; // mission status
        it->second.mission_status.error_list.clear();
        it->second.mission_status.selected_sas_ids.clear();
      }
    }

    // Compute all the MultidronePlan with the High-Level Planner, one for each list of Shooting Actions (possible mission-SAS roles combinatory).
    for (std::map< MapKey, std::vector<multidrone_msgs::ShootingAction> >::iterator it = validated_shooting_action_lists_.begin(); it != validated_shooting_action_lists_.end(); it++) {
      if (!stop_current_validation_) {

        // If this combination contains one mission which mission status is rejected, skip and don't plan this combination:
        bool dont_plan_this_combination = false;
        for(auto& k: it->first.sas_roles) {
          if (missions_[k.first].mission_status.status == multidrone_msgs::MissionStatus::STATUS_REJECTED) {
            dont_plan_this_combination = true;
            break;
          }
        }
        if (dont_plan_this_combination) {
          ROS_INFO("%s without plan.", it->first.mapKey2String(it->first).c_str() );
          continue;
        }

        std::map< int, std::vector<multidrone_msgs::DroneAction> > current_getplan_plan = high_level_planner_.getPlan(it->second, initial_state_of_drones, origin_coordinates_geo_, incoming_KML, trigger_time_of_replanning);

        if(current_getplan_plan.size()>0) { // Only save non-empty plans. Plans can be empty if no solution is possible given the initial conditions.
          ROS_INFO("%s planned successfully, pending supervisor check.", it->first.mapKey2String(it->first).c_str() );

          // This block of code is because now with the new messages the output of getPlan is not exactly the same of the plans. Better change the getPlan method output:
          multidrone_msgs::MultidronePlan current_multidrone_plan;
          for (std::map< int, std::vector<multidrone_msgs::DroneAction> >::iterator it_2 = current_getplan_plan.begin(); it_2 != current_getplan_plan.end(); it_2++) {
            multidrone_msgs::DroneActionArray current_drone_action_array;
            current_drone_action_array.drone_id = it_2->first;
            current_drone_action_array.actions = it_2->second;
            current_multidrone_plan.plan.push_back(current_drone_action_array);
          }

          // setting the state of the planned missions_
          for(auto& k: it->first.sas_roles) {
            missions_[k.first].mission_status.status = multidrone_msgs::MissionStatus::STATUS_SUPERVISING;
          }

          validated_plan_[it->first] = current_multidrone_plan;
        } else {
          // Empty plan, mission not possible, cancel the whole mission thread:
          ROS_WARN("%s not planned.", it->first.mapKey2String(it->first).c_str() );

          std::string mission_role_with_error = it->first.mission_role;
          for (std::map<std::string,MissionStruct>::iterator it_mis = missions_.begin(); it_mis != missions_.end(); it_mis++) {
            if (it_mis->second.mission_role == mission_role_with_error) {
              missions_[it_mis->first].mission_status.status = multidrone_msgs::MissionStatus::STATUS_REJECTED;

              // MissionError type planning.
              multidrone_msgs::MissionError new_planning_error;
              new_planning_error.type = multidrone_msgs::MissionError::ERROR_TYPE_PLANNER;
              new_planning_error.message = "This mission is in a rejected mission thread with unfeasible plan.";

              for(auto& k: it->first.sas_roles) {
                multidrone_msgs::SASRole mission_id_sas_role;
                mission_id_sas_role.mission_id = k.first;
                mission_id_sas_role.sas_role = k.second;
                new_planning_error.sas_roles.push_back(mission_id_sas_role);
              }

              missions_[it_mis->first].mission_status.error_list.push_back(new_planning_error);
            }
          }
        }
      } else {
        ROS_WARN("Mission Controller: New validation request received, aborting the current validation to start the new one as soon as possible.");
        validation_running_ = false;
        stop_current_validation_ = false;
        return;
      }
    }

    // Supervisor's safety check:
    if (supervisor_check_client_.exists()) {
      ROS_INFO("Mission Controller: Validation thread calling the supervisor for all the validated plans.");
      std::vector<MapKey> plans_to_delete;
      for (std::map< MapKey, multidrone_msgs::MultidronePlan >::iterator it = validated_plan_.begin(); it != validated_plan_.end(); it++) {
        if (!stop_current_validation_) {
          // If this combination contains one mission which mission status is rejected, skip and reject directly the combination:
          bool dont_supervise_this_combination = false;
          for(auto& k: it->first.sas_roles) {
            if (missions_[k.first].mission_status.status == multidrone_msgs::MissionStatus::STATUS_REJECTED) {
              dont_supervise_this_combination = true;
              plans_to_delete.push_back(it->first);
              break;
            }
          }
          if (dont_supervise_this_combination) {
            ROS_INFO("%s rejected.", it->first.mapKey2String(it->first).c_str() );
            continue;
          }
          multidrone_msgs::SafetyCheck srv;
          srv.request.plan = it->second;
          if (supervisor_check_client_.call(srv)) {
            if (srv.response.result == multidrone_msgs::SafetyCheck::Response::SAFETY_OK) {
              ROS_INFO("Mission Controller: Safety check received from Supervisor. The current plan is OK.");
              for(auto& k: it->first.sas_roles) {
                missions_[k.first].mission_status.status = multidrone_msgs::MissionStatus::STATUS_VALIDATED;
              }
            } else if (srv.response.result == multidrone_msgs::SafetyCheck::Response::SAFETY_ERROR) {
              ROS_WARN("Mission Controller: Safety check received from Supervisor. The current plan is NOT OK, discarting current plan.");
              ROS_WARN("%s rejected.", it->first.mapKey2String(it->first).c_str() );
              plans_to_delete.push_back(it->first);
              std::string mission_role_with_error = it->first.mission_role;
              for (std::map<std::string,MissionStruct>::iterator it_mis = missions_.begin(); it_mis != missions_.end(); it_mis++) {
                if (it_mis->second.mission_role == mission_role_with_error) {
                  missions_[it_mis->first].mission_status.status = multidrone_msgs::MissionStatus::STATUS_REJECTED;
                  // MissionError type safety
                  multidrone_msgs::MissionError new_safety_error;
                  new_safety_error.type = multidrone_msgs::MissionError::ERROR_TYPE_SAFETY;
                  new_safety_error.message = "This mission is in a rejected mission thread by the supervisor in this combination.";
                  for(auto& k: it->first.sas_roles) {
                    multidrone_msgs::SASRole mission_id_sas_role;
                    mission_id_sas_role.mission_id = k.first;
                    mission_id_sas_role.sas_role = k.second;
                    new_safety_error.sas_roles.push_back(mission_id_sas_role);
                  }
                  missions_[it_mis->first].mission_status.error_list.push_back(new_safety_error);
                }
              }
            }
          } else {
            ROS_WARN("Mission Controller: Safety check not received.");
          }
        } else {
          ROS_WARN("Mission Controller: New validation request received, aborting the current supervision to start the new validation as soon as possible.");
          validation_running_ = false;
          stop_current_validation_ = false;
          return;
        }
      }
      for (auto plan_to_delete: plans_to_delete) {
        validated_plan_.erase(plan_to_delete);
      }
    } else {
      ROS_WARN("Mission Controller: Supervisor not available, all missions to supervise will be automatically validated.");
      for (std::map<std::string,MissionStruct>::iterator it = missions_.begin(); it != missions_.end(); it++) {
        if (it->second.mission_status.status == multidrone_msgs::MissionStatus::STATUS_SUPERVISING) {
          it->second.mission_status.status = multidrone_msgs::MissionStatus::STATUS_VALIDATED;
        }
      }
    }

    // When _calculate_all_combinations_or_use_selected_roles == true some missions may be left planning because they are not used, mark them as validated.
    if (_calculate_all_combinations_or_use_selected_roles == true) {
      for (std::map<std::string,MissionStruct>::iterator it = missions_.begin(); it != missions_.end(); it++) {
        if (it->second.mission_status.status == multidrone_msgs::MissionStatus::STATUS_PLANNING || it->second.mission_status.status == multidrone_msgs::MissionStatus::STATUS_SUPERVISING) {
          it->second.mission_status.status = multidrone_msgs::MissionStatus::STATUS_VALIDATED;
        }
      }
    }

    ROS_INFO("Mission Controller: Validation completed.");
  } // end if initial_state_of_drones.size()>0

  validation_running_ = false;
} // end validationThread



/** Validation callback 
 */
bool MissionController::validateMissionServiceCallback(multidrone_msgs::ValidateMission::Request &_req, multidrone_msgs::ValidateMission::Response &_res) {

  // Starting validation thread that calculates all the combinations of roles (_calculate_all_combinations_or_use_selected_roles=false).
  if(validation_running_) stop_current_validation_ = true;   // If still validating, end the current validation to start the new one as soon as possible.
  if(validation_thread_.joinable()) validation_thread_.join();
  validation_thread_ = std::thread(&MissionController::validationThread, this, _req.stem_event_id, false);
  return true;

} // end validateMissionServiceCallback



/** Push drone actions thread
 */
void MissionController::pushDroneActionsThread(const multidrone_msgs::MultidronePlan& _multidrone_plan){
  push_drone_actions_running_ = true;

  for (int i=0; i<_multidrone_plan.plan.size(); i++) {
    multidrone_msgs::PushDroneAction srv;
    srv.request.action_list = _multidrone_plan.plan[i].actions;
    if (push_drone_actions_client_[_multidrone_plan.plan[i].drone_id].call(srv)) {
      ROS_INFO("Mission Controller: Push drone actions to drone %d successful.", _multidrone_plan.plan[i].drone_id);
    } else {
      ROS_ERROR("Mission Controller: Push drone actions to drone %d failed.", _multidrone_plan.plan[i].drone_id);
    }
  }

  push_drone_actions_running_ = false;
} // end pushDroneActionsThread



/** Replanning when alarm service received
 */
void MissionController::replanningThread(void) {

  replanning_running_ = true;
  ROS_INFO("Mission Controller: Replanning thread running because a drone was alarmed.");

  // Starting validation thread that only calculates the selected roles (_calculate_all_combinations_or_use_selected_roles=true).
  if(validation_running_) stop_current_validation_ = true;   // If still validating, end the current validation to start the new one as soon as possible.
  if(validation_thread_.joinable()) validation_thread_.join();
  validation_thread_ = std::thread(&MissionController::validationThread, this, stem_event_id_, true);

  // Wait until the validation finish, while checking if a new replanning is needed.
  sleep(1);   // Wait a little so that the validationThread has time to start validating.
  while (validation_running_) {
    if (!stop_current_replanning_) {
      ros::Rate loop_rate(1); //[Hz]
      loop_rate.sleep();
    } else {
      stop_current_validation_ = true;
      ROS_WARN("Mission Controller: New replanning request received, aborting the current replanning to start the new one as soon as possible.");
      replanning_running_ = false;
      stop_current_replanning_ = false;
      return;
    }
  }

  // Check if the results of the validation are successful, if not just return.
  if (validated_plan_.size()==0 || validated_plan_.count(selected_roles_)==0) {
    replanning_running_ = false;
    stop_current_replanning_ = false;
    return;
  } else {
    for(auto& k: selected_roles_.sas_roles) {
      if (missions_[k.first].mission_status.status == multidrone_msgs::MissionStatus::STATUS_REJECTED) {
        replanning_running_ = false;
        stop_current_replanning_ = false;
        return;
      }
    }
  }

  // Update selected_plan_:
  selected_plan_.plan.clear();
  selected_plan_ = validated_plan_[selected_roles_];

  // Push drone actions:
  if(push_drone_actions_thread_.joinable()) push_drone_actions_thread_.join();
  push_drone_actions_thread_ = std::thread(&MissionController::pushDroneActionsThread,this,selected_plan_);

  // Change mission status of selected missions id and SASs id to ready:
  for (std::map< std::string, std::string >::iterator it = selected_roles_.sas_roles.begin(); it != selected_roles_.sas_roles.end(); it++) {
    missions_[it->first].mission_status.status = multidrone_msgs::MissionStatus::STATUS_READY;
    missions_[it->first].mission_status.selected_sas_ids.insert(missions_[it->first].mission_status.selected_sas_ids.end(), missions_[it->first].SASs_id[it->second].begin(), missions_[it->first].SASs_id[it->second].end());
  }

  if (event_received_id_.size()!=0) { // If there is a director event already sent, wait the push drone actions and send it again.
    // Wait until push drone actions complete.
    sleep(1);   // Wait a little so that the pushDroneActionsThread has time to start pushing.
    while (push_drone_actions_running_) {
      if (!stop_current_replanning_) {
        ros::Rate loop_rate(1); //[Hz]
        loop_rate.sleep();
      } else {
        stop_current_validation_ = true;
        ROS_WARN("Mission Controller: New replanning request received, aborting the current replanning to start the new one as soon as possible.");
        replanning_running_ = false;
        stop_current_replanning_ = false;
        return;
      }
    }

    event_received_ = true; // Force to publish again the last event.
  }

  ROS_INFO("Mission Controller: Replanning successful.");

  replanning_running_ = false;
} // end replanningThread



/** Select role callback
 */
bool MissionController::selectRoleServiceCallback(multidrone_msgs::SelectRole::Request &_req, multidrone_msgs::SelectRole::Response &_res) {

  ROS_INFO("Mission Controller: Select role service received.");

  bool can_select_role = true;
  for (std::map<int, DroneState>::iterator it = drone_status_list_.begin(); it != drone_status_list_.end(); it++) {
    if (it->second.action_status.status != multidrone_msgs::ActionStatus::AS_IDLE) {
      can_select_role = false;
      break;
    }
  }

  if (can_select_role) {

    if(validation_running_) {
      _res.success = false;
      _res.message = "Validation still in progress. Ignoring role select.";
      ROS_WARN("Mission Controller: %s", _res.message.c_str());
      return true;
    }

    // Roles selected to map key (validated_plan_ map):
    selected_roles_.mission_role.clear();
    selected_roles_.sas_roles.clear();
    selected_roles_.mission_role = _req.mission_role;
    for (int i=0; i< _req.sas_roles.size(); i++) {
      selected_roles_.sas_roles[_req.sas_roles[i].mission_id] = _req.sas_roles[i].sas_role;
    }

    // With the validated_plan_ map and the key of roles selected select_map_ is extracted:
    if (validated_plan_.count(selected_roles_)>0) {

      for(auto& k: selected_roles_.sas_roles) {
        if (missions_[k.first].mission_status.status == multidrone_msgs::MissionStatus::STATUS_REJECTED) {
          _res.success = false;
          _res.message = "Selected combination in a rejected mission thread. Ignoring role select.";
          ROS_WARN("Mission Controller: %s", _res.message.c_str());
          selected_roles_.mission_role.clear();
          selected_roles_.sas_roles.clear();
          return true;
        }
      }

      selected_plan_.plan.clear();
      selected_plan_ = validated_plan_[selected_roles_];

      // Push drone actions:
      if(push_drone_actions_thread_.joinable()) push_drone_actions_thread_.join();
      push_drone_actions_thread_ = std::thread(&MissionController::pushDroneActionsThread,this,selected_plan_);

      // Put all the previous ready missions (selected) back to validated:
      for (std::map<std::string,MissionStruct>::iterator it = missions_.begin(); it != missions_.end(); it++) {
        missions_[it->first].mission_status.selected_sas_ids.clear();
        if (it->second.mission_status.status==multidrone_msgs::MissionStatus::STATUS_READY) {
          missions_[it->first].mission_status.status = multidrone_msgs::MissionStatus::STATUS_VALIDATED;
        }
      }

      // Change mission status of selected missions id and SASs id to ready:
      for (std::map< std::string, std::string >::iterator it = selected_roles_.sas_roles.begin(); it != selected_roles_.sas_roles.end(); it++) {
        missions_[it->first].mission_status.status = multidrone_msgs::MissionStatus::STATUS_READY;
        missions_[it->first].mission_status.selected_sas_ids.insert(missions_[it->first].mission_status.selected_sas_ids.end(), missions_[it->first].SASs_id[it->second].begin(), missions_[it->first].SASs_id[it->second].end());
      }

      // Send KML of the chosen SAs to the post_semantic_annotations:
      for (std::map< std::string, std::string >::iterator it_1 = selected_roles_.sas_roles.begin(); it_1 != selected_roles_.sas_roles.end(); it_1++) {
        for (std::map< std::string, std::vector<std::string> >::iterator it_2 = missions_[it_1->first].KML_strings_for_each_SA.begin(); it_2 != missions_[it_1->first].KML_strings_for_each_SA.end(); it_2++) {
          for (int i=0; i<it_2->second.size(); i++) {
            multidrone_msgs::PostSemanticAnnotations kml_srv;
            kml_srv.request.semantic_map = it_2->second[i];
            kml_srv.request.supervisor_or_dashboard = true;
            kml_srv.request.clear = false;
            if (post_kml_client_.call(kml_srv)) {
              ROS_INFO("Mission Controller: KML sent to post semantic annotations.");
            } else {
              ROS_WARN("Mission Controller: KML not sent to post semantic annotations, service didn't answer.");
            }
          }
        }
      }

      _res.success = true;
      _res.message = "Select role successful.";
      ROS_INFO("Mission Controller: %s", _res.message.c_str());

    } else {
      _res.success = false;
      _res.message = "There is no validated plan with that mission_role, mission_id or SAS_role. Wrong selection or plan not possible. Ignoring role select.";
      selected_roles_.mission_role.clear();
      selected_roles_.sas_roles.clear();
      ROS_WARN("Mission Controller: %s", _res.message.c_str());
    }

  } else {
    _res.success = false;
    _res.message = "Role not selected, there are drones not idle.";
    ROS_WARN("Mission Controller: %s", _res.message.c_str());
  }

  return true;

} // end selectRoleServiceCallback



/** Clear missions and events service callback
 */
bool MissionController::clearServiceCallback(multidrone_msgs::Clear::Request &_req, multidrone_msgs::Clear::Response &_res) {

  ROS_INFO("Mission Controller: Clear service received.");

  bool can_clear = true;
  for (std::map<int, DroneState>::iterator it = drone_status_list_.begin(); it != drone_status_list_.end(); it++) {
    if (it->second.action_status.status != multidrone_msgs::ActionStatus::AS_IDLE) {
      can_clear = false;
      break;
    }
  }

  if (can_clear) {

    if(validation_running_) stop_current_validation_ = true;   // If validating, end the current validation as soon as possible.
    if(validation_thread_.joinable()) validation_thread_.join();

    if (_req.uuid.size()==0) {  // No uuid given, clear everything.
      events_.clear();
      missions_.clear();
      time_zone_.clear();
      stem_event_id_.clear();

      _res.success = true;
      _res.message = "All missions and events successfully cleared.";
      ROS_INFO("Mission Controller: %s", _res.message.c_str());

    } else if (missions_.count(_req.uuid)==1) {
      missions_.erase(_req.uuid);

      _res.success = true;
      _res.message = "Given mission successfully cleared.";
      ROS_INFO("Mission Controller: %s", _res.message.c_str());

    } else if (events_.count(_req.uuid)==1) {
      events_.erase(_req.uuid);

      if (events_.size()==0) {
        time_zone_.clear();
        stem_event_id_.clear();
      }

      _res.success = true;
      _res.message = "Given event successfully cleared.";
      ROS_INFO("Mission Controller: %s", _res.message.c_str());

    } else {
      _res.success = false;
      _res.message = "UUID doesn't match with any enrolled mission or event.";
      ROS_INFO("Mission Controller: %s", _res.message.c_str());
    }

    if (_res.success) {
      validated_shooting_action_lists_.clear();
      validated_plan_.clear();
      selected_roles_.mission_role.clear();
      selected_roles_.sas_roles.clear();
      selected_plan_.plan.clear();
      running_missions_.clear();
      drones_last_pose_time_.clear();
      event_received_ = false;
      event_received_id_.clear();

      // mission_XML_parser may have changed the origin_coordinates_geo_ if this one was empty (0,0,0). When cleared, if the origin was changed, go back to initial values:
      if (origin_coordinates_geo_.latitude != initial_origin_coordinates_geo_.latitude || origin_coordinates_geo_.longitude != initial_origin_coordinates_geo_.longitude || origin_coordinates_geo_.altitude != initial_origin_coordinates_geo_.altitude) {
        origin_coordinates_geo_ = initial_origin_coordinates_geo_;

        pnh_.setParam("origin_latitude", initial_origin_coordinates_geo_.latitude);
        pnh_.setParam("origin_longitude", initial_origin_coordinates_geo_.longitude);
        pnh_.setParam("origin_altitude", initial_origin_coordinates_geo_.altitude);
      }

      // Push empty drone actions:
      multidrone_msgs::MultidronePlan empty_multidrone_plan;
      multidrone_msgs::DroneActionArray empty_drone_action_array;
      for (int i : drones_id_) {
        empty_drone_action_array.drone_id = i;
        empty_multidrone_plan.plan.push_back(empty_drone_action_array);
      }
      if(push_drone_actions_thread_.joinable()) {
        push_drone_actions_thread_.join();
      }
      push_drone_actions_thread_ = std::thread(&MissionController::pushDroneActionsThread,this,empty_multidrone_plan);

      // Clear Dashboards annotations in the Semantic Map Manager:
      multidrone_msgs::PostSemanticAnnotations kml_srv;
      kml_srv.request.semantic_map = "";
      kml_srv.request.supervisor_or_dashboard = true;
      kml_srv.request.clear = true;
      if (post_kml_client_.call(kml_srv)) {
        ROS_INFO("Mission Controller: clear KML sent to Semantic Map Manager.");
      } else {
        ROS_WARN("Mission Controller: clear KML not sent to Semantic Map Manager, service didn't answer.");
      }

      // Reset drone alarmed status
      for (std::map<int, DroneState>::iterator it = drone_status_list_.begin(); it != drone_status_list_.end(); it++) {
        it->second.drone_alarmed = false;
      }

    }

  } else {
    _res.success = false;
    _res.message = "Not cleared, there are drones not idle.";
    ROS_WARN("Mission Controller: %s", _res.message.c_str());
  }

  return true;

} // end clearServiceCallback



bool MissionController::directorEventCallBack(multidrone_msgs::DirectorEvent::Request &_req, multidrone_msgs::DirectorEvent::Response &_res) {
  if (events_.count(_req.event_id)==1 || _req.event_id==get_ready_event_) {
    event_received_ = true;
    event_received_id_ = _req.event_id;
    event_received_time_stamp_ = ros::Time::now();

    _res.success = true;
    _res.message = "Director event " + std::string(_req.event_id) + " received.";
    ROS_INFO("Mission Controller: %s", _res.message.c_str());

  } else {
    _res.success = false;
    _res.message = "Director event received not previously enrolled, ignoring director event.";
    ROS_WARN("Mission Controller: %s", _res.message.c_str());

  }
  return true;
} // end directorEventCallBack



/** Action status callback
 */
void MissionController::actionStatusCallback(const multidrone_msgs::ActionStatus::ConstPtr &_msg, const int _drone_id) {
// saving the action status information into the DroneState
  drone_status_list_[_drone_id].action_status = *_msg;

  if (missions_.count(_msg->mission_id)>0 && missions_[_msg->mission_id].mission_status.status!=multidrone_msgs::MissionStatus::STATUS_ABORTED && missions_[_msg->mission_id].mission_status.status!=multidrone_msgs::MissionStatus::STATUS_ENDED) {
    missions_[_msg->mission_id].mission_status.status = multidrone_msgs::MissionStatus::STATUS_RUNNING;

    // saving the last time that an action of a mission has been received to check the end of the mission
    running_missions_[_msg->mission_id] = ros::Time::now();
  }
} // end actionStatusCallback



/** Callback for drone pose
 */
void MissionController::dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg, const int _drone_id) {
  drone_status_list_[_drone_id].pose = msg->pose;
  drone_status_list_[_drone_id].drone_pose_defined = true;

  // saving the last time a pose was received to check if the drone pose is still defined
  drones_last_pose_time_[_drone_id] = ros::Time::now();
} // end dronePoseCallback



/** Callback for drone battery
 */
void MissionController::droneBateryCallback(const sensor_msgs::BatteryState::ConstPtr &_battery_status, const int _drone_id) {
  drone_status_list_[_drone_id].battery_remaining = _battery_status->percentage; 
} // end droneBateryCallback



/** Provided service for supervisor alarm
 */
bool MissionController::alarmCallback(multidrone_msgs::SupervisorAlarm::Request &_req, multidrone_msgs::SupervisorAlarm::Response &_res) {
  ROS_INFO("Mission Controller: Alarm message received from supervisor.");
  multidrone_msgs::SupervisorAlarm srv;
  srv.request = _req;
  if (alarm_service_scheduler_[_req.drone_id].call(srv)) {
    ROS_INFO("Mission Controller: Alarm message sent to scheduler %d", _req.drone_id);
    _res.received = true;

    drone_status_list_[_req.drone_id].drone_alarmed = true;

    // Check if all drones are alarmed before trying to replan:
    bool all_drones_alarmed = true;
    for (std::map<int, DroneState>::iterator it = drone_status_list_.begin(); it != drone_status_list_.end(); it++) {
      if (it->second.drone_alarmed == false) {
        all_drones_alarmed = false;
        break;
      }
    }
    if (all_drones_alarmed) {
      ROS_WARN("Mission Controller: All drones alarmed, not trying to replan.");
      return true;
    }

    // Check if there was a plan previously selected before trying to replan:
    if (selected_roles_.mission_role.size()==0 || selected_roles_.sas_roles.size()==0) {
      ROS_WARN("Mission Controller: There isn't a plan selected yet, not trying to replan. Deleting previous validation results. Validate again and select one.");
      validated_plan_.clear();
      validated_shooting_action_lists_.clear();
      return true;
    }

    // Check if the alarmed drone is used in the selected plan before trying to replan:
    bool alarmed_drone_is_used = false;
    for (int i=0; i<selected_plan_.plan.size(); i++) {
      if (selected_plan_.plan[i].drone_id == _req.drone_id) {
        alarmed_drone_is_used = true;
        break;
      }
    }
    if (! alarmed_drone_is_used) {
      ROS_WARN("Mission Controller: Alarmed drone not used in the current selected plan, not trying to replan.");
      return true;
    }

    // Start the replanning thread:
    if(replanning_running_) stop_current_replanning_ = true;   // If still replaning, end the current replanning to start the new one as soon as possible.
    if(replanning_thread_.joinable()) replanning_thread_.join();
    replanning_thread_ = std::thread(&MissionController::replanningThread, this);

  } else {
    ROS_ERROR("Mission Controller: Alarm message cannot be sent to scheduler %d", _req.drone_id);
    _res.received = false;
  }
  return true;
} // end alarmCallback



/** Callback for the event from the event manager
 */
void MissionController::eventReceived(const multidrone_msgs::Event::ConstPtr &_event) {
} // end eventReceived



/** Callback for target poses
 */
void MissionController::targetCallback(const multidrone_msgs::TargetStateArray::ConstPtr &_msg) {
  // ROS_INFO("Message from target.");

  // TODO: update current info on target if needed

} // end targetCallback


} // end namespace multidrone