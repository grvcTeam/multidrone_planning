/**
 * MULTIDRONE Project:
 *
 * High level planner.
 * 
 */

#include <multidrone_planning/high_level_planner.h>

#include <limits>
#include <algorithm>
#include <math.h>

// #define DEBUG_HIGH_LEVEL_PLANNER       // UNCOMMENT FOR VISUALIZATION OF RESULTS (DEBUG MODE)
// #define BATTERIES_START_FULL           // UNCOMMENT IF THE DRONES DON'T PUBLISH THEIR BATTERY STATUS (E.G., SIMULATION) AND YOU KNOWN THAT THE BATTERIES ARE FULL AT THE BEGINNING.
// #define SAFETY_POSTPROCESS             // UNCOMMENT TO DO A SAFETY POSTPROCESS
// #define BENCHMARK_HIGH_LEVEL_PLANNER   // UNCOMMENT FOR BENCHMARKING THE COMPUTATION TIME OF THE METHOD "getPlan". COMMENT DEBUG_HIGH_LEVEL_PLANNER OR TIME WONT'T BE REPRESENTATIVE.

#ifdef DEBUG_HIGH_LEVEL_PLANNER
#include "matplotlibcpp/matplotlibcpp.h"  // matplotlib-cpp has a MIT License (MIT), Copyright (c) 2014 Benno Evers. The full license description of matplotlib, matplotlib-cpp and its README can be found at its root.
namespace plt = matplotlibcpp;    // namespace-alias-definition: makes name a synonym for another namespace: see namespace alias
#endif

namespace multidrone {


// Brief Constructor
HighLevelPlanner::HighLevelPlanner() : path_planner_() {

  ros::NodeHandle pnh = ros::NodeHandle("~");

  pnh.param<std::string>("get_ready_event", get_ready_event_, "GET_READY");

  pnh.param<float>("height_base_stations", height_base_stations_, 0);

  pnh.param<float>("minimum_battery", minimum_battery_, 20);

  pnh.param<float>("replanning_same_SA_distance", replanning_same_SA_distance_, 5);
  pnh.param<float>("replanning_same_SA_bonus", replanning_same_SA_bonus_, 10);

  pnh.param<float>("minimum_separation_xy", minimum_separation_xy_, 3);
  pnh.param<float>("minimum_separation_z", minimum_separation_z_, 3);

  pnh.param<float>("maximum_battery_error_in_successive_aproximation_method", maximum_battery_error_in_successive_aproximation_method_, 3);
  pnh.param<int>("maximum_idle_time_in_successive_aproximation_method", maximum_idle_time_in_successive_aproximation_method_, 4);

  pnh.param<int>("orbit_path_separation_of_discretization", orbit_path_separation_of_discretization_, 1);

  pnh.param<int>("time_swap_drone", time_swap_drone_, 5);
  pnh.param<int>("time_change_battery", time_change_battery_, 30);

  pnh.param<int>("time_minimum_shooting_action", time_minimum_shooting_action_, 5);

  pnh.param<int>("time_max_hovering", time_max_hovering_, 20*60);
  pnh.param<int>("time_max_flying_full_speed", time_max_flying_full_speed_, 15*60);

  pnh.param<int>("full_speed_xy", full_speed_xy_, 12);
  pnh.param<int>("full_speed_z_down", full_speed_z_down_, 1);
  pnh.param<int>("full_speed_z_up", full_speed_z_up_, 3);
}



// Brief Destructor
HighLevelPlanner::~HighLevelPlanner() {}



std::map< int,std::vector<multidrone_msgs::DroneAction> > HighLevelPlanner::getPlan(const std::vector<multidrone_msgs::ShootingAction>& _shooting_action_list, const std::map< int,std::tuple<geometry_msgs::Point32,double,bool> >& _initial_state_of_drones, const geographic_msgs::GeoPoint& _origin_coordinates_geo, const std::string& _KML_string, int _trigger_time_of_replanning, unsigned int _max_grid_side) {

#ifdef BENCHMARK_HIGH_LEVEL_PLANNER
  clock_t t_begin;
  clock_t t_end;
  t_begin = clock();
#endif

  trigger_time_of_replanning_ = _trigger_time_of_replanning;  // trigger_time_of_replanning by default will be -1 if not specified which means no replanning. If != -1 then is a replanning.

  std::map< int,std::vector<multidrone_msgs::DroneAction> > plan;     // Solution to be returned by the getPlan method.

  bool new_non_trivial_path_planner_required = false;
  if ( (_KML_string.size() >= 10) && (_KML_string != KML_string_) ) { // if KML string received (minimum 10 characters) and it's different from the actual one.
    KML_string_ = _KML_string;                                        // Update KML attribute in High-Level Planner class.
    new_non_trivial_path_planner_required = true;
  }

  if ( (_max_grid_side > 0) && (_max_grid_side != max_grid_side_ ) )  // if max_grid_size received (minimum 1) and it's different from the actual one
    max_grid_side_ = _max_grid_side;                                  // Update max_grid_size attribute in High-Level Planner class.

  bool KML_exist = true;
  if ( KML_string_.size()<10 ) {    // if KML attribute is empty.
    ROS_WARN("High-Level Planner: getPlan using Path Planner with no KML in the constructor, paths will be straight lines.");
    KML_exist = false;
    path_planner_ = PathPlanner();  // Trivial PathPlanner. Very fast constructor, do it always if the KML attribute is empty.
  } else if ( new_non_trivial_path_planner_required ) {      // if KML attribute is not empty and has changed in this method call
    path_planner_ = PathPlanner(KML_string_, _origin_coordinates_geo, max_grid_side_); // Heavy PathPlanner, construct it only once each time the KML has changed (new_non_trivial_path_planner_required == true). May return trivial PathPlanner if invalid KML (parse error).
    if (path_planner_.getTrivialPathPlannerOrNot()) {        // Even if there is a KML, it may be invalid and the planner may use the trivial PathPlanner.
      ROS_WARN("High-Level Planner: invalid KML (parse error), getPlan using Path Planner with no KML in the constructor, paths will be straight lines.");
    }
  }
  KML_exist = ! path_planner_.getTrivialPathPlannerOrNot();  // KML exist if it's valid.

  int maximum_recording_time_possible = 0;  // Sum of all planned shooting actions' recording time or duration. The plan will try to maximize the total recorded time of the drones, if optimum assignation of task to drones, that total recorded time of the drones will be equal to maximum_recording_time_possible.
  int current_global_recorded_time = 0;     // Sum of all shooting actions' recording time currently assigned to drones.

  // Separate shooting actions planned and not planned.
  std::vector<multidrone_msgs::ShootingAction> shooting_actions_planned;
  std::vector<multidrone_msgs::ShootingAction> shooting_actions_not_planned;
  for (int i = 0; i < _shooting_action_list.size(); i++) {
    if ( _shooting_action_list[i].estimated_start_time != -1 ) {
      shooting_actions_planned.push_back( _shooting_action_list[i] );
      maximum_recording_time_possible += _shooting_action_list[i].duration.data.sec * _shooting_action_list[i].shooting_roles.size();
    } else {
      shooting_actions_not_planned.push_back( _shooting_action_list[i] );
    }
  }

  TimeGraph time_graph = initializeTimeGraph( shooting_actions_planned, _initial_state_of_drones, KML_exist ); // Returns a time graph with its tasks, base stations and initial drone states initialized, but without nodes and edges.

  int drone_counter = 0;
  while ( drone_counter < _initial_state_of_drones.size() && time_graph.task.size()>0 ) {  // Repeat until there are no more drones or tasks left available to assign

    // Method in charge of completing the Time Graph (reference call), creating all nodes (shooting and base station nodes) and edges between them. Nodes already have information of the recorded time accumulated and the navigation distance accumulated.
    completeTimeGraph( time_graph );

    if ( time_graph.base_station_node.size() > 0 ) {
      // Method that insert 1 drone solution into the plan and update current_global_recorded_time (returns them both by reference). The "const" arguments are the inputs, and the other two are the outputs by reference. The method also returns the index of the last base station node of landing of subgraph used, so that subgraph can be used to reset the time graph.
      int last_bs_node_of_subgraph_to_erase = addBetterTimeGraphSolutionToPlan( plan, current_global_recorded_time, time_graph, _shooting_action_list );

      if ( last_bs_node_of_subgraph_to_erase != 0 ) {
        drone_counter++;

        if ( (drone_counter < _initial_state_of_drones.size()) && (current_global_recorded_time < maximum_recording_time_possible) ) {
          // If there will be another iteration to assign more drones, reset time graph, else break directly and save time.

          // Method that reset the time graph (by reference) to an state similar to just initilized (with taks and base stations and without edges and nodes), but with the pieces of tasks assigned and the initial drone states used removed.
          resetTimeGraph( time_graph, last_bs_node_of_subgraph_to_erase );
        } else {
          break;
        }
      } else {
        break;
      }
    } else {
      break;
    }

  }

#ifdef SAFETY_POSTPROCESS
  safetyPostprocess(plan);
#endif

  // This block of code is to force that each drone land after doing a go to waypoint to its own takeoff position so that they don't land in the same base station.
  // TODO: solve the issue in planning time.
  for ( std::map< int,std::vector<multidrone_msgs::DroneAction> >::iterator it = plan.begin(); it != plan.end(); it++ ) { // For each drone in the plan
    geometry_msgs::PointStamped initial_drone_point;
    std::map<int, geometry_msgs::Point32>::iterator it_2;
    for (it_2 = initial_ground_drone_positions_.begin(); it_2 != initial_ground_drone_positions_.end(); it_2++) {     // Look for this drone id in initial_ground_drone_positions_ to save its position.
      if ( it->first == it_2->first ) {
        initial_drone_point.point.x = it_2->second.x;
        initial_drone_point.point.y = it_2->second.y;
        initial_drone_point.point.z = it_2->second.z == -1 ? height_base_stations_ : it_2->second.z; // -1 means in the ground!
        break;
      }
    }
    if (it_2 == initial_ground_drone_positions_.end()) std::cout << "Drone id not found." << std::endl;
    bool land_modified_in_previous_iteration = false;
    for (int i=0; i<it->second.size(); i++) {                         // For each drone action planned for this drone.
      if (land_modified_in_previous_iteration == true && it->second[i].action_type == multidrone_msgs::DroneAction::TYPE_GOTOWAYPOINT) {
        it->second[i].path[0].point.x = initial_drone_point.point.x;
        it->second[i].path[0].point.y = initial_drone_point.point.y;
        land_modified_in_previous_iteration = false;
      } else if ( it->second[i].action_type == multidrone_msgs::DroneAction::TYPE_LAND ) {  // If it's a landing drone action change its previous path for the initial drone point.
        it->second[i-1].path.clear();
        float z_0 = 3;
        for (int j=0; j<it->second[i-2].shooting_action.shooting_roles[0].shooting_parameters.size(); j++) {
          if (it->second[i-2].shooting_action.shooting_roles[0].shooting_parameters[j].param == "z_0") {
            z_0 = it->second[i-2].shooting_action.shooting_roles[0].shooting_parameters[j].value;
          }
        }
        it->second[i-1].path.push_back( initial_drone_point );
        it->second[i-1].path[ it->second[i-1].path.size()-1 ].point.z = height_base_stations_ + z_0;  // Don't give wp in the ground,
        it->second[i].path[0].point.x = initial_drone_point.point.x;
        it->second[i].path[0].point.y = initial_drone_point.point.y;
        it->second[i].path[0].point.z = height_base_stations_;
        land_modified_in_previous_iteration = true;
      }
    }
  }

#ifdef BENCHMARK_HIGH_LEVEL_PLANNER
  t_end = clock();
  double seconds = ((float)(t_end-t_begin))/CLOCKS_PER_SEC;
  std::cout << std::endl << "Computation time for getPlan: " << seconds << " seconds." << std::endl << std::endl;
#endif

#ifdef DEBUG_HIGH_LEVEL_PLANNER
  std::cout << "Maximum recording time possible: " << maximum_recording_time_possible << " seconds." << std::endl << std::endl;
  std::cout << "Recorded time in the solution: " << current_global_recorded_time << " seconds." << std::endl << std::endl;
  std::cout << "Number of drones used in the solution: " << drone_counter << " drones." << std::endl << std::endl;
  std::cout << "Number of drones available: " << _initial_state_of_drones.size() << " drones." << std::endl << std::endl;
#endif

  return plan;
} // end getPlan method



// Method in charge of initializing one Time Graph per mission (only one pair of roles). Returns an incomplete Time Graph with its tasks, base stations and initial drone states initialized, but without nodes and edges.
HighLevelPlanner::TimeGraph HighLevelPlanner::initializeTimeGraph(const std::vector<multidrone_msgs::ShootingAction>& shooting_actions_planned, const std::map< int,std::tuple<geometry_msgs::Point32,double,bool> >& _initial_state_of_drones, bool KML_exist) {

  TimeGraph time_graph;   // Solution to be returned by this method.

  // First, the base stations and the initial drone states from the time graph are build (using the information extracted from the KML and the initial position of drones).
  geometry_msgs::Point32 empty_base_station;              // First base station is empty so that there is no relevant...
  time_graph.base_station.push_back(empty_base_station);  // ... information in [0] (indexes <=-1 for base stations).
  if ( KML_exist ) {
    for (int i = 0; i < path_planner_.KML_parser_from_path_planner_.stations_cartesian_.size(); i++) {  // All the stations' positions are inserted are inserted in the time graph.
      time_graph.base_station.push_back( path_planner_.KML_parser_from_path_planner_.stations_cartesian_[i] );
      time_graph.base_station[time_graph.base_station.size()-1].z = height_base_stations_;
    }
  }

  // _initial_state_of_drones it's a map of tuples. The keys are the drone ids, the values a tuple with three elements each oen. The first in the tuple is the pose of the drone, the second its battery and the third a boolean that is false if the drone is in the ground and true if flying.
  // The initial position of the drones could be considered as special cases of base stations. Only 1 drone can leave each of them once and you can't land there.
  if (trigger_time_of_replanning_==-1) initial_ground_drone_positions_.clear();
  for ( std::map< int,std::tuple<geometry_msgs::Point32,double,bool> >::const_iterator it = _initial_state_of_drones.begin(); it != _initial_state_of_drones.end(); it++ ) {
    if (!KML_exist) {
      time_graph.base_station.push_back( std::get<0>(it->second) );  // The initial position of the drones are considered as the base stations if no KML is given.
      time_graph.base_station[time_graph.base_station.size()-1].z = height_base_stations_;
    }

    InitialDroneState actual_initial_drone_state;
    actual_initial_drone_state.drone_id = it->first;
    actual_initial_drone_state.position = std::get<0>(it->second);
    actual_initial_drone_state.battery  = std::get<1>(it->second) * 100;  // Battery given in decimal (1) and the planner work in percentage (%).
    actual_initial_drone_state.drone_flying_initially = std::get<2>(it->second);

    if (trigger_time_of_replanning_==-1) initial_ground_drone_positions_[actual_initial_drone_state.drone_id] = actual_initial_drone_state.position;

#ifdef BATTERIES_START_FULL
    actual_initial_drone_state.battery = 100;
#endif

    if ( actual_initial_drone_state.battery > 100 || actual_initial_drone_state.battery < 0 ) {
      ROS_WARN("High-Level Planner:  Initial battery of drone_id %d out of range (%f). Not planning with this drone.", actual_initial_drone_state.drone_id, actual_initial_drone_state.battery);
      continue;
    } else if ( actual_initial_drone_state.battery >= 50 && actual_initial_drone_state.battery <= 75 ) {
      ROS_WARN("High-Level Planner:  Drone_id %d has the battery partially discharged (%f). Please consider changing the battery and replan.", actual_initial_drone_state.drone_id, actual_initial_drone_state.battery);
    } else if ( actual_initial_drone_state.battery > 20 && actual_initial_drone_state.battery < 50 ) {
      ROS_WARN("High-Level Planner:  Drone_id %d has the battery half discharged (%f). Strongly consider changing the battery and replan.", actual_initial_drone_state.drone_id, actual_initial_drone_state.battery);
    } else if ( actual_initial_drone_state.battery <= 20 ) {
      ROS_WARN("High-Level Planner:  Drone_id %d has the battery discharged (%f). Not planning with this drone.", actual_initial_drone_state.drone_id, actual_initial_drone_state.battery);
      continue;
    }

    time_graph.initial_drone_state.push_back(actual_initial_drone_state);
  }

  // Second, build the tasks vector of the time_graph
  for (int i = 0; i < shooting_actions_planned.size(); i++) {
    for (int j = 0; j < shooting_actions_planned[i].shooting_roles.size(); j++ ) {   // Repeated tasks for shooting actions with several drones involved
      Task actual_task;

      // If time defined in the shooting action, calculate the estimated times.
      actual_task.time_initial = shooting_actions_planned[i].estimated_start_time;
      if ( shooting_actions_planned[i].duration.data.sec != -1 ) {
        actual_task.time_difference = shooting_actions_planned[i].duration.data.sec;
        actual_task.time_final = actual_task.time_initial + actual_task.time_difference;
      } else {
        ROS_WARN("High-Level Planner: shooting_actions_planned[i].duration.data.sec == -1");
        actual_task.time_difference = -1;
        actual_task.time_final = -1;
      }

      actual_task.delay_since_event = shooting_actions_planned[i].delay_since_event;

      actual_task.shooting_role_index = j;

      for (int k=0; k<shooting_actions_planned[i].shooting_roles[j].shooting_parameters.size(); k++)
        actual_task.shooting_parameters[ shooting_actions_planned[i].shooting_roles[j].shooting_parameters[k].param ] = shooting_actions_planned[i].shooting_roles[j].shooting_parameters[k].value;

      if (actual_task.shooting_parameters.count("x_s")>0 && actual_task.shooting_parameters.count("x_e")>0)
        actual_task.shooting_parameters["x_rate"] = (actual_task.shooting_parameters["x_e"]-actual_task.shooting_parameters["x_s"])/(float)shooting_actions_planned[i].duration.data.sec;
      if (actual_task.shooting_parameters.count("z_s")>0 && actual_task.shooting_parameters.count("z_e")>0)
        actual_task.shooting_parameters["z_rate"] = (actual_task.shooting_parameters["z_e"]-actual_task.shooting_parameters["z_s"])/(float)shooting_actions_planned[i].duration.data.sec;

      actual_task.event_id = shooting_actions_planned[i].start_event;
      actual_task.SA_id = shooting_actions_planned[i].action_id;
      actual_task.SA_sequence_id = shooting_actions_planned[i].action_sequence_id;
      actual_task.mission_id = shooting_actions_planned[i].mission_id;

      actual_task.shooting_type = shooting_actions_planned[i].shooting_roles[j].shooting_type;

      actual_task.initial_azimuth = shooting_actions_planned[i].rt_trajectory.size()>1 ? atan2( shooting_actions_planned[i].rt_trajectory[1].point.y - shooting_actions_planned[i].rt_trajectory[0].point.y , shooting_actions_planned[i].rt_trajectory[1].point.x - shooting_actions_planned[i].rt_trajectory[0].point.x ) : 0;  // If only one point given default orientation east.

      for (int k = 0; k < shooting_actions_planned[i].rt_trajectory.size(); k++) {
        geometry_msgs::Point32 current_reference_target_waypoint;
        current_reference_target_waypoint.x = shooting_actions_planned[i].rt_trajectory[k].point.x;
        current_reference_target_waypoint.y = shooting_actions_planned[i].rt_trajectory[k].point.y;
        current_reference_target_waypoint.z = shooting_actions_planned[i].rt_trajectory[k].point.z;
        actual_task.reference_target_waypoints.push_back(current_reference_target_waypoint);
      }

      actual_task.rt_displacement.x = shooting_actions_planned[i].rt_displacement.x;
      actual_task.rt_displacement.y = shooting_actions_planned[i].rt_displacement.y;
      actual_task.rt_displacement.z = shooting_actions_planned[i].rt_displacement.z;

      actual_task.initial_position.x = shooting_actions_planned[i].rt_trajectory[0].point.x;
      actual_task.initial_position.y = shooting_actions_planned[i].rt_trajectory[0].point.y;
      actual_task.initial_position.z = shooting_actions_planned[i].rt_trajectory[0].point.z;
      displacePointByTaskParameters(actual_task.initial_position, actual_task, actual_task.initial_azimuth, 0);

      // If the SA is type STATIC, initial and final positions are the same, the speed will be zero and the drone will be all the time hovering.
      if ( shooting_actions_planned[i].shooting_roles[j].shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_STATIC ) {
        actual_task.final_position = actual_task.initial_position;
        actual_task.distance = 0;

        actual_task.moving_or_hovering = true;
        actual_task.average_speed = 0;
      } else {
        actual_task.final_position.x = shooting_actions_planned[i].rt_trajectory[ shooting_actions_planned[i].rt_trajectory.size()-1 ].point.x;
        actual_task.final_position.y = shooting_actions_planned[i].rt_trajectory[ shooting_actions_planned[i].rt_trajectory.size()-1 ].point.y;
        actual_task.final_position.z = shooting_actions_planned[i].rt_trajectory[ shooting_actions_planned[i].rt_trajectory.size()-1 ].point.z;
        float final_azimuth;
        if ( actual_task.reference_target_waypoints.size()>2 )
          final_azimuth = atan2( actual_task.reference_target_waypoints[ actual_task.reference_target_waypoints.size()-1 ].y - actual_task.reference_target_waypoints[ actual_task.reference_target_waypoints.size()-2 ].y , actual_task.reference_target_waypoints[ actual_task.reference_target_waypoints.size()-1 ].x - actual_task.reference_target_waypoints[ actual_task.reference_target_waypoints.size()-2 ].x );
        else
          final_azimuth = actual_task.initial_azimuth;
        displacePointByTaskParameters(actual_task.final_position, actual_task, final_azimuth, actual_task.time_difference);

        actual_task.distance = shooting_actions_planned[i].length;

        actual_task.moving_or_hovering = false;
        actual_task.average_speed = actual_task.distance / actual_task.time_difference;
      }

      time_graph.task.push_back( actual_task );     // Push back the actual task.
    }
  }

  return time_graph;

} // end initializeTimeGraph method



// Method in charge of completing the Time Graph (reference call), creating all nodes (shooting and base station nodes) and edges between them. Nodes already have information of the recorded time accumulated and the navigation distance accumulated.
void HighLevelPlanner::completeTimeGraph(TimeGraph& time_graph) {

  std::vector<int> open_nodes;    // Vector of node indexes whose sons are to be created.

  BaseStationNode empty_base_station_node;    // First base station node is empty so that there is no relevant information in the index 0, which will be a shooting node.
  empty_base_station_node.land_or_takeoff = false;
  time_graph.base_station_node.push_back( empty_base_station_node );

  if (trigger_time_of_replanning_ == -1) { // No replanning.
    // The first shooting node at the beginning of each task is created and open.
    for (int i=0;i<time_graph.initial_drone_state.size();i++) {
      if ( time_graph.initial_drone_state[i].drone_used == false ) { // Only from initial drones not used already.
        for (int j=0;j<time_graph.task.size();j++) {
          // Create 1º node of each task, the previous node in each base station, and the edge among each pair of nodes.

          // Calculate battery drop from base station to initial point of the task:
          geometry_msgs::Point32 initial_point = time_graph.initial_drone_state[i].position;
          geometry_msgs::Point32 final_point = time_graph.task[j].initial_position;
          NavigationSeparationStruct navigation_first_takeoff = navigationSeparation(initial_point, final_point, false);
          if (navigation_first_takeoff.distance==-1) continue;        // No path found.
          float battery_drop_to_beginning_of_task = batteryDrop(0, full_speed_xy_, navigation_first_takeoff.moving_time );  // hovering_time==0 because doesn't wait hovering

          // Calculate base station with minimum battery drop from beginning of task:
          NearestBaseStationStruct nearest_bs_from_task_beginning = nearestBaseStation(time_graph.task[j].initial_position, time_graph);
          if (nearest_bs_from_task_beginning.distance==-1) continue;  // No path found.

          if ( time_graph.initial_drone_state[i].battery - battery_drop_to_beginning_of_task - nearest_bs_from_task_beginning.battery_drop >= minimum_battery_ ) { // The drone can reach the initial point of the task and return to the nearest base station from there.
            BaseStationNode current_base_station_node_previous_to_current_task;
            current_base_station_node_previous_to_current_task.index_of_task_drone_or_bs = i;
            current_base_station_node_previous_to_current_task.position = time_graph.initial_drone_state[i].position;
            current_base_station_node_previous_to_current_task.time = time_graph.task[j].time_initial - navigation_first_takeoff.moving_time;
            current_base_station_node_previous_to_current_task.battery = time_graph.initial_drone_state[i].battery;
            current_base_station_node_previous_to_current_task.navigation_distance_accumulated = 0;
            current_base_station_node_previous_to_current_task.recording_time_accumulated = 0;
            current_base_station_node_previous_to_current_task.node_has_edges_out = true;
            current_base_station_node_previous_to_current_task.land_or_takeoff = true;
            time_graph.base_station_node.push_back( current_base_station_node_previous_to_current_task );

            ShootingNode first_node_of_current_task;
            first_node_of_current_task.index_of_task_drone_or_bs = j;
            first_node_of_current_task.position = time_graph.task[j].initial_position;
            first_node_of_current_task.time = time_graph.task[j].time_initial;
            first_node_of_current_task.battery = time_graph.initial_drone_state[i].battery - battery_drop_to_beginning_of_task;
            first_node_of_current_task.navigation_distance_accumulated = navigation_first_takeoff.distance;
            first_node_of_current_task.recording_time_accumulated = 0;
            first_node_of_current_task.edge_into_node = time_graph.edge.size();
            time_graph.shooting_node.push_back( first_node_of_current_task );

            EdgeFromTimeGraph edge_from_base_station_to_beginning_of_current_task;
            edge_from_base_station_to_beginning_of_current_task.shooting_or_navigation_action = true;
            edge_from_base_station_to_beginning_of_current_task.father_node = -(time_graph.base_station_node.size()-1);
            edge_from_base_station_to_beginning_of_current_task.son_node = time_graph.shooting_node.size()-1;
            time_graph.edge.push_back( edge_from_base_station_to_beginning_of_current_task );

            // Open the new shooting node created at the beginning of the each task coming from initial drone state:
            open_nodes.push_back( edge_from_base_station_to_beginning_of_current_task.son_node );
          }
        }
      }
    }
  } else {  // Replanning, can't start assuming the drones will reach the beginning of the tasks:
    for (int i=0;i<time_graph.initial_drone_state.size();i++) {
      if ( time_graph.initial_drone_state[i].drone_used == false ) { // Only from initial drones not used already.
        for (int j=0;j<time_graph.task.size();j++) {
          // Try to connect it with all task feasibles from there.

          if ( trigger_time_of_replanning_ < time_graph.task[j].time_final && time_graph.task[j].time_difference > time_minimum_shooting_action_ ) {

            // Calculate battery drop and moving time of the navigation edge to the beginning of the task:
            geometry_msgs::Point32 initial_point = time_graph.initial_drone_state[i].position;
            geometry_msgs::Point32 final_point = time_graph.task[j].initial_position;
            NavigationSeparationStruct navigation_initial = time_graph.initial_drone_state[i].drone_flying_initially ? navigationSeparation(initial_point, final_point, true) : navigationSeparation(initial_point, final_point, false) ;
            if (navigation_initial.distance==-1) continue;  // No path found.
            float battery_drop_initial = batteryDrop(0, full_speed_xy_, navigation_initial.moving_time);  // hovering_time==0 because doesn't wait hovering

            if ( trigger_time_of_replanning_ + navigation_initial.moving_time <= time_graph.task[j].time_initial ) {  // If drone reaches in time the beginning of the shooting action
              if ( time_graph.initial_drone_state[i].battery - battery_drop_initial >= minimum_battery_ ) { // If enough battery to reach the other task

                // Calculate base station with minimum battery drop from the initial position of the task:
                NearestBaseStationStruct nearest_bs_from_task_beginning = nearestBaseStation(time_graph.task[j].initial_position, time_graph);
                if (nearest_bs_from_task_beginning.distance==-1) continue;  // No path found.

                if ( time_graph.initial_drone_state[i].battery - battery_drop_initial - nearest_bs_from_task_beginning.battery_drop >= minimum_battery_ ) {    // If enough battery to return to the nearest base station from that initial point

                  // Create new base station node:
                  BaseStationNode base_station_node;
                  base_station_node.index_of_task_drone_or_bs = i;
                  base_station_node.position = time_graph.initial_drone_state[i].position;
                  base_station_node.time = trigger_time_of_replanning_;
                  base_station_node.battery = time_graph.initial_drone_state[i].battery;
                  base_station_node.navigation_distance_accumulated = 0;
                  base_station_node.recording_time_accumulated = 0;
                  base_station_node.node_has_edges_out = true;
                  base_station_node.land_or_takeoff = true;
                  time_graph.base_station_node.push_back( base_station_node );

                  // Create new shooting node at the beginning of the task:
                  ShootingNode shooting_node_beginning_of_task;
                  shooting_node_beginning_of_task.index_of_task_drone_or_bs = j;
                  shooting_node_beginning_of_task.position = time_graph.task[j].initial_position;
                  shooting_node_beginning_of_task.time = time_graph.task[j].time_initial;
                  shooting_node_beginning_of_task.battery = time_graph.initial_drone_state[i].battery - battery_drop_initial;
                  shooting_node_beginning_of_task.navigation_distance_accumulated = navigation_initial.distance <= replanning_same_SA_distance_ ? navigation_initial.distance - replanning_same_SA_bonus_ : navigation_initial.distance ;
                  shooting_node_beginning_of_task.recording_time_accumulated = navigation_initial.distance <= replanning_same_SA_distance_ ? (int) replanning_same_SA_bonus_ : 0 ;
                  shooting_node_beginning_of_task.edge_into_node = time_graph.edge.size();
                  time_graph.shooting_node.push_back( shooting_node_beginning_of_task );

                  EdgeFromTimeGraph edge_from_new_base_station_node_to_new_shooting_node;
                  edge_from_new_base_station_node_to_new_shooting_node.shooting_or_navigation_action = true;
                  edge_from_new_base_station_node_to_new_shooting_node.father_node = -(time_graph.base_station_node.size()-1);
                  edge_from_new_base_station_node_to_new_shooting_node.son_node = time_graph.shooting_node.size()-1;
                  time_graph.edge.push_back( edge_from_new_base_station_node_to_new_shooting_node );

                  // Open the new shooting node created:
                  open_nodes.push_back( edge_from_new_base_station_node_to_new_shooting_node.son_node );
                }
              }
            } else {        // The drone doesn't reach in time the initial point of the task, but maybe could reach in time an intermediate point of the task.
              // If the drone can't reach in time the final point of the task, then it's impossible to reach any point of the task in time.
              // If the final point of the task can be reached in time, then an successive approximation method is used to calculate the intermediate point in which the wait time (idle time) of the drone, which is a incresing nonlinear function, is zero. idle_time=0 means that as soon as the drone arrives to the shooting action, the drone starts filming.

              // Calculate moving time of the navigation edge to the end of the task:
              initial_point = time_graph.initial_drone_state[i].position;
              final_point = time_graph.task[j].final_position;
              NavigationSeparationStruct navigation_final = time_graph.initial_drone_state[i].drone_flying_initially ? navigationSeparation(initial_point, final_point, true) : navigationSeparation(initial_point, final_point, false) ;
              if (navigation_final.distance==-1) continue;  // No path found.

              int idle_time_final = time_graph.task[j].time_final - (trigger_time_of_replanning_ + navigation_final.moving_time);

              if ( idle_time_final > 0 ) { // The drone can reach in time the final point of the shooting action.
                // "Idle time" is defined as the time that a drone has to wait in a point of a shooting action until it can start recording.
                // It's supposed to be always a increasing nonlinear function, this means that, going from another node, if the drone tries to enter in time in a shooting action in two different points, always the later point will have more idle time.
                // In other words, if the initial point of the task (shooting action) can't be reached in time (idle time initial point<0) and the final point can be reached in time (idle time final point>0), then there is an intermediate optimum point to enter the shooting action (idle time intermediate point=0).
                // This optimum intermediate point is calculated with a successive approximation method:

                int time_of_upper_idle_time_range = time_graph.task[j].time_final;
                int time_of_lower_idle_time_range = time_graph.task[j].time_initial;

                int time_at_the_middle_of_the_idle_time_range = (time_of_upper_idle_time_range + time_of_lower_idle_time_range)/2;

                initial_point = time_graph.initial_drone_state[i].position;

                // Calculate the intermediate point in the middle of the idle time range:
                geometry_msgs::Point32 middle_point = intermediatePointOfTaskFromTime(time_at_the_middle_of_the_idle_time_range, time_graph.task[j]);

                // Calculate moving time to the point in the middle of the idle time range:
                NavigationSeparationStruct navigation_to_the_middle_of_the_idle_time_range = time_graph.initial_drone_state[i].drone_flying_initially ? navigationSeparation(initial_point, middle_point, true) : navigationSeparation(initial_point, middle_point, false) ;

                int idle_time_in_the_middle_of_the_idle_time_range = time_at_the_middle_of_the_idle_time_range - (trigger_time_of_replanning_ + navigation_to_the_middle_of_the_idle_time_range.moving_time);

                while ( ( idle_time_in_the_middle_of_the_idle_time_range < 0 ) || ( idle_time_in_the_middle_of_the_idle_time_range > maximum_idle_time_in_successive_aproximation_method_ ) ) {

                  int previous_idle_time_middle = idle_time_in_the_middle_of_the_idle_time_range;

                  if ( idle_time_in_the_middle_of_the_idle_time_range > 0 )
                    time_of_upper_idle_time_range = time_at_the_middle_of_the_idle_time_range;
                  else
                    time_of_lower_idle_time_range = time_at_the_middle_of_the_idle_time_range;

                  time_at_the_middle_of_the_idle_time_range = (time_of_upper_idle_time_range + time_of_lower_idle_time_range)/2;

                  // Calculate the intermediate point in the middle of the idle time range:
                  middle_point = intermediatePointOfTaskFromTime(time_at_the_middle_of_the_idle_time_range, time_graph.task[j]);

                  // Calculate moving time to the point in the middle of the idle time range:
                  navigation_to_the_middle_of_the_idle_time_range = time_graph.initial_drone_state[i].drone_flying_initially ? navigationSeparation(initial_point, middle_point, true) : navigationSeparation(initial_point, middle_point, false) ;

                  idle_time_in_the_middle_of_the_idle_time_range = time_at_the_middle_of_the_idle_time_range - (trigger_time_of_replanning_ + navigation_to_the_middle_of_the_idle_time_range.moving_time);

                  if ( previous_idle_time_middle == idle_time_in_the_middle_of_the_idle_time_range ) // If idle time didn't change in this iteration and the condition is still not fulfilled, the condition won't be fulfilled ever. Break the loop.
                    break;
                }

                int hovering_time_in_the_intermediate_point = idle_time_in_the_middle_of_the_idle_time_range;
                float battery_drop_to_intermediate_point = batteryDrop(hovering_time_in_the_intermediate_point, full_speed_xy_, navigation_to_the_middle_of_the_idle_time_range.moving_time);

                // Calculate base station with minimum battery drop from the intermediate point with idle_time near to zero:
                NearestBaseStationStruct nearest_bs_from_intermediate_point = nearestBaseStation(middle_point, time_graph);
                if (nearest_bs_from_intermediate_point.distance==-1) continue;  // No path found.

                if ( time_graph.initial_drone_state[i].battery - battery_drop_to_intermediate_point - nearest_bs_from_intermediate_point.battery_drop >= minimum_battery_ ) {    // If enough battery to return to the nearest base station from that initial point

                  // Create new base station node:
                  BaseStationNode base_station_node;
                  base_station_node.index_of_task_drone_or_bs = i;
                  base_station_node.position = time_graph.initial_drone_state[i].position;
                  base_station_node.time = trigger_time_of_replanning_;
                  base_station_node.battery = time_graph.initial_drone_state[i].battery;
                  base_station_node.navigation_distance_accumulated = 0;
                  base_station_node.recording_time_accumulated = 0;
                  base_station_node.node_has_edges_out = true;
                  base_station_node.land_or_takeoff = true;
                  time_graph.base_station_node.push_back( base_station_node );

                  // Create new shooting node at the intermediate point and the edge between the current base station node and the new node:
                  ShootingNode intermediate_node;
                  intermediate_node.index_of_task_drone_or_bs = j;
                  intermediate_node.position = middle_point;
                  intermediate_node.time = time_at_the_middle_of_the_idle_time_range;
                  intermediate_node.battery = time_graph.initial_drone_state[i].battery - battery_drop_to_intermediate_point;
                  intermediate_node.navigation_distance_accumulated = navigation_to_the_middle_of_the_idle_time_range.distance <= replanning_same_SA_distance_ ? navigation_to_the_middle_of_the_idle_time_range.distance - replanning_same_SA_bonus_ : navigation_to_the_middle_of_the_idle_time_range.distance ;
                  intermediate_node.recording_time_accumulated = navigation_to_the_middle_of_the_idle_time_range.distance <= replanning_same_SA_distance_ ? (int) replanning_same_SA_bonus_ : 0 ;
                  intermediate_node.edge_into_node = time_graph.edge.size();
                  time_graph.shooting_node.push_back( intermediate_node );

                  EdgeFromTimeGraph edge_from_new_base_station_node_to_new_shooting_node;
                  edge_from_new_base_station_node_to_new_shooting_node.shooting_or_navigation_action = true;
                  edge_from_new_base_station_node_to_new_shooting_node.father_node = -(time_graph.base_station_node.size()-1);
                  edge_from_new_base_station_node_to_new_shooting_node.son_node = time_graph.shooting_node.size()-1;
                  time_graph.edge.push_back( edge_from_new_base_station_node_to_new_shooting_node );

                  // Open the new shooting node created:
                  open_nodes.push_back( edge_from_new_base_station_node_to_new_shooting_node.son_node );
                }
              }
            }
          }
        }
      }
    }
  }

  if ( time_graph.base_station_node.size()==1 ) {
#ifdef DEBUG_HIGH_LEVEL_PLANNER
    ROS_WARN("High-Level Planner: No possible solution with this conditions. Printing drone initial states:");
    for (int i=0;i<time_graph.initial_drone_state.size();i++)
      if ( time_graph.initial_drone_state[i].drone_used == false )
        std::cout << "Drone id (not used): " << time_graph.initial_drone_state[i].drone_id << ", battery: " << time_graph.initial_drone_state[i].battery << '%' << std::endl;
#endif
    return;
  }

  // Then, until the vector open_nodes is empty of open nodes, iterate:
  while ( open_nodes.size() > 0 ) {
    createNextNodes(open_nodes, time_graph);   // At each iteration, the next nodes from the first open node in the vector are created and open, and then the first open node is erased from the vector.
  }

} // end completeTimeGraph method



// Method in charge of create the next possible nodes from the first one in "open_nodes", open them, and then erase that first one.
void HighLevelPlanner::createNextNodes(std::vector<int>& open_nodes, TimeGraph& time_graph) {

  int current_node_index = open_nodes[0];   // The current node in each iteration will be the first node in the vector.

  if ( current_node_index >= 0 ) {                                     // Current node is a shooting node.
    if ( time_graph.edge[ time_graph.shooting_node[current_node_index].edge_into_node ].shooting_or_navigation_action==1 ) {
      // Current node is an entry in task (a shooting node after a navigation edge).

      // Calculate battery drop from the current node (entry to task) until the end of the task:
      int time_difference_until_end_of_task = time_graph.task[ time_graph.shooting_node[current_node_index].index_of_task_drone_or_bs ].time_final - time_graph.shooting_node[current_node_index].time;
      int hovering_time_until_end_of_task = time_graph.task[ time_graph.shooting_node[current_node_index].index_of_task_drone_or_bs ].moving_or_hovering ? time_difference_until_end_of_task : 0 ;
      int moving_time_until_end_of_task = time_graph.task[ time_graph.shooting_node[current_node_index].index_of_task_drone_or_bs ].moving_or_hovering ? 0 : time_difference_until_end_of_task ;
      float battery_drop_until_end_of_task = batteryDrop(hovering_time_until_end_of_task, time_graph.task[ time_graph.shooting_node[current_node_index].index_of_task_drone_or_bs ].average_speed, moving_time_until_end_of_task);

      // Calculate base station with minimum battery drop from the final position of the task:
      NearestBaseStationStruct nearest_bs_from_task_end = nearestBaseStation(time_graph.task[ time_graph.shooting_node[current_node_index].index_of_task_drone_or_bs ].final_position, time_graph);
      if (nearest_bs_from_task_end.distance==-1) {  // No path found.
        open_nodes.erase(open_nodes.begin());       // Delete current node (the first one of the vector).
        return;
      }

      if ( time_graph.shooting_node[current_node_index].battery - battery_drop_until_end_of_task - nearest_bs_from_task_end.battery_drop >= minimum_battery_ ) {    // If enough battery to finish the task AND return to the nearest base station
        if (time_difference_until_end_of_task >= time_minimum_shooting_action_) {
          // Create new node at the end of the task where current node is (current task) and the shooting edge between them:
          ShootingNode last_node_of_current_task;
          last_node_of_current_task.index_of_task_drone_or_bs = time_graph.shooting_node[current_node_index].index_of_task_drone_or_bs;
          last_node_of_current_task.position = time_graph.task[ last_node_of_current_task.index_of_task_drone_or_bs ].final_position;
          last_node_of_current_task.time = time_graph.task[ last_node_of_current_task.index_of_task_drone_or_bs ].time_final;
          last_node_of_current_task.battery = time_graph.shooting_node[current_node_index].battery - battery_drop_until_end_of_task;
          last_node_of_current_task.navigation_distance_accumulated = time_graph.shooting_node[current_node_index].navigation_distance_accumulated;
          last_node_of_current_task.recording_time_accumulated = time_graph.shooting_node[current_node_index].recording_time_accumulated + time_difference_until_end_of_task;
          last_node_of_current_task.edge_into_node = time_graph.edge.size();
          time_graph.shooting_node.push_back( last_node_of_current_task );

          EdgeFromTimeGraph edge_from_initial_to_final_position_of_current_task;
          edge_from_initial_to_final_position_of_current_task.shooting_or_navigation_action = false;
          edge_from_initial_to_final_position_of_current_task.father_node = current_node_index;
          edge_from_initial_to_final_position_of_current_task.son_node = time_graph.shooting_node.size()-1;
          time_graph.edge.push_back( edge_from_initial_to_final_position_of_current_task );

          time_graph.shooting_node[ edge_from_initial_to_final_position_of_current_task.father_node ].node_has_edges_out = true;

          // Open the new shooting node created at the end of the task:
          open_nodes.push_back( edge_from_initial_to_final_position_of_current_task.son_node );
        } else {  // Discarding these shooting nodes because the recording time is too small, so if the father of the previous navigation edge is a base station that base station node won't be considered with an edge out.
          if ( time_graph.edge[ time_graph.shooting_node[current_node_index].edge_into_node ].father_node<0 ) {
            time_graph.base_station_node[ - time_graph.edge[ time_graph.shooting_node[current_node_index].edge_into_node ].father_node ].node_has_edges_out = false;
            if ( time_graph.base_station_node[ - time_graph.edge[ time_graph.shooting_node[current_node_index].edge_into_node ].father_node ].edge_into_node != -1 &&
                 time_graph.edge[ time_graph.base_station_node[ - time_graph.edge[ time_graph.shooting_node[current_node_index].edge_into_node ].father_node ].edge_into_node ].father_node<0 ) {
              time_graph.base_station_node[ - time_graph.edge[ time_graph.base_station_node[ - time_graph.edge[ time_graph.shooting_node[current_node_index].edge_into_node ].father_node ].edge_into_node ].father_node ].node_has_edges_out = false;
            }
          }
        }
      } else {
        // Calculate base station with minimum battery drop from the final position of the task:
        NearestBaseStationStruct nearest_bs_from_task_start = nearestBaseStation(time_graph.task[ time_graph.shooting_node[current_node_index].index_of_task_drone_or_bs ].initial_position, time_graph);

        if (time_graph.shooting_node[current_node_index].battery - nearest_bs_from_task_start.battery_drop >= minimum_battery_ ) {    // If enough battery to finish the task but NOT to return from there
          // The drone has battery to reach the current node of the shooting action and return to land from there, but hasn't enough battery to reach the current node of the shooting action, record it entirely, and return to land.

          // In theses cases, "remaining battery" is defined as the battery that the drone has after reaching the current node, record until an intermediate point of the shooting action and then land in the nearest base station.
          // It's supposed to be always a decreasing nonlinear function, this means that, the latest the intermediate point of the task is, the lower is the remaining battery.
          // The optimum intermediate point is the one that has "remaining battery" >= and closest to "minimum battery", so even though the shooting action can't be recorded entirely, it is recorded as much as possible. This point is calculated with a successive approximation method:

          int time_of_upper_remaining_battery_range = time_graph.shooting_node[current_node_index].time;
          int time_of_lower_remaining_battery_range = time_graph.task[ time_graph.shooting_node[current_node_index].index_of_task_drone_or_bs ].time_final;

          int time_at_the_middle_of_the_remaining_battery_range = (time_of_upper_remaining_battery_range + time_of_lower_remaining_battery_range)/2;

          // Calculate battery drop from the current node to the point in the middle of the remaining battery range:
          int time_from_current_to_middle = time_at_the_middle_of_the_remaining_battery_range - time_graph.shooting_node[current_node_index].time;
          int hovering_from_current_to_middle = time_graph.task[ time_graph.shooting_node[current_node_index].index_of_task_drone_or_bs ].moving_or_hovering ? time_from_current_to_middle : 0 ;
          int moving_from_current_to_middle = time_graph.task[ time_graph.shooting_node[current_node_index].index_of_task_drone_or_bs ].moving_or_hovering ? 0 : time_from_current_to_middle ;
          float battery_drop_from_current_to_middle = batteryDrop(hovering_from_current_to_middle, time_graph.task[ time_graph.shooting_node[current_node_index].index_of_task_drone_or_bs ].average_speed, moving_from_current_to_middle);

          // Calculate the intermediate point in the middle of the battery range:
          geometry_msgs::Point32 middle_point = intermediatePointOfTaskFromTime(time_at_the_middle_of_the_remaining_battery_range, time_graph.task[ time_graph.shooting_node[current_node_index].index_of_task_drone_or_bs ]);

          // Calculate base station with minimum battery drop from the intermediate point with remaining_battery near to minimum_battery_:
          NearestBaseStationStruct nearest_bs_from_intermediate_point = nearestBaseStation(middle_point, time_graph);

          float remaining_battery_in_the_middle_of_the_remaining_battery_range = time_graph.shooting_node[current_node_index].battery - (battery_drop_from_current_to_middle + nearest_bs_from_intermediate_point.battery_drop);

          while ( ( remaining_battery_in_the_middle_of_the_remaining_battery_range < minimum_battery_ ) || ( remaining_battery_in_the_middle_of_the_remaining_battery_range > minimum_battery_ + maximum_battery_error_in_successive_aproximation_method_ ) ) {

            float previous_remaining_battery_middle = remaining_battery_in_the_middle_of_the_remaining_battery_range;

            if ( remaining_battery_in_the_middle_of_the_remaining_battery_range > minimum_battery_ )
              time_of_upper_remaining_battery_range = time_at_the_middle_of_the_remaining_battery_range;
            else
              time_of_lower_remaining_battery_range = time_at_the_middle_of_the_remaining_battery_range;

            time_at_the_middle_of_the_remaining_battery_range = (time_of_upper_remaining_battery_range + time_of_lower_remaining_battery_range)/2;

            // Calculate battery drop from the current node to the point in the middle of the remaining battery range:
            time_from_current_to_middle = time_at_the_middle_of_the_remaining_battery_range - time_graph.shooting_node[current_node_index].time;
            hovering_from_current_to_middle = time_graph.task[ time_graph.shooting_node[current_node_index].index_of_task_drone_or_bs ].moving_or_hovering ? time_from_current_to_middle : 0 ;
            moving_from_current_to_middle = time_graph.task[ time_graph.shooting_node[current_node_index].index_of_task_drone_or_bs ].moving_or_hovering ? 0 : time_from_current_to_middle ;
            battery_drop_from_current_to_middle = batteryDrop(hovering_from_current_to_middle, time_graph.task[ time_graph.shooting_node[current_node_index].index_of_task_drone_or_bs ].average_speed, moving_from_current_to_middle);

            // Calculate the intermediate point in the middle of the battery range:
            middle_point = intermediatePointOfTaskFromTime(time_at_the_middle_of_the_remaining_battery_range, time_graph.task[ time_graph.shooting_node[current_node_index].index_of_task_drone_or_bs ]);

            // Calculate base station with minimum battery drop from the intermediate point with remaining_battery near to minimum_battery_:
            nearest_bs_from_intermediate_point = nearestBaseStation(middle_point, time_graph);

            remaining_battery_in_the_middle_of_the_remaining_battery_range = time_graph.shooting_node[current_node_index].battery - (battery_drop_from_current_to_middle + nearest_bs_from_intermediate_point.battery_drop);

            if ( previous_remaining_battery_middle == remaining_battery_in_the_middle_of_the_remaining_battery_range ) // If battery didn't change in this iteration and the condition is still not fulfilled, the condition won't be fulfilled ever. Break the loop.
              break;
          }

          if (time_from_current_to_middle >= time_minimum_shooting_action_) {
            // Create new node at the intermediate point and the edge between the current shooting node and the new node:
            ShootingNode intermediate_node;
            intermediate_node.index_of_task_drone_or_bs = time_graph.shooting_node[current_node_index].index_of_task_drone_or_bs;
            intermediate_node.position = middle_point;
            intermediate_node.time = time_at_the_middle_of_the_remaining_battery_range;
            intermediate_node.battery = time_graph.shooting_node[current_node_index].battery - battery_drop_from_current_to_middle;
            intermediate_node.navigation_distance_accumulated = time_graph.shooting_node[current_node_index].navigation_distance_accumulated;
            intermediate_node.recording_time_accumulated = time_graph.shooting_node[current_node_index].recording_time_accumulated + time_from_current_to_middle;
            intermediate_node.edge_into_node = time_graph.edge.size();
            time_graph.shooting_node.push_back( intermediate_node );

            EdgeFromTimeGraph edge_from_current_node_to_intermediate_node;
            edge_from_current_node_to_intermediate_node.shooting_or_navigation_action = false;
            edge_from_current_node_to_intermediate_node.father_node = current_node_index;
            edge_from_current_node_to_intermediate_node.son_node = time_graph.shooting_node.size()-1;
            time_graph.edge.push_back( edge_from_current_node_to_intermediate_node );

            time_graph.shooting_node[ edge_from_current_node_to_intermediate_node.father_node ].node_has_edges_out = true;

            // Create new node at the nearest base station from the intermediate point and the navigation edge between them:
            BaseStationNode new_base_station_node;
            new_base_station_node.index_of_task_drone_or_bs = nearest_bs_from_intermediate_point.index;
            new_base_station_node.position = time_graph.base_station[ -nearest_bs_from_intermediate_point.index ];
            new_base_station_node.time = intermediate_node.time + nearest_bs_from_intermediate_point.moving_time;
            new_base_station_node.battery = intermediate_node.battery - nearest_bs_from_intermediate_point.battery_drop;
            new_base_station_node.navigation_distance_accumulated = time_graph.shooting_node[ edge_from_current_node_to_intermediate_node.son_node ].navigation_distance_accumulated + nearest_bs_from_intermediate_point.distance;
            new_base_station_node.recording_time_accumulated = time_graph.shooting_node[ edge_from_current_node_to_intermediate_node.son_node ].recording_time_accumulated;
            new_base_station_node.edge_into_node = time_graph.edge.size();
            new_base_station_node.land_or_takeoff = false;
            time_graph.base_station_node.push_back( new_base_station_node );

            EdgeFromTimeGraph edge_from_intermediate_node_to_nearest_base_station;
            edge_from_intermediate_node_to_nearest_base_station.shooting_or_navigation_action = true;
            edge_from_intermediate_node_to_nearest_base_station.father_node = edge_from_current_node_to_intermediate_node.son_node;
            edge_from_intermediate_node_to_nearest_base_station.son_node = -(time_graph.base_station_node.size()-1);
            time_graph.edge.push_back( edge_from_intermediate_node_to_nearest_base_station );

            time_graph.shooting_node[ edge_from_intermediate_node_to_nearest_base_station.father_node ].node_has_edges_out = true;

            // Open the new base station node created:
            open_nodes.push_back( edge_from_intermediate_node_to_nearest_base_station.son_node );
          } else {  // Discarding these shooting nodes because the recording time is too small, so if the father of the previous navigation edge is a base station that base station node won't be considered with an edge out.
            if ( time_graph.edge[ time_graph.shooting_node[current_node_index].edge_into_node ].father_node<0 ) {
              time_graph.base_station_node[ - time_graph.edge[ time_graph.shooting_node[current_node_index].edge_into_node ].father_node ].node_has_edges_out = false;
              if ( time_graph.base_station_node[ - time_graph.edge[ time_graph.shooting_node[current_node_index].edge_into_node ].father_node ].edge_into_node != -1 &&
                  time_graph.edge[ time_graph.base_station_node[ - time_graph.edge[ time_graph.shooting_node[current_node_index].edge_into_node ].father_node ].edge_into_node ].father_node<0 ) {
                time_graph.base_station_node[ - time_graph.edge[ time_graph.base_station_node[ - time_graph.edge[ time_graph.shooting_node[current_node_index].edge_into_node ].father_node ].edge_into_node ].father_node ].node_has_edges_out = false;
              }
            }
          }

        }
      }

    } else if ( time_graph.edge[ time_graph.shooting_node[current_node_index].edge_into_node ].shooting_or_navigation_action==0 ) {
      // Current node is an exit of a task (a shooting node after a shooting edge).

      for ( int i=0; i<time_graph.task.size(); i++ ) { // The current node (exit shooting node) is connected with all tasks feasibles from there.

        if ( time_graph.shooting_node[current_node_index].time < time_graph.task[i].time_final && time_graph.task[i].time_difference > time_minimum_shooting_action_ ) { // End of next task after current node and task with a minimum duration.

          // Calculate moving time of the navigation edge to the beginning of the task:
          geometry_msgs::Point32 initial_point = time_graph.shooting_node[current_node_index].position;
          geometry_msgs::Point32 final_point = time_graph.task[i].initial_position;
          NavigationSeparationStruct navigation_initial = navigationSeparation(initial_point, final_point, true);
          if (navigation_initial.distance==-1) continue;  // No path found.

          int idle_time_initial = time_graph.task[i].time_initial - (time_graph.shooting_node[current_node_index].time + navigation_initial.moving_time);

          if ( idle_time_initial >= 0 ) {  // If drone reaches in time the beginning of the shooting actions

            // Calculate battery drop of the navigation edge to the beginning of the task:
            int time_difference_initial = time_graph.task[i].time_initial - time_graph.shooting_node[current_node_index].time;
            int hovering_time_initial = time_difference_initial - navigation_initial.moving_time;
            float battery_drop_initial = batteryDrop(hovering_time_initial, full_speed_xy_, navigation_initial.moving_time);

            if ( time_graph.shooting_node[current_node_index].battery - battery_drop_initial >= minimum_battery_ ) { // If enough battery to reach the other task

              // Calculate base station with minimum battery drop from the initial position of the task:
              NearestBaseStationStruct nearest_bs_from_task_beginning = nearestBaseStation(time_graph.task[i].initial_position, time_graph);
              if (nearest_bs_from_task_beginning.distance==-1) continue;  // No path found.

              if ( time_graph.shooting_node[current_node_index].battery - battery_drop_initial - nearest_bs_from_task_beginning.battery_drop >= minimum_battery_ ) {    // If enough battery to return to the nearest base station from that initial point

                // Create new node at the beginning of the current task and the edge between the current shooting node to the new node:
                ShootingNode first_node_of_current_task;
                first_node_of_current_task.index_of_task_drone_or_bs = i;
                first_node_of_current_task.position = time_graph.task[i].initial_position;
                first_node_of_current_task.time = time_graph.task[i].time_initial;
                first_node_of_current_task.battery = time_graph.shooting_node[current_node_index].battery - battery_drop_initial;
                first_node_of_current_task.navigation_distance_accumulated = time_graph.shooting_node[current_node_index].navigation_distance_accumulated + navigation_initial.distance;
                first_node_of_current_task.recording_time_accumulated = time_graph.shooting_node[current_node_index].recording_time_accumulated;
                first_node_of_current_task.edge_into_node = time_graph.edge.size();
                time_graph.shooting_node.push_back( first_node_of_current_task );

                EdgeFromTimeGraph edge_from_current_node_to_beginning_of_current_task;
                edge_from_current_node_to_beginning_of_current_task.shooting_or_navigation_action = true;
                edge_from_current_node_to_beginning_of_current_task.father_node = current_node_index;
                edge_from_current_node_to_beginning_of_current_task.son_node = time_graph.shooting_node.size()-1;
                time_graph.edge.push_back( edge_from_current_node_to_beginning_of_current_task );

                time_graph.shooting_node[ edge_from_current_node_to_beginning_of_current_task.father_node ].node_has_edges_out = true;

                // Open the new shooting node created:
                open_nodes.push_back( edge_from_current_node_to_beginning_of_current_task.son_node );
              }
            }
          } else {        // The drone doesn't reach in time the initial point of the task, but maybe could reach in time an intermediate point of the task.
            // If the drone can't reach in time the final point of the task, then it's impossible to reach any point of the task in time.
            // If the final point of the task can be reached in time, then an successive approximation method is used to calculate the intermediate point in which the wait time (idle time) of the drone, which is a incresing nonlinear function, is zero. idle_time=0 means that as soon as the drone arrives to the shooting action, the drone starts filming.

            // Calculate moving time of the navigation edge to the end of the task:
            initial_point = time_graph.shooting_node[current_node_index].position;
            final_point = time_graph.task[i].final_position;
            NavigationSeparationStruct navigation_final = navigationSeparation(initial_point, final_point, true);
            if (navigation_initial.distance==-1) continue;  // No path found.

            int idle_time_final = time_graph.task[i].time_final - (time_graph.shooting_node[current_node_index].time + navigation_final.moving_time);

            if ( idle_time_final > 0 ) { // The drone can reach in time the final point of the shooting action.
              // "Idle time" is defined as the time that a drone has to wait in a point of a shooting action until it can start recording.
              // It's supposed to be always a increasing nonlinear function, this means that, going from another node, if the drone tries to enter in time in a shooting action in two different points, always the later point will have more idle time.
              // In other words, if the initial point of the task (shooting action) can't be reached in time (idle time initial point<0) and the final point can be reached in time (idle time final point>0), then there is an intermediate optimum point to enter the shooting action (idle time intermediate point=0).
              // This optimum intermediate point is calculated with a successive approximation method:

              int time_of_upper_idle_time_range = time_graph.task[i].time_final;
              int time_of_lower_idle_time_range = time_graph.task[i].time_initial;

              int time_at_the_middle_of_the_idle_time_range = (time_of_upper_idle_time_range + time_of_lower_idle_time_range)/2;

              initial_point = time_graph.shooting_node[current_node_index].position;

              // Calculate the intermediate point in the middle of the idle time range:
              geometry_msgs::Point32 middle_point = intermediatePointOfTaskFromTime(time_at_the_middle_of_the_idle_time_range, time_graph.task[ i ]);

              // Calculate moving time to the point in the middle of the idle time range:
              NavigationSeparationStruct navigation_to_the_middle_of_the_idle_time_range = navigationSeparation(initial_point, middle_point, true);

              int idle_time_in_the_middle_of_the_idle_time_range = time_at_the_middle_of_the_idle_time_range - (time_graph.shooting_node[current_node_index].time + navigation_to_the_middle_of_the_idle_time_range.moving_time);

              while ( ( idle_time_in_the_middle_of_the_idle_time_range < 0 ) || ( idle_time_in_the_middle_of_the_idle_time_range > maximum_idle_time_in_successive_aproximation_method_ ) ) {

                int previous_idle_time_middle = idle_time_in_the_middle_of_the_idle_time_range;

                if ( idle_time_in_the_middle_of_the_idle_time_range > 0 )
                  time_of_upper_idle_time_range = time_at_the_middle_of_the_idle_time_range;
                else
                  time_of_lower_idle_time_range = time_at_the_middle_of_the_idle_time_range;

                time_at_the_middle_of_the_idle_time_range = (time_of_upper_idle_time_range + time_of_lower_idle_time_range)/2;

                // Calculate the intermediate point in the middle of the idle time range:
                middle_point = intermediatePointOfTaskFromTime(time_at_the_middle_of_the_idle_time_range, time_graph.task[ i ]);

                // Calculate moving time to the point in the middle of the idle time range:
                navigation_to_the_middle_of_the_idle_time_range = navigationSeparation(initial_point, middle_point, true);

                idle_time_in_the_middle_of_the_idle_time_range = time_at_the_middle_of_the_idle_time_range - (time_graph.shooting_node[current_node_index].time + navigation_to_the_middle_of_the_idle_time_range.moving_time);

                if ( previous_idle_time_middle == idle_time_in_the_middle_of_the_idle_time_range ) // If idle time didn't change in this iteration and the condition is still not fulfilled, the condition won't be fulfilled ever. Break the loop.
                  break;
              }

              int hovering_time_in_the_intermediate_point = idle_time_in_the_middle_of_the_idle_time_range;
              float battery_drop_to_intermediate_point = batteryDrop(hovering_time_in_the_intermediate_point, full_speed_xy_, navigation_to_the_middle_of_the_idle_time_range.moving_time);

              // Calculate base station with minimum battery drop from the intermediate point with idle_time near to zero:
              NearestBaseStationStruct nearest_bs_from_intermediate_point = nearestBaseStation(middle_point, time_graph);
              if (nearest_bs_from_intermediate_point.distance==-1) continue;  // No path found.

              if ( time_graph.shooting_node[current_node_index].battery - battery_drop_to_intermediate_point - nearest_bs_from_intermediate_point.battery_drop >= minimum_battery_ ) {    // If enough battery to return to the nearest base station from that initial point

                // Create new node at the intermediate point and the edge between the current shooting node and the new node:
                ShootingNode intermediate_node;
                intermediate_node.index_of_task_drone_or_bs = i;
                intermediate_node.position = middle_point;
                intermediate_node.time = time_at_the_middle_of_the_idle_time_range;
                intermediate_node.battery = time_graph.shooting_node[current_node_index].battery - battery_drop_to_intermediate_point;
                intermediate_node.navigation_distance_accumulated = time_graph.shooting_node[current_node_index].navigation_distance_accumulated + navigation_to_the_middle_of_the_idle_time_range.distance;
                intermediate_node.recording_time_accumulated = time_graph.shooting_node[current_node_index].recording_time_accumulated;
                intermediate_node.edge_into_node = time_graph.edge.size();
                time_graph.shooting_node.push_back( intermediate_node );

                EdgeFromTimeGraph edge_from_current_node_to_intermediate_node;
                edge_from_current_node_to_intermediate_node.shooting_or_navigation_action = true;
                edge_from_current_node_to_intermediate_node.father_node = current_node_index;
                edge_from_current_node_to_intermediate_node.son_node = time_graph.shooting_node.size()-1;
                time_graph.edge.push_back( edge_from_current_node_to_intermediate_node );

                time_graph.shooting_node[ edge_from_current_node_to_intermediate_node.father_node ].node_has_edges_out = true;

                // Open the new shooting node created:
                open_nodes.push_back( edge_from_current_node_to_intermediate_node.son_node );
              }

            }
          }
        }
      }
      // The current node (exit shooting node) has been connected with all feasible tasks from there,
      // now it's time to connect it with the nearest base station.

      // Calculate base station with minimum battery drop from the current node:
      NearestBaseStationStruct nearest_bs_from_current_node = nearestBaseStation(time_graph.shooting_node[current_node_index].position, time_graph);

      // Create new node at the nearest base station of the final position of the current node and the shooting edge between them:
      BaseStationNode new_base_station_node;
      new_base_station_node.index_of_task_drone_or_bs = nearest_bs_from_current_node.index;
      new_base_station_node.position = time_graph.base_station[ -nearest_bs_from_current_node.index ];
      new_base_station_node.time = time_graph.shooting_node[current_node_index].time + nearest_bs_from_current_node.moving_time;
      new_base_station_node.battery = time_graph.shooting_node[current_node_index].battery - nearest_bs_from_current_node.battery_drop;
      new_base_station_node.navigation_distance_accumulated = time_graph.shooting_node[current_node_index].navigation_distance_accumulated + nearest_bs_from_current_node.distance;
      new_base_station_node.recording_time_accumulated = time_graph.shooting_node[current_node_index].recording_time_accumulated;
      new_base_station_node.edge_into_node = time_graph.edge.size();
      new_base_station_node.land_or_takeoff = false;
      time_graph.base_station_node.push_back( new_base_station_node );

      EdgeFromTimeGraph edge_from_current_node_to_nearest_base_station;
      edge_from_current_node_to_nearest_base_station.shooting_or_navigation_action = true;
      edge_from_current_node_to_nearest_base_station.father_node = current_node_index;
      edge_from_current_node_to_nearest_base_station.son_node = -(time_graph.base_station_node.size()-1);
      time_graph.edge.push_back( edge_from_current_node_to_nearest_base_station );

      time_graph.shooting_node[ edge_from_current_node_to_nearest_base_station.father_node ].node_has_edges_out = true;

      // Open the new base station node created:
      open_nodes.push_back( edge_from_current_node_to_nearest_base_station.son_node );
    }

  } else {
    // Current node is a base station node (must be a landing node).
    if ( time_graph.base_station_node[-current_node_index].land_or_takeoff==false ) {   // Current node must be a base station node of landing.

      for ( int i=0; i<time_graph.task.size(); i++ ) {
        // Try to connect it with all task feasibles from there.

        if ( time_graph.base_station_node[-current_node_index].time + time_change_battery_ < time_graph.task[i].time_final && time_graph.task[i].time_difference > time_minimum_shooting_action_ ) {       // End of next task after current node and task of a minimum duration

          // Calculate battery drop and moving time of the navigation edge to the beginning of the task:
          geometry_msgs::Point32 initial_point = time_graph.base_station_node[-current_node_index].position;
          geometry_msgs::Point32 final_point = time_graph.task[i].initial_position;
          NavigationSeparationStruct navigation_initial = navigationSeparation(initial_point, final_point, false);
          if (navigation_initial.distance==-1) continue;  // No path found.
          float battery_drop_initial = batteryDrop(0, full_speed_xy_, navigation_initial.moving_time);  // hovering_time==0 because doesn't wait hovering

          if ( time_graph.base_station_node[-current_node_index].time + time_change_battery_ + navigation_initial.moving_time <= time_graph.task[i].time_initial ) {  // If drone reaches in time the beginning of the shooting actions after changing the battery
            if ( 100 - battery_drop_initial >= minimum_battery_ ) { // If enough battery to reach the other task

              // Calculate base station with minimum battery drop from the initial position of the task:
              NearestBaseStationStruct nearest_bs_from_task_beginning = nearestBaseStation(time_graph.task[i].initial_position, time_graph);
              if (nearest_bs_from_task_beginning.distance==-1) continue;  // No path found.

              if ( 100 - battery_drop_initial - nearest_bs_from_task_beginning.battery_drop >= minimum_battery_ ) {    // If enough battery to return to the nearest base station from that initial point

                // Create new base station node delaying the takeoff as much as possible, avoiding to wait hovering too much, and the edge between them (considered navigation, but the drone is not flying):
                BaseStationNode base_station_node_before_takeoff;
                base_station_node_before_takeoff.index_of_task_drone_or_bs = time_graph.base_station_node[-current_node_index].index_of_task_drone_or_bs;
                base_station_node_before_takeoff.position = time_graph.base_station_node[-current_node_index].position;
                base_station_node_before_takeoff.time = time_graph.task[i].time_initial - navigation_initial.moving_time;
                base_station_node_before_takeoff.battery = 100;  // Battery at 100% at takeoff
                base_station_node_before_takeoff.navigation_distance_accumulated = time_graph.base_station_node[-current_node_index].navigation_distance_accumulated;
                base_station_node_before_takeoff.recording_time_accumulated = time_graph.base_station_node[-current_node_index].recording_time_accumulated;
                base_station_node_before_takeoff.edge_into_node = time_graph.edge.size();
                base_station_node_before_takeoff.land_or_takeoff = true;
                time_graph.base_station_node.push_back( base_station_node_before_takeoff );

                EdgeFromTimeGraph edge_from_current_node_to_takeoff_node;
                edge_from_current_node_to_takeoff_node.shooting_or_navigation_action = true; // Considered navigation node, even though the drone doesn't move in space, but time and battery are different.
                edge_from_current_node_to_takeoff_node.father_node = current_node_index;
                edge_from_current_node_to_takeoff_node.son_node = -(time_graph.base_station_node.size()-1);
                time_graph.edge.push_back( edge_from_current_node_to_takeoff_node );

                time_graph.base_station_node[ -edge_from_current_node_to_takeoff_node.father_node ].node_has_edges_out = true;

                // Create new shooting node at the beginning of the task:
                ShootingNode shooting_node_beginning_of_task;
                shooting_node_beginning_of_task.index_of_task_drone_or_bs = i;
                shooting_node_beginning_of_task.position = time_graph.task[i].initial_position;
                shooting_node_beginning_of_task.time = time_graph.task[i].time_initial;
                shooting_node_beginning_of_task.battery = 100 - battery_drop_initial;
                shooting_node_beginning_of_task.navigation_distance_accumulated = time_graph.base_station_node[ -edge_from_current_node_to_takeoff_node.son_node ].navigation_distance_accumulated + navigation_initial.distance;
                shooting_node_beginning_of_task.recording_time_accumulated = time_graph.base_station_node[ -edge_from_current_node_to_takeoff_node.son_node ].recording_time_accumulated;
                shooting_node_beginning_of_task.edge_into_node = time_graph.edge.size();
                time_graph.shooting_node.push_back( shooting_node_beginning_of_task );

                EdgeFromTimeGraph edge_from_new_base_station_node_to_new_shooting_node;
                edge_from_new_base_station_node_to_new_shooting_node.shooting_or_navigation_action = true;
                edge_from_new_base_station_node_to_new_shooting_node.father_node = -(time_graph.base_station_node.size()-1);
                edge_from_new_base_station_node_to_new_shooting_node.son_node = time_graph.shooting_node.size()-1;
                time_graph.edge.push_back( edge_from_new_base_station_node_to_new_shooting_node );

                time_graph.base_station_node[ -edge_from_new_base_station_node_to_new_shooting_node.father_node ].node_has_edges_out = true;

                // Open the new shooting node created:
                open_nodes.push_back( edge_from_new_base_station_node_to_new_shooting_node.son_node );
              }
            }
          } else {        // The drone doesn't reach in time the initial point of the task, but maybe could reach in time an intermediate point of the task.
            // If the drone can't reach in time the final point of the task, then it's impossible to reach any point of the task in time.
            // If the final point of the task can be reached in time, then an successive approximation method is used to calculate the intermediate point in which the wait time (idle time) of the drone, which is a incresing nonlinear function, is zero. idle_time=0 means that as soon as the drone arrives to the shooting action, the drone starts filming.

            // Calculate moving time of the navigation edge to the end of the task:
            initial_point = time_graph.base_station_node[-current_node_index].position;
            final_point = time_graph.task[i].final_position;
            NavigationSeparationStruct navigation_final = navigationSeparation(initial_point, final_point, false);
            if (navigation_final.distance==-1) continue;  // No path found.

            int idle_time_final = time_graph.task[i].time_final - (time_graph.base_station_node[-current_node_index].time + time_change_battery_ + navigation_final.moving_time);

            if ( idle_time_final > 0 ) { // The drone can reach in time the final point of the shooting action.
              // "Idle time" is defined as the time that a drone has to wait in a point of a shooting action until it can start recording.
              // It's supposed to be always a increasing nonlinear function, this means that, going from another node, if the drone tries to enter in time in a shooting action in two different points, always the later point will have more idle time.
              // In other words, if the initial point of the task (shooting action) can't be reached in time (idle time initial point<0) and the final point can be reached in time (idle time final point>0), then there is an intermediate optimum point to enter the shooting action (idle time intermediate point=0).
              // This optimum intermediate point is calculated with a successive approximation method:

              int time_of_upper_idle_time_range = time_graph.task[i].time_final;
              int time_of_lower_idle_time_range = time_graph.task[i].time_initial;

              int time_at_the_middle_of_the_idle_time_range = (time_of_upper_idle_time_range + time_of_lower_idle_time_range)/2;

              initial_point = time_graph.base_station_node[-current_node_index].position;

              // Calculate the intermediate point in the middle of the idle time range:
              geometry_msgs::Point32 middle_point = intermediatePointOfTaskFromTime(time_at_the_middle_of_the_idle_time_range, time_graph.task[ i ]);

              // Calculate moving time to the point in the middle of the idle time range:
              NavigationSeparationStruct navigation_to_the_middle_of_the_idle_time_range = navigationSeparation(initial_point, middle_point, false);

              int idle_time_in_the_middle_of_the_idle_time_range = time_at_the_middle_of_the_idle_time_range - (time_graph.base_station_node[-current_node_index].time + time_change_battery_ + navigation_to_the_middle_of_the_idle_time_range.moving_time);

              while ( ( idle_time_in_the_middle_of_the_idle_time_range < 0 ) || ( idle_time_in_the_middle_of_the_idle_time_range > maximum_idle_time_in_successive_aproximation_method_ ) ) {

                int previous_idle_time_middle = idle_time_in_the_middle_of_the_idle_time_range;

                if ( idle_time_in_the_middle_of_the_idle_time_range > 0 )
                  time_of_upper_idle_time_range = time_at_the_middle_of_the_idle_time_range;
                else
                  time_of_lower_idle_time_range = time_at_the_middle_of_the_idle_time_range;

                time_at_the_middle_of_the_idle_time_range = (time_of_upper_idle_time_range + time_of_lower_idle_time_range)/2;

                // Calculate the intermediate point in the middle of the idle time range:
                middle_point = intermediatePointOfTaskFromTime(time_at_the_middle_of_the_idle_time_range, time_graph.task[ i ]);

                // Calculate moving time to the point in the middle of the idle time range:
                navigation_to_the_middle_of_the_idle_time_range = navigationSeparation(initial_point, middle_point, false);

                idle_time_in_the_middle_of_the_idle_time_range = time_at_the_middle_of_the_idle_time_range - (time_graph.base_station_node[-current_node_index].time + time_change_battery_ + navigation_to_the_middle_of_the_idle_time_range.moving_time);

                if ( previous_idle_time_middle == idle_time_in_the_middle_of_the_idle_time_range ) // If idle time didn't change in this iteration and the condition is still not fulfilled, the condition won't be fulfilled ever. Break the loop.
                  break;
              }

              int hovering_time_in_the_intermediate_point = idle_time_in_the_middle_of_the_idle_time_range;
              float battery_drop_to_intermediate_point = batteryDrop(hovering_time_in_the_intermediate_point, full_speed_xy_, navigation_to_the_middle_of_the_idle_time_range.moving_time);

              // Calculate base station with minimum battery drop from the intermediate point with idle_time near to zero:
              NearestBaseStationStruct nearest_bs_from_intermediate_point = nearestBaseStation(middle_point, time_graph);
              if (nearest_bs_from_intermediate_point.distance==-1) continue;  // No path found.

              if ( 100 - battery_drop_to_intermediate_point - nearest_bs_from_intermediate_point.battery_drop >= minimum_battery_ ) {    // If enough battery to return to the nearest base station from that initial point

                // Create new base station node delaying the takeoff as much as possible, avoiding to wait hovering too much, and the edge between them (considered navigation, but the drone is not flying):
                BaseStationNode base_station_node_of_takeoff;
                base_station_node_of_takeoff.index_of_task_drone_or_bs = time_graph.base_station_node[-current_node_index].index_of_task_drone_or_bs;
                base_station_node_of_takeoff.position = time_graph.base_station_node[-current_node_index].position;
                base_station_node_of_takeoff.time = time_at_the_middle_of_the_idle_time_range - navigation_to_the_middle_of_the_idle_time_range.moving_time;
                base_station_node_of_takeoff.battery = 100;  // Battery at 100% at takeoff
                base_station_node_of_takeoff.navigation_distance_accumulated = time_graph.base_station_node[-current_node_index].navigation_distance_accumulated;
                base_station_node_of_takeoff.recording_time_accumulated = time_graph.base_station_node[-current_node_index].recording_time_accumulated;
                base_station_node_of_takeoff.edge_into_node = time_graph.edge.size();
                base_station_node_of_takeoff.land_or_takeoff = true;
                time_graph.base_station_node.push_back( base_station_node_of_takeoff );

                EdgeFromTimeGraph edge_from_current_node_to_takeoff_node;
                edge_from_current_node_to_takeoff_node.shooting_or_navigation_action = true; // Considered navigation node, even though the drone doesn't move in space, but time and battery are different.
                edge_from_current_node_to_takeoff_node.father_node = current_node_index;
                edge_from_current_node_to_takeoff_node.son_node = -(time_graph.base_station_node.size()-1);
                time_graph.edge.push_back( edge_from_current_node_to_takeoff_node );

                time_graph.base_station_node[ -edge_from_current_node_to_takeoff_node.father_node ].node_has_edges_out = true;

                // Create new shooting node at the intermediate point and the edge between the current base station node and the new node:
                ShootingNode intermediate_node;
                intermediate_node.index_of_task_drone_or_bs = i;
                intermediate_node.position = middle_point;
                intermediate_node.time = time_at_the_middle_of_the_idle_time_range;
                intermediate_node.battery = 100 - battery_drop_to_intermediate_point;
                intermediate_node.navigation_distance_accumulated = time_graph.base_station_node[ -edge_from_current_node_to_takeoff_node.son_node ].navigation_distance_accumulated + navigation_to_the_middle_of_the_idle_time_range.distance;
                intermediate_node.recording_time_accumulated = time_graph.base_station_node[ -edge_from_current_node_to_takeoff_node.son_node ].recording_time_accumulated;
                intermediate_node.edge_into_node = time_graph.edge.size();
                time_graph.shooting_node.push_back( intermediate_node );

                EdgeFromTimeGraph edge_from_takeoff_node_to_intermediate_node;
                edge_from_takeoff_node_to_intermediate_node.shooting_or_navigation_action = true;
                edge_from_takeoff_node_to_intermediate_node.father_node = edge_from_current_node_to_takeoff_node.son_node;
                edge_from_takeoff_node_to_intermediate_node.son_node = time_graph.shooting_node.size()-1;
                time_graph.edge.push_back( edge_from_takeoff_node_to_intermediate_node );

                time_graph.base_station_node[ -edge_from_takeoff_node_to_intermediate_node.father_node ].node_has_edges_out = true;

                // Open the new shooting node created:
                open_nodes.push_back( edge_from_takeoff_node_to_intermediate_node.son_node );
              }
            }
          }
        }
      }
    } else {
      ROS_ERROR("High-Level Planner: current_node_index = %d is a base station node but it isn't a landing one.", current_node_index);
    }
  }

  open_nodes.erase(open_nodes.begin());     // At the end of each iteration, the first node of the vector is erased.

} // end createNextNodes method



// Method that insert 1 drone solution into the plan and update current_global_recorded_time (returns them both by reference). The "const" arguments are the inputs, and the other two are the outputs by reference. The method also returns the index of the last base station node of landing of subgraph used, so that subgraph can be used to reset the time graph.
int HighLevelPlanner::addBetterTimeGraphSolutionToPlan(std::map< int,std::vector<multidrone_msgs::DroneAction> >& plan, int& current_global_recorded_time, const TimeGraph& time_graph, const std::vector<multidrone_msgs::ShootingAction>& _shooting_action_list)
{
  std::vector<multidrone_msgs::DroneAction> one_drone_plan;

  // Search the best 1 drone assignation in this time graph, which is the one with maximum recording time accumulated, and if several equal solutions choose the one with minumum navigation distance.
  int maximum_recording_time_accumulated = 0;
  float minimum_navigation_distance_accumulated = std::numeric_limits<float>::max();
  int best_last_base_station_node = 0;

  for (int i=1; i<time_graph.base_station_node.size(); i++) {
    if ( (time_graph.base_station_node[i].land_or_takeoff == false) && (time_graph.base_station_node[i].node_has_edges_out==false) ) { // base station node of landing with anything after it.
      if ( ( time_graph.base_station_node[i].recording_time_accumulated > maximum_recording_time_accumulated ) || ( ( time_graph.base_station_node[i].navigation_distance_accumulated < minimum_navigation_distance_accumulated ) && ( time_graph.base_station_node[i].recording_time_accumulated == maximum_recording_time_accumulated ) ) ) {
        // if base station node of last landing, with more recording time accumulated or equal recording time accumulated and less navigation distance accumulated, update best recording time and navigation distance accumulated.
        maximum_recording_time_accumulated = time_graph.base_station_node[i].recording_time_accumulated;
        minimum_navigation_distance_accumulated = time_graph.base_station_node[i].navigation_distance_accumulated;
        best_last_base_station_node = -i;
      }
    }
  }
  if ( best_last_base_station_node==0 ) {
#ifdef DEBUG_HIGH_LEVEL_PLANNER
    ROS_WARN("High-Level Planner: No best_last_base_station_node found.");
#endif
    return 0;
  }
  current_global_recorded_time += maximum_recording_time_accumulated;


  // Iterate the time graph's edges from the best last base station node to its root base station node, saving that edges into "best_subgraph_edges".
  std::vector<int> best_subgraph_edges;

  int son_node_index = best_last_base_station_node;
  int edge_index = time_graph.base_station_node[-son_node_index].edge_into_node;
  int father_node_index = time_graph.edge[edge_index].father_node;   // Can be positive (father is shooting node) or negative (father is another base station node).

  bool keep_in_loop = true;

  while (keep_in_loop)  {
    // Insert current edge index into "best_subgraph_edges":
    best_subgraph_edges.push_back( edge_index );

    // Update the edge and is son node and father node:
    son_node_index = father_node_index;
    if (son_node_index >= 0) edge_index = time_graph.shooting_node[son_node_index].edge_into_node;
    else                    edge_index = time_graph.base_station_node[-son_node_index].edge_into_node;
    father_node_index = time_graph.edge[edge_index].father_node;

    // Check if the condition of ending the loop is fulfilled:
    if (father_node_index >= 0) keep_in_loop = true;
    else if ( !( (time_graph.base_station_node[-father_node_index].land_or_takeoff == true) && (time_graph.base_station_node[-father_node_index].edge_into_node == -1) ) ) keep_in_loop = true;
    else keep_in_loop = false;
  }
  // Insert current (last) edge index into "best_subgraph_edges":
  best_subgraph_edges.push_back( edge_index );

  std::reverse(best_subgraph_edges.begin(),best_subgraph_edges.end());    // Reverse edges order (they were saved from last to first)


  // The drone_id will be defined in the initial drone state (first edge's father of "best_subgraph_edges", which is a base station node with positive "index_of_task_drone_or_bs"):
  int drone_id = time_graph.initial_drone_state[ time_graph.base_station_node[ -time_graph.edge[ best_subgraph_edges[0] ].father_node ].index_of_task_drone_or_bs ].drone_id;
  if (drone_id == -1) ROS_ERROR("High-Level Planner: drone_id = -1.");


  for (int current_edge : best_subgraph_edges) {
    int current_father_node = time_graph.edge[ current_edge ].father_node;
    int current_son_node = time_graph.edge[ current_edge ].son_node;

    // In each iteration of the loop, one edge is converted to drone action and inserted in the vector "one_drone_plan", then the indexes of the father, son and edge are updated, and keep_in_loop it's updated too.

    if ( time_graph.edge[ current_edge ].shooting_or_navigation_action == false ) {
      // The edge is a shooting action:

      multidrone_msgs::DroneAction shooting_drone_action;
      shooting_drone_action.final_yaw_if_gotowaypoint.x = 0;
      shooting_drone_action.final_yaw_if_gotowaypoint.y = 0;
      shooting_drone_action.final_yaw_if_gotowaypoint.z = 0;
      shooting_drone_action.final_yaw_if_gotowaypoint.w = 0;
      shooting_drone_action.action_type = multidrone_msgs::DroneAction::TYPE_SHOOTING;
      shooting_drone_action.action_id = time_graph.task[ time_graph.shooting_node[current_father_node].index_of_task_drone_or_bs ].SA_id;
      shooting_drone_action.action_sequence_id = time_graph.task[ time_graph.shooting_node[current_father_node].index_of_task_drone_or_bs ].SA_sequence_id;
      shooting_drone_action.mission_id = time_graph.task[ time_graph.shooting_node[current_father_node].index_of_task_drone_or_bs ].mission_id;
      shooting_drone_action.start_event = time_graph.task[ time_graph.shooting_node[current_father_node].index_of_task_drone_or_bs ].event_id;
      for ( int j=0; j<_shooting_action_list.size(); j++ ) {
        if ( shooting_drone_action.action_id == _shooting_action_list[j].action_id ) {  // Search in the _shooting_action_list this ShootingAction

          shooting_drone_action.shooting_action = _shooting_action_list[j];

          shooting_drone_action.shooting_action.shooting_roles.clear();
          shooting_drone_action.shooting_action.shooting_roles.push_back( _shooting_action_list[j].shooting_roles[ time_graph.task[ time_graph.shooting_node[current_father_node].index_of_task_drone_or_bs ].shooting_role_index ] );

          shooting_drone_action.shooting_action.estimated_start_time = time_graph.shooting_node[current_father_node].time;
          shooting_drone_action.shooting_action.duration.data.sec = time_graph.shooting_node[current_son_node].time - time_graph.shooting_node[current_father_node].time;

          // Delay since event >0 only if the previous drone action is a navigation action (not another shooting drone action inside a SAS) from a base station.
          if (one_drone_plan.size()!=0 && one_drone_plan[one_drone_plan.size()-1].action_type==multidrone_msgs::DroneAction::TYPE_GOTOWAYPOINT && time_graph.edge[time_graph.shooting_node[current_father_node].edge_into_node].father_node<0) {
            int fly_time_from_bs = time_graph.shooting_node[current_father_node].time - time_graph.base_station_node[-time_graph.edge[time_graph.shooting_node[current_father_node].edge_into_node].father_node].time;
            shooting_drone_action.shooting_action.delay_since_event = time_graph.task[time_graph.shooting_node[current_father_node].index_of_task_drone_or_bs].delay_since_event + (time_graph.shooting_node[current_father_node].time - time_graph.task[time_graph.shooting_node[current_father_node].index_of_task_drone_or_bs].time_initial) - fly_time_from_bs;
            if (shooting_drone_action.shooting_action.delay_since_event<0) {
              // Delay negative means that or it's the first shooting drone action after get_ready_event_ or the drone can't reach the desired shooting in time if refered to its director event, in this second case it's refered to a previous director event.
              // Search for the closest previous director event if exist.
              int start_time_current_director_event;
              for (int k=0; k<_shooting_action_list.size(); k++) {
                if (shooting_drone_action.shooting_action.start_event==_shooting_action_list[k].start_event && _shooting_action_list[k].delay_since_event==0) {
                  start_time_current_director_event = _shooting_action_list[k].estimated_start_time;
                  break;
                }
              }
              std::string previous_director_event;
              int start_time_previous_director_event = -std::numeric_limits<int>::max();
              for (int k=0; k<_shooting_action_list.size(); k++) {
                if (_shooting_action_list[k].estimated_start_time!=-1 && _shooting_action_list[k].estimated_start_time<start_time_current_director_event && _shooting_action_list[k].estimated_start_time>start_time_previous_director_event && _shooting_action_list[k].delay_since_event==0) {
                  previous_director_event = _shooting_action_list[k].start_event;
                  start_time_previous_director_event = _shooting_action_list[k].estimated_start_time;
                  break;
                }
              }
              if (previous_director_event.size()==0 || start_time_current_director_event==-1) {
                // No previous director event, this shooting drone action is the first after get_ready_event_. Delay = 0 is expected.
                shooting_drone_action.shooting_action.delay_since_event = 0;
              } else {
                // Previous director event found. This shooting drone action will be refered to this previous one
                shooting_drone_action.start_event = previous_director_event;
                shooting_drone_action.shooting_action.start_event = previous_director_event;
                shooting_drone_action.shooting_action.delay_since_event = start_time_current_director_event - start_time_previous_director_event + time_graph.task[time_graph.shooting_node[current_father_node].index_of_task_drone_or_bs].delay_since_event + (time_graph.shooting_node[current_father_node].time - time_graph.task[time_graph.shooting_node[current_father_node].index_of_task_drone_or_bs].time_initial) - fly_time_from_bs;
                if (shooting_drone_action.shooting_action.delay_since_event<0) {
                  ROS_ERROR("High-Level Planner: shooting_drone_action.shooting_action.delay_since_event < 0");
                  shooting_drone_action.shooting_action.delay_since_event = 0;
                }
              }
            }
          } else {
            shooting_drone_action.shooting_action.delay_since_event = 0;
          }

          shooting_drone_action.shooting_action.rt_trajectory.clear();
          for (int i=0; i<time_graph.task[ time_graph.shooting_node[current_father_node].index_of_task_drone_or_bs ].reference_target_waypoints.size(); i++) {
            geometry_msgs::PointStamped new_point_stamped;
            new_point_stamped.point.x = time_graph.task[ time_graph.shooting_node[current_father_node].index_of_task_drone_or_bs ].reference_target_waypoints[i].x;
            new_point_stamped.point.y = time_graph.task[ time_graph.shooting_node[current_father_node].index_of_task_drone_or_bs ].reference_target_waypoints[i].y;
            new_point_stamped.point.z = time_graph.task[ time_graph.shooting_node[current_father_node].index_of_task_drone_or_bs ].reference_target_waypoints[i].z;
            shooting_drone_action.shooting_action.rt_trajectory.push_back( new_point_stamped );
          }

          // If whole task isn't covered, edit the shooting action to remove the uncovered pieces of trajectories from the drone action.
          if ( time_graph.shooting_node[current_son_node].time != time_graph.task[ time_graph.shooting_node[current_father_node].index_of_task_drone_or_bs ].time_final ) {
            // Covered piece of task doesn't end at the end:
            trimShootingActionTrajectories(time_graph.shooting_node[current_son_node].time, true, time_graph.task[ time_graph.shooting_node[current_father_node].index_of_task_drone_or_bs ], shooting_drone_action.shooting_action.rt_trajectory );
          }
          if ( time_graph.shooting_node[current_father_node].time != time_graph.task[ time_graph.shooting_node[current_father_node].index_of_task_drone_or_bs ].time_initial ) {
            // Covered piece of task doesn't start at the beginning:
            trimShootingActionTrajectories(time_graph.shooting_node[current_father_node].time, false, time_graph.task[ time_graph.shooting_node[current_father_node].index_of_task_drone_or_bs ], shooting_drone_action.shooting_action.rt_trajectory );
          }

          shooting_drone_action.shooting_action.length = 0;
          for (int k = 0; k < shooting_drone_action.shooting_action.rt_trajectory.size() - 1; k++)
            shooting_drone_action.shooting_action.length += sqrt(pow(shooting_drone_action.shooting_action.rt_trajectory[k + 1].point.x - shooting_drone_action.shooting_action.rt_trajectory[k].point.x, 2) + pow(shooting_drone_action.shooting_action.rt_trajectory[k + 1].point.y - shooting_drone_action.shooting_action.rt_trajectory[k].point.y, 2) + pow(shooting_drone_action.shooting_action.rt_trajectory[k + 1].point.z - shooting_drone_action.shooting_action.rt_trajectory[k].point.z, 2));

          break;
        }
      }

      // Fill the path field of the (shooting) drone action with the expected trajectory.
      // In this first for loop the path is filled without time:
      int task_index = time_graph.shooting_node[current_father_node].index_of_task_drone_or_bs;
      geometry_msgs::Point32 auxiliary_displaced_point;
      geometry_msgs::PointStamped auxiliary_displaced_point_stamped;
      float distance_since_task_start = 0;
      float time_since_task_start = 0;
      float azimuth;
      for (int k=0; k<shooting_drone_action.shooting_action.rt_trajectory.size(); k++) {
        if ( shooting_drone_action.shooting_action.shooting_roles[0].shooting_type.type != multidrone_msgs::ShootingType::SHOOT_TYPE_ORBIT ) {
          if (k>0) {
            auxiliary_displaced_point.x = shooting_drone_action.shooting_action.rt_trajectory[k].point.x;
            auxiliary_displaced_point.y = shooting_drone_action.shooting_action.rt_trajectory[k].point.y;
            auxiliary_displaced_point.z = shooting_drone_action.shooting_action.rt_trajectory[k].point.z;
            distance_since_task_start += sqrt( pow(shooting_drone_action.shooting_action.rt_trajectory[k].point.x-shooting_drone_action.shooting_action.rt_trajectory[k-1].point.x, 2) + pow(shooting_drone_action.shooting_action.rt_trajectory[k].point.y-shooting_drone_action.shooting_action.rt_trajectory[k-1].point.y, 2) + pow(shooting_drone_action.shooting_action.rt_trajectory[k].point.z-shooting_drone_action.shooting_action.rt_trajectory[k-1].point.z, 2) );
            time_since_task_start = (float)( shooting_drone_action.shooting_action.duration.data.sec ) * distance_since_task_start/shooting_drone_action.shooting_action.length;
            azimuth = atan2( shooting_drone_action.shooting_action.rt_trajectory[k].point.y - shooting_drone_action.shooting_action.rt_trajectory[k-1].point.y , shooting_drone_action.shooting_action.rt_trajectory[k].point.x - shooting_drone_action.shooting_action.rt_trajectory[k-1].point.x );

            displacePointByTaskParameters(auxiliary_displaced_point, time_graph.task[task_index], azimuth, time_since_task_start);

            auxiliary_displaced_point_stamped.point.x = auxiliary_displaced_point.x;
            auxiliary_displaced_point_stamped.point.y = auxiliary_displaced_point.y;
            auxiliary_displaced_point_stamped.point.z = auxiliary_displaced_point.z;
            auxiliary_displaced_point_stamped.header.stamp.sec = -1;
            shooting_drone_action.path.push_back(auxiliary_displaced_point_stamped);
          }
          if (k<shooting_drone_action.shooting_action.rt_trajectory.size()-1) {
            auxiliary_displaced_point.x = shooting_drone_action.shooting_action.rt_trajectory[k].point.x;
            auxiliary_displaced_point.y = shooting_drone_action.shooting_action.rt_trajectory[k].point.y;
            auxiliary_displaced_point.z = shooting_drone_action.shooting_action.rt_trajectory[k].point.z;
            azimuth = atan2( shooting_drone_action.shooting_action.rt_trajectory[k+1].point.y - shooting_drone_action.shooting_action.rt_trajectory[k].point.y , shooting_drone_action.shooting_action.rt_trajectory[k+1].point.x - shooting_drone_action.shooting_action.rt_trajectory[k].point.x );

            displacePointByTaskParameters(auxiliary_displaced_point, time_graph.task[task_index], azimuth, time_since_task_start);

            auxiliary_displaced_point_stamped.point.x = auxiliary_displaced_point.x;
            auxiliary_displaced_point_stamped.point.y = auxiliary_displaced_point.y;
            auxiliary_displaced_point_stamped.point.z = auxiliary_displaced_point.z;
            auxiliary_displaced_point_stamped.header.stamp.sec = -1;
            shooting_drone_action.path.push_back(auxiliary_displaced_point_stamped);
          }
          if (shooting_drone_action.shooting_action.rt_trajectory.size()==1) {
            auxiliary_displaced_point.x = shooting_drone_action.shooting_action.rt_trajectory[0].point.x;
            auxiliary_displaced_point.y = shooting_drone_action.shooting_action.rt_trajectory[0].point.y;
            auxiliary_displaced_point.z = shooting_drone_action.shooting_action.rt_trajectory[0].point.z;

            displacePointByTaskParameters(auxiliary_displaced_point, time_graph.task[task_index], time_graph.task[task_index].initial_azimuth, 0);

            auxiliary_displaced_point_stamped.point.x = auxiliary_displaced_point.x;
            auxiliary_displaced_point_stamped.point.y = auxiliary_displaced_point.y;
            auxiliary_displaced_point_stamped.point.z = auxiliary_displaced_point.z;
            auxiliary_displaced_point_stamped.header.stamp.sec = -1;
            shooting_drone_action.path.push_back(auxiliary_displaced_point_stamped);
          }
        } else {  // ORBIT
          if (shooting_drone_action.shooting_action.rt_trajectory.size()==1 ) {
            bool insert_last_point_manually = shooting_drone_action.shooting_action.duration.data.sec % orbit_path_separation_of_discretization_ > 0 ? true : false;
            for (int m=0; m<shooting_drone_action.shooting_action.duration.data.sec/orbit_path_separation_of_discretization_; m++) {
              auxiliary_displaced_point.x = shooting_drone_action.shooting_action.rt_trajectory[0].point.x;
              auxiliary_displaced_point.y = shooting_drone_action.shooting_action.rt_trajectory[0].point.y;
              auxiliary_displaced_point.z = shooting_drone_action.shooting_action.rt_trajectory[0].point.z;

              time_since_task_start = m*orbit_path_separation_of_discretization_;

              displacePointByTaskParameters(auxiliary_displaced_point, time_graph.task[task_index], time_graph.task[task_index].initial_azimuth, time_since_task_start);

              auxiliary_displaced_point_stamped.point.x = auxiliary_displaced_point.x;
              auxiliary_displaced_point_stamped.point.y = auxiliary_displaced_point.y;
              auxiliary_displaced_point_stamped.point.z = auxiliary_displaced_point.z;
              auxiliary_displaced_point_stamped.header.stamp.sec = shooting_drone_action.shooting_action.estimated_start_time + time_since_task_start;
              shooting_drone_action.path.push_back(auxiliary_displaced_point_stamped);
            }
            if (insert_last_point_manually) {
              auxiliary_displaced_point.x = shooting_drone_action.shooting_action.rt_trajectory[0].point.x;
              auxiliary_displaced_point.y = shooting_drone_action.shooting_action.rt_trajectory[0].point.y;
              auxiliary_displaced_point.z = shooting_drone_action.shooting_action.rt_trajectory[0].point.z;

              time_since_task_start = (float)( shooting_drone_action.shooting_action.duration.data.sec );

              displacePointByTaskParameters(auxiliary_displaced_point, time_graph.task[task_index], time_graph.task[task_index].initial_azimuth, time_since_task_start);

              auxiliary_displaced_point_stamped.point.x = auxiliary_displaced_point.x;
              auxiliary_displaced_point_stamped.point.y = auxiliary_displaced_point.y;
              auxiliary_displaced_point_stamped.point.z = auxiliary_displaced_point.z;
              auxiliary_displaced_point_stamped.header.stamp.sec = shooting_drone_action.shooting_action.estimated_start_time + shooting_drone_action.shooting_action.duration.data.sec;
              shooting_drone_action.path.push_back(auxiliary_displaced_point_stamped);
            }
          } else if (k<shooting_drone_action.shooting_action.rt_trajectory.size()-1) {
            float length_current_pair_of_waypoints = sqrt( pow(shooting_drone_action.shooting_action.rt_trajectory[k].point.x-shooting_drone_action.shooting_action.rt_trajectory[k+1].point.x, 2) + pow(shooting_drone_action.shooting_action.rt_trajectory[k].point.y-shooting_drone_action.shooting_action.rt_trajectory[k+1].point.y, 2) + pow(shooting_drone_action.shooting_action.rt_trajectory[k].point.z-shooting_drone_action.shooting_action.rt_trajectory[k+1].point.z, 2) );
            float number_of_iterations_float = (float)shooting_drone_action.shooting_action.duration.data.sec/(float)orbit_path_separation_of_discretization_ * length_current_pair_of_waypoints/shooting_drone_action.shooting_action.length;
            int number_of_iterations = (int)number_of_iterations_float;
            for (int m=0; m<number_of_iterations; m++) {
              auxiliary_displaced_point.x = shooting_drone_action.shooting_action.rt_trajectory[k].point.x + ( shooting_drone_action.shooting_action.rt_trajectory[k+1].point.x - shooting_drone_action.shooting_action.rt_trajectory[k].point.x ) * m/number_of_iterations;
              auxiliary_displaced_point.y = shooting_drone_action.shooting_action.rt_trajectory[k].point.y + ( shooting_drone_action.shooting_action.rt_trajectory[k+1].point.y - shooting_drone_action.shooting_action.rt_trajectory[k].point.y ) * m/number_of_iterations;
              auxiliary_displaced_point.z = shooting_drone_action.shooting_action.rt_trajectory[k].point.z + ( shooting_drone_action.shooting_action.rt_trajectory[k+1].point.z - shooting_drone_action.shooting_action.rt_trajectory[k].point.z ) * m/number_of_iterations;

              float azimuth = atan2( shooting_drone_action.shooting_action.rt_trajectory[k+1].point.y - shooting_drone_action.shooting_action.rt_trajectory[k].point.y , shooting_drone_action.shooting_action.rt_trajectory[k+1].point.x - shooting_drone_action.shooting_action.rt_trajectory[k].point.x );

              displacePointByTaskParameters(auxiliary_displaced_point, time_graph.task[task_index], azimuth, time_since_task_start);

              auxiliary_displaced_point_stamped.point.x = auxiliary_displaced_point.x;
              auxiliary_displaced_point_stamped.point.y = auxiliary_displaced_point.y;
              auxiliary_displaced_point_stamped.point.z = auxiliary_displaced_point.z;
              auxiliary_displaced_point_stamped.header.stamp.sec = shooting_drone_action.shooting_action.estimated_start_time + time_since_task_start;
              shooting_drone_action.path.push_back(auxiliary_displaced_point_stamped);

              time_since_task_start += orbit_path_separation_of_discretization_;
            }
            distance_since_task_start += sqrt( pow(shooting_drone_action.shooting_action.rt_trajectory[k].point.x-shooting_drone_action.shooting_action.rt_trajectory[k+1].point.x, 2) + pow(shooting_drone_action.shooting_action.rt_trajectory[k].point.y-shooting_drone_action.shooting_action.rt_trajectory[k+1].point.y, 2) + pow(shooting_drone_action.shooting_action.rt_trajectory[k].point.z-shooting_drone_action.shooting_action.rt_trajectory[k+1].point.z, 2) );
            time_since_task_start = (float)shooting_drone_action.shooting_action.duration.data.sec * distance_since_task_start/shooting_drone_action.shooting_action.length;
          } else {
            auxiliary_displaced_point.x = shooting_drone_action.shooting_action.rt_trajectory[k].point.x;
            auxiliary_displaced_point.y = shooting_drone_action.shooting_action.rt_trajectory[k].point.y;
            auxiliary_displaced_point.z = shooting_drone_action.shooting_action.rt_trajectory[k].point.z;

            float azimuth = atan2( shooting_drone_action.shooting_action.rt_trajectory[shooting_drone_action.shooting_action.rt_trajectory.size()-1].point.y - shooting_drone_action.shooting_action.rt_trajectory[shooting_drone_action.shooting_action.rt_trajectory.size()-2].point.y , shooting_drone_action.shooting_action.rt_trajectory[shooting_drone_action.shooting_action.rt_trajectory.size()-1].point.x - shooting_drone_action.shooting_action.rt_trajectory[shooting_drone_action.shooting_action.rt_trajectory.size()-2].point.x );

            displacePointByTaskParameters(auxiliary_displaced_point, time_graph.task[task_index], azimuth, (float)shooting_drone_action.shooting_action.duration.data.sec);

            auxiliary_displaced_point_stamped.point.x = auxiliary_displaced_point.x;
            auxiliary_displaced_point_stamped.point.y = auxiliary_displaced_point.y;
            auxiliary_displaced_point_stamped.point.z = auxiliary_displaced_point.z;
            auxiliary_displaced_point_stamped.header.stamp.sec = shooting_drone_action.shooting_action.estimated_start_time + shooting_drone_action.shooting_action.duration.data.sec;
            shooting_drone_action.path.push_back(auxiliary_displaced_point_stamped);
          }
        }
      }
      // Total distance of the drone in this path:
      float drone_distance = 0;
      for (int k=0; k<shooting_drone_action.path.size()-1; k++) {
        drone_distance += sqrt(pow(shooting_drone_action.path[k+1].point.x - shooting_drone_action.path[k].point.x, 2) + pow(shooting_drone_action.path[k+1].point.y - shooting_drone_action.path[k].point.y, 2) + pow(shooting_drone_action.path[k+1].point.z - shooting_drone_action.path[k].point.z, 2));
      }
      // Fill the time in each waypoint of the path if not already included:
      if (shooting_drone_action.path[0].header.stamp.sec == -1) {
        float aux_path_distance = 0;
        if (shooting_drone_action.path.size()>1 && shooting_drone_action.shooting_action.shooting_roles[0].shooting_type.type!=multidrone_msgs::ShootingType::SHOOT_TYPE_STATIC && drone_distance!=0) {
          for (int k=0; k<shooting_drone_action.path.size()-1; k++) {
            shooting_drone_action.path[k].header.stamp.sec = shooting_drone_action.shooting_action.estimated_start_time + shooting_drone_action.shooting_action.duration.data.sec * (double)aux_path_distance/drone_distance;   // Estimate time by distance done at the waypoint in proportion with the total and the duration.
            // CAUTION: seconds in header.stamp.sec so big that using float results in a low resolution of seconds and the information is lost (approximated with several seconds of error) USE DOUBLE INSTEAD TO AVOID THIS.
            aux_path_distance += sqrt(pow(shooting_drone_action.path[k+1].point.x - shooting_drone_action.path[k].point.x, 2) + pow(shooting_drone_action.path[k+1].point.y - shooting_drone_action.path[k].point.y, 2) + pow(shooting_drone_action.path[k+1].point.z - shooting_drone_action.path[k].point.z, 2));
          }
          shooting_drone_action.path[shooting_drone_action.path.size()-1].header.stamp.sec = shooting_drone_action.shooting_action.estimated_start_time + shooting_drone_action.shooting_action.duration.data.sec;
        } else if (shooting_drone_action.path.size()==1) {
          shooting_drone_action.path[0].header.stamp.sec = shooting_drone_action.shooting_action.estimated_start_time;
        } else {
          for (int k=0; k<shooting_drone_action.path.size()-1; k++) {
            shooting_drone_action.path[k].header.stamp.sec = shooting_drone_action.shooting_action.estimated_start_time;
          }
          shooting_drone_action.path[shooting_drone_action.path.size()-1].header.stamp.sec = shooting_drone_action.shooting_action.estimated_start_time + shooting_drone_action.shooting_action.duration.data.sec;
        }
      }

      shooting_drone_action.delay_since_event = shooting_drone_action.shooting_action.delay_since_event;
      one_drone_plan.push_back(shooting_drone_action);

    } else if ( time_graph.edge[ current_edge ].shooting_or_navigation_action == true ) {
      // The edge is a navigation action (may have takeoff or land):

      if ( (current_father_node < 0) && (current_son_node < 0) ) continue;   // Fictional navigation action while changing battery in base station, do nothing.

      // If the navigation edge is between two SA in the same SAS, don't insert a navigation action between them.
      if (current_father_node>=0 && current_son_node>=0) {
        if (time_graph.task[time_graph.shooting_node[current_father_node].index_of_task_drone_or_bs].SA_sequence_id == time_graph.task[time_graph.shooting_node[current_son_node].index_of_task_drone_or_bs].SA_sequence_id) {
          continue;
        }
      }


      // Navigation action (don't push_back yet! before check if there is a takeoff in the navigation edge):
      multidrone_msgs::DroneAction navigation_drone_action;
      navigation_drone_action.final_yaw_if_gotowaypoint.x = 0;
      navigation_drone_action.final_yaw_if_gotowaypoint.y = 0;
      navigation_drone_action.final_yaw_if_gotowaypoint.z = 0;
      navigation_drone_action.final_yaw_if_gotowaypoint.w = 0;
      navigation_drone_action.action_type = multidrone_msgs::DroneAction::TYPE_GOTOWAYPOINT;
      navigation_drone_action.action_id = current_son_node >= 0 ? time_graph.task[ time_graph.shooting_node[current_son_node].index_of_task_drone_or_bs ].SA_id : "0" ;
      navigation_drone_action.action_sequence_id = current_son_node >= 0 ? time_graph.task[ time_graph.shooting_node[current_son_node].index_of_task_drone_or_bs ].SA_sequence_id : "" ;
      navigation_drone_action.mission_id = current_son_node >= 0 ? time_graph.task[ time_graph.shooting_node[current_son_node].index_of_task_drone_or_bs ].mission_id : "" ;
      navigation_drone_action.start_event = current_son_node >= 0 && current_father_node >= 0 ? time_graph.task[ time_graph.shooting_node[current_son_node].index_of_task_drone_or_bs ].event_id : "";
      navigation_drone_action.delay_since_event = 0;

      // Initial and final points' calculation (x, y, z and time) of the navigation action:
      geometry_msgs::PointStamped initial_point;
      geometry_msgs::PointStamped final_point;
      initial_point.point.x = current_father_node >=0 ? time_graph.shooting_node[current_father_node].position.x : time_graph.base_station_node[-current_father_node].position.x;
      initial_point.point.y = current_father_node >=0 ? time_graph.shooting_node[current_father_node].position.y : time_graph.base_station_node[-current_father_node].position.y;
      initial_point.point.z = current_father_node >=0 ? time_graph.shooting_node[current_father_node].position.z : time_graph.base_station_node[-current_father_node].position.z;
      final_point.point.x = current_son_node >= 0 ? time_graph.shooting_node[current_son_node].position.x : time_graph.base_station_node[-current_son_node].position.x;
      final_point.point.y = current_son_node >= 0 ? time_graph.shooting_node[current_son_node].position.y : time_graph.base_station_node[-current_son_node].position.y;
      final_point.point.z = current_son_node >= 0 ? time_graph.shooting_node[current_son_node].position.z : time_graph.base_station_node[-current_son_node].position.z;
      if ( current_son_node < 0 ) {                    // If the navigation action ends up with a landing, the z of all the waypoints is the z of the initial point.
        final_point.point.z = initial_point.point.z;
      } else {                                         // Else if the navigation action ends up in a shooting action, the z of all the waypoints is the one of that next shooting action.
        initial_point.point.z = final_point.point.z;
      }
      initial_point.header.stamp.sec = current_father_node >=0 ? time_graph.shooting_node[current_father_node].time : time_graph.base_station_node[-current_father_node].time + (int)((initial_point.point.z - time_graph.base_station_node[-current_father_node].position.z)/full_speed_z_up_ + 0.5);
      final_point.header.stamp.sec = current_son_node >=0 ? time_graph.shooting_node[current_son_node].time : time_graph.base_station_node[-current_son_node].time - (int)((final_point.point.z - time_graph.base_station_node[-current_son_node].position.z)/full_speed_z_down_ + 0.5);
      if ( current_father_node < 0 )
        if ( initial_point.point.z <= time_graph.base_station_node[-current_father_node].position.z )
          ROS_WARN("High-Level Planner: CAUTION (1), drone will crash trying to take off, base station higher than the trajectory of the drone. High point: %f. Low point: %f", initial_point.point.z, time_graph.base_station_node[-current_father_node].position.z);
      if ( current_son_node < 0 )
        if ( final_point.point.z <= time_graph.base_station_node[-current_son_node].position.z )
          ROS_WARN("High-Level Planner: CAUTION (2), drone will crash trying to land, base station higher than the trajectory of the drone. High point: %f. Low point: %f", final_point.point.z, time_graph.base_station_node[-current_son_node].position.z);

      // Path calculation of the navigation action (waypoints with x, y, z and time):
      navigation_drone_action.path = path_planner_.getPathWithTimePredictionsAndInitialPoint(initial_point, final_point, full_speed_xy_, full_speed_z_down_, full_speed_z_up_);         // Calculate path of the navigation action edge while predicting the time in each waypoint.

      int k;
      for (k=0; k<time_graph.initial_drone_state.size(); k++) {   // Search the index in time_graph.initial_drone_state of this drone (k)
        if (time_graph.initial_drone_state[k].drone_id == drone_id) {
          break;
        }
      }

      if ( current_father_node < 0 ) {
        if ( time_graph.base_station_node[-current_father_node].land_or_takeoff == true ) {
          if ( ! time_graph.initial_drone_state[k].drone_flying_initially ) { // Only takeoff if the drone is in the ground.
            // Takeoff before navigation action:

            multidrone_msgs::DroneAction takeoff_drone_action;
            takeoff_drone_action.final_yaw_if_gotowaypoint.x = 0;
            takeoff_drone_action.final_yaw_if_gotowaypoint.y = 0;
            takeoff_drone_action.final_yaw_if_gotowaypoint.z = 0;
            takeoff_drone_action.final_yaw_if_gotowaypoint.w = 0;
            takeoff_drone_action.action_type = multidrone_msgs::DroneAction::TYPE_TAKEOFF;
            takeoff_drone_action.start_event = get_ready_event_;
            takeoff_drone_action.delay_since_event = 0;
            geometry_msgs::PointStamped takeoff_waypoint;
            takeoff_waypoint.header.stamp.sec = time_graph.base_station_node[-current_father_node].time + (initial_point.point.z - time_graph.base_station_node[-current_father_node].position.z)/full_speed_z_up_;
            takeoff_waypoint.point.x = initial_point.point.x;
            takeoff_waypoint.point.y = initial_point.point.y;
            takeoff_waypoint.point.z = initial_point.point.z;
            takeoff_drone_action.path.push_back(takeoff_waypoint);
            takeoff_drone_action.action_id = "0";   // action_id of the drone action is zero when taking off.
            takeoff_drone_action.action_sequence_id = "";
            takeoff_drone_action.mission_id = "";
            one_drone_plan.push_back(takeoff_drone_action);

          }
        }
      }


      if (one_drone_plan.size() > 0 || path_planner_.getFlatDistance() > replanning_same_SA_distance_) {  // Don't do the navigation action if it's the first drone action (drone already flying so no previous takeoff) and the drone was flying near (which means continuing the same shooting action).
        one_drone_plan.push_back(navigation_drone_action);
      }


      if ( current_son_node < 0 ) {
        if ( time_graph.base_station_node[-current_son_node].land_or_takeoff == false ) {
          // Land after navigation action:

          multidrone_msgs::DroneAction land_drone_action;
          land_drone_action.final_yaw_if_gotowaypoint.x = 0;
          land_drone_action.final_yaw_if_gotowaypoint.y = 0;
          land_drone_action.final_yaw_if_gotowaypoint.z = 0;
          land_drone_action.final_yaw_if_gotowaypoint.w = 0;
          land_drone_action.action_type = multidrone_msgs::DroneAction::TYPE_LAND;
          land_drone_action.action_id = "0";    // action_id of the drone action is zero when landing.
          land_drone_action.action_sequence_id = "";
          land_drone_action.mission_id = "";
          land_drone_action.delay_since_event = 0;
          geometry_msgs::PointStamped land_waypoint;
          land_waypoint.header.stamp.sec = time_graph.base_station_node[-current_son_node].time;
          land_waypoint.point.x = time_graph.base_station_node[-current_son_node].position.x;
          land_waypoint.point.y = time_graph.base_station_node[-current_son_node].position.y;
          land_waypoint.point.z = time_graph.base_station_node[-current_son_node].position.z;
          land_drone_action.path.push_back(land_waypoint);
          one_drone_plan.push_back(land_drone_action);

        }
      }
    }
  }


  for (int i=0; i<one_drone_plan.size(); i++) {
    if (one_drone_plan[i].action_type==multidrone_msgs::DroneAction::TYPE_TAKEOFF && one_drone_plan[i+2].delay_since_event>0) {
      one_drone_plan[i].start_event = one_drone_plan[i+2].start_event;
      one_drone_plan[i].delay_since_event = one_drone_plan[i+2].delay_since_event;
    }
    if (one_drone_plan[i].action_type==multidrone_msgs::DroneAction::TYPE_GOTOWAYPOINT && i+1<one_drone_plan.size() && one_drone_plan[i+1].action_type==multidrone_msgs::DroneAction::TYPE_SHOOTING) {
      double final_yaw;

      if (one_drone_plan[i+1].shooting_action.rt_trajectory.size()==1 || one_drone_plan[i+1].path.size()==1 || one_drone_plan[i+1].shooting_action.length==0 || one_drone_plan[i+1].shooting_action.shooting_roles[0].shooting_type.type==multidrone_msgs::ShootingType::SHOOT_TYPE_STATIC || one_drone_plan[i+1].shooting_action.shooting_roles[0].shooting_type.type==multidrone_msgs::ShootingType::SHOOT_TYPE_ELEVATOR  || one_drone_plan[i+1].shooting_action.shooting_roles[0].shooting_type.type==multidrone_msgs::ShootingType::SHOOT_TYPE_FLYBY || one_drone_plan[i+1].shooting_action.shooting_roles[0].shooting_type.type==multidrone_msgs::ShootingType::SHOOT_TYPE_ESTABLISH || one_drone_plan[i+1].shooting_action.shooting_roles[0].shooting_type.type==multidrone_msgs::ShootingType::SHOOT_TYPE_CHASE || one_drone_plan[i+1].shooting_action.shooting_roles[0].shooting_type.type==multidrone_msgs::ShootingType::SHOOT_TYPE_LEAD) {
        // The yaw at the final of the navigation makes the drone look to where the target will be in the next shooting:
        final_yaw = atan2( one_drone_plan[i+1].shooting_action.rt_trajectory[0].point.y - one_drone_plan[i+1].path[0].point.y , one_drone_plan[i+1].shooting_action.rt_trajectory[0].point.x - one_drone_plan[i+1].path[0].point.x );
      } else if (one_drone_plan[i+1].shooting_action.shooting_roles[0].shooting_type.type==multidrone_msgs::ShootingType::SHOOT_TYPE_ORBIT || one_drone_plan[i+1].shooting_action.shooting_roles[0].shooting_type.type==multidrone_msgs::ShootingType::SHOOT_TYPE_FLY_THROUGH || one_drone_plan[i+1].shooting_action.shooting_roles[0].shooting_type.type==multidrone_msgs::ShootingType::SHOOT_TYPE_LATERAL) {
        // The yaw at the final of the navigation makes the drone look to where it's going to move:
        final_yaw = atan2( one_drone_plan[i+1].path[1].point.y - one_drone_plan[i+1].path[0].point.y , one_drone_plan[i+1].path[1].point.x - one_drone_plan[i+1].path[0].point.x );
      } else {
        final_yaw = 0;
      }

      // pitch = 0, roll = 0 and yaw = final_yaw from sexagesimal to quaternion.

      // Abbreviations for the various angular functions
      double yaw = final_yaw;
      double pitch = 0;
      double roll = 0;

      double cy = cos(yaw * 0.5);
      double sy = sin(yaw * 0.5);
      double cp = cos(pitch * 0.5);
      double sp = sin(pitch * 0.5);
      double cr = cos(roll * 0.5);
      double sr = sin(roll * 0.5);

      one_drone_plan[i].final_yaw_if_gotowaypoint.w = cy * cp * cr + sy * sp * sr;
      one_drone_plan[i].final_yaw_if_gotowaypoint.x = cy * cp * sr - sy * sp * cr;
      one_drone_plan[i].final_yaw_if_gotowaypoint.y = sy * cp * sr + cy * sp * cr;
      one_drone_plan[i].final_yaw_if_gotowaypoint.z = sy * cp * cr - cy * sp * sr;
    }
  }


  // All drone actions of the current drone calculated, insert it into the plan:
  plan[drone_id] = one_drone_plan;


#ifdef DEBUG_HIGH_LEVEL_PLANNER
  showTimeGraphAndDroneActionsForDebug(time_graph, plan, drone_id);
#endif


  return best_last_base_station_node;

} // end addBetterTimeGraphSolutionToPlan method



// Method that reset the time graph (by reference) to an state similar to just initilized (with taks and base stations and without edges and nodes), but with the pieces of tasks assigned and the initial drone states used removed.
void HighLevelPlanner::resetTimeGraph(TimeGraph& time_graph, const int last_bs_node_of_subgraph_to_erase) {

  std::vector<Task> new_tasks;

  struct ShootingEdgeRegistry {
    int t_ini;
    int t_end;
  };
  std::map< int,std::vector<ShootingEdgeRegistry> > shooting_edges_registries;  // Map which keys are task indexes and values shooting edges registries.

  // Iterate the time graph's edges from the last base station node to its root base station node, removing or cutting the used tasks.
  int son_node_index = last_bs_node_of_subgraph_to_erase;
  int edge_index = time_graph.base_station_node[-son_node_index].edge_into_node;
  int father_node_index = time_graph.edge[edge_index].father_node;   // Can be positive (father is shooting node) or negative (father is another base station node).

  bool keep_in_loop = true;

  while (keep_in_loop)  {

    // if shooting edge, remove the used task in that shooting edge if all the task has been covered, and if only a piece of task has been covered erase and create the remaining piece of task.
    if ( time_graph.edge[edge_index].shooting_or_navigation_action == false ) {

      int task_index = time_graph.shooting_node[father_node_index].index_of_task_drone_or_bs;

      ShootingEdgeRegistry current_shooting_edge_registry;
      current_shooting_edge_registry.t_ini = time_graph.shooting_node[father_node_index].time;
      current_shooting_edge_registry.t_end = time_graph.shooting_node[son_node_index].time;

      shooting_edges_registries[ task_index ].push_back(current_shooting_edge_registry);

    }

    // Update the edge and its son node and father node:
    son_node_index = father_node_index;
    if (son_node_index >= 0) edge_index = time_graph.shooting_node[son_node_index].edge_into_node;
    else                    edge_index = time_graph.base_station_node[-son_node_index].edge_into_node;
    father_node_index = time_graph.edge[edge_index].father_node;

    // Check if the condition of ending the loop is fulfilled:
    if (father_node_index >= 0) keep_in_loop = true;
    else if ( !( (time_graph.base_station_node[-father_node_index].land_or_takeoff == true) && (time_graph.base_station_node[-father_node_index].edge_into_node == -1) ) ) keep_in_loop = true;
    else keep_in_loop = false;
  }
  // IMPORTANT: this loop doesn't iterate with the last edge, but as it's a navigation edge it doesn't matter.

  for (int i=0; i<time_graph.task.size(); i++) {

    if ( shooting_edges_registries.count(i) <= 0 ) {
      ShootingEdgeRegistry empty_shooting_registry;
      empty_shooting_registry.t_ini = -1;
      empty_shooting_registry.t_end = -1;
      shooting_edges_registries[i].push_back(empty_shooting_registry);
    } else {
      std::reverse(shooting_edges_registries[i].begin(), shooting_edges_registries[i].end());
    }

  }


  for ( std::map< int,std::vector<ShootingEdgeRegistry> >::iterator it = shooting_edges_registries.begin(); it != shooting_edges_registries.end(); it++ ) {

    if ( it->second.size()==1 && it->second[0].t_ini == -1 && it->second[0].t_end == -1 ) {
      new_tasks.push_back( time_graph.task[it->first] );
    } else {

      for (int i=0; i<it->second.size(); i++) {
        if ( i == 0 &&  ( it->second[0].t_ini != time_graph.task[it->first].time_initial ) ) {
          if ( it->second[0].t_ini - time_swap_drone_ > time_graph.task[it->first].time_initial ) {
            Task new_first_trimmed_task = time_graph.task[it->first];

            new_first_trimmed_task.time_initial = time_graph.task[it->first].time_initial;
            new_first_trimmed_task.time_final = it->second[0].t_ini - time_swap_drone_;
            trimTask(new_first_trimmed_task, time_graph.task[it->first]);

            new_tasks.push_back(new_first_trimmed_task);
          }
        }

        if ( i < it->second.size()-1 ) {
          if ( it->second[i+1].t_ini - time_swap_drone_ > it->second[i].t_end + time_swap_drone_ ) {
            Task new_intermedium_trimmed_task = time_graph.task[it->first];

            new_intermedium_trimmed_task.time_initial = it->second[i].t_end + time_swap_drone_;
            new_intermedium_trimmed_task.time_final = it->second[i+1].t_ini - time_swap_drone_;
            trimTask(new_intermedium_trimmed_task, time_graph.task[it->first]);

            new_tasks.push_back(new_intermedium_trimmed_task);
          }
        } else if ( i == it->second.size()-1 &&  ( it->second[i].t_end != time_graph.task[it->first].time_final ) ) {
          if ( time_graph.task[it->first].time_final > it->second[i].t_end + time_swap_drone_ ) {
            Task new_last_trimmed_task = time_graph.task[it->first];

            new_last_trimmed_task.time_initial = it->second[i].t_end + time_swap_drone_;
            new_last_trimmed_task.time_final = time_graph.task[it->first].time_final;
            trimTask(new_last_trimmed_task, time_graph.task[it->first]);

            new_tasks.push_back(new_last_trimmed_task);
          }
        }

      }
    }

  }


  // Change the tasks with the ones with the shooting edges removed:
  time_graph.task = new_tasks;


  // Set as used the current initial drone state assigned to drone_id in this method call so that next assignations won't use takeoff from there:
  time_graph.initial_drone_state[ time_graph.base_station_node[ -father_node_index ].index_of_task_drone_or_bs ].drone_used = true;


  // Prepare the time graph for the next drone assignation only if there are more drones left to assign and not all the recording time has been covered.
  time_graph.base_station_node.clear(); // Clear all base station nodes.
  time_graph.shooting_node.clear();     // Clear all shooting nodes.
  time_graph.edge.clear();              // Clear all edges.

} // end resetTimeGraph method



// Brief method that returns the distance and time separation between two waypoints in navigation actions (only valid if no landing involved in the navigation action).
HighLevelPlanner::NavigationSeparationStruct HighLevelPlanner::navigationSeparation(const geometry_msgs::Point32& from_here, const geometry_msgs::Point32& to_here, bool takeoff_or_flat_movement) {

  NavigationSeparationStruct solution_to_return;
  solution_to_return.distance = -1;

  if ( takeoff_or_flat_movement ) {
    // Flat movement:
    auto path = path_planner_.getPath(from_here, to_here);  // Calculate distance of the path between nodes with A* algorithm.
    if (path.size()==0) return solution_to_return;          // No path found.
    float distance_xy = path_planner_.getFlatDistance();
    float first_distance_xy = sqrt(pow(path[0].x-from_here.x,2)+pow(path[0].y-from_here.y,2));  // Distance from "from_here" to the first waypoint of the path.

    float moving_time_xy = distance_xy/full_speed_xy_;              // Navigation actions always at full speed to save time.
    float first_moving_time_xy = first_distance_xy/full_speed_xy_;  // Navigation actions always at full speed to save time.


    // Vertical movement:
    float distance_z = to_here.z - from_here.z;                   // Difference of height between the nodes.

    float moving_time_z = distance_z >= 0 ? distance_z/full_speed_z_up_ : abs(distance_z)/full_speed_z_down_;  // Navigation actions always at full speed to save time.


    // Resultant movement: the vertical movement (z) is done while reaching the first waypoint, between the rest of waypoints the movement is only flat (xy).
    solution_to_return.distance = distance_xy - first_moving_time_xy + sqrt( pow(distance_z,2) + pow(first_moving_time_xy,2) );  // Pythagorean theorem for the first waypoint.
    solution_to_return.moving_time = first_moving_time_xy >= moving_time_z ? (int)(moving_time_xy + 0.5) : (int)(moving_time_xy - first_moving_time_xy + moving_time_z + 0.5) ; // if first_moving_time_xy dominant above moving_time_z, ignore vertical movement time, else consider it only in the first waypoint movement.

    return solution_to_return;
  } else {
    // Takeoff movement:
    float takeoff_distance_z = to_here.z - from_here.z;
    if ( takeoff_distance_z <= 0 )
      ROS_WARN("High-Level Planner: CAUTION (3), drone will crash trying to take off, base station higher than the trajectory of the drone. High point: %f. Low point: %f", to_here.z, from_here.z);
    float takeoff_time_z = takeoff_distance_z/full_speed_z_up_; // Navigation actions always at full speed to save time.

    // Flat movement:
    auto path = path_planner_.getPath(from_here, to_here);    // Calculate distance of the path between nodes with A* algorithm.
    if (path.size()==0) return solution_to_return;            // No path found.
    float distance_xy = path_planner_.getFlatDistance();
    float moving_time_xy = distance_xy/full_speed_xy_;          // Navigation actions always at full speed to save time.

    // Separete movements one after another:
    solution_to_return.distance = takeoff_distance_z + distance_xy;
    solution_to_return.moving_time = (int)(takeoff_time_z + moving_time_xy + 0.5);

    return solution_to_return;
  }

} // end navigationSeparation method



// Brief method that returns the index of the nearest base station given an inital point, the battery drop and the moving time of going to the base station.
HighLevelPlanner::NearestBaseStationStruct HighLevelPlanner::nearestBaseStation(const geometry_msgs::Point32& from_here, const TimeGraph& time_graph) {

  NearestBaseStationStruct solution_to_return;
  solution_to_return.distance = -1;

  solution_to_return.battery_drop = std::numeric_limits<float>::max();

  // Calculate base station with minimum battery drop from the current node:
  for ( int i=1; i<time_graph.base_station.size(); i++ ) {

    // Flat movement to the base station:
    auto path = path_planner_.getPath(from_here, time_graph.base_station[i]); // Calculate distance of the path between nodes with A* algorithm.
    if (path.size()==0) continue;       // No path found.
    float distance_xy = path_planner_.getFlatDistance();
    float moving_time_xy = distance_xy/full_speed_xy_;                        // Navigation actions always at full speed to save time.

    // Landing movement:
    float landing_distance_z = from_here.z - time_graph.base_station[i].z;
    if ( landing_distance_z <= 0 )
      ROS_WARN("High-Level Planner: CAUTION (4), drone will crash trying to land, base station higher than the trajectory of the drone. High point: %f. Low point: %f", from_here.z, time_graph.base_station[i].z);
    float landing_time_z = landing_distance_z/full_speed_z_down_;  // Navigation actions always at full speed to save time.

    // Separete movements one after another and battery drop:
    float battery_drop = batteryDrop(0, full_speed_xy_, (int)(moving_time_xy + landing_time_z + 0.5));  // Navigation actions like going to a base station are done at full speed and without hovering waits (hovering_time==0).

    if ( solution_to_return.battery_drop > battery_drop ) {
      solution_to_return.battery_drop = battery_drop;
      solution_to_return.distance = distance_xy + landing_distance_z;
      solution_to_return.index = -i;
      solution_to_return.moving_time = (int)(moving_time_xy + landing_time_z + 0.5);
    }
  }

  return solution_to_return;
} // end nearestBaseStation method



// Brief method that returns an intermediate point in a trajectory (vector of waypoints) given a time. The initial point of the trajectory has its start time defined and the time to do that trajectory is also defined, so the time given for the intermediate point has to be between them.
geometry_msgs::Point32 HighLevelPlanner::intermediatePointOfTaskFromTime(int time_at_intermediate_point, const Task& task, bool return_displaced_by_parameters) {

  geometry_msgs::Point32 intermediate_point;

  if ( (time_at_intermediate_point >= task.time_initial) && (time_at_intermediate_point <= task.time_final) ) {

    if ( task.moving_or_hovering ) {        // If shooting action without movement then the middle point, first waypoint and last waypoint are all the same.
      intermediate_point = task.reference_target_waypoints[0];
      displacePointByTaskParameters(intermediate_point, task, task.initial_azimuth, time_at_intermediate_point-task.time_initial);
      return intermediate_point;
    }

    // To calculate the intermediate Cartesian point of the trajectory given a time, the first thing is to find the segment of the trajectory (pair of reference target waypoints) in which it is placed.
    // For this, the distance of task done at that intermediate point is calculated...
    float decimal_task_done_at_the_intermediate_point = (float)( time_at_intermediate_point - task.time_initial )/task.time_difference; // Task left (percentage not multiplied by 100)
    float distance_of_task_done_at_the_intermediate_point = decimal_task_done_at_the_intermediate_point * task.distance;

    if ( distance_of_task_done_at_the_intermediate_point <= 0 ) {    // If no distance done yet, return first waypoint.
      intermediate_point = task.reference_target_waypoints[0];
      displacePointByTaskParameters(intermediate_point, task, task.initial_azimuth, time_at_intermediate_point-task.time_initial);
      return intermediate_point;
    }

    // ... then it's iteratively checked if that distance is between pair of reference target waypoints.
    float distance_accumulated_in_j_minus_1 = 0;
    float distance_between_pair_of_wp;
    for (int j=1; j < task.reference_target_waypoints.size(); j++) {

      distance_between_pair_of_wp = sqrt( pow(task.reference_target_waypoints[j].x-task.reference_target_waypoints[j-1].x, 2) + pow(task.reference_target_waypoints[j].y-task.reference_target_waypoints[j-1].y, 2) + pow(task.reference_target_waypoints[j].z-task.reference_target_waypoints[j-1].z, 2) );

      if ( distance_of_task_done_at_the_intermediate_point <= distance_accumulated_in_j_minus_1 + distance_between_pair_of_wp ) {
        // Once found that segment (pair of waypoints), the Cartesian point is calculated with a parametric equation of a line.
        intermediate_point.x = task.reference_target_waypoints[j-1].x + ( task.reference_target_waypoints[j].x - task.reference_target_waypoints[j-1].x ) * (distance_of_task_done_at_the_intermediate_point - distance_accumulated_in_j_minus_1)/distance_between_pair_of_wp;
        intermediate_point.y = task.reference_target_waypoints[j-1].y + ( task.reference_target_waypoints[j].y - task.reference_target_waypoints[j-1].y ) * (distance_of_task_done_at_the_intermediate_point - distance_accumulated_in_j_minus_1)/distance_between_pair_of_wp;
        intermediate_point.z = task.reference_target_waypoints[j-1].z + ( task.reference_target_waypoints[j].z - task.reference_target_waypoints[j-1].z ) * (distance_of_task_done_at_the_intermediate_point - distance_accumulated_in_j_minus_1)/distance_between_pair_of_wp;
        if ( return_displaced_by_parameters ) {
          float azimuth = atan2( task.reference_target_waypoints[j].y - task.reference_target_waypoints[j-1].y , task.reference_target_waypoints[j].x - task.reference_target_waypoints[j-1].x );
          displacePointByTaskParameters(intermediate_point, task, azimuth, time_at_intermediate_point-task.time_initial);
        }
        break;  // intermediate_point found, break the loop.
      }

      distance_accumulated_in_j_minus_1 = distance_accumulated_in_j_minus_1 + distance_between_pair_of_wp;
    }
  } else {
    ROS_ERROR("High-Level Planner: Time at intermediate point not between the initial and final time of the task.");
  }

  return intermediate_point;
} // end intermediatePointOfTaskFromTime method



// Brief method that trim the task, adjusting its values to the new the initial and final times of the task to edit.
void HighLevelPlanner::trimTask(Task& task_to_edit, Task& task_not_edited)
{
  // Update times:
  task_to_edit.time_difference = task_to_edit.time_final - task_to_edit.time_initial;
  task_to_edit.previous_time_accumulated_for_rates_displacement += task_to_edit.time_initial - task_not_edited.time_initial;
  task_to_edit.delay_since_event += task_to_edit.time_initial - task_not_edited.time_initial;


  if ( task_to_edit.moving_or_hovering ) return;   // If shooting action without movement then no more changes are needed.


  // Trim the trajectory and update the initial_position, final_position and initial_azimuth:
  std::vector< int > wp_index_to_erase; // Dont't erase directly the waypoints of the trajectory, it's safer erase later with a sorted decreasing vector.

  if ( task_to_edit.time_initial != task_not_edited.time_initial )
    wp_index_to_erase.push_back( 0 );

  // The first thing is to find the segment of the trajectory (pair of reference target waypoints) in which are placed the initial and final points of the task edited.
  // For this, the distance of task done at that initial and final points of the task edited are calculated...
  float decimal_task_done_at_the_initial_position = (float)( task_to_edit.time_initial - task_not_edited.time_initial )/task_not_edited.time_difference; // Task left (percentage not multiplied by 100)
  float distance_of_task_done_at_the_initial_position = decimal_task_done_at_the_initial_position * task_not_edited.distance;
  float decimal_task_done_at_the_final_position = (float)( task_to_edit.time_final - task_not_edited.time_initial )/task_not_edited.time_difference; // Task left (percentage not multiplied by 100)
  float distance_of_task_done_at_the_final_position = decimal_task_done_at_the_final_position * task_not_edited.distance;

  bool initial_position_found = false;
  bool final_position_found = false;

  // ... then it's iteratively checked if that distance is between pair of reference target waypoints.
  float distance_accumulated_in_j_minus_1 = 0;
  float distance_between_pair_of_wp;
  for (int j=1; j < task_to_edit.reference_target_waypoints.size(); j++) {

    distance_between_pair_of_wp = sqrt( pow(task_to_edit.reference_target_waypoints[j].x-task_to_edit.reference_target_waypoints[j-1].x, 2) + pow(task_to_edit.reference_target_waypoints[j].y-task_to_edit.reference_target_waypoints[j-1].y, 2) + pow(task_to_edit.reference_target_waypoints[j].z-task_to_edit.reference_target_waypoints[j-1].z, 2) );

    // initial_position of the drone at the task found.
    if ( !initial_position_found && distance_of_task_done_at_the_initial_position <= distance_accumulated_in_j_minus_1 + distance_between_pair_of_wp ) {
      task_to_edit.initial_position.x = task_to_edit.reference_target_waypoints[j-1].x + ( task_to_edit.reference_target_waypoints[j].x - task_to_edit.reference_target_waypoints[j-1].x ) * (distance_of_task_done_at_the_initial_position - distance_accumulated_in_j_minus_1)/distance_between_pair_of_wp;
      task_to_edit.initial_position.y = task_to_edit.reference_target_waypoints[j-1].y + ( task_to_edit.reference_target_waypoints[j].y - task_to_edit.reference_target_waypoints[j-1].y ) * (distance_of_task_done_at_the_initial_position - distance_accumulated_in_j_minus_1)/distance_between_pair_of_wp;
      task_to_edit.initial_position.z = task_to_edit.reference_target_waypoints[j-1].z + ( task_to_edit.reference_target_waypoints[j].z - task_to_edit.reference_target_waypoints[j-1].z ) * (distance_of_task_done_at_the_initial_position - distance_accumulated_in_j_minus_1)/distance_between_pair_of_wp;
      task_to_edit.initial_azimuth = atan2( task_to_edit.reference_target_waypoints[j].y - task_to_edit.reference_target_waypoints[j-1].y , task_to_edit.reference_target_waypoints[j].x - task_to_edit.reference_target_waypoints[j-1].x );
      initial_position_found = true;
    }
    // final_position of the drone at the task found.
    if ( !final_position_found && distance_of_task_done_at_the_final_position <= distance_accumulated_in_j_minus_1 + distance_between_pair_of_wp ) {
      task_to_edit.final_position.x = task_to_edit.reference_target_waypoints[j-1].x + ( task_to_edit.reference_target_waypoints[j].x - task_to_edit.reference_target_waypoints[j-1].x ) * (distance_of_task_done_at_the_final_position - distance_accumulated_in_j_minus_1)/distance_between_pair_of_wp;
      task_to_edit.final_position.y = task_to_edit.reference_target_waypoints[j-1].y + ( task_to_edit.reference_target_waypoints[j].y - task_to_edit.reference_target_waypoints[j-1].y ) * (distance_of_task_done_at_the_final_position - distance_accumulated_in_j_minus_1)/distance_between_pair_of_wp;
      task_to_edit.final_position.z = task_to_edit.reference_target_waypoints[j-1].z + ( task_to_edit.reference_target_waypoints[j].z - task_to_edit.reference_target_waypoints[j-1].z ) * (distance_of_task_done_at_the_final_position - distance_accumulated_in_j_minus_1)/distance_between_pair_of_wp;
      final_position_found = true;
    }

    if ( distance_accumulated_in_j_minus_1 + distance_between_pair_of_wp <= distance_of_task_done_at_the_initial_position ) {
      wp_index_to_erase.push_back( j ); // Dont't erase directly, is safer erase later with a sorted decreasing vector.
    }
    if ( distance_of_task_done_at_the_final_position <= distance_accumulated_in_j_minus_1 + distance_between_pair_of_wp ) {
      wp_index_to_erase.push_back( j ); // Dont't erase directly, is safer erase later with a sorted decreasing vector.
    }

    distance_accumulated_in_j_minus_1 = distance_accumulated_in_j_minus_1 + distance_between_pair_of_wp;
  }

  // Reverse order of wp indexes to erase (from more to less now):
  std::reverse(wp_index_to_erase.begin(), wp_index_to_erase.end());

  // Erase waypoints of the trajectory:
  for ( int current_wp_index_to_erase: wp_index_to_erase )
    task_to_edit.reference_target_waypoints.erase( task_to_edit.reference_target_waypoints.begin() + current_wp_index_to_erase );


  // Insert new first point in the trajectory:
  task_to_edit.reference_target_waypoints.insert(task_to_edit.reference_target_waypoints.begin(), task_to_edit.initial_position);
  // Insert new last point in the trajectory:
  task_to_edit.reference_target_waypoints.push_back(task_to_edit.final_position);

  // Displace initial and final position:
  displacePointByTaskParameters(task_to_edit.initial_position, task_to_edit, task_to_edit.initial_azimuth, task_to_edit.previous_time_accumulated_for_rates_displacement);
  float final_azimuth = task_to_edit.reference_target_waypoints.size()>=2 ? atan2( task_to_edit.reference_target_waypoints[ task_to_edit.reference_target_waypoints.size()-1 ].y - task_to_edit.reference_target_waypoints[ task_to_edit.reference_target_waypoints.size()-2 ].y , task_to_edit.reference_target_waypoints[ task_to_edit.reference_target_waypoints.size()-1 ].x - task_to_edit.reference_target_waypoints[ task_to_edit.reference_target_waypoints.size()-2 ].x ) : task_to_edit.initial_azimuth;
  displacePointByTaskParameters(task_to_edit.final_position, task_to_edit, final_azimuth, task_to_edit.previous_time_accumulated_for_rates_displacement + task_to_edit.time_difference);

  // Update distance:
  task_to_edit.distance = 0;
  for (int j = 0; j < task_to_edit.reference_target_waypoints.size() - 1; j++)
    task_to_edit.distance += sqrt(pow(task_to_edit.reference_target_waypoints[j + 1].x - task_to_edit.reference_target_waypoints[j].x, 2) + pow(task_to_edit.reference_target_waypoints[j + 1].y - task_to_edit.reference_target_waypoints[j].y, 2) + pow(task_to_edit.reference_target_waypoints[j + 1].z - task_to_edit.reference_target_waypoints[j].z, 2));


} // end trimTask method



// Brief method that erase the waypoints in the trajectories (reference_target_waypoints and target_waypoints) of a shooting action before or after a given time (intermediate point), and insert the new intermediate point at the beginning or end of the shooting action's trajectory respectively.
void HighLevelPlanner::trimShootingActionTrajectories(int time_at_intermediate_point, bool erase_before_or_after_time, const Task& task, std::vector<geometry_msgs::PointStamped>& reference_target_waypoints_to_edit) {

  if ( task.moving_or_hovering ) return; // If shooting action without movement then the middle point, first waypoint and last waypoint are all the same.

  std::vector< int > wp_index_to_erase; // Dont't erase directly, is safer erase later with a sorted decreasing vector.
  if ( !erase_before_or_after_time ) {
    wp_index_to_erase.push_back( 0 );
  }

  // The first thing is to find the segment of the trajectory (pair of reference target waypoints) in which it is placed the intermediate point.
  // For this, the distance of task done at that intermediate point is calculated...
  float decimal_task_done_at_the_intermediate_point = (float)( time_at_intermediate_point - task.time_initial )/task.time_difference; // Task left (percentage not multiplied by 100)
  float distance_of_task_done_at_the_intermediate_point = decimal_task_done_at_the_intermediate_point * task.distance;

  // ... then it's iteratively checked if that distance is between pair of reference target waypoints.
  float distance_accumulated_in_j_minus_1 = 0;
  float distance_between_pair_of_wp;
  for (int j=1; j < reference_target_waypoints_to_edit.size(); j++) {

    distance_between_pair_of_wp = sqrt( pow(reference_target_waypoints_to_edit[j].point.x-reference_target_waypoints_to_edit[j-1].point.x, 2) + pow(reference_target_waypoints_to_edit[j].point.y-reference_target_waypoints_to_edit[j-1].point.y, 2) + pow(reference_target_waypoints_to_edit[j].point.z-reference_target_waypoints_to_edit[j-1].point.z, 2) );

    if ( !erase_before_or_after_time ) {
      if ( distance_accumulated_in_j_minus_1 + distance_between_pair_of_wp <= distance_of_task_done_at_the_intermediate_point ) {
        wp_index_to_erase.push_back( j ); // Dont't erase directly, is safer erase later with a sorted decreasing vector.
      }
    } else {
      if ( distance_accumulated_in_j_minus_1 + distance_between_pair_of_wp >= distance_of_task_done_at_the_intermediate_point ) {
        wp_index_to_erase.push_back( j ); // Dont't erase directly, is safer erase later with a sorted decreasing vector.
      }
    }

    distance_accumulated_in_j_minus_1 = distance_accumulated_in_j_minus_1 + distance_between_pair_of_wp;
  }

  std::reverse(wp_index_to_erase.begin(), wp_index_to_erase.end()); // Reverse indexes order (from more to less now)

  for ( int current_wp_index_to_erase: wp_index_to_erase ) {
    reference_target_waypoints_to_edit.erase( reference_target_waypoints_to_edit.begin() + current_wp_index_to_erase );
  }

  geometry_msgs::Point32 intermediate_point = intermediatePointOfTaskFromTime(time_at_intermediate_point, task, false);
  geometry_msgs::PointStamped reference_intermediate_point_stamped;
  reference_intermediate_point_stamped.point.x = intermediate_point.x;
  reference_intermediate_point_stamped.point.y = intermediate_point.y;
  reference_intermediate_point_stamped.point.z = intermediate_point.z;
  if ( !erase_before_or_after_time ) {
    reference_target_waypoints_to_edit.insert(reference_target_waypoints_to_edit.begin(), reference_intermediate_point_stamped);
  } else {
    reference_target_waypoints_to_edit.push_back( reference_intermediate_point_stamped );
  }

} // end trimShootingActionTrajectories method



// Brief method that displace a point of a task according to its parameters, its shooting type, etc.
void HighLevelPlanner::displacePointByTaskParameters(geometry_msgs::Point32& point_to_displace, const Task& task, float azimuth, float time_since_task_start, bool reverse_displacement) {

  float x_displacement = task.rt_displacement.x;
  float y_displacement = task.rt_displacement.y;
  float z_displacement = task.rt_displacement.z;

  float x_rate = 0;
  float y_rate = 0;
  float z_rate = 0;

  if (task.shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_ORBIT) {

    float initial_azimuth = task.shooting_parameters.count("azimuth_s")==0 ? 0 : task.shooting_parameters.at("azimuth_s");
    float radius = task.shooting_parameters.count("r_0")==0 ? 0 : task.shooting_parameters.at("r_0");
    float azimuth_speed = task.shooting_parameters.count("azimuth_speed")==0 ? 0 : task.shooting_parameters.at("azimuth_speed");

    x_displacement += radius * cos( initial_azimuth + azimuth_speed * time_since_task_start );
    y_displacement += radius * sin( initial_azimuth + azimuth_speed * time_since_task_start );
    z_displacement += task.shooting_parameters.count("z_0")==0 ? 0 : task.shooting_parameters.at("z_0");
  } else {
    x_displacement += task.shooting_parameters.count("x_s")==0 ? 0 : task.shooting_parameters.at("x_s");
    y_displacement += task.shooting_parameters.count("y_0")==0 ? 0 : task.shooting_parameters.at("y_0");
    z_displacement += task.shooting_parameters.count("z_0")==0 ? 0 : task.shooting_parameters.at("z_0");
    z_displacement += task.shooting_parameters.count("z_s")==0 ? 0 : task.shooting_parameters.at("z_s");

    x_rate = task.shooting_parameters.count("x_rate")==0 ? 0 : task.shooting_parameters.at("x_rate");
    y_rate = task.shooting_parameters.count("y_rate")==0 ? 0 : task.shooting_parameters.at("y_rate");
    z_rate = task.shooting_parameters.count("z_rate")==0 ? 0 : task.shooting_parameters.at("z_rate");
  }

  if (task.shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_STATIC || task.shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_ELEVATOR) {
    azimuth = 0;
  }

  if ( reverse_displacement ) {
    x_displacement *= -1;
    y_displacement *= -1;
    z_displacement *= -1;

    x_rate *= -1;
    y_rate *= -1;
    z_rate *= -1;
  }

  x_displacement += x_rate * time_since_task_start;
  y_displacement += y_rate * time_since_task_start;
  z_displacement += z_rate * time_since_task_start;

  point_to_displace.x += x_displacement * cos(azimuth) - y_displacement * sin(azimuth) ;
  point_to_displace.y += y_displacement * cos(azimuth) + x_displacement * sin(azimuth) ;
  point_to_displace.z += z_displacement;

} // end displacePointByTaskParameters method



// Brief method that returns the battery drop of a drone during a certain time graph's edge.
float HighLevelPlanner::batteryDrop(int hovering_time, float average_speed, int moving_time) {

  // NOTE: in navigation actions the "average_speed" received should be "full_speed_xy_".

  if ( time_max_hovering_ < time_max_flying_full_speed_ ) {  // time_max_hovering_ must be greater than time_max_flying_full_speed_
    ROS_ERROR("High-Level Planner: time_max_hovering_ must be greater than time_max_flying_full_speed_.");
    return 100;
  } else {
    float hovering_battery_drop = (float)hovering_time/(float)time_max_hovering_ * 100.0;     // Battery consumption hovering supposed linear in hovering time.

    float flying_battery_drop = average_speed>0 ? (float)moving_time*( (float)average_speed *(1.0/(float)time_max_flying_full_speed_ - 1.0/(float)time_max_hovering_)/(float)full_speed_xy_ +1.0/(float)time_max_hovering_ )*100.0 : 0;  // Battery consumption flying supposed linear in flying time and average speed.

    return hovering_battery_drop + flying_battery_drop;
  }
} // end batteryDrop method



// Postprocess after the planning phase to fix possible collisions between drones or shooting drone actions flying over no fly zones.
void HighLevelPlanner::safetyPostprocess(std::map< int,std::vector<multidrone_msgs::DroneAction> >& _plan) {

  // Check if any shooting drone action is inside an obstacle, if it is delete all the plan directly and return.
  for (std::vector<geometry_msgs::Point32> obstacle : path_planner_.KML_parser_from_path_planner_.no_fly_zones_cartesian_) {
    for (std::map< int,std::vector<multidrone_msgs::DroneAction> >::iterator it = _plan.begin(); it != _plan.end(); it++) {
      for (int i=0; i<it->second.size(); i++) {
        if (it->second[i].action_type==multidrone_msgs::DroneAction::TYPE_SHOOTING) {
          for (int j=0; j<it->second[i].path.size(); j++) {
            if (path_planner_.checkIfPointInsidePolygon(obstacle,it->second[i].path[j])) {
              // If found points inside an obstacle, delete all the plan directly and return:
              _plan.clear();
              return;
            }
          }
        }
      }
    }
  }

  if (_plan.size()<2) return;  // The rest of code in this function is to recognize if there can be collisions between drones, so if it's only one drone or no plan just return.

  struct MinMaxValuesOfEachDroneAction {
    float x_min = std::numeric_limits<float>::max();
    float y_min = std::numeric_limits<float>::max();
    float z_min = std::numeric_limits<float>::max();
    float x_max = -std::numeric_limits<float>::max();
    float y_max = -std::numeric_limits<float>::max();
    float z_max = -std::numeric_limits<float>::max();
    int t_min, t_max;
  };
  std::map< int, std::map<int,MinMaxValuesOfEachDroneAction> > minmax_values_map;  // First key is the drone_id, second key it's the drone action index, value is the struct with minimum and maximum time, x, y and z of the drone action.

  // Build the map with the minimum and maximum values of time, x, y and z for each drone action of each drone. This will speed up the later search of conflicts.
  for (std::map< int,std::vector<multidrone_msgs::DroneAction> >::iterator it = _plan.begin(); it != _plan.end(); it++) {
    for (int i=0; i<it->second.size(); i++) {
      if (it->second[i].action_type==multidrone_msgs::DroneAction::TYPE_GOTOWAYPOINT || it->second[i].action_type==multidrone_msgs::DroneAction::TYPE_SHOOTING) {
        MinMaxValuesOfEachDroneAction current_minmax;
        current_minmax.t_min = it->second[i].path[0].header.stamp.sec;
        current_minmax.t_max = it->second[i].path[it->second[i].path.size()-1].header.stamp.sec;
        for (int j=0; j<it->second[i].path.size(); j++) {
          current_minmax.x_min = current_minmax.x_min > it->second[i].path[j].point.x ? it->second[i].path[j].point.x : current_minmax.x_min;
          current_minmax.x_max = current_minmax.x_max < it->second[i].path[j].point.x ? it->second[i].path[j].point.x : current_minmax.x_max;
          current_minmax.y_min = current_minmax.y_min > it->second[i].path[j].point.y ? it->second[i].path[j].point.y : current_minmax.y_min;
          current_minmax.y_max = current_minmax.y_max < it->second[i].path[j].point.y ? it->second[i].path[j].point.y : current_minmax.y_max;
          current_minmax.z_min = current_minmax.z_min > it->second[i].path[j].point.z ? it->second[i].path[j].point.z : current_minmax.z_min;
          current_minmax.z_max = current_minmax.z_max < it->second[i].path[j].point.z ? it->second[i].path[j].point.z : current_minmax.z_max;
        }
        minmax_values_map[it->first][i] = current_minmax;
      }
    }
  }

  struct ConflictivePairOfDroneActions {
    std::vector< std::pair<int,int> > conflictive_time_windows;
    int drone_id_1;
    int drone_id_2;
    int drone_action_1;
    int drone_action_2;
  };
  std::vector<ConflictivePairOfDroneActions> vector_of_conflicts;

  // Search for conflictive pair of drone actions with collisions and save them.
  // This first block of for-if lines is a quick check if the two pair of t-x-y-z volumes intersect.
  for (std::map< int,std::vector<multidrone_msgs::DroneAction> >::iterator it_1 = _plan.begin(); it_1 != std::prev(_plan.end(), 1); it_1++) {
    for (std::map< int,std::vector<multidrone_msgs::DroneAction> >::iterator it_2 = std::next(it_1, 1); it_2 != _plan.end(); it_2++) {
      for (int i_1=0; i_1<it_1->second.size(); i_1++) {
        for (int i_2=0; i_2<it_2->second.size(); i_2++) {
          if ((it_1->second[i_1].action_type==multidrone_msgs::DroneAction::TYPE_GOTOWAYPOINT || it_1->second[i_1].action_type==multidrone_msgs::DroneAction::TYPE_SHOOTING) && (it_2->second[i_2].action_type==multidrone_msgs::DroneAction::TYPE_GOTOWAYPOINT || it_2->second[i_2].action_type==multidrone_msgs::DroneAction::TYPE_SHOOTING)) {
            if ((minmax_values_map[it_1->first][i_1].t_min>=minmax_values_map[it_2->first][i_2].t_min)&&(minmax_values_map[it_1->first][i_1].t_min<=minmax_values_map[it_2->first][i_2].t_max) || (minmax_values_map[it_1->first][i_1].t_max>=minmax_values_map[it_2->first][i_2].t_min)&&(minmax_values_map[it_1->first][i_1].t_max<=minmax_values_map[it_2->first][i_2].t_max)) {
              if ((minmax_values_map[it_1->first][i_1].z_min>=minmax_values_map[it_2->first][i_2].z_min-minimum_separation_z_)&&(minmax_values_map[it_1->first][i_1].z_min<=minmax_values_map[it_2->first][i_2].z_max+minimum_separation_z_) || (minmax_values_map[it_1->first][i_1].z_max>=minmax_values_map[it_2->first][i_2].z_min-minimum_separation_z_)&&(minmax_values_map[it_1->first][i_1].z_max<=minmax_values_map[it_2->first][i_2].z_max+minimum_separation_z_)) {
                if ((minmax_values_map[it_1->first][i_1].x_min>=minmax_values_map[it_2->first][i_2].x_min-minimum_separation_xy_)&&(minmax_values_map[it_1->first][i_1].x_min<=minmax_values_map[it_2->first][i_2].x_max+minimum_separation_xy_) || (minmax_values_map[it_1->first][i_1].x_max>=minmax_values_map[it_2->first][i_2].x_min-minimum_separation_xy_)&&(minmax_values_map[it_1->first][i_1].x_max<=minmax_values_map[it_2->first][i_2].x_max+minimum_separation_xy_)) {
                  if ((minmax_values_map[it_1->first][i_1].y_min>=minmax_values_map[it_2->first][i_2].y_min-minimum_separation_xy_)&&(minmax_values_map[it_1->first][i_1].y_min<=minmax_values_map[it_2->first][i_2].y_max+minimum_separation_xy_) || (minmax_values_map[it_1->first][i_1].y_max>=minmax_values_map[it_2->first][i_2].y_min-minimum_separation_xy_)&&(minmax_values_map[it_1->first][i_1].y_max<=minmax_values_map[it_2->first][i_2].y_max+minimum_separation_xy_)) {

                    // Collision possible, t-x-y-z volumes of both drone actions intersect in space and time. That doesn't mean that the trajectories intersect for sure, have to do a more precisely check of the trajectories.

                    // // Debug:
                    // ROS_WARN("COLLISION POSSIBLE! DEEP CHECK");

                    // Calculate the time coincident parts of the trajectories and charge it in a discretized way into 2 point stamped vectors:
                    int minimum_time_coincidence = minmax_values_map[it_1->first][i_1].t_min > minmax_values_map[it_2->first][i_2].t_min ? minmax_values_map[it_1->first][i_1].t_min : minmax_values_map[it_2->first][i_2].t_min;
                    int maximum_time_coincidence = minmax_values_map[it_1->first][i_1].t_max < minmax_values_map[it_2->first][i_2].t_max ? minmax_values_map[it_1->first][i_1].t_max : minmax_values_map[it_2->first][i_2].t_max;

                    std::vector<geometry_msgs::PointStamped> trajectory_1, trajectory_2;

                    if (minimum_time_coincidence!=maximum_time_coincidence) {

                      trajectory_1 = buildTrajectoryForCollisionDetection(it_1->second[i_1].path, minimum_time_coincidence, maximum_time_coincidence);
                      trajectory_2 = buildTrajectoryForCollisionDetection(it_2->second[i_2].path, minimum_time_coincidence, maximum_time_coincidence);

                      // // Debug trajectories:
                      // std::cout << "trajectory_1.size() = " << trajectory_1.size() << "     trajectory_2.size() = " << trajectory_2.size() << std::endl;
                      // std::cout << std::endl;
                      // for (int i=0; i<trajectory_1.size(); i++) {
                      //   std::cout << "trajectory_1 [" << i << "]:" << std::endl << trajectory_1[i];
                      // }
                      // for (int i=0; i<trajectory_2.size(); i++) {
                      //   std::cout << "trajectory_2 [" << i << "]:" << std::endl << trajectory_2[i];
                      // }
                      // for (int i=0; i<it_1->second[i_1].path.size(); i++) {
                      //   std::cout << "path_1 [" << i << "]:" << std::endl << it_1->second[i_1].path[i];
                      // }
                      // for (int i=0; i<it_2->second[i_2].path.size(); i++) {
                      //   std::cout << "path_2 [" << i << "]:" << std::endl << it_2->second[i_2].path[i];
                      // }

                      // Check if the trajectories are close enough, if they are save the information in "vector_of_conflicts":
                      if (trajectory_1.size() != trajectory_2.size()) {
                        ROS_ERROR("High-Level Planner: different size of time coincident trajectories in safetyPostprocess. Return plan as is.");
                        return;
                      } else if (trajectory_1[0].header.stamp.sec != trajectory_2[0].header.stamp.sec || trajectory_1[trajectory_1.size()-1].header.stamp.sec != trajectory_2[trajectory_2.size()-1].header.stamp.sec) {
                        ROS_ERROR("High-Level Planner: different times of trajectories in safetyPostprocess. Return plan as is.");
                        return;
                      } else {

                        ConflictivePairOfDroneActions pair_of_drone_actions;
                        pair_of_drone_actions.drone_id_1 = it_1->first;
                        pair_of_drone_actions.drone_id_2 = it_2->first;
                        pair_of_drone_actions.drone_action_1 = i_1;
                        pair_of_drone_actions.drone_action_2 = i_2;
                        std::pair<int,int> time_window_pair (-1,-1);
                        bool collision_recognized = false;
                        for (int k=0; k<trajectory_1.size(); k++) {
                          float xy_separation = sqrt( pow(trajectory_1[k].point.x-trajectory_2[k].point.x, 2) + pow(trajectory_1[k].point.y-trajectory_2[k].point.y, 2) );
                          float z_separation = abs( trajectory_1[k].point.z-trajectory_2[k].point.z );
                          if (xy_separation < minimum_separation_xy_ && z_separation < minimum_separation_z_) {
                            if (collision_recognized == false) {
                              collision_recognized = true;
                              time_window_pair.first = trajectory_1[k].header.stamp.sec;
                            }
                            if (k==trajectory_1.size()-1) {
                              collision_recognized = false;
                              time_window_pair.second = trajectory_1[k].header.stamp.sec;
                              if (time_window_pair.first != -1) {
                                pair_of_drone_actions.conflictive_time_windows.push_back(time_window_pair);
                              }
                            }
                          } else if (collision_recognized) {
                            collision_recognized = false;
                            if (k-1>=0) time_window_pair.second = trajectory_1[k-1].header.stamp.sec;
                            else        time_window_pair.second = trajectory_1[k].header.stamp.sec;
                            if (time_window_pair.first != -1) {
                              pair_of_drone_actions.conflictive_time_windows.push_back(time_window_pair);
                            }
                          }
                        }
                        if (pair_of_drone_actions.conflictive_time_windows.size()>0) {
                          vector_of_conflicts.push_back(pair_of_drone_actions);
                        }

                      }
                    }

                  }
                }
              }
            }
          }
        }
      }
    }
  }

  // // Debug vector_of_conflicts:
  // std::cout << std::endl;
  // for (int i=0; i<vector_of_conflicts.size(); i++) {
  //   std::cout << "vector_of_conflicts [" << i << "]:" << std::endl << "   drone_id_1 " << vector_of_conflicts[i].drone_id_1 << std::endl << "   drone_id_2 " << vector_of_conflicts[i].drone_id_2 << std::endl << "   drone_action_1 " << vector_of_conflicts[i].drone_action_1 << std::endl << "   drone_action_2 " << vector_of_conflicts[i].drone_action_2 << std::endl;
  //   for (int j=0; j<vector_of_conflicts[i].conflictive_time_windows.size(); j++) {
  //     std::cout << "   conflictive_time_windows[" << j << "] : " << vector_of_conflicts[i].conflictive_time_windows[j].first << " - " << vector_of_conflicts[i].conflictive_time_windows[j].second << std::endl;
  //   }
  // }
  // std::cout << std::endl;


  for (int i=0; i<vector_of_conflicts.size(); i++) {  // Check the conflicts, if exist:
    if ( (_plan[vector_of_conflicts[i].drone_id_1][vector_of_conflicts[i].drone_action_1].action_type==multidrone_msgs::DroneAction::TYPE_SHOOTING) && (_plan[vector_of_conflicts[i].drone_id_2][vector_of_conflicts[i].drone_action_2].action_type==multidrone_msgs::DroneAction::TYPE_SHOOTING) ) {
      // Search for conflict involving 2 shooting drone actions, if found delete all the plan directly and return:
      _plan.clear();
      return;
    }
  }

  bool check_again_because_of_conflict = false;
  for (int i=0; i<vector_of_conflicts.size(); i++) {  // Check the conflicts, if exist:

    if ( (_plan[vector_of_conflicts[i].drone_id_1][vector_of_conflicts[i].drone_action_1].action_type==multidrone_msgs::DroneAction::TYPE_GOTOWAYPOINT) && (_plan[vector_of_conflicts[i].drone_id_2][vector_of_conflicts[i].drone_action_2].action_type==multidrone_msgs::DroneAction::TYPE_GOTOWAYPOINT) ) {
      // If conflict involving 2 navigation actions, raise the height of the one higher, and if same height raise the one with less priority (greater drone id). Then rerun the safetyPostprocess at the end.
      if ( (_plan[vector_of_conflicts[i].drone_id_1][vector_of_conflicts[i].drone_action_1].path[0].point.z > _plan[vector_of_conflicts[i].drone_id_2][vector_of_conflicts[i].drone_action_2].path[0].point.z) || (_plan[vector_of_conflicts[i].drone_id_1][vector_of_conflicts[i].drone_action_1].path[0].point.z == _plan[vector_of_conflicts[i].drone_id_2][vector_of_conflicts[i].drone_action_2].path[0].point.z && vector_of_conflicts[i].drone_id_1 > vector_of_conflicts[i].drone_id_2) ) {
        for (int j=0; j<_plan[vector_of_conflicts[i].drone_id_1][vector_of_conflicts[i].drone_action_1].path.size(); j++) {
          _plan[vector_of_conflicts[i].drone_id_1][vector_of_conflicts[i].drone_action_1].path[j].point.z = _plan[vector_of_conflicts[i].drone_id_1][vector_of_conflicts[i].drone_action_1].path[j].point.z + minimum_separation_z_ - (_plan[vector_of_conflicts[i].drone_id_1][vector_of_conflicts[i].drone_action_1].path[j].point.z - _plan[vector_of_conflicts[i].drone_id_2][vector_of_conflicts[i].drone_action_2].path[j].point.z);
        }
      } else {
        for (int j=0; j<_plan[vector_of_conflicts[i].drone_id_2][vector_of_conflicts[i].drone_action_2].path.size(); j++) {
          _plan[vector_of_conflicts[i].drone_id_2][vector_of_conflicts[i].drone_action_2].path[j].point.z = _plan[vector_of_conflicts[i].drone_id_2][vector_of_conflicts[i].drone_action_2].path[j].point.z + minimum_separation_z_ - (_plan[vector_of_conflicts[i].drone_id_2][vector_of_conflicts[i].drone_action_2].path[j].point.z - _plan[vector_of_conflicts[i].drone_id_1][vector_of_conflicts[i].drone_action_1].path[j].point.z);
        }
      }
      check_again_because_of_conflict = true;

    } else {
      // If conflict involving a shooting drone action and a navigation action, raise the height of the navigation action and rerun the safetyPostprocess at the end:
      int drone_of_navigation_action, drone_of_shooting_drone_action, index_of_navigation_action, index_of_shooting_drone_action;
      if (_plan[vector_of_conflicts[i].drone_id_1][vector_of_conflicts[i].drone_action_1].action_type==multidrone_msgs::DroneAction::TYPE_GOTOWAYPOINT) {
        drone_of_navigation_action = vector_of_conflicts[i].drone_id_1;
        drone_of_shooting_drone_action = vector_of_conflicts[i].drone_id_2;
        index_of_navigation_action = vector_of_conflicts[i].drone_action_1;
        index_of_shooting_drone_action = vector_of_conflicts[i].drone_action_2;
      } else {
        drone_of_navigation_action = vector_of_conflicts[i].drone_id_2;
        drone_of_shooting_drone_action = vector_of_conflicts[i].drone_id_1;
        index_of_navigation_action = vector_of_conflicts[i].drone_action_2;
        index_of_shooting_drone_action = vector_of_conflicts[i].drone_action_1;
      }
      for (int j=0; j<_plan[drone_of_navigation_action][index_of_navigation_action].path.size(); j++) {
        _plan[drone_of_navigation_action][index_of_navigation_action].path[j].point.z = _plan[drone_of_navigation_action][index_of_navigation_action].path[j].point.z + minimum_separation_z_ - (_plan[drone_of_navigation_action][index_of_navigation_action].path[j].point.z - _plan[drone_of_shooting_drone_action][index_of_shooting_drone_action].path[j].point.z);
      }
      check_again_because_of_conflict = true;

    }
  }
  if (check_again_because_of_conflict) safetyPostprocess(_plan);

} // end safetyPostprocess method



// Method that show in the terminal the time graph for debug purposes.
void HighLevelPlanner::showTimeGraphAndDroneActionsForDebug(const TimeGraph& time_graph, std::map< int,std::vector<multidrone_msgs::DroneAction> > plan, int drone_id) {
#ifdef DEBUG_HIGH_LEVEL_PLANNER
  std::vector<multidrone_msgs::DroneAction> one_drone_plan = plan[drone_id];

  // ... and the base station node index of the last landing of that assignation.
  int maximum_recording_time_accumulated = 0;
  float minimum_navigation_distance_accumulated = std::numeric_limits<float>::max();
  int last_base_station_node_of_last_assignation;

  for (int i=1; i<time_graph.base_station_node.size(); i++) {
    if ( (time_graph.base_station_node[i].land_or_takeoff == false) && (time_graph.base_station_node[i].node_has_edges_out==false) ) { // base station node of landing with anything after it.
      if ( ( time_graph.base_station_node[i].recording_time_accumulated > maximum_recording_time_accumulated ) || ( ( time_graph.base_station_node[i].navigation_distance_accumulated < minimum_navigation_distance_accumulated ) && ( time_graph.base_station_node[i].recording_time_accumulated == maximum_recording_time_accumulated ) ) ) {
        // if base station node of last landing, with more recording time accumulated or equal recording time accumulated and less navigation distance accumulated, update best recording time and navigation distance accumulated.
        maximum_recording_time_accumulated = time_graph.base_station_node[i].recording_time_accumulated;
        minimum_navigation_distance_accumulated = time_graph.base_station_node[i].navigation_distance_accumulated;
        last_base_station_node_of_last_assignation = -i;
      }
    }
  }



  // Drone Actions: show in the terminal the drone actions assigned to one drone in this method call.
  std::cout << std::endl << std::endl;
  for (int i=0; i < one_drone_plan.size(); i++) {
    std::cout << "DRONE ID: " << drone_id << "  , DRONE ACTION: " << i << std::endl;
    std::cout << "action_id: " << one_drone_plan[i].action_id << std::endl;
    std::cout << "action_sequence_id: " << one_drone_plan[i].action_sequence_id << std::endl;
    std::cout << "mission_id: " << one_drone_plan[i].mission_id << std::endl;
    std::cout << "start_event: " << one_drone_plan[i].start_event << std::endl;
    std::cout << "delay_since_event: " << one_drone_plan[i].delay_since_event << std::endl;
    for (int j=0; j<one_drone_plan[i].path.size(); j++)
      std::cout << "path, waypoint[" << j << "]: " << std::endl << one_drone_plan[i].path[j];
    std::cout << "action_type: ";
    if ( one_drone_plan[i].action_type == multidrone_msgs::DroneAction::TYPE_TAKEOFF )           std::cout << "TYPE_TAKEOFF" << std::endl;
    else if ( one_drone_plan[i].action_type == multidrone_msgs::DroneAction::TYPE_LAND )         std::cout << "TYPE_LAND" << std::endl;
    else if ( one_drone_plan[i].action_type == multidrone_msgs::DroneAction::TYPE_GOTOWAYPOINT ) std::cout << "TYPE_GOTOWAYPOINT" << std::endl;
    else if ( one_drone_plan[i].action_type == multidrone_msgs::DroneAction::TYPE_SHOOTING ) {   std::cout << "TYPE_SHOOTING" << std::endl;
      std::cout << "SA: " << std::endl;
      std::cout << "   shooting_type: ";
      if ( one_drone_plan[i].shooting_action.shooting_roles[0].shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_STATIC )           std::cout << "SHOOT_TYPE_STATIC" << std::endl;
      else if ( one_drone_plan[i].shooting_action.shooting_roles[0].shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_FLY_THROUGH ) std::cout << "SHOOT_TYPE_FLY_THROUGH" << std::endl;
      else if ( one_drone_plan[i].shooting_action.shooting_roles[0].shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_ESTABLISH )   std::cout << "SHOOT_TYPE_ESTABLISH" << std::endl;
      else if ( one_drone_plan[i].shooting_action.shooting_roles[0].shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_CHASE )       std::cout << "SHOOT_TYPE_CHASE" << std::endl;
      else if ( one_drone_plan[i].shooting_action.shooting_roles[0].shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_LEAD )        std::cout << "SHOOT_TYPE_LEAD" << std::endl;
      else if ( one_drone_plan[i].shooting_action.shooting_roles[0].shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_FLYBY )       std::cout << "SHOOT_TYPE_FLYBY" << std::endl;
      else if ( one_drone_plan[i].shooting_action.shooting_roles[0].shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_LATERAL )     std::cout << "SHOOT_TYPE_LATERAL" << std::endl;
      else if ( one_drone_plan[i].shooting_action.shooting_roles[0].shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_ELEVATOR )    std::cout << "SHOOT_TYPE_ELEVATOR" << std::endl;
      else if ( one_drone_plan[i].shooting_action.shooting_roles[0].shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_ORBIT )       std::cout << "SHOOT_TYPE_ORBIT" << std::endl;
      std::cout << "   duration: " << (int) one_drone_plan[i].shooting_action.duration.data.sec << std::endl;
      std::cout << "   action_id: " << one_drone_plan[i].shooting_action.action_id << std::endl;
      std::cout << "   action_sequence_id: " << one_drone_plan[i].shooting_action.action_sequence_id << std::endl;
      std::cout << "   mission_id: " << one_drone_plan[i].shooting_action.mission_id << std::endl;
      std::cout << "   start_event: " << one_drone_plan[i].shooting_action.start_event << std::endl;
      std::cout << "   estimated_start_time: " << one_drone_plan[i].shooting_action.estimated_start_time << std::endl;
      std::cout << "   delay_since_event: " << one_drone_plan[i].shooting_action.delay_since_event << std::endl;
      std::cout << "   length: " << one_drone_plan[i].shooting_action.length << std::endl;
      if (one_drone_plan[i].shooting_action.target_type.type == multidrone_msgs::TargetType::TARGET_NONE)          std::cout << "   target_type: TARGET_NONE" << std::endl;
      else if (one_drone_plan[i].shooting_action.target_type.type == multidrone_msgs::TargetType::TARGET_BOAT)     std::cout << "   target_type: TARGET_BOAT" << std::endl;
      else if (one_drone_plan[i].shooting_action.target_type.type == multidrone_msgs::TargetType::TARGET_CYCLIST)  std::cout << "   target_type: TARGET_CYCLIST" << std::endl;
      else if (one_drone_plan[i].shooting_action.target_type.type == multidrone_msgs::TargetType::TARGET_RUNNER)   std::cout << "   target_type: TARGET_RUNNER" << std::endl;
      else if (one_drone_plan[i].shooting_action.target_type.type == multidrone_msgs::TargetType::TARGET_MONUMENT) std::cout << "   target_type: TARGET_MONUMENT" << std::endl;
      std::cout << "   formation_speed: " << one_drone_plan[i].shooting_action.formation_speed << std::endl;
      if (one_drone_plan[i].shooting_action.rt_mode == multidrone_msgs::ShootingAction::RT_MODE_VIRTUAL_TRAJ)       std::cout << "   rt_mode: RT_MODE_VIRTUAL_TRAJ" << std::endl;
      else if (one_drone_plan[i].shooting_action.rt_mode == multidrone_msgs::ShootingAction::RT_MODE_VIRTUAL_PATH)  std::cout << "   rt_mode: RT_MODE_VIRTUAL_PATH" << std::endl;
      else if (one_drone_plan[i].shooting_action.rt_mode == multidrone_msgs::ShootingAction::RT_MODE_ACTUAL_TARGET) std::cout << "   rt_mode: RT_MODE_ACTUAL_TARGET" << std::endl;
      std::cout << "   rt_id: " << (int) one_drone_plan[i].shooting_action.rt_id << std::endl;
      std::cout << "   ndrones: " << (int) one_drone_plan[i].shooting_action.ndrones << std::endl;
      std::cout << "   rt_displacement: " << std::endl << one_drone_plan[i].shooting_action.rt_displacement;
      for (int j=0; j<one_drone_plan[i].shooting_action.rt_trajectory.size(); j++) {
        std::cout << "   reference target trajectory, waypoint[" << j << "]: " << std::endl << one_drone_plan[i].shooting_action.rt_trajectory[j];
      }
      if (one_drone_plan[i].shooting_action.shooting_roles[0].framing_type.type == multidrone_msgs::FramingType::FRAMING_TYPE_ELS) std::cout      << "   framing_type: FRAMING_TYPE_ELS" << std::endl;
      else if (one_drone_plan[i].shooting_action.shooting_roles[0].framing_type.type == multidrone_msgs::FramingType::FRAMING_TYPE_VLS) std::cout << "   framing_type: FRAMING_TYPE_VLS" << std::endl;
      else if (one_drone_plan[i].shooting_action.shooting_roles[0].framing_type.type == multidrone_msgs::FramingType::FRAMING_TYPE_LS)  std::cout << "   framing_type: FRAMING_TYPE_LS" << std::endl;
      else if (one_drone_plan[i].shooting_action.shooting_roles[0].framing_type.type == multidrone_msgs::FramingType::FRAMING_TYPE_MS)  std::cout << "   framing_type: FRAMING_TYPE_MS" << std::endl;
      else if (one_drone_plan[i].shooting_action.shooting_roles[0].framing_type.type == multidrone_msgs::FramingType::FRAMING_TYPE_MCU) std::cout << "   framing_type: FRAMING_TYPE_MCU" << std::endl;
      else if (one_drone_plan[i].shooting_action.shooting_roles[0].framing_type.type == multidrone_msgs::FramingType::FRAMING_TYPE_CU)  std::cout << "   framing_type: FRAMING_TYPE_CU" << std::endl;
      else if (one_drone_plan[i].shooting_action.shooting_roles[0].framing_type.type == multidrone_msgs::FramingType::FRAMING_TYPE_OTS) std::cout << "   framing_type: FRAMING_TYPE_OTS" << std::endl;
      if (one_drone_plan[i].shooting_action.shooting_roles[0].target_identifier_type.type == multidrone_msgs::TargetIdentifierType::NONE) std::cout         << "   target_identifier_type: NONE" << std::endl;
      else if (one_drone_plan[i].shooting_action.shooting_roles[0].target_identifier_type.type == multidrone_msgs::TargetIdentifierType::VIRTUAL) std::cout << "   target_identifier_type: VIRTUAL" << std::endl;
      else if (one_drone_plan[i].shooting_action.shooting_roles[0].target_identifier_type.type == multidrone_msgs::TargetIdentifierType::GPS) std::cout     << "   target_identifier_type: GPS" << std::endl;
      else if (one_drone_plan[i].shooting_action.shooting_roles[0].target_identifier_type.type == multidrone_msgs::TargetIdentifierType::VISUAL) std::cout  << "   target_identifier_type: VISUAL" << std::endl;
      else if (one_drone_plan[i].shooting_action.shooting_roles[0].target_identifier_type.type == multidrone_msgs::TargetIdentifierType::STATIC) std::cout  << "   target_identifier_type: STATIC" << std::endl;
      else if (one_drone_plan[i].shooting_action.shooting_roles[0].target_identifier_type.type == multidrone_msgs::TargetIdentifierType::VISUAL_GPS) std::cout  << "   target_identifier_type: VISUAL_GPS" << std::endl;
      for (int j=0; j<one_drone_plan[i].shooting_action.shooting_roles[0].shooting_parameters.size(); j++) {
        std::cout << "   shooting_parameters[" << j << "]: " << one_drone_plan[i].shooting_action.shooting_roles[0].shooting_parameters[j].param << " = " << one_drone_plan[i].shooting_action.shooting_roles[0].shooting_parameters[j].value << std::endl;
      }
      for (int j=0; j<one_drone_plan[i].shooting_action.shooting_roles[0].target_parameters.size(); j++) {
        std::cout << "   target_parameters[" << j << "]: " << one_drone_plan[i].shooting_action.shooting_roles[0].target_parameters[j].param << " = " << one_drone_plan[i].shooting_action.shooting_roles[0].target_parameters[j].value << std::endl;
      }
    }
    std::cout << std::endl;
  }
  std::cout << std::endl << std::endl;



  // Time Graph: show all its members in the terminal and plot all the subgraphs.
  std::cout << "TIME GRAPH: " << plan.size() << std::endl << std::endl;
  for (int i=0; i < time_graph.task.size(); i++) {
    std::cout << "task[" << i << "]" << std::endl;
    std::cout << "event_id: " << time_graph.task[i].event_id << std::endl;
    std::cout << "SA_id: " << time_graph.task[i].SA_id << std::endl;
    std::cout << "SA_sequence_id: " << time_graph.task[i].SA_sequence_id << std::endl;
    std::cout << "mission_id: " << time_graph.task[i].mission_id << std::endl;
    std::cout << "shooting_type: ";
    if ( time_graph.task[i].shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_STATIC )            std::cout << "SHOOT_TYPE_STATIC" << std::endl;
    else if ( time_graph.task[i].shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_FLY_THROUGH )  std::cout << "SHOOT_TYPE_FLY_THROUGH" << std::endl;
    else if ( time_graph.task[i].shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_ESTABLISH )    std::cout << "SHOOT_TYPE_ESTABLISH" << std::endl;
    else if ( time_graph.task[i].shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_CHASE )        std::cout << "SHOOT_TYPE_CHASE" << std::endl;
    else if ( time_graph.task[i].shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_LEAD )         std::cout << "SHOOT_TYPE_LEAD" << std::endl;
    else if ( time_graph.task[i].shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_FLYBY )        std::cout << "SHOOT_TYPE_FLYBY" << std::endl;
    else if ( time_graph.task[i].shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_LATERAL )      std::cout << "SHOOT_TYPE_LATERAL" << std::endl;
    else if ( time_graph.task[i].shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_ELEVATOR )     std::cout << "SHOOT_TYPE_ELEVATOR" << std::endl;
    else if ( time_graph.task[i].shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_ORBIT )        std::cout << "SHOOT_TYPE_ORBIT" << std::endl;
    std::cout << "initial_position:" << std::endl << time_graph.task[i].initial_position;
    std::cout << "final_position:" << std::endl << time_graph.task[i].final_position;
    std::cout << "initial_azimuth: " << time_graph.task[i].initial_azimuth << std::endl;
    std::cout << "distance: " << time_graph.task[i].distance << std::endl;
    for (int j=0; j < time_graph.task[i].reference_target_waypoints.size(); j++) {
      std::cout << "reference_target_waypoints[" << j << "]: " << std::endl << time_graph.task[i].reference_target_waypoints[j];
    }
    std::cout << "time_initial: " << time_graph.task[i].time_initial << std::endl;
    std::cout << "time_final: " << time_graph.task[i].time_final << std::endl;
    std::cout << "time_difference: " << time_graph.task[i].time_difference << std::endl;
    std::cout << "delay_since_event: " << time_graph.task[i].delay_since_event << std::endl;
    std::cout << "average_speed: " << time_graph.task[i].average_speed << std::endl;
    std::cout << "moving_or_hovering: " << time_graph.task[i].moving_or_hovering << std::endl;
    std::cout << std::endl;
  }
  std::cout << std::endl << std::endl;
  for (int i=1; i < time_graph.base_station.size(); i++) {
    std::cout << "base_station[" << i << "]" << std::endl;
    std::cout << "position: " << std::endl << time_graph.base_station[i];
    std::cout << std::endl;
  }
  std::cout << std::endl << std::endl;
  for (int i=0; i < time_graph.initial_drone_state.size(); i++) {
    std::cout << "initial_drone_state[" << i << "]" << std::endl;
    std::cout << "position: " << std::endl << time_graph.initial_drone_state[i].position;
    std::cout << "battery: " << time_graph.initial_drone_state[i].battery << std::endl;
    std::cout << "drone_id: " << time_graph.initial_drone_state[i].drone_id << std::endl;
    std::cout << std::endl;
  }
  std::cout << std::endl << std::endl;
  for (int i=0; i < time_graph.shooting_node.size(); i++) {
    std::cout << "shooting_node[" << i << "]" << std::endl;
    std::cout << "index_of_task_drone_or_bs: " << time_graph.shooting_node[i].index_of_task_drone_or_bs << std::endl;
    std::cout << "position:" << std::endl << time_graph.shooting_node[i].position;
    std::cout << "battery: " << time_graph.shooting_node[i].battery << std::endl;
    std::cout << "time: " << time_graph.shooting_node[i].time << std::endl;
    std::cout << "recording_time_accumulated: " << time_graph.shooting_node[i].recording_time_accumulated << std::endl;
    std::cout << "navigation_distance_accumulated: " << time_graph.shooting_node[i].navigation_distance_accumulated << std::endl;
    std::cout << "edge_into_node: " << time_graph.shooting_node[i].edge_into_node << std::endl;
    std::cout << "node_has_edges_out: " << time_graph.shooting_node[i].node_has_edges_out << std::endl;
    std::cout << std::endl << std::endl;
  }
  std::cout << std::endl << std::endl;
  for (int i=1; i < time_graph.base_station_node.size(); i++) {
    std::cout << "base_station_node[" << i << "]" << std::endl;
    std::cout << "land_or_takeoff: " << time_graph.base_station_node[i].land_or_takeoff << std::endl;
    std::cout << "index_of_task_drone_or_bs: " << time_graph.base_station_node[i].index_of_task_drone_or_bs << std::endl;
    std::cout << "position:" << std::endl << time_graph.base_station_node[i].position;
    std::cout << "battery: " << time_graph.base_station_node[i].battery << std::endl;
    std::cout << "time: " << time_graph.base_station_node[i].time << std::endl;
    std::cout << "recording_time_accumulated: " << time_graph.base_station_node[i].recording_time_accumulated << std::endl;
    std::cout << "navigation_distance_accumulated: " << time_graph.base_station_node[i].navigation_distance_accumulated << std::endl;
    std::cout << "edge_into_node: " << time_graph.base_station_node[i].edge_into_node << std::endl;
    std::cout << "node_has_edges_out: " << time_graph.base_station_node[i].node_has_edges_out << std::endl;
    std::cout << std::endl << std::endl;
  }
  std::cout << std::endl << std::endl;
  for (int i=0; i < time_graph.edge.size(); i++) {
    std::cout << "edge[" << i << "] " << std::endl;
    std::cout << "shooting_or_navigation_action: " << time_graph.edge[i].shooting_or_navigation_action << std::endl;
    std::cout << "father_node: " << time_graph.edge[i].father_node << std::endl;
    std::cout << "son_node: " << time_graph.edge[i].son_node << std::endl;
    std::cout << std::endl;
  }
  std::cout << std::endl << std::endl;



  // Time Graph: plot the different subgraphs in this time graph.
  system("mkdir debug_plots");  // Create a folder where the plots will be saved ("system" calls a shell terminal and run the command in the string).
  int graph_plot_counter = 0;

  static double min_x = std::numeric_limits<double>::max();  // static so that all the graphs has the same xlim and ylim.
  static double min_y = std::numeric_limits<double>::max();  // static so that all the graphs has the same xlim and ylim.
  static double max_x = -std::numeric_limits<double>::max(); // static so that all the graphs has the same xlim and ylim.
  static double max_y = -std::numeric_limits<double>::max(); // static so that all the graphs has the same xlim and ylim.

  for (int i=1; i<time_graph.base_station_node.size(); i++) {
    if ( (time_graph.base_station_node[i].land_or_takeoff == false) && (time_graph.base_station_node[i].node_has_edges_out==false) ) { // base station node of landing with anything after it.

      graph_plot_counter++;

      bool subgraph_is_the_solution = last_base_station_node_of_last_assignation==-i ? true : false;

      // Plots of the tasks (shooting actions):
      bool only_once = true;
      for (int j=0; j < time_graph.task.size(); j++) {
        std::vector<double> x_plot, y_plot;
        std::vector<double> x_plot_displaced, y_plot_displaced;

        float distance_since_task_start = 0;
        float time_since_task_start = 0;
        float azimuth;
        geometry_msgs::Point32 auxiliary_displaced_point;

        for (int k=0; k<time_graph.task[j].reference_target_waypoints.size(); k++) {

          x_plot.push_back( time_graph.task[j].reference_target_waypoints[k].x );
          y_plot.push_back( time_graph.task[j].reference_target_waypoints[k].y );

          min_x = min_x > time_graph.task[j].reference_target_waypoints[k].x ? time_graph.task[j].reference_target_waypoints[k].x : min_x;
          min_y = min_y > time_graph.task[j].reference_target_waypoints[k].y ? time_graph.task[j].reference_target_waypoints[k].y : min_y;
          max_x = max_x < time_graph.task[j].reference_target_waypoints[k].x ? time_graph.task[j].reference_target_waypoints[k].x : max_x;
          max_y = max_y < time_graph.task[j].reference_target_waypoints[k].y ? time_graph.task[j].reference_target_waypoints[k].y : max_y;

          if ( k==0 || !(time_graph.task[j].shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_STATIC) ) {

            if ( time_graph.task[j].shooting_type.type != multidrone_msgs::ShootingType::SHOOT_TYPE_ORBIT ) {
              if (k>0) {
                auxiliary_displaced_point = time_graph.task[j].reference_target_waypoints[k];

                distance_since_task_start += sqrt( pow(time_graph.task[j].reference_target_waypoints[k].x-time_graph.task[j].reference_target_waypoints[k-1].x, 2) + pow(time_graph.task[j].reference_target_waypoints[k].y-time_graph.task[j].reference_target_waypoints[k-1].y, 2) + pow(time_graph.task[j].reference_target_waypoints[k].z-time_graph.task[j].reference_target_waypoints[k-1].z, 2) );
                time_since_task_start = (float)( time_graph.task[j].time_difference ) * distance_since_task_start/time_graph.task[j].distance;
                azimuth = atan2( time_graph.task[j].reference_target_waypoints[k].y - time_graph.task[j].reference_target_waypoints[k-1].y , time_graph.task[j].reference_target_waypoints[k].x - time_graph.task[j].reference_target_waypoints[k-1].x );

                displacePointByTaskParameters(auxiliary_displaced_point, time_graph.task[j], azimuth, time_since_task_start);

                x_plot_displaced.push_back( auxiliary_displaced_point.x );
                y_plot_displaced.push_back( auxiliary_displaced_point.y );

                min_x = min_x > auxiliary_displaced_point.x ? auxiliary_displaced_point.x : min_x;
                min_y = min_y > auxiliary_displaced_point.y ? auxiliary_displaced_point.y : min_y;
                max_x = max_x < auxiliary_displaced_point.x ? auxiliary_displaced_point.x : max_x;
                max_y = max_y < auxiliary_displaced_point.y ? auxiliary_displaced_point.y : max_y;
              }
              if (k<time_graph.task[j].reference_target_waypoints.size()-1) {
                auxiliary_displaced_point = time_graph.task[j].reference_target_waypoints[k];

                azimuth = atan2( time_graph.task[j].reference_target_waypoints[k+1].y - time_graph.task[j].reference_target_waypoints[k].y , time_graph.task[j].reference_target_waypoints[k+1].x - time_graph.task[j].reference_target_waypoints[k].x );

                displacePointByTaskParameters(auxiliary_displaced_point, time_graph.task[j], azimuth, time_since_task_start);

                x_plot_displaced.push_back( auxiliary_displaced_point.x );
                y_plot_displaced.push_back( auxiliary_displaced_point.y );

                min_x = min_x > auxiliary_displaced_point.x ? auxiliary_displaced_point.x : min_x;
                min_y = min_y > auxiliary_displaced_point.y ? auxiliary_displaced_point.y : min_y;
                max_x = max_x < auxiliary_displaced_point.x ? auxiliary_displaced_point.x : max_x;
                max_y = max_y < auxiliary_displaced_point.y ? auxiliary_displaced_point.y : max_y;
              }
              if (time_graph.task[j].reference_target_waypoints.size()==1) {
                auxiliary_displaced_point = time_graph.task[j].reference_target_waypoints[0];

                displacePointByTaskParameters(auxiliary_displaced_point, time_graph.task[j], time_graph.task[j].initial_azimuth, 0);

                x_plot_displaced.push_back( auxiliary_displaced_point.x );
                y_plot_displaced.push_back( auxiliary_displaced_point.y );

                min_x = min_x > auxiliary_displaced_point.x ? auxiliary_displaced_point.x : min_x;
                min_y = min_y > auxiliary_displaced_point.y ? auxiliary_displaced_point.y : min_y;
                max_x = max_x < auxiliary_displaced_point.x ? auxiliary_displaced_point.x : max_x;
                max_y = max_y < auxiliary_displaced_point.y ? auxiliary_displaced_point.y : max_y;
              }
            } else {  // ORBIT
              if (time_graph.task[j].reference_target_waypoints.size()==1) {
                bool insert_last_point_manually = time_graph.task[j].time_difference % orbit_path_separation_of_discretization_ > 0 ? true : false;
                for (int m=0; m<time_graph.task[j].time_difference/orbit_path_separation_of_discretization_; m++) {
                  auxiliary_displaced_point = time_graph.task[j].reference_target_waypoints[0];

                  time_since_task_start = m*orbit_path_separation_of_discretization_;

                  displacePointByTaskParameters(auxiliary_displaced_point, time_graph.task[j], time_graph.task[j].initial_azimuth, time_since_task_start);

                  x_plot_displaced.push_back( auxiliary_displaced_point.x );
                  y_plot_displaced.push_back( auxiliary_displaced_point.y );

                  min_x = min_x > auxiliary_displaced_point.x ? auxiliary_displaced_point.x : min_x;
                  min_y = min_y > auxiliary_displaced_point.y ? auxiliary_displaced_point.y : min_y;
                  max_x = max_x < auxiliary_displaced_point.x ? auxiliary_displaced_point.x : max_x;
                  max_y = max_y < auxiliary_displaced_point.y ? auxiliary_displaced_point.y : max_y;
                }
                if (insert_last_point_manually) {
                  auxiliary_displaced_point = time_graph.task[j].reference_target_waypoints[0];

                  time_since_task_start = (float)( time_graph.task[j].time_difference );

                  displacePointByTaskParameters(auxiliary_displaced_point, time_graph.task[j], time_graph.task[j].initial_azimuth, time_since_task_start);

                  x_plot_displaced.push_back( auxiliary_displaced_point.x );
                  y_plot_displaced.push_back( auxiliary_displaced_point.y );

                  min_x = min_x > auxiliary_displaced_point.x ? auxiliary_displaced_point.x : min_x;
                  min_y = min_y > auxiliary_displaced_point.y ? auxiliary_displaced_point.y : min_y;
                  max_x = max_x < auxiliary_displaced_point.x ? auxiliary_displaced_point.x : max_x;
                  max_y = max_y < auxiliary_displaced_point.y ? auxiliary_displaced_point.y : max_y;
                }
              } else if (k<time_graph.task[j].reference_target_waypoints.size()-1) {
                float length_current_pair_of_waypoints = sqrt( pow(time_graph.task[j].reference_target_waypoints[k].x-time_graph.task[j].reference_target_waypoints[k+1].x, 2) + pow(time_graph.task[j].reference_target_waypoints[k].y-time_graph.task[j].reference_target_waypoints[k+1].y, 2) + pow(time_graph.task[j].reference_target_waypoints[k].z-time_graph.task[j].reference_target_waypoints[k+1].z, 2) );
                float number_of_iterations_float = (float)time_graph.task[j].time_difference/(float)orbit_path_separation_of_discretization_ * length_current_pair_of_waypoints/time_graph.task[j].distance;
                int number_of_iterations = (int)number_of_iterations_float;
                for (int m=0; m<number_of_iterations; m++) {
                  auxiliary_displaced_point.x = time_graph.task[j].reference_target_waypoints[k].x + ( time_graph.task[j].reference_target_waypoints[k+1].x - time_graph.task[j].reference_target_waypoints[k].x ) * m/number_of_iterations;
                  auxiliary_displaced_point.y = time_graph.task[j].reference_target_waypoints[k].y + ( time_graph.task[j].reference_target_waypoints[k+1].y - time_graph.task[j].reference_target_waypoints[k].y ) * m/number_of_iterations;
                  auxiliary_displaced_point.z = time_graph.task[j].reference_target_waypoints[k].z + ( time_graph.task[j].reference_target_waypoints[k+1].z - time_graph.task[j].reference_target_waypoints[k].z ) * m/number_of_iterations;

                  float azimuth = atan2( time_graph.task[j].reference_target_waypoints[k+1].y - time_graph.task[j].reference_target_waypoints[k].y , time_graph.task[j].reference_target_waypoints[k+1].x - time_graph.task[j].reference_target_waypoints[k].x );

                  displacePointByTaskParameters(auxiliary_displaced_point, time_graph.task[j], azimuth, time_since_task_start);

                  x_plot_displaced.push_back( auxiliary_displaced_point.x );
                  y_plot_displaced.push_back( auxiliary_displaced_point.y );

                  min_x = min_x > auxiliary_displaced_point.x ? auxiliary_displaced_point.x : min_x;
                  min_y = min_y > auxiliary_displaced_point.y ? auxiliary_displaced_point.y : min_y;
                  max_x = max_x < auxiliary_displaced_point.x ? auxiliary_displaced_point.x : max_x;
                  max_y = max_y < auxiliary_displaced_point.y ? auxiliary_displaced_point.y : max_y;

                  time_since_task_start += orbit_path_separation_of_discretization_;
                }
                distance_since_task_start += sqrt( pow(time_graph.task[j].reference_target_waypoints[k].x-time_graph.task[j].reference_target_waypoints[k+1].x, 2) + pow(time_graph.task[j].reference_target_waypoints[k].y-time_graph.task[j].reference_target_waypoints[k+1].y, 2) + pow(time_graph.task[j].reference_target_waypoints[k].z-time_graph.task[j].reference_target_waypoints[k+1].z, 2) );
                time_since_task_start = (float)time_graph.task[j].time_difference * distance_since_task_start/time_graph.task[j].distance;
              } else {
                auxiliary_displaced_point = time_graph.task[j].reference_target_waypoints[k];

                float azimuth = atan2( time_graph.task[j].reference_target_waypoints[time_graph.task[j].reference_target_waypoints.size()-1].y - time_graph.task[j].reference_target_waypoints[time_graph.task[j].reference_target_waypoints.size()-2].y , time_graph.task[j].reference_target_waypoints[time_graph.task[j].reference_target_waypoints.size()-1].x - time_graph.task[j].reference_target_waypoints[time_graph.task[j].reference_target_waypoints.size()-2].x );

                displacePointByTaskParameters(auxiliary_displaced_point, time_graph.task[j], azimuth, (float)time_graph.task[j].time_difference );

                x_plot_displaced.push_back( auxiliary_displaced_point.x );
                y_plot_displaced.push_back( auxiliary_displaced_point.y );

                min_x = min_x > auxiliary_displaced_point.x ? auxiliary_displaced_point.x : min_x;
                min_y = min_y > auxiliary_displaced_point.y ? auxiliary_displaced_point.y : min_y;
                max_x = max_x < auxiliary_displaced_point.x ? auxiliary_displaced_point.x : max_x;
                max_y = max_y < auxiliary_displaced_point.y ? auxiliary_displaced_point.y : max_y;
              }
            }
          }
          if (only_once) {
            only_once = false;
            plt::named_plot("Task: reference target trajectory", x_plot, y_plot, "xg-");                    // plot with legend
            plt::named_plot("Task: estimated drone trajectory", x_plot_displaced, y_plot_displaced, "oy-"); // plot with legend
          } else {
            plt::plot(x_plot, y_plot, "xg-");                                   // Regular plot
            plt::plot(x_plot_displaced, y_plot_displaced, "oy-");               // Regular plot
          }
        }
      }

      // Plots of the base stations:
      for (int j=1; j < time_graph.base_station.size(); j++) {
        std::vector<double> x_plot, y_plot;
        x_plot.push_back( time_graph.base_station[j].x );
        y_plot.push_back( time_graph.base_station[j].y );

        min_x = min_x > time_graph.base_station[j].x ? time_graph.base_station[j].x : min_x;
        min_y = min_y > time_graph.base_station[j].y ? time_graph.base_station[j].y : min_y;
        max_x = max_x < time_graph.base_station[j].x ? time_graph.base_station[j].x : max_x;
        max_y = max_y < time_graph.base_station[j].y ? time_graph.base_station[j].y : max_y;

        if (j==1) plt::named_plot("Base stations", x_plot, y_plot, "xr");             // plot with legend
        else      plt::plot(x_plot, y_plot, "xr");                                    // Regular plot
      }

      // Plot of the current subgraph:
      int son_node_index = -i;      // Negative always because base station node.
      int edge_index = time_graph.base_station_node[-son_node_index].edge_into_node;
      int father_node_index = time_graph.edge[edge_index].father_node;   // Can be positive (father is shooting node) or negative (father is another base station node).

      std::vector<double> x_plot_1, y_plot_1;

      x_plot_1.push_back( time_graph.base_station_node[-son_node_index].position.x );                                   y_plot_1.push_back( time_graph.base_station_node[-son_node_index].position.y );
      if (father_node_index >= 0) { x_plot_1.push_back( time_graph.shooting_node[father_node_index].position.x );       y_plot_1.push_back( time_graph.shooting_node[father_node_index].position.y ); }
      else                        { x_plot_1.push_back( time_graph.base_station_node[-father_node_index].position.x );  y_plot_1.push_back( time_graph.base_station_node[-father_node_index].position.y ); }

      plt::named_plot("Subgraph "+ std::to_string(graph_plot_counter), x_plot_1, y_plot_1, "b-");

      // Plot text (numbers converted to strings) of the son node, father node and the edge:
      plt::text( x_plot_1[0], y_plot_1[0], son_node_index>=0? std::to_string(son_node_index) : "     " + std::to_string(son_node_index) );
      plt::text( x_plot_1[1], y_plot_1[1], std::to_string(father_node_index) );
      plt::text( (x_plot_1[0]+x_plot_1[1])/2, (y_plot_1[0]+y_plot_1[1])/2, std::to_string(edge_index) );

      son_node_index = father_node_index;
      if (son_node_index >= 0) edge_index = time_graph.shooting_node[son_node_index].edge_into_node;
      else                     edge_index = time_graph.base_station_node[-son_node_index].edge_into_node;
      father_node_index = time_graph.edge[edge_index].father_node;

      bool keep_in_loop;
      if (father_node_index >= 0) keep_in_loop = true;
      else if ( !( (time_graph.base_station_node[-father_node_index].land_or_takeoff == true) && (time_graph.base_station_node[-father_node_index].edge_into_node == -1) ) ) keep_in_loop = true;
      else keep_in_loop = false;

      while (keep_in_loop)  {
        std::vector<double> x_plot_loop, y_plot_loop;

        if (son_node_index >= 0)    { x_plot_loop.push_back( time_graph.shooting_node[son_node_index].position.x );          y_plot_loop.push_back( time_graph.shooting_node[son_node_index].position.y ); }
        else                        { x_plot_loop.push_back( time_graph.base_station_node[-son_node_index].position.x );     y_plot_loop.push_back( time_graph.base_station_node[-son_node_index].position.y ); }
        if (father_node_index >= 0) { x_plot_loop.push_back( time_graph.shooting_node[father_node_index].position.x );       y_plot_loop.push_back( time_graph.shooting_node[father_node_index].position.y ); }
        else                        { x_plot_loop.push_back( time_graph.base_station_node[-father_node_index].position.x );  y_plot_loop.push_back( time_graph.base_station_node[-father_node_index].position.y ); }

        plt::plot(x_plot_loop, y_plot_loop, "b-");

        // Plot text (numbers converted to strings) of the son node, father node and the edge:
        plt::text( x_plot_loop[0], y_plot_loop[0], son_node_index>=0? std::to_string(son_node_index) : "     " + std::to_string(son_node_index) );
        plt::text( x_plot_loop[1], y_plot_loop[1], std::to_string(father_node_index) );
        plt::text( (x_plot_loop[0]+x_plot_loop[1])/2, (y_plot_loop[0]+y_plot_loop[1])/2, std::to_string(edge_index) );

        son_node_index = father_node_index;
        if (son_node_index >= 0) edge_index = time_graph.shooting_node[son_node_index].edge_into_node;
        else                    edge_index = time_graph.base_station_node[-son_node_index].edge_into_node;
        father_node_index = time_graph.edge[edge_index].father_node;

        if (father_node_index >= 0) keep_in_loop = true;
        else if ( !( (time_graph.base_station_node[-father_node_index].land_or_takeoff == true) && (time_graph.base_station_node[-father_node_index].edge_into_node == -1) ) ) keep_in_loop = true;
        else keep_in_loop = false;
      }

      std::vector<double> x_plot_2, y_plot_2;

      if (son_node_index >= 0)    { x_plot_2.push_back( time_graph.shooting_node[son_node_index].position.x );          y_plot_2.push_back( time_graph.shooting_node[son_node_index].position.y ); }
      else                        { x_plot_2.push_back( time_graph.base_station_node[-son_node_index].position.x );     y_plot_2.push_back( time_graph.base_station_node[-son_node_index].position.y ); }
      if (father_node_index >= 0) { x_plot_2.push_back( time_graph.shooting_node[father_node_index].position.x );       y_plot_2.push_back( time_graph.shooting_node[father_node_index].position.y ); }
      else                        { x_plot_2.push_back( time_graph.base_station_node[-father_node_index].position.x );  y_plot_2.push_back( time_graph.base_station_node[-father_node_index].position.y ); }

      plt::named_plot("Navigation after first takeoff", x_plot_2, y_plot_2, "b--");

      // Plot text (numbers converted to strings) of the son node, father node and the edge:
      plt::text( x_plot_2[0], y_plot_2[0], son_node_index>=0? std::to_string(son_node_index) : "     " + std::to_string(son_node_index) );
      plt::text( x_plot_2[1], y_plot_2[1], std::to_string(father_node_index) );
      plt::text( (x_plot_2[0]+x_plot_2[1])/2, (y_plot_2[0]+y_plot_2[1])/2, std::to_string(edge_index) );

      plt::xlim(min_x-10, max_x+10);  // Set x-axis to interval. OVERRIDEN IF USING axis("equal")
      plt::ylim(min_y-10, max_y+10);  // Set y-axis to interval. OVERRIDEN IF USING axis("equal")
      if ( subgraph_is_the_solution ) {
        plt::title("Subgraph " + std::to_string(graph_plot_counter) + ". drone_id: " + std::to_string(drone_id) + ". #assig.: " + std::to_string(plan.size()) + ". t_rec: " + std::to_string(maximum_recording_time_accumulated) + " s. Nav. dist.: " + std::to_string(minimum_navigation_distance_accumulated) + " m.");

        // Plot the reference target waypoints of the shooting actions in the drone actions.
        bool only_once_flag = false;
        for (int j=0; j<plan[drone_id].size(); j++) {
          if (plan[drone_id][j].action_type == multidrone_msgs::DroneAction::TYPE_SHOOTING) {
            std::vector<double> x_plot_SA, y_plot_SA;
            for (int k=0; k<plan[drone_id][j].shooting_action.rt_trajectory.size(); k++) {
              x_plot_SA.push_back( plan[drone_id][j].shooting_action.rt_trajectory[k].point.x );
              y_plot_SA.push_back( plan[drone_id][j].shooting_action.rt_trajectory[k].point.y );
            }
            if (!only_once_flag) {
              plt::named_plot("Shooting Drone Action: rel. tar. tra.", x_plot_SA, y_plot_SA, "or-");     // plot with legend
            } else {
              plt::plot(x_plot_SA, y_plot_SA, "or-");                                   // Regular plot
            }
            only_once_flag = true;
          }
        }

      } else {
        plt::title("Subgraph " + std::to_string(graph_plot_counter) + ". t_rec: " + std::to_string(time_graph.base_station_node[i].recording_time_accumulated) + " s. Nav. dist.: " + std::to_string(time_graph.base_station_node[i].navigation_distance_accumulated) + " m.");
      }
      plt::legend();      // Enable legend.
      plt::axis("equal"); // Axis with the same scale. Commented because it's incompatible with xlim and ylim.
      plt::save("./debug_plots/subgraph_"+ std::to_string(graph_plot_counter) + ".png"); // Save the plot in the folder created
      plt::clf();         // Clear figure, so the next ones doesn't start from this one.
    }
  }

  system("xdg-open ./debug_plots/subgraph_1.png");    // Open the first image, the rest of them can be visualized with the left and right buttons. IMPOTANT: eog (Eye Of Gnome or Ubuntu Image Viewer) must be the default application to open images so that the plots can be viewed.
  system("while [ -n \"$(pidof eog)\" ]; do sleep 2; done; rm -r ./debug_plots"); // Wait (code paralyzed) until the eog process (Eye Of Gnome or Ubuntu Image Viewer) is finished (windows closed), and then delete the folder with the plots inside it (selfdestruct the plots!).
#endif
} // end showTimeGraphAndDroneActionsForDebug method



// Method used in the safetyPostprocess for building the trajectories for collision detection:
std::vector<geometry_msgs::PointStamped> HighLevelPlanner::buildTrajectoryForCollisionDetection(const std::vector<geometry_msgs::PointStamped>& drone_action_path, int minimum_time_coincidence, int maximum_time_coincidence) {

  std::vector<geometry_msgs::PointStamped> trajectory;

  const int postprocess_time_discretization = 1;   // Time discretization when checking collisions (distance calculation) between pairs of waypoints with the same timestamp in different segments (of path) in drone actions of different drones. Must be one because of the int seconds resolution of the considered time.
  bool inside_time_window = false;

  for (int j=0; j<drone_action_path.size()-1; j++) {

    int time_between_pair_of_wp = drone_action_path[j+1].header.stamp.sec - drone_action_path[j].header.stamp.sec;
    geometry_msgs::PointStamped point_to_insert;

    if ((drone_action_path[j].header.stamp.sec<=minimum_time_coincidence)&&(minimum_time_coincidence<=drone_action_path[j+1].header.stamp.sec) && (drone_action_path[j].header.stamp.sec<=maximum_time_coincidence)&&(maximum_time_coincidence<=drone_action_path[j+1].header.stamp.sec)) {
      // Start and end of coincident time window in this segment.
      inside_time_window = true;
      for (int k=0; k <= maximum_time_coincidence - minimum_time_coincidence; k+=postprocess_time_discretization) {
        // Insert first point (when k=0), the rest of discretized points of the segment, and the last one (when k=maximum_time_coincidence-minimum_time_coincidence):
        point_to_insert.header.stamp.sec = minimum_time_coincidence + k;
        point_to_insert.point.x = time_between_pair_of_wp>0 ? drone_action_path[j].point.x + ( drone_action_path[j+1].point.x - drone_action_path[j].point.x ) * (float)(point_to_insert.header.stamp.sec - drone_action_path[j].header.stamp.sec)/(float)time_between_pair_of_wp : drone_action_path[j].point.x;
        point_to_insert.point.y = time_between_pair_of_wp>0 ? drone_action_path[j].point.y + ( drone_action_path[j+1].point.y - drone_action_path[j].point.y ) * (float)(point_to_insert.header.stamp.sec - drone_action_path[j].header.stamp.sec)/(float)time_between_pair_of_wp : drone_action_path[j].point.y;
        point_to_insert.point.z = time_between_pair_of_wp>0 ? drone_action_path[j].point.z + ( drone_action_path[j+1].point.z - drone_action_path[j].point.z ) * (float)(point_to_insert.header.stamp.sec - drone_action_path[j].header.stamp.sec)/(float)time_between_pair_of_wp : drone_action_path[j].point.z;
        trajectory.push_back(point_to_insert);
      }
      inside_time_window = false;
      break;

    } else if ((drone_action_path[j].header.stamp.sec<=minimum_time_coincidence)&&(minimum_time_coincidence<=drone_action_path[j+1].header.stamp.sec)) {
      // Start of the coincident time window in this segment.
      inside_time_window = true;
      for (int k=0; k <= drone_action_path[j+1].header.stamp.sec - minimum_time_coincidence; k+=postprocess_time_discretization) {
        // Insert first point (when k=0) and the rest of discretized points of the segment:
        point_to_insert.header.stamp.sec = minimum_time_coincidence + k;
        point_to_insert.point.x = time_between_pair_of_wp>0 ? drone_action_path[j].point.x + ( drone_action_path[j+1].point.x - drone_action_path[j].point.x ) * (float)(point_to_insert.header.stamp.sec - drone_action_path[j].header.stamp.sec)/(float)time_between_pair_of_wp : drone_action_path[j].point.x;
        point_to_insert.point.y = time_between_pair_of_wp>0 ? drone_action_path[j].point.y + ( drone_action_path[j+1].point.y - drone_action_path[j].point.y ) * (float)(point_to_insert.header.stamp.sec - drone_action_path[j].header.stamp.sec)/(float)time_between_pair_of_wp : drone_action_path[j].point.y;
        point_to_insert.point.z = time_between_pair_of_wp>0 ? drone_action_path[j].point.z + ( drone_action_path[j+1].point.z - drone_action_path[j].point.z ) * (float)(point_to_insert.header.stamp.sec - drone_action_path[j].header.stamp.sec)/(float)time_between_pair_of_wp : drone_action_path[j].point.z;
        trajectory.push_back(point_to_insert);
      }

    } else if ((drone_action_path[j].header.stamp.sec<=maximum_time_coincidence)&&(maximum_time_coincidence<=drone_action_path[j+1].header.stamp.sec)) {
      // End of the coincident time window in this segment.
      for (int k=postprocess_time_discretization; k <= drone_action_path[j+1].header.stamp.sec - drone_action_path[j].header.stamp.sec; k+=postprocess_time_discretization) {
        // Insert discretized points of the segment (not inserting the first one), and the last one (when k=drone_action_path[j+1].header.stamp.sec-drone_action_path[j].header.stamp.sec):
        point_to_insert.header.stamp.sec = drone_action_path[j].header.stamp.sec + k;
        point_to_insert.point.x = time_between_pair_of_wp>0 ? drone_action_path[j].point.x + ( drone_action_path[j+1].point.x - drone_action_path[j].point.x ) * (float)(point_to_insert.header.stamp.sec - drone_action_path[j].header.stamp.sec)/(float)time_between_pair_of_wp : drone_action_path[j].point.x;
        point_to_insert.point.y = time_between_pair_of_wp>0 ? drone_action_path[j].point.y + ( drone_action_path[j+1].point.y - drone_action_path[j].point.y ) * (float)(point_to_insert.header.stamp.sec - drone_action_path[j].header.stamp.sec)/(float)time_between_pair_of_wp : drone_action_path[j].point.y;
        point_to_insert.point.z = time_between_pair_of_wp>0 ? drone_action_path[j].point.z + ( drone_action_path[j+1].point.z - drone_action_path[j].point.z ) * (float)(point_to_insert.header.stamp.sec - drone_action_path[j].header.stamp.sec)/(float)time_between_pair_of_wp : drone_action_path[j].point.z;
        trajectory.push_back(point_to_insert);
      }
      inside_time_window = false;
      break;

    } else if (inside_time_window) {
      // Inside time window:
      for (int k=postprocess_time_discretization; k <= drone_action_path[j+1].header.stamp.sec - drone_action_path[j].header.stamp.sec; k+=postprocess_time_discretization) {
        // Insert discretized points of the segment (not inserting the first one), and the last one (when k = maximum_time_coincidence - drone_action_path[j].header.stamp.sec):
        point_to_insert.header.stamp.sec = drone_action_path[j].header.stamp.sec + k;
        point_to_insert.point.x = time_between_pair_of_wp>0 ? drone_action_path[j].point.x + ( drone_action_path[j+1].point.x - drone_action_path[j].point.x ) * (float)(point_to_insert.header.stamp.sec - drone_action_path[j].header.stamp.sec)/(float)time_between_pair_of_wp : drone_action_path[j].point.x;
        point_to_insert.point.y = time_between_pair_of_wp>0 ? drone_action_path[j].point.y + ( drone_action_path[j+1].point.y - drone_action_path[j].point.y ) * (float)(point_to_insert.header.stamp.sec - drone_action_path[j].header.stamp.sec)/(float)time_between_pair_of_wp : drone_action_path[j].point.y;
        point_to_insert.point.z = time_between_pair_of_wp>0 ? drone_action_path[j].point.z + ( drone_action_path[j+1].point.z - drone_action_path[j].point.z ) * (float)(point_to_insert.header.stamp.sec - drone_action_path[j].header.stamp.sec)/(float)time_between_pair_of_wp : drone_action_path[j].point.z;
        trajectory.push_back(point_to_insert);
      }
    }

  }

  return trajectory;
} // end buildTrajectoryForCollisionDetection method


} // end namespace multidrone

// TODO: solve in planning time the land in base station problem. Maybe little displacement for each drone.
// TODO: take into account flying time when XML duration = 0 (in XML parser). Maybe impossible.
// TODO: remove point from base station nodes.
// HEAVY TODO: more efficient data structure creation process.