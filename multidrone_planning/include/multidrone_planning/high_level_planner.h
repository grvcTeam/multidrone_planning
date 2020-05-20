/**
 * MULTIDRONE Project:
 *
 * High level planner.
 * 
 */

#ifndef HIGH_LEVEL_PLANNER_H
#define HIGH_LEVEL_PLANNER_H

#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <geographic_msgs/GeoPoint.h>

#include <multidrone_msgs/DroneAction.h>
#include <multidrone_msgs/ShootingAction.h>
#include <multidrone_msgs/ShootingType.h>
#include <multidrone_msgs/ShootingActionError.h>
#include <multidrone_planning/path_planner.h>

namespace multidrone {

/// HighLevelPlanner class that works as interface
class HighLevelPlanner {

public:
    HighLevelPlanner();
    ~HighLevelPlanner();

    std::map< int,std::vector<multidrone_msgs::DroneAction> > getPlan(const std::vector<multidrone_msgs::ShootingAction>& _shooting_action_list, const std::map< int,std::tuple<geometry_msgs::Point32,double,bool> >& _initial_state_of_drones, const geographic_msgs::GeoPoint& _origin_coordinates_geo, const std::string& _KML_string = "", int _trigger_time_of_replanning = -1, unsigned int _max_grid_side = 100);

private:

    std::string KML_string_ = "";       // For the path_planner_ initializer list in the HighLevelPlanner constructor. Can be changed when calling the getPlan method.
    unsigned int max_grid_side_ = 100;  // For the path_planner_ initializer list in the HighLevelPlanner constructor. Can be changed when calling the getPlan method.

    PathPlanner path_planner_;

    int trigger_time_of_replanning_;    // trigger_time_of_replanning by default will be -1 if not specified which means no replanning. If != -1 then is a replanning.

    float height_base_stations_;

    std::map<int, geometry_msgs::Point32> initial_ground_drone_positions_;   // Map with drone ids as keys and position of the drones initially at the ground as values. Won't be updated when replanning.



    ////////////// <Time Graph components> //////////////

    // Struct that will be initialized for each shooting node.
    // Shooting nodes of the time graph represent the initial, intermediate or final points of a certain shooting action, so there are two shooting nodes per shooting action.
    struct ShootingNode {
        geometry_msgs::Point32 position;    // Position of the actual node.
        float battery;                      // Battery at the node.

        float navigation_distance_accumulated;  // Total distance (of navigation actions only) accumulated since the initial takeoff to this node. The solution for each drone has to maximize the time recording shooting actions, and if several identical solutions the one with less distance accumulated is chosen.
        int recording_time_accumulated;         // Total time (of shooting actions only) accumulated since the initial takeoff to this node. The solution for each drone has to maximize the time recording shooting actions, and if several identical solutions the one with less distance accumulated is chosen.

        int edge_into_node = -1;            // Index (in time graph edges) corresponding to the edge entering the actual node (only one father possible). -1 if the node has no father (initial takeoff)

        int time;                           // Time estimated (in seconds) for the actual node to be reached. Only if shooting node.

        int index_of_task_drone_or_bs;      // Identifier of the task (if shooting node), initial drone state (if initial drone state) or base station (if base station node, negative index in this case)

        bool node_has_edges_out = false;    // Used to check if edges out of node exist or not, not really necessary to know the edges.
    };

    // Struct that will be initialized for each base station node.
    // Base station nodes are similar to shooting nodes, but they have also information of landings (battery != 100%) and takeoff (battery == 100%) in base stations. "index_of_task_drone_or_bs" contain negative indexes for base stations, but positive for initial drone states.
    struct BaseStationNode {
        geometry_msgs::Point32 position;    // Position of the actual node.
        float battery;                      // Battery at the node.

        float navigation_distance_accumulated;  // Total distance (of navigation actions only) accumulated since the initial takeoff to this node. The solution for each drone has to maximize the time recording shooting actions, and if several identical solutions the one with less distance accumulated is chosen.
        int recording_time_accumulated;         // Total time (of shooting actions only) accumulated since the initial takeoff to this node. The solution for each drone has to maximize the time recording shooting actions, and if several identical solutions the one with less distance accumulated is chosen.

        int edge_into_node = -1;            // Index (in time graph edges) corresponding to the edge entering the actual node (only one father possible). -1 if the node has no father (initial takeoff)

        int time;                           // Time estimated (in seconds) for the actual node to be reached. Only if shooting node.

        int index_of_task_drone_or_bs;      // Identifier of the task (if shooting node), initial drone state (if initial drone state) or base station (if base station node, negative index in this case)

        bool land_or_takeoff;               // false if land and true if takeoff

        bool node_has_edges_out = false;    // Used to check if edges out of node exist or not, not really necessary to know the edges.  In the end, all shooting nodes should have true in here.
    };

    // Struct that will be initialized for each edge of the time graph.
    // Edges of the time graph are connections between nodes.
    struct EdgeFromTimeGraph {
        int father_node;                // Index (in time graph shooting_node if positive or zero index, base_station_node if negative index) of the father node of the edge. In order to distinguish shooting nodes and base station nodes, if the referenced node is a base station node, it will have a negative (<=-1) index.
        int son_node;                   // Index (in time graph shooting_node if positive or zero index, base_station_node if negative index) of the son node of the edge. In order to distinguish shooting nodes and base station nodes, if the referenced node is a base station node, it will have a negative (<=-1) index.

        bool shooting_or_navigation_action; // Binary variable which value is false if the edge represents a shooting action, and true if it's a navigation action.
    };

    // Struct that will be initialized for each drone available with pose defined when the planner is called. Initial drone states are stored as positive indexes in the field of "index_of_task_drone_or_bs" in base station nodes.
    struct InitialDroneState {
        geometry_msgs::Point32 position;    // Position of the drone in the moment that the planner is called (may be flying already!).
        float battery;                      // Battery that the drone has when the planner is called (may not be fully charged!).
        int drone_id;                       // Drone_id of the drone.
        bool drone_flying_initially;        // True if the drones are flying when the planner is called (replanning), false if not.
        bool drone_used = false;            // Will change during planning algorithm!! True if a plan has been assigned to the drone already by the planner.
    };

    // Struct that will be initialized for each task (shooting action).
    // Tasks are the raw information that will be used to build the time graph.
    struct Task {
        std::map<std::string,float> shooting_parameters;    // Used to displace waypoints of the reference target before sending them to drones.

        std::vector<geometry_msgs::Point32> reference_target_waypoints;  // Waypoints of the reference target.

        std::string event_id;                           // Identifier of the event.
        std::string SA_id;                              // Identifier of the shooting action.
        std::string SA_sequence_id;                     // Identifier of the shooting action sequence.
        std::string mission_id;                         // Identifier of the mission.

        geometry_msgs::Point32 initial_position;        // Initial point of the task.
        geometry_msgs::Point32 final_position;          // Final point of the task.
        geometry_msgs::Point32 rt_displacement;         // Displacement of the formation relative to the reference target waypoints.
        float distance;                                 // Distance in meters of the path between initial and final positions.

        float average_speed;                            // Estimated average speed in meter/second during the shooting action.

        float initial_azimuth;                          // Azimuth at the beginning of the task.

        multidrone_msgs::ShootingType shooting_type;    // Shooting type of the shooting actions.

        int time_initial;                   // Time estimated (in seconds) for the start of the task.
        int time_final;                     // Time estimated (in seconds) for the end of the task.
        int time_difference;                // Difference of time in seconds between the son node and the father node (always positive).

        int delay_since_event;              // Delay of time in seconds between the start event and the start of the task.

        int previous_time_accumulated_for_rates_displacement = 0;   // When tasks are partially assigned and then trimmed, here it's stored the time to take into account in the rates displacemnt by parameters of the task.

        unsigned int shooting_role_index;   // There is a task for each shooting role in a shooting action, this is the index of the shooting role of the current task.

        bool moving_or_hovering;            // Binary variable which value is false if the task is to film while moving, and true if the task is to film staying still.
    };

    // Struct that will be initialized for each time graph.
    // There will be one time graph for each pair of shooting roles given in the mission.
    struct TimeGraph {
        std::vector<BaseStationNode> base_station_node;     // - nodes corresponding to landings or takeoffs (true) in base stations. Base station nodes here will have negative (<=-1) indexes to distinguish from shooting action nodes.
        std::vector<ShootingNode> shooting_node;            // - nodes corresponding to the initial, intermediate or final points of the shooting actions.
        std::vector<EdgeFromTimeGraph> edge;                // - edges of the graph. Some values will be used by the algorithm to solve the problem and store the solution.
        std::vector<Task> task;                             // - tasks that will be the mould for the nodes and edges.
        std::vector<InitialDroneState> initial_drone_state; // - initial drone states, that will also be bases stations that will be the mould for the base station nodes.
        std::vector<geometry_msgs::Point32> base_station;   // - bases stations that will be the mould for the base station nodes.
    };

    ////////////// </Time Graph components> //////////////



    TimeGraph initializeTimeGraph(const std::vector<multidrone_msgs::ShootingAction>& shooting_actions_planned, const std::map< int,std::tuple<geometry_msgs::Point32,double,bool> >& _initial_state_of_drones, bool KML_exist);

    void completeTimeGraph(TimeGraph& time_graph);

    void createNextNodes(std::vector<int>& open_nodes, TimeGraph& time_graph);

    int addBetterTimeGraphSolutionToPlan(std::map< int,std::vector<multidrone_msgs::DroneAction> >& plan, int& current_recorded_time, const TimeGraph& time_graph, const std::vector<multidrone_msgs::ShootingAction>& _shooting_action_list);

    void resetTimeGraph( TimeGraph& time_graph, const int last_bs_node_of_subgraph_to_erase );

    struct NavigationSeparationStruct {
        float distance;
        int moving_time;
    };
    NavigationSeparationStruct navigationSeparation(const geometry_msgs::Point32& from_here, const geometry_msgs::Point32& to_here, bool takeoff_or_flat_movement);

    struct NearestBaseStationStruct {
        float battery_drop;
        float distance;
        int index;
        int moving_time;
    };
    NearestBaseStationStruct nearestBaseStation(const geometry_msgs::Point32& from_here, const TimeGraph& time_graph);

    geometry_msgs::Point32 intermediatePointOfTaskFromTime(int time_at_intermediate_point, const Task& task, bool return_displaced_by_parameters=true);

    void trimTask(Task& task_to_edit, Task& task_not_edited);
    void trimShootingActionTrajectories(int time_at_intermediate_point, bool erase_before_or_after_time, const Task& task, std::vector<geometry_msgs::PointStamped>& reference_target_waypoints_to_edit );

    void displacePointByTaskParameters(geometry_msgs::Point32& point_to_displace, const Task& task, float azimuth, float time_since_task_start, bool reverse_displacement = false);

    float batteryDrop(int hovering_time, float speed, int moving_time);

    void showTimeGraphAndDroneActionsForDebug(const TimeGraph& time_graph, std::map< int,std::vector<multidrone_msgs::DroneAction> > plan, int drone_id);

    void safetyPostprocess(std::map< int,std::vector<multidrone_msgs::DroneAction> >& _plan);

    std::vector<geometry_msgs::PointStamped> buildTrajectoryForCollisionDetection(const std::vector<geometry_msgs::PointStamped>& drone_action_path, int minimum_time_coincidence, int maximum_time_coincidence);    // Method used in the safetyPostprocess for building the trajectories for collision detection,



    ////////////// < Parameters of the planning problem > //////////////

    // Director event id (request of the director event service) that take off the drones and get them ready when receibed:
    std::string get_ready_event_;

    // LiPo battery constraint:
    float minimum_battery_;         // Minimum battery charge (%) that the battery can have during the plan. LiPo batteries should never discharge to less than 20% or else the life span (number of charge/discharge cycles) will be dramatically reduced.

    // Parameters specific for replanning:
    float replanning_same_SA_distance_;         // When replanning, if a flying drone has to fly less distance than this parameter in meters it will be considered the same SA Sequence, this new replanning solution will be bonus.
    float replanning_same_SA_bonus_;   // When replanning, bonus of the new solution if a drone is in the same SA Sequence. In seconds for the time record, and meters in accumulated distance.

    // Drones separation management during navigation actions of multiple drones:
    float minimum_separation_xy_;   // Minimum drone separation (meters) between drones during navigation actions.
    float minimum_separation_z_;    // Minimum drone separation (meters) between drones during navigation actions.

    // Parameters for the successive aproximation method:
    float maximum_battery_error_in_successive_aproximation_method_; // For those nodes that end recording in the middle of a shooting action (because of low battery), % of battery above minimum_battery_ that the drone can have after land.
    int maximum_idle_time_in_successive_aproximation_method_;       // For those shooting nodes that start recording in the middle of a shooting action, maximum time in seconds that the drone can arrive earlier to the node.

    // Shooting drone actions type orbit are discretized to build its path, this parameter is the separation time in seconds between its waypoints in terms of rt_trajectory:
    int orbit_path_separation_of_discretization_;

    // Waste of time estimations:
    int time_swap_drone_;               // When swapping drones inside an edge (SA) because the battery is low and there is no other way to cover the SA, for security the drone that leaves the SA will be separated an amount "time_swap_drone" of seconds of the one that arrives.
    int time_change_battery_;           // Time in seconds estimated for a battery change in a base station.

    // Minimum recording time for shooting actions:
    int time_minimum_shooting_action_;  // In seconds, minimum time for a shooting edge. If found a shooting action with shorter rec time discard it.

    // Battery drop estimation:
    int time_max_hovering_;             // Estimated maximum hovering time (in seconds) before the drone runs out of battery. Must be greater than time_max_flying_full_speed_.
    int time_max_flying_full_speed_;    // Estimated maximum flying time at full speed (in seconds) before the drone runs out of battery. Must be lower than time_max_hovering_.

    // Drone maximum speed values from PX4, information extracted from here (30-01-2019): https://dev.px4.io/en/advanced/parameter_reference.html
    // Parameters not changed in the px4cmd file right now, so default values assigned:
    int full_speed_xy_;     // MPC_XY_VEL_MAX    [min, default, max] : [ 0.0 , 12 , 20.0 ]  Maximum horizontal velocity (m/s) of the drone (in AUTO mode, if higher speeds are commanded in a mission they will be capped to this velocity).
    int full_speed_z_down_; // MPC_Z_VEL_MAX_DN  [min, default, max] : [ 0.5 ,  1 ,  4.0 ]  Maximum vertical descent velocity (m/s) of the drone (in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL)).
    int full_speed_z_up_;   // MPC_Z_VEL_MAX_UP  [min, default, max] : [ 0.5 ,  3 ,  8.0 ]  Maximum vertical ascent velocity (m/s) of the drone (in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL)).
    // TODO?: the drone could have its default parameters changed in "/grvc-ual/robots_description/models/typhoon_h480/px4cmd" or elsewhere if different drone, but in this case that parameters aren't changed. Take this into account. Maybe parse the px4cmd file?

    ////////////// </ Parameters of the planning problem > //////////////


};  // end HighLevelPlanner class

}   // end namespace multidrone

#endif  // HIGH_LEVEL_PLANNER_H