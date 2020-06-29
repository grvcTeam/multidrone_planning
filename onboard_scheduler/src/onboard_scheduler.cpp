/**
 * MULTIDRONE Project:
 *    Prototyping functions to test the overall system specifications.
 *
 * Onboard scheduler node.
 *
 */

#include <onboard_scheduler.h>

/** \brief Constructor: Initialize subscriptions, publications, services, actionlib and action status thread.
 */
OnBoardScheduler::OnBoardScheduler()
{
  nh_ = ros::NodeHandle();
  pnh_ = ros::NodeHandle("~");

  pnh_.param<int>("drone_id", drone_id_, 1);
  ROS_INFO("Setting up Scheduler %d",drone_id_);


  
  // Subscriptions
  event_sub_ = nh_.subscribe<multidrone_msgs::Event>("/mission_controller/event", 10, &OnBoardScheduler::eventReceived, this);
  drone_telemetry_sub_ = nh_.subscribe<sensor_msgs::BatteryState>("mavros/battery", 10, &OnBoardScheduler::droneTelemetryCallback, this);
  drone_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("ual/pose", 1, &OnBoardScheduler::dronePoseCallback, this);
  ual_state_sub_ = nh_.subscribe<uav_abstraction_layer::State>("ual/state",1, &OnBoardScheduler::ualStateCallback,this);
  // Publications
  action_status_pub_ = nh_.advertise<multidrone_msgs::ActionStatus>("action_status", 1);

  // Services
  action_list_service_ = nh_.advertiseService("push_drone_action", &OnBoardScheduler::droneactionlistServiceCallback, this);
  emergency_service_ = nh_.advertiseService("emergency", &OnBoardScheduler::emergencyCallback, this);
  safe_to_go_service_ = nh_.advertiseService("safe_to_go", &OnBoardScheduler::safeToGoCallback, this);
  abort_service_ = nh_.advertiseService("abort", &OnBoardScheduler::abortServiceCallback, this);

  KML_client_ = nh_.serviceClient<multidrone_msgs::GetSemanticMap>("/get_semantic_map");
  // execute action client
  action_client_ = new Drone_action_client("action_server", true);

  // Timer callback for KML update when drone idle:
  timer_ = nh_.createTimer(ros::Duration(10), &OnBoardScheduler::kmlUpdateCallback,this); // 0.1 Hz
  
  bool connected = false;
  bool print_only_once = true;
  while(!connected){
    ros::Duration d(1);
    if (print_only_once) {
      ROS_INFO("[Scheduler %d] Waiting for action server",drone_id_);
      print_only_once = false;
    }
    connected = action_client_->waitForServer(d);
  }
  // action status thread
  status_thread_ = std::thread(&OnBoardScheduler::updatingActionsStatus, this);

  // main loop thread
  loop_thread_ = std::thread(&OnBoardScheduler::loop, this);

}



/** \brief Destructor
 */
OnBoardScheduler::~OnBoardScheduler()
{
  delete action_client_;
}



/** \brief  Event callback. This callback find a drone action for the event. If it is found, the previous drone actions would be erased.
 * If any drone action has been found, nothing happens
 * \param event     event received
 */
void OnBoardScheduler::eventReceived(const multidrone_msgs::Event::ConstPtr &event)
{
 
  int cont = 0;
  if(!safe_to_go_flag_){
    ROS_WARN("Scheduler [%d]: event %s received and rejected because drone %d is not ready", drone_id_, event->event_id.c_str(), drone_id_);
    return;
  }
  if(push_drone_action_.empty()){
    ROS_WARN("Scheduler [%d]: event %s received and rejected because there is no drone action", drone_id_, event->event_id.c_str());
    return;
  }
  for(int i = 0; i<push_drone_action_.size();++i){
    if(event->event_id == push_drone_action_[i].start_event){
       event_timestamp_ = event->header.stamp.toSec();
       last_event_received_time_ = ros::Time::now();
       event_ = event->event_id;
      // flag when an event has been received and a corresponding drone action has been found
      event_received_ = true;
      // erase the previous drone actions to the drone action that is going to be executed
      if(i!=0) push_drone_action_.erase(push_drone_action_.begin(),push_drone_action_.begin()+i);
      ROS_INFO("Scheduler [%d]: event %s received and drone action found", drone_id_, event->event_id.c_str());
      break;
    }
    if(i==(push_drone_action_.size()-1)){ // any drone actions have been found for this event
      ROS_WARN("Scheduler [%d]: event %s received and drone action not found",drone_id_, event->event_id.c_str());
    } 

  }
}
/** \brief abort service callback
*/
bool OnBoardScheduler::abortServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
  abort_service_received_ = true;
  res.success = true;
  return true;
}
/** \brief safe to go service callback
 */
bool OnBoardScheduler::safeToGoCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
  safe_to_go_flag_ = true;
  res.success = true;
  if (action_status_ == multidrone_msgs::ActionStatus::AS_WAIT_FOR_SAFE_TO_GO) {
    action_status_ = multidrone_msgs::ActionStatus::AS_WAIT_FOR_GET_READY;
  }
  return true;
}

/** \brief callback of the UAL state
 */
void OnBoardScheduler::ualStateCallback(const uav_abstraction_layer::State::ConstPtr &_msg){
  state_.state = _msg->state;
}

/** \brief  Callback for the drone pose
 */
void OnBoardScheduler::dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg)
{
  drone_pose_previous_ = drone_pose_;
  drone_pose_.pose = _msg->pose;
}
/** \brief  Callback for the drone telemetry. This callback will launch emergency when the baterry level is low
 */
void OnBoardScheduler::droneTelemetryCallback(const sensor_msgs::BatteryState::ConstPtr &battery_status)
{
  if (battery_status->percentage < BATTERY_LIMIT)
  {
    emergency_ = true;
  }
}


/** \brief Drone action list received from mission controller
*/
bool OnBoardScheduler::droneactionlistServiceCallback(multidrone_msgs::PushDroneAction::Request &req, multidrone_msgs::PushDroneAction::Response &res)
{
  ROS_INFO("Scheduler [%d]: Drone actions received from the mission controller", drone_id_);

  // if a new drone action list is received, turn a flag on to reject the older
  if (!push_drone_action_.empty())
  {
    new_drone_action_list_ = true;
    push_drone_action_.clear();
  }

  /// push the drone action list into a member of the scheduler class
  for (int i = 0; i < req.action_list.size(); ++i)
  {
    push_drone_action_.push_back(req.action_list[i]);

    // Warning if complex missions with delays for the final tests in Germany.
    if (req.action_list[i].shooting_action.delay_since_event > 0) {
      std::cout << std::endl;
      ROS_WARN("Scheduler [%d]: shooting action %d will be executed after a time delay.",drone_id_,i);
      std::cout << std::endl;
    }
  }
  
  return true;
}

/** \brief Utility function to check communication with Executor
 *  \param last_time    last time that the msg was received
 *  \param delay_allowed  delay allowed between msgs
 * **/
void OnBoardScheduler::checkCommunication(time_t last_time, double delay_allowed){
  if(time(NULL)-last_time>delay_allowed) ROS_WARN("Scheduler [%d]: delay in communications with executor",drone_id_);
}

/** \brief Drone action feedback callback 
*/
void OnBoardScheduler::feedbackCallback(const multidrone_msgs::ExecuteFeedbackConstPtr& feedback){
  if(feedback->status == false) // Nan of inf in the executor
  { 
    ROS_WARN("Scheduler [%d]: NaN or inf in the executor",drone_id_);
    
    if(feedback->action_id != previous_id_NaN_inf_){
      cont_nan_inf_executor_++;
      previous_id_NaN_inf_ = feedback->action_id;
    } 

    if(cont_nan_inf_executor_<MAX_NUMBER_NAN){ //first time nan or inf in the executor, sending another SA
      nan_inf_executor_ = true;
    }else{ // second time receiving nan or inf, emergency state is launched
      emergency_ = true;
    }
  }
  time_feedback_ = time(NULL);
}

/** \brief Callback to check if the drone action is active
 */
void OnBoardScheduler::activeCallback(){
  ROS_INFO("Scheduler [%d]: The drone action was received by the executor and it just went active",drone_id_);
  drone_action_actived_ = true;
}

/** \brief Callback for an emergency
 */
bool OnBoardScheduler::emergencyCallback(multidrone_msgs::SupervisorAlarm::Request &req, multidrone_msgs::SupervisorAlarm::Response &res)
{
  ROS_INFO("Emergency service from supervisor");
  emergency_ = true;
  res.received = true;
  return true;
}

/** \brief Action status thread executes this function
 */
void OnBoardScheduler::updatingActionsStatus() //const ros::TimerEvent&
{
  while (ros::ok)
  {
    ros::Rate loop_rate(1); //[Hz]
    multidrone_msgs::ActionStatus action_status;
    action_status.status = action_status_;
    action_status.mission_id = mission_id_;   // During the following drone actions this information is not available (because the SA field is empty), so it won't be published: navigation action, land and takeoff. Only available in shooting drone actions.
    action_status.SAS_id = SAS_id_;           // During the following drone actions this information is not available (because the SA field is empty), so it won't be published: navigation action, land and takeoff. Only available in shooting drone actions.
    action_status.action_id = action_id_;
    action_status.header.stamp = ros::Time::now();
    action_status_pub_.publish(action_status); // publishing action_status (1 Hz)
    if (action_status.status == multidrone_msgs::ActionStatus::AS_IDLE) { // If not idle stop updating the KML.
      timer_.start(); // start() does nothing if already started.
    } else {
      timer_.stop();  // stop() does nothing if already ended.
    }
    loop_rate.sleep();
  }
}



// When drone idle, the drone should ask periodically for a new KML of the static map:
void OnBoardScheduler::kmlUpdateCallback(const ros::TimerEvent&) {
  // KML stuff and create a path planner to emergency alarms:
  multidrone_msgs::GetSemanticMap kml_srv;
  if (KML_client_.call(kml_srv)) {
    if (!kml_srv.response.semantic_map.empty()) {
      if (print_new_KML_rosinfo_!=1) {
        ROS_INFO("Scheduler %d: KML received.", drone_id_);
        print_new_KML_rosinfo_ = 1;
      }
      try {
        std::vector<double> map_origin_geo_param;
        ros::param::get("ual/map_origin_geo", map_origin_geo_param);
        geographic_msgs::GeoPoint origin_coordinates_geo;
        origin_coordinates_geo.latitude  = map_origin_geo_param.at(0);
        origin_coordinates_geo.longitude = map_origin_geo_param.at(1);
        origin_coordinates_geo.altitude  = map_origin_geo_param.at(2);
        path_planner_ = multidrone::PathPlanner(std::string(kml_srv.response.semantic_map),origin_coordinates_geo);
      } catch (...) {
        ROS_ERROR("Scheduler %d: param map_origin_geo from UAL not available.", drone_id_);
        path_planner_ = multidrone::PathPlanner();
      }
    } else {
      if (print_new_KML_rosinfo_!=2) {
        ROS_WARN("Scheduler %d: KML received empty.", drone_id_);
        print_new_KML_rosinfo_ = 2;
      }
      path_planner_ = multidrone::PathPlanner();
    }
  } else {
    if (print_new_KML_rosinfo_!=3) {
      ROS_WARN("Scheduler %d: KML service not available.", drone_id_);
      print_new_KML_rosinfo_ = 3;
    }
    path_planner_ = multidrone::PathPlanner();
  }
} // end kmlUpdateCallback


/** \brief this function command the drone to go to the emergency site
 *  \param x,y,z  position of the emergencty landing site
 *  \param abort  this flag differenciate between abort and emergency
 */
void OnBoardScheduler::goToEmergencySite(const float x, const float y, const float z, const bool abort, const bool emergency)
{
  // wait for the executor to finish the landing or the take off  
  if(action_status_ == multidrone_msgs::ActionStatus::AS_TAKING_OFF || action_status_ == multidrone_msgs::ActionStatus::AS_LANDING){
    ros::Time begin = ros::Time::now();
    float duration = 0.0;
    ros::Rate rate(1); //[Hz]
    while(action_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
      duration = ros::Time::now().toSec() - begin.toSec();
      if(action_status_ == multidrone_msgs::ActionStatus::AS_TAKING_OFF) ROS_INFO("Scheduler [%d]: waiting for the Executor to finish the take off", drone_id_);
      if(action_status_ == multidrone_msgs::ActionStatus::AS_LANDING) ROS_INFO("Scheduler [%d]: waiting for the Executor to finish the landing", drone_id_);
      if(duration>max_time_landing_takeoff_){
        ROS_WARN("Scheduler [%d]: too much time waiting for The Executor. Aborting the wait",drone_id_);
        break;
      }
      rate.sleep();
      ros::spinOnce();
    }
  }
  // if the UAV is flying, it will perfom a contigency plan
  if(state_.state == uav_abstraction_layer::State::FLYING_AUTO)
  {
    ROS_INFO("drone %d: going to emergency site", drone_id_);

    //TODO search for the nearest landing site (if path_planner_.getTrivialPathPlannerOrNot() == false)
    //std::vector<std::vector<geometry_msgs::Point32>> emergency_polygon = path_planner_.KML_parser_from_path_planner_.emergency_landing_sites_cartesian_;

    geometry_msgs::PointStamped emergency_pos;
    ros::Rate rate(1); //[Hz]

    // emergency landing site pose
    emergency_pos.point.x = x;
    emergency_pos.point.y = y;
    emergency_pos.point.z = z;

    // initial pose
    geometry_msgs::PointStamped initial_pose;
    initial_pose.point.x = drone_pose_.pose.position.x;
    initial_pose.point.y = drone_pose_.pose.position.y;
    initial_pose.point.z = drone_pose_.pose.position.z;
    // ask for path
    std::vector<geometry_msgs::PointStamped> path;
    path = path_planner_.getPath(initial_pose, emergency_pos);

    /// send this drone action to executor
    multidrone_msgs::DroneAction go_to_emergency_site;
    go_to_emergency_site.path = path;
    go_to_emergency_site.action_type = multidrone_msgs::DroneAction::TYPE_GOTOWAYPOINT;
    multidrone_msgs::ExecuteGoal goal;

    goal.action_goal = go_to_emergency_site;
    drone_action_actived_ = false;
    action_client_->sendGoal(goal, Drone_action_client::SimpleDoneCallback(), boost::bind(&OnBoardScheduler::activeCallback, this));
    if(abort)           action_status_ = multidrone_msgs::ActionStatus::AS_GOING_HOME;
    else if (emergency) action_status_ = multidrone_msgs::ActionStatus::AS_EMERGENCY;

    // wait for executor
    while(!drone_action_actived_){
      ROS_INFO("Scheduler [%d]: waiting for goToWaypoint to be activated",drone_id_);
      sleep(1);
      ros::spinOnce();
    }

    /// wait for drone arrives to home
    while(action_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Scheduler [%d]: Waiting for the result of the navigation action", drone_id_); // wait for a result from executor when a drone action finish
      rate.sleep();
    }

    // command land to executor
    multidrone_msgs::DroneAction land_home;
    land_home.action_type = multidrone_msgs::DroneAction::TYPE_LAND;
    goal.action_goal = land_home;
    drone_action_actived_ = false;
    action_client_->sendGoal(goal, Drone_action_client::SimpleDoneCallback(), boost::bind(&OnBoardScheduler::activeCallback, this));
    if(abort) action_status_ = multidrone_msgs::ActionStatus::AS_LANDING;

    while(!drone_action_actived_){ // wait for executor
      ROS_INFO("Scheduler [%d]: waiting for landing to be activated",drone_id_);
      sleep(1);
      ros::spinOnce();
    }

    /// wait for landing
    while(action_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
      rate.sleep();
      ROS_INFO("Scheduler [%d]: Waiting for landing", drone_id_); // wait for a result from executor when a drone action finish
    }
  }
  emergency_ = false; 
  safe_to_go_flag_ = false; // the safe to go is needed again to fly
  abort_service_received_ = false;
  nan_inf_executor_ = false;
}

/** \brief utility function to check the travelled distance of a shooting action and return if the total distance was reached
*/
bool OnBoardScheduler::checkDistance(float total_distance, float &dist_sum){
   if(total_distance == 0) return false;
   dist_sum =+ sqrt(powf(drone_pose_.pose.position.x- drone_pose_previous_.pose.position.x, 2.0) +
   powf(drone_pose_.pose.position.y- drone_pose_previous_.pose.position.y, 2.0)+
   powf(drone_pose_.pose.position.z- drone_pose_previous_.pose.position.z, 2.0));
   
   float diff = total_distance-dist_sum;
   float diff_min = 1.0;
   if(diff<diff_min) return true;
   else return false;
}

/**  \brief utility function to check the time during a shooting action and return whether max time was reached
*/
bool OnBoardScheduler::checkTime(float max_dur_sec, ros::Time begin){
  if(max_dur_sec == 0) return false;
  float duration = ros::Time::now().toSec() - begin.toSec();
  if(max_dur_sec<duration){
    return true;
  } 
  else return false;
}
/** \brief Main loop.
 */
void OnBoardScheduler::loop()
{
  // Main loop
  ROS_INFO("Scheduler [%d] initialized!", drone_id_);
  ros::Time begin;
  float sum_distance;
  int waiting_for_event_cont = 0;
  while (ros::ok)
  {
    ros::Rate loop_rate(10); //[Hz]
    loop_rate.sleep();
    ros::spinOnce();
    if(emergency_ || abort_service_received_){ // check emergency and abort service
      //TODO search for the nearest landing site
      ROS_WARN("Scheduler [%d]: abort or emergency. Rejecting drone action list.", drone_id_);
      event_ = "NULL";
      if(emergency_){
        goToEmergencySite(home_pose_.pose.position.x,home_pose_.pose.position.y,drone_pose_.pose.position.z, false, true);
      }
      else{
        goToEmergencySite(home_pose_.pose.position.x,home_pose_.pose.position.y,drone_pose_.pose.position.z, true, false); //abort
      } 
      push_drone_action_.clear();
    }
    else if(new_drone_action_list_ && !push_drone_action_.empty()){ // check if a new drone action list has been received
      ROS_INFO("Scheduler [%d]: new shooting action list. Rejecting the old drone action list", drone_id_);
      event_ = "NULL";
      new_drone_action_list_ = false;
    }
    else if (!push_drone_action_.empty() && safe_to_go_flag_) // drone action list received and is safe to go
    {
      multidrone_msgs::ExecuteGoal goal;
      ros::Duration delay = ros::Time::now()-last_event_received_time_;
      if(push_drone_action_[0].start_event == "" || (event_ == push_drone_action_[0].start_event && push_drone_action_[0].delay_since_event < delay.toSec())) // if the first drone should wait for an event + delay, wait for it
      {
        delay_countdown_ = 0;
        waiting_for_event_cont = 0;
        event_received_ = false;
        /// publish action to executor
        goal.action_goal = push_drone_action_[0];

        // TODO: shooting_parameters doesn't have now "event_timestamp"

        // if the UAL is not flying auto, the scheduler will wait for it to command next order
        if(state_.state != uav_abstraction_layer::State::FLYING_AUTO && (goal.action_goal.action_type == multidrone_msgs::DroneAction::TYPE_GOTOWAYPOINT || goal.action_goal.action_type == multidrone_msgs::DroneAction::TYPE_SHOOTING)){
         ROS_WARN("Scheduler [%d]: waiting for FlYING_AUTO", drone_id_);
         sleep(1);
         continue;
        }
        drone_action_actived_ = false;
        action_client_->sendGoal(goal, Drone_action_client::SimpleDoneCallback(), boost::bind(&OnBoardScheduler::activeCallback, this), boost::bind(&OnBoardScheduler::feedbackCallback, this, _1));
        action_id_ = goal.action_goal.action_id;
        mission_id_ = goal.action_goal.shooting_action.mission_id;
        SAS_id_ = goal.action_goal.action_sequence_id;
        if(goal.action_goal.action_type == multidrone_msgs::DroneAction::TYPE_TAKEOFF){
          ROS_INFO("Scheduler [%d]: sending take off to the executor", drone_id_);
          if(!home_pose_saved_){ // Scheduler saves the pose before taking offf to back home later
            home_pose_ = drone_pose_;
            home_pose_saved_ = true;
          } 
        }
        else if(goal.action_goal.action_type == multidrone_msgs::DroneAction::TYPE_LAND){
           home_pose_saved_ = false;  // once the drone has landed, the home pose has to be saved again
           ROS_INFO("Scheduler [%d]: sending land to the executor", drone_id_);
           safe_to_go_flag_ = false; //once the drones have landed, the safe to go service is needed again
        }
        else if(goal.action_goal.action_type == multidrone_msgs::DroneAction::TYPE_GOTOWAYPOINT) ROS_INFO("Scheduler [%d]: sending going to the start pose of SA %s to the executor. x=%f, y=%f", drone_id_,goal.action_goal.action_id.c_str(),home_pose_.pose.position.x,home_pose_.pose.position.y);
        else if(goal.action_goal.action_type == multidrone_msgs::DroneAction::TYPE_SHOOTING) ROS_INFO("Scheduler [%d]: shooting action %s to the executor", drone_id_,goal.action_goal.action_id.c_str());
      }
      else if (event_ == push_drone_action_[0].start_event && push_drone_action_[0].delay_since_event >= delay.toSec()) { // if waiting for a delay
        if (push_drone_action_[0].action_type == multidrone_msgs::DroneAction::TYPE_TAKEOFF && push_drone_action_[0].delay_since_event>0) {
          if (delay_countdown_ != (int)(push_drone_action_[0].delay_since_event-delay.toSec()) && (int)(push_drone_action_[0].delay_since_event-delay.toSec()) > 0) {
            delay_countdown_ = (int)(push_drone_action_[0].delay_since_event-delay.toSec());
            ROS_WARN("Scheduler [%d]: takeoff in %d seconds.", drone_id_, delay_countdown_);
          }
        }
        action_id_ = push_drone_action_[0].action_id;
        SAS_id_ = push_drone_action_[0].action_sequence_id;
        continue;
      }
      else{  // waiting for event
        delay_countdown_ = 0;
        if(waiting_for_event_cont == 0) ROS_INFO("Scheduler [%d]: waiting for event %s", drone_id_, push_drone_action_[0].start_event.c_str());
        waiting_for_event_cont++;
        if (push_drone_action_[0].start_event == "GET_READY") { // TODO: read from somewhere the name of the director event to avoid hard-code
          action_status_ = multidrone_msgs::ActionStatus::AS_WAIT_FOR_GET_READY;
        } else {
          action_status_ = multidrone_msgs::ActionStatus::AS_WAIT_FOR_EVENT;
        }
        action_id_ = push_drone_action_[0].action_id;
        SAS_id_ = push_drone_action_[0].action_sequence_id;
        continue;
      }
        begin = ros::Time::now();
        sum_distance = 0.0;
      while(ros::ok){
        if(emergency_ || abort_service_received_){ // check emergency
          //TODO search for the nearest landing site
          ROS_INFO("Scheduler [%d]: abort or emergency. Rejecting drone action list.", drone_id_);
          event_ = "NULL";
          if(emergency_){
            goToEmergencySite(home_pose_.pose.position.x,home_pose_.pose.position.y,drone_pose_.pose.position.z, false, true);
          }
          else{
            goToEmergencySite(home_pose_.pose.position.x,home_pose_.pose.position.y,drone_pose_.pose.position.z, true, false);
          } 
          push_drone_action_.clear();
          break;
          }
        else if(new_drone_action_list_){ // check whether a new drone action list has been received
          if (push_drone_action_[0].start_event == "GET_READY") { // TODO: read from somewhere the name of the director event to avoid hard-code
            action_status_ = multidrone_msgs::ActionStatus::AS_WAIT_FOR_GET_READY;
          } else {
            action_status_ = multidrone_msgs::ActionStatus::AS_WAIT_FOR_EVENT;
          }
          waiting_for_event_cont = 0;
          action_id_ = "";
          mission_id_ = "";
          SAS_id_ = "";
          ROS_INFO("Scheduler [%d]: new shooting action list. Rejecting the old drone action list", drone_id_);
          event_ = "NULL";
          new_drone_action_list_ = false;
          break;
          }
        else if(event_received_ == true){ // check if an event has been received
          waiting_for_event_cont = 0;
          break;
          }
        else if(!drone_action_actived_){ //check if the executor has received the last drone action
          ROS_INFO("Scheduler [%d]: waiting for the drone action to be activated",drone_id_);
          sleep(1);
        }else{
        // if any of the previous conditions has happened, check the distance and the time
          if(goal.action_goal.action_type == multidrone_msgs::DroneAction::TYPE_SHOOTING){ //shooting action
            double dangerous_delay = 2.0;
            // checkCommunication(time_feedback_,dangerous_delay); //check the feedback sent by the executor
            action_status_ = multidrone_msgs::ActionStatus::AS_RUNNING;
            if(checkDistance(goal.action_goal.shooting_action.length, sum_distance )){  // check the max distance of the shooting action
              ROS_INFO("Scheduler [%d]: Max distance reached of the drone action %s", drone_id_, goal.action_goal.action_id.c_str());
            push_drone_action_.erase(push_drone_action_.begin());
            break;
            } 
            if(checkTime(goal.action_goal.shooting_action.duration.data.sec, begin)){ // check the max time of the shooting action
              ROS_INFO("Scheduler [%d]: Max time of the drone action %s", drone_id_, goal.action_goal.action_id.c_str());
              push_drone_action_.erase(push_drone_action_.begin());
              // if drone action ends by max time, and if next drone action is a navigation action followed by a shooting drone action, delete the associated event of the navigation action
              // so that the drone go directly to where the next shooting drone action starts and wait there for the director event.
              if (push_drone_action_.size()>1) {
                if (push_drone_action_[0].action_type == multidrone_msgs::DroneAction::TYPE_GOTOWAYPOINT && push_drone_action_[1].action_type == multidrone_msgs::DroneAction::TYPE_SHOOTING) {
                  push_drone_action_[0].start_event.clear();
                }
              }

              break;
            }
            if(nan_inf_executor_){ // if executor reports NaN or Inf, the scheduler will try to send next SA
              ROS_INFO("Scheduler [%d]: rejecting shooting action %s because the executor reported NaN or Inf", drone_id_, goal.action_goal.action_id.c_str());
              push_drone_action_.erase(push_drone_action_.begin());
              nan_inf_executor_ = false;
              break;
            }
          }
          else{ // navigation action
            if(goal.action_goal.action_type == multidrone_msgs::DroneAction::TYPE_TAKEOFF) action_status_ = multidrone_msgs::ActionStatus::AS_TAKING_OFF;
            else if(push_drone_action_.size()<3 && goal.action_goal.action_type != multidrone_msgs::DroneAction::TYPE_LAND) action_status_ = multidrone_msgs::ActionStatus::AS_GOING_HOME;
            else if(goal.action_goal.action_type == multidrone_msgs::DroneAction::TYPE_LAND) action_status_ = multidrone_msgs::ActionStatus::AS_LANDING;
            else{ // go to waypoint
              double dangerous_delay = 2.0;
              // checkCommunication(time_feedback_,dangerous_delay); // Check the feedback sent by executor
              action_status_ = multidrone_msgs::ActionStatus::AS_GOING_TO_START_POSE;
            }
            

            if(action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) //check the succees of the navigation action
            {
              if(goal.action_goal.action_type == multidrone_msgs::DroneAction::TYPE_TAKEOFF) ROS_INFO("Scheduler [%d]: take off finished", drone_id_);
              else if(goal.action_goal.action_type == multidrone_msgs::DroneAction::TYPE_LAND) ROS_INFO("Scheduler [%d]: land finished", drone_id_);
              else if(goal.action_goal.action_type == multidrone_msgs::DroneAction::TYPE_GOTOWAYPOINT) ROS_INFO("Scheduler [%d]: going to the start pose of SA %s finished", drone_id_,goal.action_goal.action_id.c_str());
              push_drone_action_.erase(push_drone_action_.begin());
              break;
            }
          }
      } 
        loop_rate.sleep();
        ros::spinOnce();
      }
    }
    else if (push_drone_action_.empty()) {
      action_status_ = multidrone_msgs::ActionStatus::AS_IDLE;
      action_id_ = "0";
      SAS_id_ = "";
      mission_id_ = "";
    }   // if any drone action list has been sent, AS_IDLE status will be activated
    else if (!safe_to_go_flag_) {  // Drone action list sent but the safe to go service not receibed
      action_status_ = multidrone_msgs::ActionStatus::AS_WAIT_FOR_SAFE_TO_GO;
      action_id_ = "0";
      SAS_id_ = "";
      mission_id_ = "";
    }

  }

}