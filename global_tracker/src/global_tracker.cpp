/**
 * MULTIDRONE Project:
 *    Prototyping functions to test the overall system specifications.
 *
 * Mission controller.
 *
 */

#include <global_tracker/global_tracker.h>
#include <functional>
#include <multidrone_kml_parser/geographic_to_cartesian.hpp>

namespace multidrone
{

// Brief Constructor
GlobalTracker::GlobalTracker()
{

  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  // Read parameters
  // drones [id_1,...,id_N]
  pn.getParam("drones", drones_);
  pn.getParam("targets", targets_vector_);
  pn.getParam("filter", filter);
  pn.getParam("height", height);
  std::vector<double> origin_geo_vector;
  pn.getParam("origin_geo",origin_geo_vector);
  origin_geo_.latitude = origin_geo_vector[0];
  origin_geo_.longitude = origin_geo_vector[1];
  origin_geo_.altitude = origin_geo_vector[2];

  // Subscribers
  for (auto target_id : targets_vector_)
  {
    boost::shared_ptr<sensor_msgs::NavSatFix const> sharedPtr;
    sensor_msgs::NavSatFix ori;
    sharedPtr  = ros::topic::waitForMessage<sensor_msgs::NavSatFix>("target_"+std::to_string(target_id)+"/mavros/global_position/raw/fix", ros::Duration(5));
    ori= *sharedPtr;
    std::cout << ori <<"\n";
    for(int i=0; i < filter; i++)
      filter_position[target_id].push_back(ori);
    
    ROS_INFO("Subscribed to target %d",target_id);
    target_gps_sub_[target_id]      = n.subscribe<sensor_msgs::NavSatFix>("target_"+std::to_string(target_id)+"/mavros/global_position/raw/fix", 1, std::bind(&GlobalTracker::targetGpsCallback, this, std::placeholders::_1, target_id));
    target_pose_sub_[target_id]     = n.subscribe<geometry_msgs::PoseStamped>("target_"+std::to_string(target_id)+"/mavros/local_position/pose", 1, std::bind(&GlobalTracker::targetPoseCallback, this, std::placeholders::_1, target_id));
    target_velocity_sub_[target_id] = n.subscribe<geometry_msgs::TwistStamped>("target_"+std::to_string(target_id)+"/mavros/local_position/velocity", 1, std::bind(&GlobalTracker::targetVelocityCallback, this, std::placeholders::_1, target_id));
    target_delay_sub_[target_id]    = n.subscribe<mavros_msgs::TimesyncStatus>("target_"+std::to_string(target_id)+"/mavros/timesync_status", 1, std::bind(&GlobalTracker::delayCallback, this, std::placeholders::_1, target_id));
  }
  
  target_pub_        = n.advertise<geometry_msgs::PoseStamped>("fake_target_pose",1);

  for (int i = 0; i < drones_.size(); i++)
    drone_target_sub_[drones_[i]] = n.subscribe<multidrone_msgs::TargetStateArray>("drone_" + std::to_string(drones_[i]) + "/target_3d_pose", 1, std::bind(&GlobalTracker::targetDroneCallback, this, std::placeholders::_1, drones_[i])); // Change for each drone ID

  // publishers
  global_target_pub_ = n.advertise<multidrone_msgs::TargetStateArray>("targets_pose",1);

  // Make communications spin!
  spin_thread_ = std::thread([this]() {
    ros::MultiThreadedSpinner spinner(2); // Use 2 threads
    spinner.spin();
  });

  /// Main loop
  mainloop_thread_ = std::thread([this]() {
    // Publish @ 10Hz
    ros::Rate loop_rate(5);
    while (ros::ok())
    {
      multidrone_msgs::TargetStateArray targets_msg;
      for(auto elem : targets_) {
	      targets_msg.targets.push_back(elem.second);
      }
      targets_msg.header.stamp = ros::Time::now();
      global_target_pub_.publish(targets_msg);
      target_pub_.publish(lala);

      loop_rate.sleep();
    }
  });
}

// Brief Destructor
GlobalTracker::~GlobalTracker()
{
}

/**\Brief GPS target position
 */
void GlobalTracker::targetGpsCallback(const sensor_msgs::NavSatFix::ConstPtr& _msg, const int _target_id)
{
  geographic_msgs::GeoPoint actual_coordinate_geo;
  filter_position[_target_id][0].longitude               = _msg->longitude;
  filter_position[_target_id][0].latitude                = _msg->latitude;
  filter_position[_target_id][0].altitude                = _msg->altitude;
  filter_position[_target_id][0].position_covariance[4]  = _msg->position_covariance[4];
  filter_position[_target_id][0].position_covariance[0]  = _msg->position_covariance[0];
  filter_position[_target_id][0].position_covariance[8]  = _msg->position_covariance[8];
    
  for(auto x: filter_position[_target_id]){
    actual_coordinate_geo.longitude          += (x.longitude)/filter_position[_target_id].size();
    actual_coordinate_geo.latitude           += (x.latitude) /filter_position[_target_id].size();
    actual_coordinate_geo.altitude           += (x.altitude) /filter_position[_target_id].size();
    targets_[_target_id].pose.covariance[0]  += (x.position_covariance[4])/filter_position[_target_id].size();
    targets_[_target_id].pose.covariance[7]  += (x.position_covariance[0])/filter_position[_target_id].size();
    targets_[_target_id].pose.covariance[14] += (x.position_covariance[8])/filter_position[_target_id].size();
  }


  geometry_msgs::Point32 target_pose = multidrone::geographic_to_cartesian(actual_coordinate_geo, origin_geo_);

  targets_[_target_id].pose.pose.position.x = target_pose.x;
  targets_[_target_id].pose.pose.position.y = target_pose.y;
  targets_[_target_id].pose.pose.position.z = height;
  targets_[_target_id].target_id = _target_id;
  
  lala.pose.position.x = target_pose.x;
  lala.pose.position.y = target_pose.y;
  lala.pose.position.z = height;


  std::rotate(filter_position[_target_id].rbegin(), filter_position[_target_id].rbegin() + 1, filter_position[_target_id].rend());
}


void GlobalTracker::targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg, const int _target_id)
{
  targets_[_target_id].pose.pose.orientation = _msg->pose.orientation;
}

void GlobalTracker::targetVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& _msg, const int _target_id)
{
  targets_[_target_id].velocity.twist = _msg->twist;
}

/** Callback for target estimation from each drone
 */
void GlobalTracker::targetDroneCallback(const multidrone_msgs::TargetStateArray::ConstPtr& _msg, const int _drone_id)
{
  // ROS_INFO("Received target pose from drone %d.", _drone_id);
  // TODO: act acordingly
}

void GlobalTracker::delayCallback(const mavros_msgs::TimesyncStatus::ConstPtr& _msg, const int _target_id){
  targets_[_target_id].delay = _msg->round_trip_time_ms;
}

}
