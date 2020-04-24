/**
 * MULTIDRONE Project:
 *
 * Global 3D Tracker.
 * 
 */

#ifndef GLOBAL_TRACKER_H
#define GLOBAL_TRACKER_H

#include <ros/ros.h>
#include <thread>
#include <vector>
#include <map>
#include <multidrone_msgs/TargetStateArray.h>
#include <geographic_msgs/GeoPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/TimesyncStatus.h>
#include <geometry_msgs/Point32.h>


namespace multidrone {

/// GlobalTracker class
class GlobalTracker {

public:
    GlobalTracker();
    ~GlobalTracker();

private:
    void targetGpsCallback(const sensor_msgs::NavSatFix::ConstPtr& _msg,  const int _target_id);
    void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg, const int _target_id);
    void targetVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& _msg, const int _target_id);
    void targetDroneCallback(const multidrone_msgs::TargetStateArray::ConstPtr& _msg, const int _drone_id);
    void delayCallback(const mavros_msgs::TimesyncStatus::ConstPtr& _msg, const int _target_id);


    std::map<int, multidrone_msgs::TargetState> targets_;

    // Subscribers
    std::map<int, ros::Subscriber> target_gps_sub_;
    std::map<int, ros::Subscriber> target_pose_sub_;
    std::map<int, ros::Subscriber> target_velocity_sub_;
    std::map<int, ros::Subscriber> drone_target_sub_;
    std::map<int, ros::Subscriber> target_delay_sub_;

    // Publishers
    ros::Publisher global_target_pub_;

    std::thread spin_thread_;
    std::thread mainloop_thread_;

    std::vector<int> drones_; // drones [id_1,...,id_N] will be received with rosparam
    std::vector<int> targets_vector_; // targets_vector [id_1,...,id_N] will be received with rosparam
    std::vector<int> targets_velocity_vector_; // targets_vector [id_1,...,id_N] will be received with rosparam

    std::map <int, std::vector<sensor_msgs::NavSatFix> > filter_position;
    geographic_msgs::GeoPoint origin_geo_;
    
    geometry_msgs::PoseStamped lala;
    ros::Publisher target_pub_;

    int filter;
    double height;
};  // GlobalTracker class

} // namespace multidrone

#endif  // GLOBAL_TRACKER_H