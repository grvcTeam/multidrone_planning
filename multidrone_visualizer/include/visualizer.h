#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <string>                   //to string
#include <sstream>                  //stringstream
#include <vector>                   //vector3
#include <iomanip>                  // setprecision
#include <tf/transform_datatypes.h> //to get yaw
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <multidrone_msgs/ActionStatus.h>
#include <multidrone_msgs/TargetStateArray.h>
#include <multidrone_msgs/GimbalStatus.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>
#include <tf2_msgs/TFMessage.h>
#include <nav_msgs/Path.h>


/** \brief Class for drone visualization
 * */
class Drone
{
  public:
    void publishMarker();
    Drone(int drone_id);
    ~Drone();

  private:
    ros::NodeHandle *nh_;
    int drone_id_;
    ros::Subscriber drone_sub_;
    ros::Subscriber action_status_sub_;
    ros::Subscriber tf_topic_;
    ros::Subscriber drone_trajectory_sub;
    ros::Subscriber gimbal_status_sub_;
    ros::Subscriber gimbal_sub_;
    ros::Publisher marker_robot_pub_;
    ros::Publisher marker_id_pub_;
    ros::Publisher action_status_pub_;
    ros::Publisher action_id_pub_;
    ros::Publisher path_pub;
    ros::Publisher path_rviz_pub;
    geometry_msgs::PoseStamped drone_pose_;
    ros::Publisher gimbal_odometry_pub_;
    int action_status_ = 0;
    std::string action_id_ = "";
    void dronePoseReceived(const geometry_msgs::PoseStamped::ConstPtr &uav_pose);
    void actionStatusCallback(const multidrone_msgs::ActionStatus::ConstPtr &_msg);
    void gimbalStatusCallback(const multidrone_msgs::GimbalStatus::ConstPtr &_msg);
    void tfCallback(const tf2_msgs::TFMessage::ConstPtr &_msg);
    nav_msgs::Odometry gimbal_odometry_;
    geometry_msgs::PointStamped odom_;
    ros::Time begin;
    std::vector<geometry_msgs::PoseStamped> path;
};

/** \brief class for visualizing no fly zones
 */
class NoFlyZones
{
  public:
    NoFlyZones(); /*!constructor*/
    ~NoFlyZones(); /*! destructor*/

  private:
  ros::NodeHandle *nh_;
  ros::Publisher path_rviz_pub;

};

/** \brief class for visualizing the target */
class Target
{
  public:
    Target(int target_id);
    ~Target();
    void publishMarker(); // publish the topic to visualize in rviz
    void targetarrayCallback(const multidrone_msgs::TargetStateArray::ConstPtr& _msg); // callback to the target pose
    void targetarrayFilterCallback(const multidrone_msgs::TargetStateArray::ConstPtr& _msg); // callback to the target pose filter

  private:
    ros::NodeHandle *nh_;
    void targetPose(const geometry_msgs::PoseStamped::ConstPtr &msg);
    ros::Subscriber target_sub_;
    ros::Subscriber target_array_sub_;
    ros::Subscriber target_array_filter_sub_;
    ros::Publisher  target_marker_pub_;
    ros::Publisher  target_marker_filter_pub_;
    ros::Publisher  target_marker_filter_id_pub_;

    ros::Publisher target_id_pub_;
    geometry_msgs::PoseStamped target_pose_;
    geometry_msgs::PoseStamped target_pose_filter_;

};