#include <visualizer.h>
/** \brief Constructor of the drone class
 */
Drone::Drone(int drone_id)
{
    // subscriptions/publications
    nh_ = new ros::NodeHandle();
    drone_id_ = drone_id;
    begin = ros::Time::now();
    std::string uav_topic_name = "drone_" + std::to_string(drone_id_) + "/ual/pose";
    drone_sub_ = nh_->subscribe<geometry_msgs::PoseStamped>(uav_topic_name.c_str(), 1, &Drone::dronePoseReceived, this);
    // gimbal_sub_ = nh_->subscribe<geometry_msgs::Vector3>("drone_" + std::to_string(drone_id_) + "/",0);
    marker_id_pub_ = nh_->advertise<visualization_msgs::Marker>("drone_" + std::to_string(drone_id_) + "/id",0);
    marker_robot_pub_ = nh_->advertise<visualization_msgs::Marker>("drone_" + std::to_string(drone_id) + "/robot", 0);
    action_id_pub_ = nh_->advertise<visualization_msgs::Marker>("drone_" + std::to_string(drone_id) + "/action_id_marker", 0);
    gimbal_odometry_pub_ = nh_->advertise<nav_msgs::Odometry>("drone_" + std::to_string(drone_id) + "/gimbal_marker", 0);
    action_status_sub_ = nh_->subscribe<multidrone_msgs::ActionStatus>("drone_" + std::to_string(drone_id) + "/action_status", 10, &Drone::actionStatusCallback, this); // Change for each drone ID
    gimbal_status_sub_ = nh_->subscribe<multidrone_msgs::GimbalStatus>("drone_" + std::to_string(drone_id) + "/gimbal/status", 10, &Drone::gimbalStatusCallback, this); // Change for each drone ID
    tf_topic_ = nh_-> subscribe<tf2_msgs::TFMessage>("/tf",1,&Drone::tfCallback,this);
    path_pub = nh_->advertise<nav_msgs::Path>("drone_" + std::to_string(drone_id) + "/Path", 0);
    path_rviz_pub = nh_->advertise<nav_msgs::Path>("drone_" + std::to_string(drone_id) + "/solver_path",1);


}
/**\brief Drone class destructor
 */
Drone::~Drone() {}

/** the drone tf callbacks
 */
void Drone::tfCallback(const tf2_msgs::TFMessage::ConstPtr &_msg){
    for(int i = 0; i< _msg->transforms.size();i++){
        if(_msg->transforms[i].header.frame_id =="drone_"+std::to_string(drone_id_)+"/base_link"){
            drone_pose_.pose.position.x =odom_.point.x+_msg->transforms[i].transform.translation.x;
            drone_pose_.pose.position.y =odom_.point.y+_msg->transforms[i].transform.translation.y;
            drone_pose_.pose.position.z =odom_.point.z+_msg->transforms[i].transform.translation.z;
        }
        if(_msg->transforms[i].header.frame_id =="drone_"+std::to_string(drone_id_)+"/odom"){
            drone_pose_.pose.position.x  =_msg->transforms[i].transform.translation.x;
            drone_pose_.pose.position.y = _msg->transforms[i].transform.translation.y;
            drone_pose_.pose.position.z =_msg->transforms[i].transform.translation.z;    
        }
    }   

}
void Drone::dronePoseReceived(const geometry_msgs::PoseStamped::ConstPtr &uav_pose)
{
    drone_pose_ = *uav_pose;
}

void Drone::actionStatusCallback(const multidrone_msgs::ActionStatus::ConstPtr &_msg)
{
    action_status_ = _msg->status;
    action_id_ = _msg->action_id;
   
}

void Drone::gimbalStatusCallback(const multidrone_msgs::GimbalStatus::ConstPtr &_msg){

    gimbal_odometry_.pose.pose.position = drone_pose_.pose.position;
    gimbal_odometry_.pose.pose.orientation = _msg->orientation;
    gimbal_odometry_.header.frame_id = "map";
}

void Drone::publishMarker()
{
    // gimbal marker
    visualization_msgs::Marker marker_gimbal;
    marker_gimbal.header.frame_id = "/map";
    marker_gimbal.header.stamp = ros::Time();
    marker_gimbal.id = drone_id_;
    marker_gimbal.ns = "uavs";
    marker_gimbal.type = visualization_msgs::Marker::ARROW;
    marker_gimbal.color.a = 1;
    marker_gimbal.action = visualization_msgs::Marker::ADD;
    marker_gimbal.pose = drone_pose_.pose;
    marker_gimbal.scale.x = 0.001;
    marker_gimbal.scale.y = 0.001;
    marker_gimbal.scale.z = 0.001;
    std::vector<geometry_msgs::PointStamped> points;
    geometry_msgs::PointStamped start_point;
    start_point.point.x = drone_pose_.pose.position.x;
    start_point.point.y = drone_pose_.pose.position.y;
    start_point.point.z = drone_pose_.pose.position.z;
    
    points.push_back(start_point);
    marker_gimbal.mesh_use_embedded_materials = true;

    // robot marker
    visualization_msgs::Marker marker_robot;
    marker_robot.header.frame_id = "/map";
    marker_robot.header.stamp = ros::Time();
    marker_robot.id = drone_id_;
    marker_robot.ns = "uavs";
    marker_robot.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_robot.mesh_resource = "package://robots_description/models/mbzirc/meshes/multirotor.dae";
    marker_robot.color.a = 1;
    marker_robot.action = visualization_msgs::Marker::ADD;
    ros::Duration difference;
    difference = ros::Time::now()-begin;
    marker_robot.pose= drone_pose_.pose;
    marker_robot.scale.x = 0.003;
    marker_robot.scale.y = 0.003;
    marker_robot.scale.z = 0.003;
    marker_robot.mesh_use_embedded_materials = true;

    // robot id marker
    visualization_msgs::Marker marker_robot_id;
    marker_robot_id.header.frame_id = "/map";
    marker_robot_id.header.stamp = ros::Time();
    marker_robot_id.id = drone_id_;
    marker_robot_id.ns = "uavs_state";
    marker_robot_id.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_robot_id.text = "Drone " + std::to_string(drone_id_);
    marker_robot_id.pose.position.z = drone_pose_.pose.position.z;
    marker_robot_id.pose.position.y = drone_pose_.pose.position.y-0.6;
    marker_robot_id.pose.position.x = drone_pose_.pose.position.x-3;
    marker_robot_id.color.a = 1;
    marker_robot_id.scale.x = 3;
    marker_robot_id.scale.y = 3;
    marker_robot_id.scale.z = 3;
    marker_robot_id.mesh_use_embedded_materials = true;

    // action status marker

    std::string status_text;
    switch (action_status_)
    {
        case 0:
         status_text = "\nIdle";
            break;
        case 1:
        //  status_text = drone_id_!=2 ? " Waiting for\ndirector event" : "Waiting for\n  relay";
         status_text = " Waiting for\ndirector event";
            break;
        case 2:
        status_text = "\nTaking off";
            break;
        case 3:
         status_text = "\nLanding";
            break;
        case 4:
         status_text = "Going to\nstart pose";
            break;
        case 5:
         status_text = "\nGoing home";
            break;
        case 6:
         status_text = "Shooting action\n   running";
            break;
        case 7:
         status_text = "\nEmergency";
            break;
        case 8:
         status_text = "Waiting for\nsafe to go";
            break;
        case 9:
         status_text = "Waiting for\nget ready";
            break;
        default:
            break;
    }
  

    // action id marker
    visualization_msgs::Marker marker_action_id;
    marker_action_id.header.frame_id = "/map";
    marker_action_id.header.stamp = ros::Time();
    marker_action_id.id = drone_id_;
    marker_action_id.ns = "uavs_state";
    marker_action_id.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_action_id.text = status_text;
    marker_action_id.pose.position.z = drone_pose_.pose.position.z;
    marker_action_id.pose.position.y = drone_pose_.pose.position.y+0.8;
    marker_action_id.pose.position.x = drone_pose_.pose.position.x+4;
    marker_action_id.color.a = 1;
    marker_action_id.scale.x = 3;
    marker_action_id.scale.y = 3;
    marker_action_id.scale.z = 3;
    marker_action_id.mesh_use_embedded_materials = true;

    switch (drone_id_)
    {
    case 1:
        // orange
        marker_action_id.color.r = 0.0;
        marker_action_id.color.g = 0;
        marker_action_id.color.b = 0;

        marker_robot_id.color.r = 0.0;
        marker_robot_id.color.g = 0;
        marker_robot_id.color.b = 0;

        marker_robot.color.r = 0.0;
        marker_robot.color.g = 0.0;
        marker_robot.color.b = 0.0;

        break;
    case 4:
        // orange
        marker_action_id.color.r = 1.0;
        marker_action_id.color.g = 0.0;
        marker_action_id.color.b = 0.0;

         marker_robot_id.color.r = 1.0;
        marker_robot_id.color.g = 0.0;
        marker_robot_id.color.b = 0.0;

        marker_robot.color.r = 1.0;
        marker_robot.color.g = 0.0;
        marker_robot.color.b = 0.0;
        break;
    case 2:
        // ingigo
        marker_action_id.color.r = 0.0;
        marker_action_id.color.g = 0.0;
        marker_action_id.color.b = 1;


        marker_robot_id.color.r = 0.0;
        marker_robot_id.color.g = 0.0;
        marker_robot_id.color.b = 1;

        marker_robot.color.r = 0.0;
        marker_robot.color.g = 0.0;
        marker_robot.color.b = 1.0;
        break;
    case 3:
        // zinc yellow
        marker_action_id.color.r = 1;
        marker_action_id.color.g = 0;
        marker_action_id.color.b = 0;


        marker_robot_id.color.r = 1;
        marker_robot_id.color.g = 0;
        marker_robot_id.color.b = 0;

        marker_robot.color.r = 1.0;
        marker_robot.color.g = 0.0;
        marker_robot.color.b = 0.0;
        break;
    }
    path.push_back(drone_pose_);
    nav_msgs::Path path_to_publish;
    path_to_publish.poses = path;
    path_to_publish.header.frame_id = "map";
    path_pub.publish(path_to_publish);
    action_id_pub_.publish(marker_action_id);
    marker_robot_pub_.publish(marker_robot);
    marker_id_pub_.publish(marker_robot_id);
    gimbal_odometry_pub_.publish(gimbal_odometry_);

}


Target::Target(int marker_robot_id){
    nh_ = new ros::NodeHandle();
    target_sub_ =  nh_->subscribe<geometry_msgs::PoseStamped>("/target_1/target_pose", 1, &Target::targetPose,this);
    target_array_sub_  = nh_->subscribe<multidrone_msgs::TargetStateArray>("/targets_pose", 1, &Target::targetarrayCallback, this);
    target_array_filter_sub_  = nh_->subscribe<multidrone_msgs::TargetStateArray>("/target_3d_state", 1, &Target::targetarrayFilterCallback, this);
    target_marker_pub_ = nh_->advertise<visualization_msgs::Marker>("target/marker", 0);
    target_marker_filter_pub_ = nh_->advertise<visualization_msgs::Marker>("target_filter/marker", 0);
    target_id_pub_ = nh_->advertise<visualization_msgs::Marker>("target/id", 0);
    target_marker_filter_id_pub_ = nh_->advertise<visualization_msgs::Marker>("target_filter/id", 0);
}
Target::~Target(){}

void Target::targetarrayFilterCallback(const multidrone_msgs::TargetStateArray::ConstPtr& _msg){
    target_pose_filter_.pose = _msg->targets.at(0).pose.pose;
} // real target callback

void Target::targetarrayCallback(const multidrone_msgs::TargetStateArray::ConstPtr& _msg){
    target_pose_.pose = _msg->targets.at(0).pose.pose;
} // real target callback

/** \brief Callback to receive UAVs poses
*/
void Target::targetPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    
}

void Target::publishMarker(){
    // Publish uav cylinder
    visualization_msgs::Marker target_marker;
    target_marker.header.frame_id = "/map";
    target_marker.header.stamp = ros::Time();
    target_marker.id = 1;
    target_marker.ns = "target";
    target_marker.type = visualization_msgs::Marker::CUBE;
    target_marker.text = "1";
    target_marker.pose.position.z = target_pose_.pose.position.z;
    target_marker.pose.position.y = target_pose_.pose.position.y;
    target_marker.pose.position.x = target_pose_.pose.position.x;
    target_marker.pose.orientation.x = target_pose_.pose.orientation.x;
    target_marker.pose.orientation.y = target_pose_.pose.orientation.y;
    target_marker.pose.orientation.z = target_pose_.pose.orientation.z;
    target_marker.pose.orientation.w = target_pose_.pose.orientation.w;
    // orange
    target_marker.color.r = 1.0;
    target_marker.color.g = 0.0;
    target_marker.color.b = 0.0;
    target_marker.color.a = 1;
    target_marker.scale.x = 1.5;
    target_marker.scale.y = 1.5;
    target_marker.scale.z = 1.5;
    target_marker.mesh_use_embedded_materials = true;
    visualization_msgs::Marker marker_robot_id;
    marker_robot_id.header.frame_id = "/map";
    marker_robot_id.header.stamp = ros::Time();
    marker_robot_id.id = 1;
    marker_robot_id.ns = "uavs_state";
    marker_robot_id.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_robot_id.text = "Target";
    marker_robot_id.pose.position.z = target_pose_.pose.position.z;
    marker_robot_id.pose.position.y = target_pose_.pose.position.y+0.6;
    marker_robot_id.pose.position.x = target_pose_.pose.position.x+3;
    marker_robot_id.color.a = 1;
    marker_robot_id.color.r = 1.0;
    marker_robot_id.color.g = 0.0;
    marker_robot_id.color.b = 0.0;
    marker_robot_id.scale.x = 3;
    marker_robot_id.scale.y = 3;
    marker_robot_id.scale.z = 3;
    marker_robot_id.mesh_use_embedded_materials = true;

    // target filter
    
    visualization_msgs::Marker target_filter_marker;
    target_filter_marker.header.frame_id = "/map";
    target_filter_marker.header.stamp = ros::Time();
    target_filter_marker.id = 1;
    target_filter_marker.ns = "target";
    target_filter_marker.type = visualization_msgs::Marker::CUBE;
    target_filter_marker.text = "1";
    target_filter_marker.pose.position.z = target_pose_filter_.pose.position.z;
    target_filter_marker.pose.position.y = target_pose_filter_.pose.position.y;
    target_filter_marker.pose.position.x = target_pose_filter_.pose.position.x;
    // orange
    target_filter_marker.color.r = 0.0;
    target_filter_marker.color.g = 1.0;
    target_filter_marker.color.b = 0.0;
    target_filter_marker.color.a = 1;

    target_filter_marker.scale.x = 1;
    target_filter_marker.scale.y = 1;
    target_filter_marker.scale.z = 1;
    target_filter_marker.mesh_use_embedded_materials = true;

    visualization_msgs::Marker marker_target_filter_id;
    marker_target_filter_id.header.frame_id = "/map";
    marker_target_filter_id.header.stamp = ros::Time();
    marker_target_filter_id.id = 1;
    marker_target_filter_id.ns = "uavs_state";
    marker_target_filter_id.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_target_filter_id.text = "Target filter";
    marker_target_filter_id.pose.position.z = target_pose_filter_.pose.position.z;
    marker_target_filter_id.pose.position.y = target_pose_filter_.pose.position.y;
    marker_target_filter_id.pose.position.x = target_pose_filter_.pose.position.x + 3;
    marker_target_filter_id.color.a = 1;
    marker_target_filter_id.color.r = 0.0;
    marker_target_filter_id.color.g = 1.0;
    marker_target_filter_id.color.b = 0.0;
    marker_target_filter_id.scale.x = 1;
    marker_target_filter_id.scale.y = 1;
    marker_target_filter_id.scale.z = 1;
    marker_target_filter_id.mesh_use_embedded_materials = true;
    target_marker_filter_id_pub_.publish(marker_target_filter_id);
    target_marker_filter_pub_.publish(target_filter_marker);
    target_marker_pub_.publish(target_marker);
    target_id_pub_.publish(marker_robot_id);
}

NoFlyZones::NoFlyZones(){
    nh_ = new ros::NodeHandle();
    std::vector<double> wps_x = {0,60,60,0,0}; 
    std::vector<double> wps_y = {5,5,-50,-50,5};
    std::vector<double> wps_z = {2,2,2,2};
    nav_msgs::Path msg;
    path_rviz_pub = nh_->advertise<nav_msgs::Path>("/noflyzones",0);
    std::vector<geometry_msgs::PoseStamped> poses(wps_x.size());
    msg.header.frame_id = "map";
    for (int i = 0; i < wps_x.size(); i++) {
        poses.at(i).pose.position.x = wps_x[i];
        poses.at(i).pose.position.y = wps_y[i];
        poses.at(i).pose.position.z = wps_z[i];
        poses.at(i).pose.orientation.x = 0;
        poses.at(i).pose.orientation.y = 0;
        poses.at(i).pose.orientation.z = 0;
        poses.at(i).pose.orientation.w = 1;
    }
    msg.poses = poses;
    ROS_INFO("publishing topic");
    sleep(15);
    path_rviz_pub.publish(msg);
}

NoFlyZones::~NoFlyZones(){}

