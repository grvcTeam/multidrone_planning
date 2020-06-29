#ifndef TRAILER_H
#define TRAILER_H

#include <ros/ros.h>
#include <vector>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_eigen/tf2_eigen.h>

// ROS types
#include <std_srvs/Empty.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamGet.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Int32.h>
#include "tf/transform_listener.h"
#include <tf2_msgs/TFMessage.h>

// MULTIDRONE types
#include <uav_abstraction_layer/ual.h>
#include <uav_abstraction_layer/GoToWaypoint.h>
#include <uav_abstraction_layer/TakeOff.h>
#include <uav_abstraction_layer/Land.h>
#include <uav_abstraction_layer/State.h>
#include <multidrone_msgs/ShootingAction.h>
#include <multidrone_msgs/ShootingType.h>
#include <multidrone_msgs/DroneAction.h>
#include <multidrone_msgs/GimbalStatus.h>
#include <multidrone_msgs/ExecuteAction.h>  // Note: "Action" is appended
#include <multidrone_msgs/FollowTarget.h>
#include <multidrone_msgs/SetFramingType.h>
#include <multidrone_msgs/ManualControls.h>
#include <multidrone_msgs/CameraControl.h>
#include <multidrone_msgs/TargetStateArray.h>
#include <multidrone_msgs/TargetIdentifierType.h>
#include <multidrone_msgs/ActionStatus.h>
#include <multidrone_msgs/CameraStatus.h>
#include <action_executor/TargetFeedbackMode.h>
#include <actionlib/server/simple_action_server.h>

#define TRAILER_TYPE_INERTIAL           0
#define TRAILER_TYPE_CONSTANT           1
#define TRAILER_TYPE_ORBIT              2
#define TRAILER_TYPE_VARIABLE           3
#define TRAILER_TYPE_ORBIT_GIMBAL       4

class Trailer {

  public:
    int trailer_type;
    std::vector<double>            t;              // current time
    std::vector<double>            Ts;             // current time
    double                         t_0;            // initial time step
    double                         a;              // current trailer heading
    double                         a_;             // last trailer heading
    double                         d;              // trailer length
    double                         w_trailer;      // trailer angular velocity
    double                         v_trailer;      // trailer linear velocity
    Eigen::Matrix2d                R;              // rotation matrix with current trailer heading
    Eigen::Matrix2d                R_;             // rotation matrix with last trailer heading
    Eigen::Matrix2d                R_leader;       // rotation matrix with leader heading
    Eigen::Vector2d                v_leader;       // leader linear velocity
    Eigen::Vector2d                r;              // displacement vector
    Eigen::Vector2d                r_dot;          // displacement vector
    Eigen::Vector2d                p;              // current trailer position
    Eigen::Vector2d                p_;             // last trailer position
    Eigen::Vector2d                vehicle_pos_;   // 2D vehicle position
    Eigen::Vector2d                _vehicle_pos;   // last 2D vehicle position
    Eigen::Vector2d                desired_point_; // desired point;
    Eigen::Vector2d                v_trailer_;
    Eigen::Vector2d                v_desired;
    Eigen::Vector2d                v_target;
    std::vector<Eigen::Vector3d>   _target_pos_;
    std::vector<Eigen::Vector3d>   _target_pos_s;

    Eigen::Matrix<double, 2, 3> I_1_2;
    Eigen::Matrix2d S_w_trailer;

    Eigen::Vector2d originOfFormation_;
    
    double altitude         = 0;  // desired altitude
    double altitude_        = 0;  // last altitude
    double altitude_dot     = 0;  // altitude rate
    double rx               = 0;  // displacement in x direction
    double ry               = 0;  // displacement in y direction
    double rx_              = 0;  // last displacement in x direction
    double ry_              = 0;  // last displacement in y direction
    double rx_dot           = 0;  // linear velocity in x direction
    double ry_dot           = 0;  // linear velocity in y direction
    double rx_e             = 0;  // trailer final desired position in x direction
    double ry_e             = 0;  // trailer final desired position in y direction
    double z_e              = 0;  // trailer final desired position in z direction
    double rx_s             = 0;  // trailer initial desired position in x direction
    double z_s              = 0;  // trailer initial desired position in z direction
    double initial_azimuth  = 0;  // initial azimuth relative to the target heading (orbit)
    double azimuth          = 0;  // current azimuth relative to the target heading (orbit)
    double azimuth_         = 0;  // last azimuth relative to the target heading (orbit)
    double radius           = 0;  // 2D distance to the target (orbit)
    double angular_speed    = 0;  // angular speed around the target (orbit) [deg/s]
    uint8_t filter          =10;  // order of filter of target position
    double r_safety         = 2;  // safety radius around target
    double area             = 0;  // safety area around target

    void init(double _vehicle_yaw, Eigen::Vector3d _vehicle_pos);
    void run_trailer(Eigen::Vector3d _target_pos);
};

#endif // TRAILER_H