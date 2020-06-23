#include <action_executer/action_executer.h>
#include <chrono>
#include <thread>
#include <boost/bind.hpp>
#include <fstream>


#define SHOOT_TYPE_DEFAULT              0
#define EXECUTER_IDLE                 200
#define K_GIMBAL                      1.0
#define K_DRONE                       1.0
#define K_DRONE_YAW                   0.3
#define GIMBAL_FREQ                   0.1
#define TRAILER_FREQ                  0.1 
#define MAIN_FREQ                   0.033
#define RT_FREQ                     0.033
#define TO_DEG                   180/M_PI
#define TO_RAD                   M_PI/180
#define MAX_DRONE_SPEED                 1
#define K1_GIMBAL                     1.0
#define K2_GIMBAL                     0.5
#define K_OFFSET                      2.0

// Constructor
Executer::Executer(int _argc, char** _argv)
{
  ros::init(_argc, _argv, "Executer");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param<int>("drone_id", drone_id_, 1);
  pnh.param<float>("k1_gimbal", k_1,     K1_GIMBAL);
  pnh.param<float>("k2_gimbal", k_2,     K2_GIMBAL);
  pnh.param<float>("k_offset", k_offset, K_OFFSET);
  pnh.param<int>("image_width", w_im, 640);
  pnh.param<int>("image_height",h_im, 480);
  pnh.param<float>("MAX_YAW_RATE", MAX_YAW_RATE, 0.2);
  pnh.param<double>("trailer", trailer_size, 0.5);
  std::vector<double> p_C_B_aux;
  pnh.getParam("p_C_B",p_C_B_aux);
  p_C_B.x() = p_C_B_aux[0];
  p_C_B.y() = p_C_B_aux[1];
  p_C_B.z() = p_C_B_aux[2];
  pnh.param<bool>("simulation",simulation,true);
  pnh.param<bool>("onboard",onboard, true);
  pnh.param<bool>("offset", offset, false); 
  pnh.param<bool>("f_target", f_target, false); 

  target_image_offset_x_MAX = w_im/2;
  target_image_offset_x_MIN = -target_image_offset_x_MAX;
  target_image_offset_x_13  = w_im/3-w_im/2;
  target_image_offset_x_23  = w_im*2/3-w_im/2;
  target_image_offset_y_MAX = h_im/2;
  target_image_offset_y_MIN = -target_image_offset_y_MAX;
  target_image_offset_y_13  = h_im/3-h_im/2;
  target_image_offset_y_23  = h_im*2/3-h_im/2;

  ROS_INFO("Setting up Executer %d", drone_id_);

  // Subscribers
  ros::Subscriber target_array_sub_;
  if (onboard)
    target_array_sub_                          = nh.subscribe<multidrone_msgs::TargetStateArray>("/target_3d_state", 1, &Executer::targetarrayCallback, this);
  else
    target_array_sub_                          = nh.subscribe<multidrone_msgs::TargetStateArray>("/targets_pose", 1, &Executer::targetarrayCallback, this);
  ros::Subscriber drone_pose_sub               = nh.subscribe<geometry_msgs::PoseStamped>("ual/pose", 1, &Executer::dronePose, this);
  ros::Subscriber drone_vel_sub                = nh.subscribe<geometry_msgs::TwistStamped>("ual/velocity",1, &Executer::droneVelocity, this);
  ros::Subscriber drone_state_sub              = nh.subscribe<uav_abstraction_layer::State>("ual/state",1, &Executer::droneState, this);
  ros::Subscriber gimbal_status_sub            = nh.subscribe<multidrone_msgs::GimbalStatus>("gimbal/status",1, &Executer::gimbalStatus, this);
  
  ros::Subscriber focus_sub_;
  if(f_target)
    focus_sub_                                 = nh.subscribe<std_msgs::Int32>("visual_analysis/focus_value_target", 1, &Executer::focusCallback, this);
  else
    focus_sub_                                 = nh.subscribe<std_msgs::Int32>("visual_analysis/focus_value_image", 1, &Executer::focusCallback, this);

  // ###########  Communication with SWAP  ########## //
  ros::Subscriber confl_warning_sub_           = nh.subscribe("collision_warning", 1, &Executer::collisionWarningCallback, this);
  wished_mov_dir_pub_                          = nh.advertise<geometry_msgs::Vector3>("wished_movement_direction",1 , true);  // the final true is required
  ros::Subscriber avoid_mov_dir_sub_           = nh.subscribe("avoid_movement_direction", 1, &Executer::avoidMovementCallback, this);
  // ###########  #######################  ########### //

  // Publishers
  gimbal_cmd_basecam_pub                       = nh.advertise<geometry_msgs::Vector3>("gimbal/cmd",1);
  go_to_waypoint_pub_                          = nh.advertise<geometry_msgs::PoseStamped>("ual/set_pose",1);
  set_velocity_pub_                            = nh.advertise<geometry_msgs::TwistStamped>("ual/set_velocity",1);
  trajectory_                                  = nh.advertise<nav_msgs::Odometry>("desired_trajectory",1);
  lyapunov_                                    = nh.advertise<geometry_msgs::Point32>("gimbal_lyapunov",1);
  pixel_publish                                = nh.advertise<geometry_msgs::Vector3>("pixel_position",1);

  // Service Client
  cmd_long_client_                             = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");
  set_param_client_                            = nh.serviceClient<mavros_msgs::ParamSet>("mavros/param/set");
  get_param_client_                            = nh.serviceClient<mavros_msgs::ParamGet>("mavros/param/get");
  take_off_client_                             = nh.serviceClient<uav_abstraction_layer::TakeOff>("ual/take_off");
  go_to_waypoint_client_                       = nh.serviceClient<uav_abstraction_layer::GoToWaypoint>("ual/go_to_waypoint");
  land_client_                                 = nh.serviceClient<uav_abstraction_layer::Land>("ual/land");
  follow_target_client_                        = nh.serviceClient<multidrone_msgs::FollowTarget>("follow_target");
  set_framing_type_client_                     = nh.serviceClient<multidrone_msgs::SetFramingType>("visual_analysis/set_framing_type");
  camera_control_client_                       = nh.serviceClient<multidrone_msgs::CameraControl>("camera_control");

  // Service Server
  ros::ServiceServer manual_controls_service_  = nh.advertiseService("manual_controls", &Executer::ManualControlServiceCallback, this);

  if(simulation) {
    std::vector<double> camera_matrix;
    double cols, rows;
    pnh.getParam("camera_matrix/rows",rows);
    pnh.getParam("camera_matrix/cols",cols);
    camera_matrix.reserve(cols*rows);
    pnh.getParam("camera_matrix/data",camera_matrix);

    K  <<  K_min ,     0.000000,   319.5,    // BMMCC camera_matrix minimum zoom
          0.000000,      K_min,   239.5,
          0.000000,     0.000000, 1.000;

    ROS_INFO("Executer [%d] initialized in simulation mode!", drone_id_);
    // Initialise gimbal cmd
    #ifdef MAVROS_VERSION_BELOW_0_25_0
    gimbal_cmd_.request.command = mavros_msgs::CommandCode::CMD_DO_MOUNT_CONTROL;
    #else
    gimbal_cmd_.request.command = mavros_msgs::CommandCode::DO_MOUNT_CONTROL;
    #endif
    gimbal_cmd_.request.confirmation = false;
    gimbal_cmd_.request.param7 = 2; // MAV_MOUNT_MODE::MAV_MOUNT_MODE_MAVLINK_TARGETING

    p_C_B << -0.051, 0, 0.162; // gimbal position with respect to drone CG

    // Set MNT_MODE parameters on FCU for gimbal control via Mavlink
    setMountModeParameters();

  } 
  else{

    K  <<  K_min ,     0.000000,   319.5,    // BMMCC camera_matrix minimum zoom
          0.000000,      K_min,   239.5,
          0.000000,     0.000000, 1.000;

    ROS_INFO("Executer [%d] initialized in real mode", drone_id_);    
  }

  multidrone_msgs::ManualControls manual_control_msg_;
  manual_control_msg_.request.control = multidrone_msgs::ManualControls::Request::CAMERA_RESET;
  ManualControlServiceCallback(manual_control_msg_.request, manual_control_msg_.response);
  

  w_est_ << 0.0, 0.0, 0.0;
  w_est << 0.0, 0.0, 0.0;
  
  R_S_I <<    1,  0,  0,
              0, -1,  0,
              0,  0, -1;

  std::cout << "Gimbal position is \n" << p_C_B << "\n\n";
  
  // Wait until having initial drone pose
  while (!has_drone_pose_ || !has_drone_vel_ && ros::ok()) { 
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    ros::spinOnce();
  }

  // action service
  server_ = new Server(nh, "action_server", false);
  server_->registerGoalCallback(boost::bind(&Executer::actionCallback, this));
  server_->registerPreemptCallback(boost::bind(&Executer::preemptCallback, this));

  server_->start();

  t_0 = ros::Time::now().toSec();

  // start Timer
  timer_        = nh.createTimer(ros::Duration(MAIN_FREQ), &Executer::timerCallback, this); //33Hz
  timer_trailer = nh.createTimer(ros::Duration(TRAILER_FREQ), &Executer::timerCallbackTrailer, this, false, false); //10Hz
  timer_gimbal  = nh.createTimer(ros::Duration(GIMBAL_FREQ), &Executer::timerCallbackGimbal, this, false, false); //5Hz
  timer_rt      = nh.createTimer(ros::Duration(RT_FREQ), &Executer::referenceTrajectory, this, false, false); //33Hz

  ROS_INFO("Executer [%d] running", drone_id_);

  ros::MultiThreadedSpinner spinner(4); // Use 4 threads
  spinner.spin();
}

// Destructor
Executer::~Executer()
{
  delete server_;
};

// Timer callback
void Executer::timerCallback(const ros::TimerEvent&)
{
  lyapunov_publisher();
  
  if(confl_warning_){
    setpoint_vel_.twist.linear.x = avoid_mov_direction_.x;
    setpoint_vel_.twist.linear.y = avoid_mov_direction_.y;
    setpoint_vel_.twist.linear.z = 0;
    set_velocity_pub_.publish(setpoint_vel_);
  }
  else
  {
   switch (action_type_){
     case multidrone_msgs::DroneAction::TYPE_TAKEOFF:{
      timer_trailer.stop();
      takeOff();
      multidrone_msgs::ManualControls manual_control_msg_;
      manual_control_msg_.request.control = multidrone_msgs::ManualControls::Request::FOCUS_RESET;
      ManualControlServiceCallback(manual_control_msg_.request, manual_control_msg_.response);   
      break;
     }
    case multidrone_msgs::DroneAction::TYPE_LAND:     
      running_trailer = false;
      timer_trailer.stop();
      timer_gimbal.stop();
      timer_trajectory.stop();
      gimbalSafeMode();
      land();
      break;
    case multidrone_msgs::DroneAction::TYPE_GOTOWAYPOINT:
      timer_trailer.stop();
      timer_gimbal.stop();
      timer_trajectory.stop();
      gimbalSafeMode();
      running_trailer = false;
      feedback_.status= true;
      server_->publishFeedback(feedback_);
      goToWaypoint();
      break;
    case multidrone_msgs::DroneAction::TYPE_SHOOTING:
      if (!running_trailer) {                
        timer_gimbal.start();
        if (parameters_set){
          if (list_wp_.size() == 1 || list_wp_.size() == 0)
            orientation_0 = 0;
          else
            orientation_0 = atan2 (list_wp_[1].point.y - list_wp_[0].point.y, list_wp_[1].point.x - list_wp_[0].point.x);
          
          gimbalSafeMode();
          running_trailer = true;
          trailer.init(orientation_0,rt_target_pos_);
          timer_trailer.start();
          timer_trajectory.start();
          parameters_set = false;
          feedback_.status= true;
          server_->publishFeedback(feedback_);
          std::cout << "GOT A SA\n\n";
        }
      }
      break;
    default:
      if (running_trailer) {
        timer_trailer.stop();
        timer_gimbal.stop();
        timer_trajectory.stop();
        running_trailer = false;
        gimbalSafeMode();
      }
    }
  }
}

void Executer::timerCallbackGimbal(const ros::TimerEvent&)
{
  if (has_gimbal_status_ || simulation) {
    if (shooting_mode_ == multidrone_msgs::TargetIdentifierType::NONE)
      gimbalAutomatic(pan_s,tilt_s,pan_e,tilt_e, duration_.data.sec, start_time); 
    else
      gimbalController();     
  }else
      gimbalSafeMode();

   if(shooting_mode_ == multidrone_msgs::TargetIdentifierType::VISUAL || shooting_mode_ == multidrone_msgs::TargetIdentifierType::VISUAL_GPS) 
    if(has_visual_2D_target_status_)
      CameraControl();

  if (count < count_lim/4)
    K(0) = K_min;
  else if (count < count_lim/2)
    K(0) = (3*K_min+K_max)/4;
  else if (count < 3*count_lim/4)
    K(0) = (K_min+3*K_max)/4;
  else   
    K(0) = K_max;

  K(4) = K(0);  

  double t_now;
  t_now = ros::Time::now().toSec();
  if (t_now - time_of_last_gimbal_status > 1)
    has_gimbal_status_ = false;
  if (t_now - time_of_last_target_status > 1)
    has_shooting_target_status_ = false;
  if (t_now - time_of_last_target_visual_2D_status > 1)
    has_visual_2D_target_status_ = false;
  if (t_now - time_of_last_drone_pose > 1)
    has_drone_pose_ = false;
}

void Executer::timerCallbackTrailer(const ros::TimerEvent&)
{
  if (has_rt_target_status_){
    trailer.run_trailer(rt_target_pos_);
    followVehicle();
    trajectory_publisher();
  }
  has_rt_target_status_ = false;
}

void Executer::trajectory_publisher ()
{
  nav_msgs::Odometry _msg;
  desired_point << trailer.desired_point_, trailer.altitude;
  desired_vel << trailer.v_desired;

  _msg.header.stamp = ros::Time::now();
  _msg.header.frame_id = "map";
  _msg.child_frame_id = "drone_" + std::to_string(drone_id_) + "/base_link";
  _msg.pose.pose.position.x = desired_point[0];
  _msg.pose.pose.position.y = desired_point[1];
  _msg.pose.pose.position.z = trailer.altitude;
  _msg.pose.pose.orientation.x = 0;
  _msg.pose.pose.orientation.y = 0;
  _msg.pose.pose.orientation.z = 0;
  _msg.pose.pose.orientation.w = 1;
  _msg.twist.twist.linear.x  = desired_vel[0];
  _msg.twist.twist.linear.y  = desired_vel[1];
  _msg.twist.twist.linear.z  = 0;
  _msg.twist.twist.angular.x = 0;
  _msg.twist.twist.angular.y = 0;
  _msg.twist.twist.angular.z = setpoint_vel_.twist.angular.z;
  trajectory_.publish(_msg);
}

void Executer::lyapunov_publisher()
{
  geometry_msgs::Point32 _msg;
  _msg.x = lyapunov_visual;
  _msg.y = drone_yaw_;
  lyapunov_.publish(_msg);
}

void Executer::targetarrayCallback(const multidrone_msgs::TargetStateArray::ConstPtr& _msg) // real target callback
{
  
  //rt target
  if (rt_mode_ != multidrone_msgs::ShootingAction::RT_MODE_VIRTUAL_TRAJ){
    for (auto target:_msg->targets) {
      if (target.target_id == rt_id_) {
        tf::twistMsgToEigen(target.velocity.twist, rt_target_twist_);
      
        if (rt_mode_ == multidrone_msgs::ShootingAction::RT_MODE_ACTUAL_TARGET)
          tf2::fromMsg(target.pose.pose.position, rt_target_pos_);
        targetStatus(true);
      }
    }
  }
  //gimbal target
  if (shooting_mode_ == multidrone_msgs::TargetIdentifierType::GPS || shooting_mode_ == multidrone_msgs::TargetIdentifierType::VISUAL_GPS){
    for (auto target:_msg->targets) {
      if (target.target_id == shooting_id_) {
        tf2::fromMsg(target.pose.pose.position,shooting_target_pos_);
        tf2::fromMsg(target.pose.pose.orientation,shooting_target_att_);
        tf::twistMsgToEigen(target.velocity.twist, shooting_target_twist_);
        targetStatus(false);
      }
    }
  }
}

void Executer::referenceTrajectory(const ros::TimerEvent&){ 

  if (rt_mode_ == multidrone_msgs::ShootingAction::RT_MODE_VIRTUAL_TRAJ){
    rt_target_pos_.x()  = rt_target_pos_.x() + total_displ.x()*formation_speed_*RT_FREQ/(total_displ.norm());
    rt_target_pos_.y()  = rt_target_pos_.y() + total_displ.y()*formation_speed_*RT_FREQ/(total_displ.norm());
    rt_target_pos_.z()  = rt_target_pos_.z() + total_displ.z()*formation_speed_*RT_FREQ/(total_displ.norm());
    if (abs(rt_target_pos_.x()-list_wp_[0].point.x) >= abs(total_displ.x()) && abs(rt_target_pos_.y()-list_wp_[0].point.y) >= abs(total_displ.y()) && abs(rt_target_pos_.z()-list_wp_[0].point.z) >= abs(total_displ.z()))
      formation_speed_=0;
    targetStatus(true);
  }
  
  if (shooting_mode_ == multidrone_msgs::TargetIdentifierType::VIRTUAL){
    shooting_target_pos_.x()   = shooting_target_pos_.x() + total_displ.x()*formation_speed_*RT_FREQ/(total_displ.norm());
    shooting_target_pos_.y()   = shooting_target_pos_.y() + total_displ.y()*formation_speed_*RT_FREQ/(total_displ.norm());
    shooting_target_pos_.z()   = shooting_target_pos_.z() + total_displ.z()*formation_speed_*RT_FREQ/(total_displ.norm());
  
    desired_yaw = atan2 (total_displ.y() - list_wp_[0].point.y, total_displ.x() - list_wp_[0].point.x);
    shooting_target_att_.z()   = sin(desired_yaw/2);
    shooting_target_att_.y()   = 0;
    shooting_target_att_.x()   = 0;
    shooting_target_att_.w()   = cos(desired_yaw/2);
    shooting_target_twist_.x() = total_displ.x()*formation_speed_/(total_displ.norm());
    shooting_target_twist_.y() = total_displ.y()*formation_speed_/(total_displ.norm());
    shooting_target_twist_.z() = total_displ.z()*formation_speed_/(total_displ.norm());
    if (abs(shooting_target_pos_.x()-list_wp_[0].point.x) >= abs(total_displ.x()) && abs(shooting_target_pos_.y()-list_wp_[0].point.y) >= abs(total_displ.y()) && abs(shooting_target_pos_.z()-list_wp_[0].point.z) >= abs(total_displ.z()))
     formation_speed_=0;
    targetStatus(false);
  } 
}

void Executer::targetStatus(bool rt)
{
  if(rt)
    has_rt_target_status_ = true; 
  else{  
    p_T_I = R_S_I*shooting_target_pos_;
    if(shooting_mode_ == multidrone_msgs::TargetIdentifierType::GPS || shooting_mode_ == multidrone_msgs::TargetIdentifierType::VISUAL_GPS){
      geometry_msgs::Vector3 pixel_msg;
      q_I = p_T_I - p_B_I - R_B_I*p_C_B;
      q_C = R_C_I.transpose()*q_I;
                       
      q_C2.x() = q_C.y()/q_C.z();
      q_C2.y() = -q_C.x()/q_C.z();
      q_C2.z() = q_C.z()/q_C.z();
      pixel = K*q_C2;

      pixel_msg.x = pixel[0];
      pixel_msg.y = pixel[1];
      pixel_msg.z = pixel[2];

      pixel_publish.publish(pixel_msg);
    }
    has_shooting_target_status_ = true;
  }
}


void Executer::focusCallback(const std_msgs::Int32::ConstPtr& _msg)
{
  focus_value = _msg->data;
}

void Executer::run_w_estimate()
{
  Eigen::Vector3d w_est_dot;  

  dT_w = GIMBAL_FREQ;
  w_est_dot = k_2*w_e;
  w_est = w_est_+w_est_dot*dT_w;

  w_est_ = w_est;
}

// Drone pose callback
void Executer::dronePose(const geometry_msgs::PoseStamped::ConstPtr& _msg)
{
  tf::pointMsgToEigen(_msg->pose.position, drone_pos_);
  tf::quaternionMsgToEigen(_msg->pose.orientation, drone_att_);
  drone_yaw_ = atan2(2*(drone_att_.w()*drone_att_.z()+drone_att_.x()*drone_att_.y()),1-2*(drone_att_.y()*drone_att_.y()+drone_att_.z()*drone_att_.z()));

  R_B_S = drone_att_.normalized().toRotationMatrix();
  R_B_I = R_S_I*R_B_S;
  p_B_I = R_S_I*drone_pos_;

  time_of_last_drone_pose = ros::Time::now().toSec();
  has_drone_pose_ = true;
}

// Drone velocity callback
void Executer::droneVelocity(const geometry_msgs::TwistStamped::ConstPtr& _msg)
{

  tf::twistMsgToEigen(_msg->twist, drone_twist_);

  drone_vel_ << drone_twist_(0,0), drone_twist_(1,0), drone_twist_(2,0);
  drone_ang_vel_ << drone_twist_(3,0), drone_twist_(4,0), drone_twist_(5,0);

  p_B_I_dot = R_S_I*drone_vel_;
  w_B = drone_ang_vel_;

  has_drone_vel_ = true;
}

// Drone state callback
void Executer::droneState(const uav_abstraction_layer::State::ConstPtr& _msg)
{
  drone_state_.state = _msg->state;
}

// Gimbal status callback
void Executer::gimbalStatus(const multidrone_msgs::GimbalStatus::ConstPtr& _msg)
{

  tf::quaternionMsgToEigen(_msg->orientation, gimbal_att_);
  R_C_I = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitX())*gimbal_att_.normalized().toRotationMatrix();
 
  time_of_last_gimbal_status = ros::Time::now().toSec();
  has_gimbal_status_ = true;
}

bool Executer::ManualControlServiceCallback(multidrone_msgs::ManualControls::Request& req, multidrone_msgs::ManualControls::Response& res)
{
  multidrone_msgs::CameraControl camera_control_msg_;
  if (req.control == multidrone_msgs::ManualControls::Request::GIMBAL_MOVE_UP)
    target_image_offset_y -= target_image_offset_step;
  else if (req.control == multidrone_msgs::ManualControls::Request::GIMBAL_MOVE_DOWN)
    target_image_offset_y += target_image_offset_step;
  else if (req.control == multidrone_msgs::ManualControls::Request::GIMBAL_MOVE_LEFT)   
    target_image_offset_x -= target_image_offset_step;
  else if (req.control == multidrone_msgs::ManualControls::Request::GIMBAL_MOVE_RIGHT)
    target_image_offset_x += target_image_offset_step;
  else if (req.control == multidrone_msgs::ManualControls::Request::CAMERA_ZOOM_IN){
    if (count < 20){
      count++;
      camera_control_msg_.request.control_function = multidrone_msgs::CameraControl::Request::CONTROL_FUNCTION_ZOOM;
      camera_control_msg_.request.value = multidrone_msgs::CameraControl::Request::VALUE_INCREASE;
      camera_control_client_.call(camera_control_msg_);
    }
  }
  else if (req.control == multidrone_msgs::ManualControls::Request::CAMERA_ZOOM_OUT){
    if (count > 0){
      count--;
      camera_control_msg_.request.control_function = multidrone_msgs::CameraControl::Request::CONTROL_FUNCTION_ZOOM;
      camera_control_msg_.request.value = multidrone_msgs::CameraControl::Request::VALUE_DECREASE;
      camera_control_client_.call(camera_control_msg_);
    }
  }
  else if (req.control == multidrone_msgs::ManualControls::Request::CONTROL_FOCUS_INCREASE) {
    camera_control_msg_.request.control_function = multidrone_msgs::CameraControl::Request::CONTROL_FUNCTION_FOCUS;
    if(cmd < 212)
      cmd += inc;
    camera_control_msg_.request.value = cmd;     
    camera_control_client_.call(camera_control_msg_); 
  }   
  else if (req.control == multidrone_msgs::ManualControls::Request::CONTROL_FOCUS_DECREASE) {
    camera_control_msg_.request.control_function = multidrone_msgs::CameraControl::Request::CONTROL_FUNCTION_FOCUS;
    if(cmd > 44)
      cmd -= inc;
    camera_control_msg_.request.value = cmd;
    camera_control_client_.call(camera_control_msg_);
  }
  else if (req.control == multidrone_msgs::ManualControls::Request::CONTROL_RECORD_START) {
    if(!recording){
      camera_control_msg_.request.control_function = multidrone_msgs::CameraControl::Request::CONTROL_FUNCTION_RECORD;
      camera_control_msg_.request.value = multidrone_msgs::CameraControl::Request::VALUE_INCREASE;
      camera_control_client_.call(camera_control_msg_);
      recording = true;
    } else ROS_WARN("Already recording");
  }
  else if (req.control == multidrone_msgs::ManualControls::Request::CONTROL_RECORD_STOP) {

    if(recording){
      camera_control_msg_.request.control_function = multidrone_msgs::CameraControl::Request::CONTROL_FUNCTION_RECORD;
      camera_control_msg_.request.value = multidrone_msgs::CameraControl::Request::VALUE_DECREASE;
      camera_control_client_.call(camera_control_msg_);
      recording = false;
    } else ROS_WARN("Not recording");
  }
  else if (req.control == multidrone_msgs::ManualControls::Request::CAMERA_RESET) {
    camera_control_msg_.request.control_function = multidrone_msgs::CameraControl::Request::CAMERA_RESET;
    camera_control_client_.call(camera_control_msg_);
    count = 0;
  } else if (req.control == multidrone_msgs::ManualControls::Request::FOCUS_RESET) {
    focus_reset_tree(54,92);
  } else if (req.control == multidrone_msgs::ManualControls::Request::GIMBAL_RETURN_HOME) {
    gimbal_home = true;
  }

  if (target_image_offset_x > target_image_offset_x_MAX)
    target_image_offset_x = target_image_offset_x_MAX;
  else if (target_image_offset_x < target_image_offset_x_MIN)
    target_image_offset_x = target_image_offset_x_MIN;
  if (target_image_offset_y > target_image_offset_y_MAX)
    target_image_offset_y = target_image_offset_y_MAX;
  else if (target_image_offset_y < target_image_offset_y_MIN)
    target_image_offset_y = target_image_offset_y_MIN;
  
    return true;
}  

void Executer::focus_reset_tree(int beg, int end){
  std::cout << " Starting Auto Focus" << "\n";
  int BEST, low_left, high_left = -1, low_right = -1, high_right, curr, curr_left, curr_right;
  bool  stop_left, stop_right;
  multidrone_msgs::CameraControl camera_control_msg_;
  camera_control_msg_.request.control_function = multidrone_msgs::CameraControl::Request::CONTROL_FUNCTION_FOCUS;
  camera_control_msg_.request.manual_ctrl = false;
  f_reset = false;
  buffer[0] = -1;
  curr = (end+beg)/2 + (end+beg)%4;
  low_left = beg;
  high_right = end;

  camera_control_msg_.request.value = curr;
  camera_control_client_.call(camera_control_msg_);
  std::this_thread::sleep_for(std::chrono::milliseconds(400));

  BEST = curr;
  buffer[0] = focus_value;

  high_left = curr;
  low_right = curr;
    
  curr_left = (high_left+low_left)/2 - ((high_left+low_left)/2)%4;
  curr_right = (high_right+low_right)/2 + ((high_right+low_right)/2)%4;

  if(curr_left == curr){
    stop_left = true;
    buffer[1] = -1;
  }else stop_left = false;

  if(curr_right == curr){
    stop_right = true;
    buffer[2] = -1;
  }else stop_right = false;

  while(!stop_left || !stop_right){
    if(!stop_left){
      camera_control_msg_.request.value = curr_left;
      camera_control_client_.call(camera_control_msg_);
      std::this_thread::sleep_for(std::chrono::milliseconds(400));
      
      buffer[1] = focus_value;
      if(buffer[1] > buffer[0]){
        buffer[0] = buffer[1];
        BEST = curr_left;
      }
    }
    
    if(!stop_right){
      camera_control_msg_.request.value = curr_right;
      camera_control_client_.call(camera_control_msg_);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      
      buffer[2] = focus_value;
      if(buffer[2] > buffer[0]){
        buffer[0] = buffer[2];
        BEST = curr_right;
      }  
    }
    

    if(buffer[1] >= buffer[2]){
      curr = curr_left;
      high_right = curr;
    } else {  
      curr = curr_right;
      low_left = curr;
    }

    high_left = curr;
    low_right = curr;
    
    curr_left = (high_left+low_left)/2 - ((high_left+low_left)/2)%4;
    curr_right = (high_right+low_right)/2 + ((high_right+low_right)/2)%4;

    if(curr_left == curr){
      stop_left = true;
      buffer[1] = -1;
    } else stop_left = false;

    if(curr_right == curr){
      stop_right = true;
      buffer[2] = -1;
    } else stop_right = false;

  }


  camera_control_msg_.request.value = BEST;
  camera_control_client_.call(camera_control_msg_);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  f_reset = true;
  std::cout << " Done. Focus value is  " << BEST <<"\n";
}

void Executer::setMountModeParameters()
{
  mavros_msgs::ParamGet get_param_in, get_param_out;
  get_param_in.request.param_id = "MNT_MODE_IN";
  get_param_out.request.param_id = "MNT_MODE_OUT";

  mavros_msgs::ParamSet set_param;

  ros::Time last_request = ros::Time::now();
  while(ros::ok())
  {
    if((ros::Time::now() - last_request > ros::Duration(5.0)))
    {
      if(get_param_client_.call(get_param_in) && get_param_in.response.success && get_param_in.response.value.integer != 3)
      {
        set_param.request.param_id = "MNT_MODE_IN";
        set_param.request.value.integer = 3;
        set_param_client_.call(set_param);
      }

      if(get_param_client_.call(get_param_out) && get_param_out.response.success && get_param_out.response.value.integer != 0)
      {
        set_param.request.param_id = "MNT_MODE_OUT";
        set_param.request.value.integer = 0;
        set_param_client_.call(set_param);
      }

      if(get_param_in.response.value.integer == 3 && get_param_out.response.value.integer == 0)
        break;

      last_request = ros::Time::now();
    }
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
}

void Executer::toEulerAngles_YXZ(const Eigen::Quaterniond& _q, Eigen::Vector3d& euler)
{
  Eigen::Quaterniond q(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitX())*_q.normalized().toRotationMatrix());

  double t0 = -2.0 * (q.x() * q.y() - q.w() * q.z());
  double t1 = q.w()*q.w() - q.x()*q.x() + q.y()*q.y() - q.z()*q.z();
  double t2 = 2.0 * (q.y()*q.z() + q.w()*q.x());
  double t3 = -2.0 * (q.x()*q.z() - q.w()*q.y());
  double t4 = q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z();

  // 0
  euler.y() = atan2(t3, t4);

  // 1
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  euler.x() = asin(t2);

  // 2
  euler.z() = atan2(t0, t1);
}

void Executer::rotationMatrixToEulerAngles_XYZ(Eigen::Matrix3d& R, Eigen::Vector3d& euler){
  if (R(2,0) < 1) {
    if (R(2,0) > -1) {
      euler.y() = asin(-R(2,0));
      euler.z() = atan2(R(1,0),R(0,0));
      euler.x() = atan2(R(2,1),R(2,2));
    } else { // R(2,0) = -1
      // Not a unique solution: euler.x() - euler.z() = atan2(-R(1,2),R(1,1))
      euler.y() = M_PI/2;
      euler.z() = -atan2(-R(1,2),R(1,1));
      euler.x() = 0;
    }
  } else { // R(2,0) = 1
// Not a unique solution: euler.x() + euler.z() = atan2(-R(1,2),R(1,1))
    euler.y() = -M_PI/2;
    euler.z() = atan2(-R(1,2),R(1,1));
    euler.x() = 0;
  }
}

void Executer::preemptCallback()
{
goal_preempt_ = true;
}

void Executer::toSMatrix(Eigen::Vector3d& w, Eigen::Matrix3d& S)
{
  S <<      0,  -w.z(),   w.y(),
        w.z(),       0,  -w.x(),
       -w.y(),   w.x(),       0;
}

void Executer::toSInvMatrix(Eigen::Matrix3d& R__, Eigen::Vector3d& S_inv__)
{
  S_inv__ << R__(2,1), R__(0,2), R__(1,0);
}

void Executer::toPIMatrix(Eigen::Vector3d& rx, Eigen::Matrix3d& PI)
{
  PI = Eigen::Matrix3d::Identity() - rx*rx.transpose();
}

void Executer::takeOff()
{
  feedback_.status= true;
  server_->publishFeedback(feedback_);
  uav_abstraction_layer::TakeOff take_off_msg;
  take_off_msg.request.height = list_wp_[0].point.z;
  take_off_msg.request.blocking = true;
  result_.goal_achieved = take_off_client_.call(take_off_msg);
  server_->setSucceeded(result_);
  action_type_ = EXECUTER_IDLE;
}

void Executer::land()
{
  feedback_.status= true;
  server_->publishFeedback(feedback_);
  uav_abstraction_layer::Land land_msg;
  land_msg.request.blocking = true;
  result_.goal_achieved = land_client_.call(land_msg);
  server_->setSucceeded(result_);
  action_type_ = EXECUTER_IDLE;
}

void Executer::goToWaypoint()
{
  if (cont < list_wp_.size()) {
    result_.goal_achieved=false;

    setpoint_pose_.header.stamp     = ros::Time::now();
    setpoint_pose_.header.frame_id  = "map";
    setpoint_pose_.pose.position.x  = list_wp_[cont].point.x;
    setpoint_pose_.pose.position.y  = list_wp_[cont].point.y;
    setpoint_pose_.pose.position.z  = list_wp_[cont].point.z; 

    setpoint_pose_.pose.orientation.z = drone_att_.z();
    setpoint_pose_.pose.orientation.y = drone_att_.y();
    setpoint_pose_.pose.orientation.x = drone_att_.x();
    setpoint_pose_.pose.orientation.w = drone_att_.w();
        
    go_to_waypoint_pub_.publish(setpoint_pose_);

    if (drone_alarmed_when_doing_gotowaypoint_==true) { // Alarm received while doing the gotowaypoint, don't keep iterating.
      action_type_ = EXECUTER_IDLE;
      action_received_ = false;
      drone_alarmed_when_doing_gotowaypoint_ = false;
      cont = 0;
      return;
    }
    wp << setpoint_pose_.pose.position.x , setpoint_pose_.pose.position.y , setpoint_pose_.pose.position.z;
    diff = (wp - drone_pos_).norm();

    if(diff<=0.5) { // if wp is reached, command next waypoint
      cont = cont +1;
    }

  } else if (cont == list_wp_.size()) {
    if (final_yaw_if_gotowaypoint_.z==0 && final_yaw_if_gotowaypoint_.y==0 && final_yaw_if_gotowaypoint_.x==0 && final_yaw_if_gotowaypoint_.w==0) {
      result_.goal_achieved=true;
      server_->setSucceeded(result_);
      action_type_ = EXECUTER_IDLE;
      action_received_ = false;
      drone_alarmed_when_doing_gotowaypoint_ = false;
      cont = 0;
      return;
    }
    // Last go to wp at the final position of the path but with the final yaw:
    setpoint_pose_.header.stamp     = ros::Time::now();
    setpoint_pose_.header.frame_id  = "map";
    setpoint_pose_.pose.position.x  = list_wp_[cont-1].point.x;
    setpoint_pose_.pose.position.y  = list_wp_[cont-1].point.y;
    setpoint_pose_.pose.position.z  = list_wp_[cont-1].point.z;

    setpoint_pose_.pose.orientation.z = final_yaw_if_gotowaypoint_.z;
    setpoint_pose_.pose.orientation.y = final_yaw_if_gotowaypoint_.y;
    setpoint_pose_.pose.orientation.x = final_yaw_if_gotowaypoint_.x;
    setpoint_pose_.pose.orientation.w = final_yaw_if_gotowaypoint_.w;

    go_to_waypoint_pub_.publish(setpoint_pose_);
    if (drone_alarmed_when_doing_gotowaypoint_==true) { // Alarm received while doing the gotowaypoint, don't keep iterating.
      action_type_ = EXECUTER_IDLE;
      action_received_ = false;
      drone_alarmed_when_doing_gotowaypoint_ = false;
      cont = 0;
      return;
    }

    // Desired yaw (z-axis rotation) in radians:
    double desired_yaw = atan2(2*(final_yaw_if_gotowaypoint_.w*final_yaw_if_gotowaypoint_.z+final_yaw_if_gotowaypoint_.x*final_yaw_if_gotowaypoint_.y),\
    1-2*(final_yaw_if_gotowaypoint_.y*final_yaw_if_gotowaypoint_.y+final_yaw_if_gotowaypoint_.z*final_yaw_if_gotowaypoint_.z));
    
    diff = abs(desired_yaw - drone_yaw_);

    if(diff<=0.261799388) { // if wp is reached (difference <= 10 sexagesimal degrees), goal achieved
      result_.goal_achieved=true;
      server_->setSucceeded(result_);
      action_type_ = EXECUTER_IDLE;
      action_received_ = false;
      drone_alarmed_when_doing_gotowaypoint_ = false;
      cont = 0;
    }
  }
}

void Executer::followVehicle()
{
  desired_point << trailer.desired_point_, trailer.altitude;

  double pos_error_tanh_x = MAX_DRONE_SPEED*tanh(K_DRONE*(drone_pos_[0]-desired_point[0])/MAX_DRONE_SPEED);
  double pos_error_tanh_y = MAX_DRONE_SPEED*tanh(K_DRONE*(drone_pos_[1]-desired_point[1])/MAX_DRONE_SPEED);
  double pos_error_tanh_z = MAX_DRONE_SPEED*tanh(K_DRONE*(drone_pos_[2]-desired_point[2])/MAX_DRONE_SPEED);
  Eigen::Vector3d pos_error_tanh(pos_error_tanh_x,pos_error_tanh_y,pos_error_tanh_z);
  Eigen::Vector3d v_;
  v_ << trailer.v_desired, 0;
  cmd_linear_velocity = -pos_error_tanh + v_;

  tf::vectorEigenToMsg(cmd_linear_velocity, cmd_linear_velocity_);

  setpoint_vel_.header.stamp = ros::Time::now();
  setpoint_vel_.header.frame_id = "map";
  if(!std::isnan(cmd_linear_velocity.x()) && !std::isnan(cmd_linear_velocity.y()) && !std::isnan(cmd_linear_velocity.z())
  && !std::isinf(cmd_linear_velocity.x()) && !std::isinf(cmd_linear_velocity.y()) && !std::isinf(cmd_linear_velocity.z())) 
    setpoint_vel_.twist.linear = cmd_linear_velocity_;
  else {
    action_type_ = EXECUTER_IDLE; 
    feedback_.status = false;
    feedback_.action_id = action_id_;
    server_->publishFeedback(feedback_);
    ROS_ERROR("Got NAN, shooting action will be discarded. \ndesired_point =  %f, %f, %f  \nv_ = %f, %f, %f \nv_target =  %f, %f \nv_trailer_ = %f, %f \nw_trailer_ = %f \nTs = %f \n", desired_point.x(), desired_point.y(), desired_point.z(), v_.x(),v_.y(),v_.z(), trailer.v_target.x(), trailer.v_target.y(), trailer.v_trailer_.x(), trailer.v_trailer_.y(), trailer.w_trailer, trailer.Ts[trailer.filter-2]);
    return;
  } 

  setpoint_vel_.twist.angular.x = 0;
  setpoint_vel_.twist.angular.y = 0;

  if (shooting_action_ == multidrone_msgs::ShootingType::SHOOT_TYPE_FLY_THROUGH || shooting_action_ == multidrone_msgs::ShootingType::SHOOT_TYPE_LATERAL || shooting_mode_ == multidrone_msgs::TargetIdentifierType::NONE)
  	    drone_heading_error = remainder((drone_yaw_ - trailer.a_), 2*M_PI);
  else if(shooting_action_ == multidrone_msgs::ShootingType::SHOOT_TYPE_ORBIT){
    Eigen::Vector3d heading = (drone_pos_ - rt_target_pos_);
    if (trailer.angular_speed > 0)
      heading = heading.cross(-Eigen::Vector3d::UnitZ());
    else
      heading = heading.cross(Eigen::Vector3d::UnitZ()); 
    drone_heading_error = remainder((drone_yaw_ - atan2(heading.y(),heading.x())), 2*M_PI);
  } 
  else 
    drone_heading_error = remainder((drone_yaw_ - atan2 (shooting_target_pos_.y() - drone_pos_.y(), shooting_target_pos_.x() - drone_pos_.x())), 2*M_PI);

  if (drone_heading_error > M_PI)
    drone_heading_error = -2*M_PI + drone_heading_error;
  setpoint_vel_.twist.angular.z = MAX_YAW_RATE*tanh((-K_DRONE_YAW*drone_heading_error + trailer.w_trailer)/MAX_YAW_RATE);

  goal_direction_ = setpoint_vel_.twist.linear;
  wished_mov_dir_pub_.publish(goal_direction_);
  set_velocity_pub_.publish(setpoint_vel_);
}

/**
 *
 */
void Executer::collisionWarningCallback(const std_msgs::Bool::ConstPtr& collision_warning)
{
  confl_warning_ = collision_warning->data;
}

/**
 *
 */
void Executer::avoidMovementCallback(const geometry_msgs::Vector3::ConstPtr& avoidance_direction)
{
    avoid_mov_direction_.x = avoidance_direction->x;
    avoid_mov_direction_.y = avoidance_direction->y;
    avoid_mov_direction_.z = avoidance_direction->z;
}

void Executer::actionCallback()
{
  // If the new drone action is a gotowaypoint and right now is running another gotowaypoint, it means that an alarm has been triggered while doing it.
  // In this case wait until the current gotowaypoint finish to start the next alarm's gotowaypoint.
  if (action_type_==multidrone_msgs::DroneAction::TYPE_GOTOWAYPOINT && goal_.action_type==multidrone_msgs::DroneAction::TYPE_GOTOWAYPOINT) {
    drone_alarmed_when_doing_gotowaypoint_ = true;
    while (drone_alarmed_when_doing_gotowaypoint_ && ros::ok) {
      ros::Rate loop_rate(1); // [Hz]
      loop_rate.sleep();
    }
  }

  timer_rt.stop();
  timer_trailer.stop();
  running_trailer = false;
  action_received_ = true;
  start_time = ros::Time::now().toSec();
  goal_ = server_->acceptNewGoal()->action_goal;

  // saving list of waypoints
  cont = 0;
  list_wp_.clear();
  action_type_= goal_.action_type;
  action_id_ = goal_.action_id;

  // Save final_yaw_if_gotowaypoint from this drone action into its class attribute, so that later a new last waypoint with different yaw can be sent in the navigation actions.
  final_yaw_if_gotowaypoint_.x = goal_.final_yaw_if_gotowaypoint.x;
  final_yaw_if_gotowaypoint_.y = goal_.final_yaw_if_gotowaypoint.y;
  final_yaw_if_gotowaypoint_.z = goal_.final_yaw_if_gotowaypoint.z;
  final_yaw_if_gotowaypoint_.w = goal_.final_yaw_if_gotowaypoint.w;

  if (action_type_ == multidrone_msgs::DroneAction::TYPE_SHOOTING) {
    shooting_parameters_.clear();
    target_parameters_.clear();
    for(int i = 0; i<goal_.shooting_action.shooting_roles[0].shooting_parameters.size(); i++)
      shooting_parameters_[goal_.shooting_action.shooting_roles[0].shooting_parameters[i].param] = goal_.shooting_action.shooting_roles[0].shooting_parameters[i].value;
    for(int i = 0; i<goal_.shooting_action.shooting_roles[0].target_parameters.size(); i++)
      target_parameters_[goal_.shooting_action.shooting_roles[0].target_parameters[i].param] = goal_.shooting_action.shooting_roles[0].target_parameters[i].value;
    for(int i = 0; i<goal_.shooting_action.rt_trajectory.size(); i++)
      list_wp_.push_back(goal_.shooting_action.rt_trajectory[i]);

    shooting_action_                =  goal_.shooting_action.shooting_roles[0].shooting_type.type;
    trailer.originOfFormation_(0)   =  goal_.shooting_action.rt_displacement.x + list_wp_[0].point.x;
    trailer.originOfFormation_(1)   =  goal_.shooting_action.rt_displacement.y + list_wp_[0].point.y;
    duration_                       =  goal_.shooting_action.duration;
    length_                         =  goal_.shooting_action.length;
    rt_mode_                        =  goal_.shooting_action.rt_mode;
    std::cout << "rt_mode_ is " << (int)rt_mode_ << "\n";
    if (rt_mode_ != multidrone_msgs::ShootingAction::RT_MODE_VIRTUAL_TRAJ){
      rt_id_                        =  goal_.shooting_action.rt_id;
      std::cout << "rt_id_ is " << (int)rt_id_ << "\n"; 
    }      
    shooting_mode_                  =  goal_.shooting_action.shooting_roles[0].target_identifier_type.type;  

    if (shooting_mode_ == multidrone_msgs::TargetIdentifierType::GPS){
      shooting_id_                  =  (uint8_t) target_parameters_["ID"];
      std::cout << "shooting_id_ is " << (int)shooting_id_ << "\n";      
    }      
    else if (shooting_mode_ == multidrone_msgs::TargetIdentifierType::VISUAL){
      multidrone_msgs::FollowTarget follow_target_msg;
      follow_target_msg.request.target_id = 0; 
      follow_target_msg.request.target_type.type = 0;
      follow_target_client_.call(follow_target_msg);
    }
    else if (shooting_mode_ == multidrone_msgs::TargetIdentifierType::VISUAL_GPS){
      multidrone_msgs::FollowTarget follow_target_msg;
      shooting_id_                  =  (uint8_t) target_parameters_["ID"];
      follow_target_msg.request.target_id = 0;
      follow_target_msg.request.target_type.type = 1;
      follow_target_client_.call(follow_target_msg);      
      std::cout << "VISUAL_GPS shooting_id_ is " << (int)shooting_id_ << "\n";
    }
    else if (shooting_mode_ == multidrone_msgs::TargetIdentifierType::NONE){
      pan_s                         =  target_parameters_["pan_s"];
      tilt_s                        =  target_parameters_["tilt_s"];
      pan_e                         =  target_parameters_["pan_e"];
      tilt_e                        =  target_parameters_["tilt_e"];
      } 
    else if (shooting_mode_ == multidrone_msgs::TargetIdentifierType::STATIC){
      latitude                      =  target_parameters_["latitude"];
      longitude                     =  target_parameters_["longitude"];        
      altitude                      =  target_parameters_["altitude"];        
    }

    if (rt_mode_ == multidrone_msgs::ShootingAction::RT_MODE_VIRTUAL_TRAJ || shooting_mode_ == multidrone_msgs::TargetIdentifierType::VIRTUAL){
      total_displ << (list_wp_[list_wp_.size()-1].point.x - list_wp_[0].point.x) , (list_wp_[list_wp_.size()-1].point.y -list_wp_[0].point.y) , (list_wp_[list_wp_.size()-1].point.z -list_wp_[0].point.z);
      formation_speed_ = goal_.shooting_action.formation_speed;
      if (total_displ.norm() == 0){
        total_displ << 1,1,1;
        formation_speed_ = 0;
      }
      if (rt_mode_ == multidrone_msgs::ShootingAction::RT_MODE_VIRTUAL_TRAJ){
        rt_target_pos_.x() = list_wp_[0].point.x;
        rt_target_pos_.y() = list_wp_[0].point.y;
        rt_target_pos_.z() = list_wp_[0].point.z;
      }
      if (shooting_mode_ == multidrone_msgs::TargetIdentifierType::VIRTUAL){
        shooting_target_pos_.x() = list_wp_[0].point.x;
        shooting_target_pos_.y() = list_wp_[0].point.y;
        shooting_target_pos_.z() = list_wp_[0].point.z; 
      }    
      timer_rt.start();
    }

    //trailer parameters
    trailer.initial_azimuth   = 0;
    trailer.radius            = 0;
    trailer.angular_speed     = 0;
    trailer.rx                = 0;
    trailer.rx_e              = 0;
    trailer.rx_s              = 0;
    trailer.ry                = 0;
    trailer.rx_dot            = 0;
    trailer.ry_dot            = 0;
    trailer.altitude          = 0; 
    trailer.z_e               = 0;
    trailer.altitude_dot      = 0;
    trailer.d                 = trailer_size;
    if (rt_mode_ != multidrone_msgs::ShootingAction::RT_MODE_ACTUAL_TARGET)
      trailer.d               = 0;
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if(shooting_action_ == multidrone_msgs::ShootingType::SHOOT_TYPE_STATIC) //0
    {
      trailer.altitude          = shooting_parameters_["z_0"];
      trailer.trailer_type      = TRAILER_TYPE_INERTIAL;
    } 
    else if(shooting_action_ == multidrone_msgs::ShootingType::SHOOT_TYPE_FLY_THROUGH && rt_mode_ == multidrone_msgs::ShootingAction::RT_MODE_VIRTUAL_TRAJ) //1
    {
      trailer.altitude          = shooting_parameters_["z_0"]; 
      trailer.trailer_type      = TRAILER_TYPE_VARIABLE;
    } 
    else if(shooting_action_ == multidrone_msgs::ShootingType::SHOOT_TYPE_ESTABLISH) //2
    {
      trailer.rx_s              = shooting_parameters_["x_s"];
      trailer.rx_e              = shooting_parameters_["x_e"];
      trailer.rx_dot            = (shooting_parameters_["x_e"] - shooting_parameters_["x_s"])/duration_.data.sec;
      trailer.altitude          = shooting_parameters_["z_s"];
      trailer.z_e               = shooting_parameters_["z_e"]; 
      trailer.altitude_dot      = (shooting_parameters_["z_e"] - shooting_parameters_["z_s"])/duration_.data.sec;
      trailer.trailer_type      = TRAILER_TYPE_VARIABLE;
    }
    else if(shooting_action_ == multidrone_msgs::ShootingType::SHOOT_TYPE_CHASE || shooting_action_ == multidrone_msgs::ShootingType::SHOOT_TYPE_LEAD) //3
    {
      trailer.rx_s              = shooting_parameters_["x_s"];
      trailer.rx_e              = shooting_parameters_["x_e"];
      trailer.rx_dot            = (shooting_parameters_["x_e"] - shooting_parameters_["x_s"])/duration_.data.sec;
      trailer.altitude          = shooting_parameters_["z_0"]; 
      trailer.trailer_type      = TRAILER_TYPE_VARIABLE;
    } 
    else if(shooting_action_ == multidrone_msgs::ShootingType::SHOOT_TYPE_FLYBY) //5
    {
      trailer.rx_s              = shooting_parameters_["x_s"];
      trailer.rx_e              = shooting_parameters_["x_e"];
      trailer.ry                = shooting_parameters_["y_0"];
      trailer.rx_dot            = (shooting_parameters_["x_e"] - shooting_parameters_["x_s"])/duration_.data.sec;
      trailer.altitude          = shooting_parameters_["z_0"]; 
      trailer.trailer_type      = TRAILER_TYPE_VARIABLE;
    } 
    else if(shooting_action_ == multidrone_msgs::ShootingType::SHOOT_TYPE_LATERAL) //6
    {
      trailer.ry                = shooting_parameters_["y_0"];
      trailer.altitude          = shooting_parameters_["z_0"]; 
      trailer.trailer_type      = TRAILER_TYPE_VARIABLE;
    } 
    else if(shooting_action_ == multidrone_msgs::ShootingType::SHOOT_TYPE_ELEVATOR) //7
    {
      trailer.altitude          = shooting_parameters_["z_s"];
      trailer.z_e               = shooting_parameters_["z_e"]; 
      trailer.altitude_dot      = (shooting_parameters_["z_e"] - shooting_parameters_["z_s"])/duration_.data.sec;
      trailer.trailer_type      = TRAILER_TYPE_INERTIAL;
    } 
    else  if(shooting_action_ == multidrone_msgs::ShootingType::SHOOT_TYPE_ORBIT) //8
    {
      trailer.initial_azimuth  = shooting_parameters_["azimuth_s"];
      trailer.radius           = shooting_parameters_["r_0"];
      trailer.angular_speed    = shooting_parameters_["azimuth_speed"];
      trailer.altitude         = shooting_parameters_["z_0"]; 
      trailer.trailer_type     = TRAILER_TYPE_ORBIT;
    }
    else {
      action_type_ = EXECUTER_IDLE; 
      feedback_.status = false;
      feedback_.action_id = action_id_;
      server_->publishFeedback(feedback_);
      return;
    }
    std::cout << "shooting_action_ is " << (int)shooting_action_ << "\n";
    std::cout << "trailer.ry is "       << trailer.ry            << "\n";
    std::cout << "trailer.altitude is " << trailer.altitude      << "\n";
      
    trailer.r << trailer.rx, trailer.ry;
    trailer.r_dot << trailer.rx_dot, trailer.ry_dot;

    parameters_set = true;
    
    multidrone_msgs::SetFramingType set_framing_type_msg;
    set_framing_type_msg.request.target_framing_type.type = goal_.shooting_action.shooting_roles[0].framing_type.type;
    set_framing_type_client_.call(set_framing_type_msg);
  }
  else //Takeoff, land or gotowaypoint
    for(int i = 0; i<goal_.path.size(); i++)
      list_wp_.push_back(goal_.path[i]);
}

void Executer::gimbalAutomatic(double pan_s, double tilt_s, double pan_e, double tilt_e, double duration, double start_time)
{ 
  double t    = ros::Time::now().toSec() - start_time;
  double pan  = pan_s + t*(pan_e - pan_s)/duration;
  double tilt = tilt_s + t*(tilt_e - tilt_s)/duration;

  toEulerAngles_YXZ(gimbal_att_,gimbal_euler);
  geometry_msgs::Vector3 msg_gimbal_cmd;
  
  msg_gimbal_cmd.x = - gimbal_euler.x()*TO_DEG;
  msg_gimbal_cmd.y = 1.5*(tilt - gimbal_euler.y()*TO_DEG);
  msg_gimbal_cmd.z = 1.5*(pan - (gimbal_euler.z()+drone_yaw_)*TO_DEG);
  
  if(t>=duration){
    msg_gimbal_cmd.y = 0;
    msg_gimbal_cmd.z = 0;
  }
    gimbal_cmd_basecam_pub.publish(msg_gimbal_cmd);

  if(simulation){
    rotationMatrixToEulerAngles_XYZ(R_x, gimbal_euler);

    if(std::abs(gimbal_euler[0]) >= 3){ //to avoid singularity on simulated gimbal
      gimbal_euler[0] = gimbal_euler[0] - M_PI;
      gimbal_euler[1] = M_PI - gimbal_euler[1];
      gimbal_euler[2] = gimbal_euler[2] - M_PI;
    }
    if (!has_gimbal_status_)
      R_C_I = R_x;

    gimbal_cmd_.request.param1 = tilt -90;  // pitch
		gimbal_cmd_.request.param2 = gimbal_euler.x()*TO_DEG;;// roll
		gimbal_cmd_.request.param3 = pan +drone_yaw_*TO_DEG;  // yaw
		if(t<duration)
      cmd_long_client_.call(gimbal_cmd_);  
  }
}

void Executer::gimbalSafeMode()
{
    w_est << 0.0, 0.0, 0.0;
    w_est_ << 0.0, 0.0, 0.0;
    t_w_ = ros::Time::now().toSec();

    geometry_msgs::Vector3 msg_gimbal_cmd;
    msg_gimbal_cmd.x = 0;
    msg_gimbal_cmd.y = 0;
    msg_gimbal_cmd.z = 0;
    gimbal_cmd_basecam_pub.publish(msg_gimbal_cmd);
}

void Executer::gimbalController()
{ 

  if (has_drone_pose_ && has_shooting_target_status_ ){ //GPS
    o(0) = K.inverse()(1, 1) * -target_image_offset_y;  
    o(1) = K.inverse()(0, 0) * target_image_offset_x;
    o(2) = 0.0;
    q_I = p_T_I - p_B_I - R_B_I*p_C_B + R_C_I*o*q_C.z();
    q_I_dot << 0.0, 0.0, 0.0;
    q_norm = R_C_I.transpose()*(q_I/q_I.norm());
    r_x_3 = q_I/q_I.norm();
    a = r_x_3.cross(-Eigen::Vector3d::UnitZ());
    has_visual_2D_target_status_ = false;
  }
  else if (has_drone_pose_ && shooting_mode_ == multidrone_msgs::TargetIdentifierType::STATIC){ // static
    p_T_I << latitude, longitude, altitude;
    q_I = p_T_I - p_B_I - R_B_I*p_C_B;
    q_I_dot << 0.0, 0.0, 0.0;
    q_norm = R_C_I.transpose()*(q_I/q_I.norm());
    r_x_3 = q_I/q_I.norm();
    a = r_x_3.cross(-Eigen::Vector3d::UnitZ());
    has_visual_2D_target_status_ = false;
  }
  else{
    gimbalSafeMode();
    return;
  }

  r_x_2 = a/a.norm();
  b = r_x_2.cross(r_x_3);
  r_x_1 = b/b.norm();
  R_x.col(0) = r_x_1;
  R_x.col(1) = r_x_2;
  R_x.col(2) = r_x_3;

  if (!has_visual_2D_target_status_)
    R_e = R_C_I.transpose()*R_x;
  else
    R_e = R_x;
  
  R_e_R_e = R_e-R_e.transpose();
  toSInvMatrix(R_e_R_e,w_e);      

  // Calculate Lyapunov function
  V_1_ = Eigen::MatrixXd::Identity(3,3)-R_e;
  lyapunov_visual = V_1_(0,0)+V_1_(1,1)+V_1_(2,2);

  w = k_1*w_e;

  if(shooting_action_ != multidrone_msgs::ShootingType::SHOOT_TYPE_LATERAL && shooting_action_ != multidrone_msgs::ShootingType::SHOOT_TYPE_STATIC && 
     shooting_action_ != multidrone_msgs::ShootingType::SHOOT_TYPE_ELEVATOR){ 
       run_w_estimate();   
       w = w_est + k_1*w_e;
  }
  
  toEulerAngles_YXZ(gimbal_att_,gimbal_euler);
  
  Q_inv <<                       cos(gimbal_euler.y()),  0,                         sin(gimbal_euler.y()),
           sin(gimbal_euler.x())*sin(gimbal_euler.y()),  1,  -sin(gimbal_euler.x())*cos(gimbal_euler.y()),
                                -sin(gimbal_euler.y()),  0,                         cos(gimbal_euler.y());
  euler_dot_cmd = Q_inv*w;

  geometry_msgs::Vector3 msg_gimbal_cmd;
  msg_gimbal_cmd.x = euler_dot_cmd[0]*TO_DEG;
  msg_gimbal_cmd.y = euler_dot_cmd[1]*TO_DEG;
  msg_gimbal_cmd.z = euler_dot_cmd[2]*TO_DEG;
  
  gimbal_cmd_basecam_pub.publish(msg_gimbal_cmd);  
  
  if(simulation){
    if ((shooting_mode_ == multidrone_msgs::TargetIdentifierType::VISUAL || shooting_mode_ == multidrone_msgs::TargetIdentifierType::VISUAL_GPS) && has_visual_2D_target_status_) {R_x = R_C_I*R_e;}
    
    rotationMatrixToEulerAngles_XYZ(R_x, gimbal_euler);

    if(std::abs(gimbal_euler[0]) >= 3){ //to avoid singularity on simulated gimbal
      gimbal_euler[0] = gimbal_euler[0] - M_PI;
      gimbal_euler[1] = M_PI - gimbal_euler[1];
      gimbal_euler[2] = gimbal_euler[2] - M_PI;
    }
    if (!has_gimbal_status_)
      R_C_I = R_x;

    gimbal_cmd_.request.param1 = gimbal_euler[1]*TO_DEG-90;  // pitch
		gimbal_cmd_.request.param2 = gimbal_euler[0]*TO_DEG;     // roll
		gimbal_cmd_.request.param3 = gimbal_euler[2]*TO_DEG+drone_yaw_*TO_DEG;  // yaw
    
		cmd_long_client_.call(gimbal_cmd_);  
  }
}

void Executer::CameraControl(){
  multidrone_msgs::CameraControl camera_control_msg_;
  camera_control_msg_.request.control_function = multidrone_msgs::CameraControl::Request::CONTROL_FUNCTION_ZOOM;
  camera_control_msg_.request.manual_ctrl = false;

  if (zoom_error > 0.05){
    if (count > 0){
      camera_control_msg_.request.value = uint8_t(127 - 88 * (zoom_error-0.05));
      camera_control_client_.call(camera_control_msg_);
      count--;
    }  
  }
  else if (zoom_error < -0.05){
    if (count < count_lim){
       camera_control_msg_.request.value = uint8_t(129 - 88 * (zoom_error+0.05));
       camera_control_client_.call(camera_control_msg_);
       count++;  
  }
}
}