#ifndef ACTION_EXECUTER_H
#define ACTION_EXECUTER_H


#include <action_executer/trailer.h>


class Executer
{
private:
    ros::Subscriber vechile_odometry_sub;

    ros::ServiceClient    cmd_long_client_;
    ros::ServiceClient    set_param_client_;
    ros::ServiceClient    get_param_client_;
    ros::ServiceClient    take_off_client_;
    ros::ServiceClient    go_to_waypoint_client_;
    ros::ServiceClient    land_client_;
    ros::ServiceClient    follow_target_client_;
    ros::ServiceClient    set_framing_type_client_;
    ros::ServiceClient    camera_control_client_;
    
    ros::Publisher        go_to_waypoint_pub_;
    ros::Publisher        set_velocity_pub_;
    ros::Publisher        gimbal_cmd_basecam_pub;
    ros::Publisher        wished_mov_dir_pub_;
    ros::Publisher        trajectory_;
    ros::Publisher        lyapunov_;
    ros::Publisher        pixel_publish;


    ros::Timer timer_;
    ros::Timer timer_trailer;
    ros::Timer timer_gimbal;
    ros::Timer timer_trajectory;
    ros::Timer timer_rt;

    Trailer trailer;

    actionlib::SimpleActionServer<multidrone_msgs::ExecuteAction>* server_;
    typedef actionlib::SimpleActionServer<multidrone_msgs::ExecuteAction> Server;
    multidrone_msgs::ExecuteResult result_;


    geometry_msgs::PoseStamped  setpoint_pose_;
    geometry_msgs::TwistStamped setpoint_vel_;
    geometry_msgs::Vector3      cmd_linear_velocity_;
    geometry_msgs::Vector3      goal_direction_;

    uav_abstraction_layer::State drone_state_;

    int cont = 0;

    bool confl_warning_ = false;                        //!< Flag to know if there is a conflict to avoid
    geometry_msgs::Vector3 avoid_mov_direction_;


    double t_w_, dT_w = 0.0;

    //rt_target
    Eigen::Vector3d               rt_target_pos_;
    Eigen::Matrix<double, 6, 1>   rt_target_twist_;
    Eigen::Vector3d               rt_target_vel_;
    Eigen::Vector3d               rt_target_ang_vel_;
    
    //shooting_target
    Eigen::Vector3d               shooting_target_pos_;
    Eigen::Matrix<double, 6, 1>   shooting_target_twist_;
    Eigen::Quaterniond            shooting_target_att_;
    Eigen::Vector3d               shooting_target_vel_;
    Eigen::Vector3d               shooting_target_ang_vel_;

    Eigen::Vector3d               desired_point;
    Eigen::Vector2d               desired_vel;
    double                        drone_yaw_;
    double                        desired_yaw;
    double                        drone_heading_error;
    Eigen::Vector3d               cmd_linear_velocity;
    Eigen::Vector3d               drone_pos_;
    Eigen::Quaterniond            drone_att_;
    Eigen::Matrix<double, 6, 1>   drone_twist_;
    Eigen::Vector3d               drone_vel_;
    Eigen::Vector3d               drone_ang_vel_;
    Eigen::Quaterniond            gimbal_att_;
    Eigen::Vector3d               w_est_;
    Eigen::Vector3d               w_est;
    Eigen::Vector3d               w_e;
    Eigen::Matrix3d               R_C_I = Eigen::Matrix3d::Zero();
    Eigen::Vector3d               p_pix;

    Eigen::Matrix3d               R_B_S;        // "Drone" orientation in the MoCap reference frame
    Eigen::Matrix3d               R_B_I;        // "Drone" orientation in the Inertial frame
    Eigen::Matrix3d               R_T_S;        // Target orientation in the MoCap reference frame
    Eigen::Matrix3d               R_T_I;
    Eigen::Matrix3d               R_S_I;        // Camera orientation in the Inertial frame. Gimbal has z-axis pointing down unlike ENU 
    Eigen::Matrix3d               R_x;          // Desired rotation matrix for the gimbal
    Eigen::Matrix3d               R;            // Actual gimbal rotation matrix
    Eigen::Matrix3d               R_e;          // error rotation matrix
    Eigen::Matrix3d               R_e_R_e;
    Eigen::Matrix3d               Q_inv;
    Eigen::Matrix3d               K;
    double                        K_min =  609.947387;
    double                        K_max = 2000.39947387;
    Eigen::Vector3d               q_C;
    Eigen::Vector3d               q_Cv;
    Eigen::Vector3d               q_C2;
    Eigen::Vector3d               p_T_I;
    Eigen::Vector3d               p_T_I_dot;
    Eigen::Vector3d               o;
    Eigen::Vector3d               q_norm, o_norm, a_norm, a_off;
    float                         aux;
                        
    Eigen::Vector3d               p_B_I;
            
    Eigen::Vector3d               p_B_I_dot;
    Eigen::Vector3d               p_C_B; // gimbal position with respect to drone CG. x front z up
    Eigen::Vector3d               w_T;
    Eigen::Vector3d               q_I;
    Eigen::Vector3d               q_I_visual;
    Eigen::Vector3d               q_I_dot;
    Eigen::Vector3d               r_x_1;
    Eigen::Vector3d               r_x_2;
    Eigen::Vector3d               r_x_3;
    Eigen::Vector3d               a;
    Eigen::Vector3d               b;
    Eigen::Vector3d               w;
    Eigen::Vector3d               w_B;
    Eigen::Vector3d               gimbal_euler;
    Eigen::Vector3d               euler_dot_cmd;
    Eigen::Matrix3d               V_1_;
    Eigen::Vector3d               pixel;

    
    double lyapunov_visual = 0.0; 

    multidrone_msgs::DroneAction  goal_;
    std::map<std::string, double> shooting_parameters_;
    std::map<std::string, double> target_parameters_;
    std_msgs::Duration            duration_;
    double                        length_;
    double                        formation_speed_;
    uint8_t                       rt_id_;
    uint8_t                       shooting_id_;
    uint8_t                       shooting_mode_ = 6;
    uint8_t                       shooting_action_;
    uint8_t                       rt_mode_ = 4;
    double                        trailer_size;
    int                           action_type_ = 0;
    std::string action_id_ = "";

    std::vector<geometry_msgs::PointStamped> list_wp_;
    geometry_msgs::Quaternion final_yaw_if_gotowaypoint_;
    mavros_msgs::CommandLong gimbal_cmd_;
    uint8_t cmd = 76;
    uint8_t inc = 4;

    double pan_s;
    double tilt_s;
    double pan_e;
    double tilt_e;
    double latitude;
    double longitude;
    double altitude;
    double start_time;

    std::atomic<bool> drone_alarmed_when_doing_gotowaypoint_ = {false};

    int  drone_id_;
    bool parameters_set = false;
    bool onboard = false;
    bool simulation = false;
    bool offset = false;    
    bool has_drone_vel_ = false;
    bool has_gimbal_status_ = false;
    bool running_trailer = false;
    bool has_drone_pose_ = false;
    bool has_rt_target_status_ = false;
    bool has_shooting_target_status_ = false;
    bool has_visual_2D_target_status_ = false;
    bool gimbal_home = false;
    double time_of_last_gimbal_status;
    double time_of_last_target_visual_2D_status;
    double time_of_last_target_visual_3D_status;
    double time_of_last_target_status;
    double time_of_last_drone_pose;
    float  k_1;
    float  k_2;
    float  k_offset;
    float  MAX_YAW_RATE;
    double x_visual_target = 0.0;
    double y_visual_target = 0.0;
    double zoom_error = 0.0;
    int    count = 0;
    int    count_lim = 24;
    int    focus_value = -1;
    int    best_focus_value = -1;
    int    focus_ctr = 0;
    int    buffer[5] = {-1,-1,-1,-1, -1};
    double RT_FREQ;
    Eigen::Vector3d total_displ;
    Eigen::Vector3d wp;
    double diff;

    double t_0;
    double orientation_0;

    bool action_received_ = false;
    bool goal_preempt_ = false;
    bool f_reset = true;
    bool recording = false;
    bool f_target = false;

    int w_im;
    int h_im;
    int target_image_offset_x = 0;
    int target_image_offset_y = 0;
    int target_image_offset_step = 50;
    int target_image_offset_x_MAX;
    int target_image_offset_x_MIN;
    int target_image_offset_x_13;
    int target_image_offset_x_23;
    int target_image_offset_y_MAX;
    int target_image_offset_y_MIN;
    int target_image_offset_y_13;
    int target_image_offset_y_23;

    multidrone_msgs::ExecuteFeedback feedback_;

    void timerCallback(const ros::TimerEvent&);
    void timerCallbackTrailer(const ros::TimerEvent&);
    void timerCallbackGimbal(const ros::TimerEvent&);
    void referenceTrajectory(const ros::TimerEvent&);
    void trajectory_publisher ();
    void lyapunov_publisher();
    void dronePose(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void droneVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void droneState(const uav_abstraction_layer::State::ConstPtr& msg);
    void gimbalStatus(const multidrone_msgs::GimbalStatus::ConstPtr& msg);
    bool ManualControlServiceCallback (multidrone_msgs::ManualControls::Request &req, multidrone_msgs::ManualControls::Response &res);
    void setMountModeParameters();
    void toEulerAngles_YXZ(const Eigen::Quaterniond& q, Eigen::Vector3d& euler);
    void rotationMatrixToEulerAngles_XYZ(Eigen::Matrix3d& R, Eigen::Vector3d& euler);
    void toSMatrix(Eigen::Vector3d& w, Eigen::Matrix3d& S);
    void toSInvMatrix(Eigen::Matrix3d& R, Eigen::Vector3d& S_inv);
    void toPIMatrix(Eigen::Vector3d& rx, Eigen::Matrix3d& PI);
    void goToWaypoint();
    void followVehicle();
    void takeOff();
    void land();
    void preemptCallback();
    void run_w_estimate();
    void gimbalSafeMode();
    void gimbalController();
    void gimbalAutomatic(double pan_s,double tilt_s,double pan_e,double tilt_e, double duration, double start_time);
    void targetarrayCallback(const multidrone_msgs::TargetStateArray::ConstPtr& msg);
    void CameraControl();
    void targetStatus(bool rt);
    void collisionWarningCallback(const std_msgs::Bool::ConstPtr& collision_warning);
    void focusCallback(const std_msgs::Int32::ConstPtr& _msg);
    void avoidMovementCallback(const geometry_msgs::Vector3::ConstPtr& avoidance_direction);
    void focus_reset_tree(int beg, int end);

public:

    Executer(int _argc, char** _argv);
    ~Executer();
    void actionCallback();
};

#endif // ACTION_EXECUTER_H