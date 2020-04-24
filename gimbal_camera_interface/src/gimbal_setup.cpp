#include <ros/ros.h>
#include <ros/timer.h>
#include <ros/console.h>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#define DEBUG_PRINT 0
#define SERIAL_PORT_READ_BUF_SIZE 256
#define SBGC_CMD_MAX_BYTES 255
#define SBGC_CMD_NON_PAYLOAD_BYTES 5
#define SBGC_CMD_DATA_SIZE (SBGC_CMD_MAX_BYTES - SBGC_CMD_NON_PAYLOAD_BYTES)
#define CMD_REALTIME_DATA_3       23
#define CMD_REALTIME_DATA_4       25
#define CMD_CONTROL		            67
#define CMD_READ_PARAMS_3         21
#define CMD_WRITE_PARAMS_3        22

#define _PROFILE_ID_                         0
#define _MOTOR_STAB_DATA_0_P_               10
#define _MOTOR_STAB_DATA_0_I_               10
#define _MOTOR_STAB_DATA_0_D_               20
#define _MOTOR_STAB_DATA_0_POWER_           94
#define _MOTOR_STAB_DATA_0_INVERT_           0
#define _MOTOR_STAB_DATA_0_POLES_           22
#define _MOTOR_STAB_DATA_1_P_               10
#define _MOTOR_STAB_DATA_1_I_               10
#define _MOTOR_STAB_DATA_1_D_               10
#define _MOTOR_STAB_DATA_1_POWER_           68
#define _MOTOR_STAB_DATA_1_INVERT_           0
#define _MOTOR_STAB_DATA_1_POLES_           22
#define _MOTOR_STAB_DATA_2_P_               10
#define _MOTOR_STAB_DATA_2_I_               10
#define _MOTOR_STAB_DATA_2_D_               24
#define _MOTOR_STAB_DATA_2_POWER_           75
#define _MOTOR_STAB_DATA_2_INVERT_           0
#define _MOTOR_STAB_DATA_2_POLES_           22
#define _ACC_LIMITER_ALL_                  100
#define _EXT_FC_GAIN_                        0
#define _RESERVED_                           0
#define _RC_DATA_0_MIN_ANGLE_              -70
#define _RC_DATA_0_MAX_ANGLE_               70
#define _RC_DATA_0_MODE_                     1
#define _RC_DATA_0_LPF_                      3
#define _RC_DATA_0_SPEED_                   20
#define _RC_DATA_0_FOLLOW_                   0
#define _RC_DATA_1_MIN_ANGLE_             -140
#define _RC_DATA_1_MAX_ANGLE_               40
#define _RC_DATA_1_MODE_                     1
#define _RC_DATA_1_LPF_                      3
#define _RC_DATA_1_SPEED_                   20
#define _RC_DATA_1_FOLLOW_                   0
#define _RC_DATA_2_MIN_ANGLE_              -45
#define _RC_DATA_2_MAX_ANGLE_               45
#define _RC_DATA_2_MODE_                     1
#define _RC_DATA_2_LPF_                      3
#define _RC_DATA_2_SPEED_                   20
#define _RC_DATA_2_FOLLOW_                   0
#define _GYRO_TRUST_                       100
#define _USE_MODEL_                          0
#define _PWM_FREQ_                           1
#define _SERIAL_SPEED_                       0
#define _RC_TRIM_0_                          0
#define _RC_TRIM_1_                          0
#define _RC_TRIM_2_                          0
#define _RC_DEADBAND_                       15
#define _RC_EXPO_RATE_                       0
#define _RC_VIRT_MODE_                       0
#define _RC_MAP_ROLL_                        0
#define _RC_MAP_PITCH_                       0
#define _RC_MAP_YAW_                         0
#define _RC_MAP_CMD_                         0
#define _RC_MAP_FC_ROLL_                     0
#define _RC_MAP_FC_PITCH_                    0
#define _RC_MIX_FC_ROLL_                     0
#define _RC_MIX_FC_PITCH_                    0
#define _FOLLOW_MODE_                        0
#define _FOLLOW_DEADBAND_                   50
#define _FOLLOW_EXPO_RATE_                  50
#define _FOLLOW_OFFSET_0_                   59
#define _FOLLOW_OFFSET_1_                    5
#define _FOLLOW_OFFSET_2_                    5
#define _AXIS_TOP_                           1
#define _AXIS_RIGHT_                        -2
#define _FRAME_AXIS_TOP_                    -2
#define _FRAME_AXIS_RIGHT_                  -1
#define _FRAME_IMU_POS_                      0
#define _GYRO_DEADBAND_                      0
#define _GYRO_SENS_                          0
#define _I2C_SPEED_FAST_                     1
#define _SKIP_GYRO_CALIB_                    0
#define _RC_CMD_LOW_                         0
#define _RC_CMD_MID_                         0
#define _RC_CMD_HIGH_                        0
#define _MENU_CMD_0_                         1
#define _MENU_CMD_1_                         2
#define _MENU_CMD_2_                         3
#define _MENU_CMD_3_                        14
#define _MENU_CMD_4_                         4
#define _MENU_CMD_LONG_                      9
#define _MOTOR_OUTPUT_0_                     1
#define _MOTOR_OUTPUT_1_                     2
#define _MOTOR_OUTPUT_2_                     3
#define _BAT_THRESHOLD_ALARM_             1080
#define _BAT_THRESHOLD_MOTORS_             990
#define _BAT_COMP_REF_                    1260
#define _BEEPER_MODES_                       0
#define _FOLLOW_ROLL_MIX_START_             40
#define _FOLLOW_ROLL_MIX_RANGE_             20
#define _BOOSTER_POWER_0_                    0
#define _BOOSTER_POWER_1_                    0
#define _BOOSTER_POWER_2_                    0
#define _FOLLOW_SPEED_0_                    10
#define _FOLLOW_SPEED_1_                    10
#define _FOLLOW_SPEED_2_                    10
#define _FRAME_ANGLE_FROM_MOTORS_            0
#define _RC_MEMORY_0_                        0
#define _RC_MEMORY_1_                    -5234
#define _RC_MEMORY_2_                        0
#define _SERVO_OUT_0_                        0
#define _SERVO_OUT_1_                        0
#define _SERVO_OUT_2_                        0
#define _SERVO_OUT_3_                        0
#define _SERVO_RATE_                         5
#define _ADAPTIVE_PID_ENABLED_               0
#define _ADAPTIVE_PID_THRESHOLD_            10
#define _ADAPTIVE_PID_RATE_                 50
#define _ADAPTIVE_PID_RECOVERY_FACTOR_       6
#define _FOLLOW_LPF_0_                       3
#define _FOLLOW_LPF_1_                       3
#define _FOLLOW_LPF_2_                       3
#define _GENERAL_FLAGS_1_                  274
#define _PROFILE_FLAGS_1_                 1168
#define _SPEKTRUM_MODE_                      0
#define _ORDER_OF_AXES_                      0
#define _EULER_ORDER_                        0
#define _CUR_IMU_                            1
#define _CUR_PROFILE_ID_                     0


enum STATE
{
  STATE_WAIT, STATE_GOT_MARKER, STATE_GOT_ID, STATE_GOT_LEN, STATE_GOT_HEADER, STATE_GOT_DATA
};

enum TIMER_STATE
{
  TIMER_STATE_READ_1, TIMER_STATE_READ_2, TIMER_STATE_WRITE
};

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

// CMD_READ_PARAMS_3
typedef struct __attribute__((packed)) {
  // uint8_t mode;     // legacy format: mode is common for all axes
  uint8_t profile_id;	// Possible values: 0..4         Current (avtive) profile: 255
  struct {
    uint8_t gain_P;
    uint8_t gain_I;
    uint8_t gain_D;
    uint8_t power;
    uint8_t invert;
    uint8_t poles;
  } motor_stab_data[3];
  uint8_t acc_limiter_all;
  int8_t ext_fc_gain;
  uint8_t reserved;
  struct {
    int16_t rc_min_angle;   // degrees
    int16_t rc_max_angle;   // degrees
    uint8_t rc_mode;        // 0..2 bits - mode: RC_MODE_ANGLE = 0; RC_MODE_SPEED = 1;; 3rd bit - control is inverted, if set to 1
    uint8_t rc_lpf;
    uint8_t rc_speed;
    uint8_t rc_follow;      // ROLL, PITCH: this value specify follow rate for flight controller. YAW: if value != 0, "follow motor" mode is enabled.
  } rc_data[3];
  uint8_t gyro_trust;
  uint8_t use_model;
  uint8_t pwm_freq;
  uint8_t serial_speed;
  int8_t rc_trim[3];
  uint8_t rc_deadband;
  uint8_t rc_expo_rate;
  uint8_t rc_virt_mode;
  uint8_t rc_map_roll;
  uint8_t rc_map_pitch;
  uint8_t rc_map_yaw;
  uint8_t rc_map_cmd;
  uint8_t rc_map_fc_roll;
  uint8_t rc_map_fc_pitch;
  uint8_t rc_mix_fc_roll;
  uint8_t rc_mix_fc_pitch;
  uint8_t follow_mode;
  uint8_t follow_deadband;
  uint8_t follow_expo_rate;
  int8_t follow_offset[3];
  int8_t axis_top;
  int8_t axis_right;
  int8_t frame_axis_top;
  int8_t frame_axis_right;
  uint8_t frame_imu_pos;
  uint8_t gyro_deadband;
  uint8_t gyro_sens;
  uint8_t i2c_speed_fast;
  uint8_t skip_gyro_calib;
  uint8_t rc_cmd_low;
  uint8_t rc_cmd_mid;
  uint8_t rc_cmd_high;
  uint8_t menu_cmd[5];
  uint8_t menu_cmd_long;
  uint8_t motor_output[3];
  int16_t bat_threshold_alarm;
  int16_t bat_threshold_motors;
  int16_t bat_comp_ref;
  uint8_t beeper_modes;
  uint8_t follow_roll_mix_start;
  uint8_t follow_roll_mix_range;
  uint8_t booster_power[3];
  uint8_t follow_speed[3];
  uint8_t frame_angle_from_motors;
  int16_t rc_memory[3];
  uint8_t servo_out[4];
  uint8_t servo_rate;
  uint8_t adaptive_pid_enabled;
  uint8_t adaptive_pid_threshold;
  uint8_t adaptive_pid_rate;
  uint8_t adaptive_pid_recovery_factor;
  uint8_t follow_lpf[3];
  uint16_t general_flags_1;
  uint16_t profile_flags_1;
  uint8_t spektrum_mode;
  uint8_t order_of_axes;
  uint8_t euler_order;
  uint8_t cur_imu;
  uint8_t cur_profile_id;
} SBGC_cmd_read_params_t;

// CMD_READ_PARAMS_EXT
typedef struct __attribute__((packed)) {
  // uint8_t mode;     // legacy format: mode is common for all axes
  uint8_t profile_id;	// Possible values: 0..4         Current (avtive) profile: 255
  struct {
    uint8_t notch_freq;
    uint8_t notch_width;
  } notch_data[3];
  uint16_t lpf_freq[3];
  uint8_t filters_en[3];
  int16_t encoder_offset[3];
  int16_t encoder_fld_offset[3];
  uint8_t encoder_manual_set_time[3];
  uint8_t motor_heating_factor[3];
  uint8_t motor_cooling_factor[3];
  uint8_t reserved[2];
  uint8_t follow_inside_deadband;
  uint8_t motor_mag_link[3];
  uint16_t motor_gearing[3];
  int8_t encoder_limit_min[3];
  int8_t encoder_limit_max[3];
  int8_t notch_gain[9];
  uint8_t beeper_volume;
  uint16_t encoder_gear_ratio[3];
  uint8_t encoder_type[3];
  uint8_t encoder_cfg[3];
  uint8_t outer_p[3];
  uint8_t outer_i[3];
  int8_t mag_axis_top;
  int8_t mag_axis_right;
  uint8_t mag_trust;
  int8_t mag_declination;
  uint16_t acc_lpf_freq;
  uint8_t d_term_lpf_freq[3];
} SBGC_cmd_read_params_ext_t;

// CMD_READ_PARAMS_EXT2
typedef struct __attribute__((packed)) {
  // uint8_t mode;     // legacy format: mode is common for all axes
  uint8_t profile_id;	// Possible values: 0..4         Current (avtive) profile: 255
  struct {
    uint8_t mav_src;
    uint8_t mav_sys_id;
    uint8_t mav_comp_id;
    uint8_t mav_cfg_flags;
    uint8_t mav_reserved[4];
  } mav_data[2];
  uint16_t motor_mag_link_fine[3];
  uint8_t acc_limiter[3];
  uint8_t pid_gain[3];
  uint8_t frame_imu_lpf_freq;
  uint8_t auto_pid_cfg;
  uint8_t auto_pid_gain;
  int16_t frame_cam_angle_min[3];
  int16_t frame_cam_angle_max[3];
  uint16_t general_flags_2;
  uint8_t auto_speed;
  uint8_t auto_acc_limiter;
  int16_t imu_orientation_corr[3];
  uint16_t timelapse_time;
  uint16_t emergency_stop_restart_delay;
  uint8_t timelapse_acc_part;
  uint16_t momentum[3];
  uint8_t momentum_calib_stimulus[3];
  uint8_t momentum_elitpicity[3];
  uint8_t follow_range[3];
  uint8_t stab_axis[3];
  uint8_t reserved[74];
} SBGC_cmd_read_params_ext2_t;

class SerialCommand {
public:
  uint8_t pos;
  uint8_t id;
  uint8_t data[SBGC_CMD_DATA_SIZE];
  uint8_t len;

  void init(uint8_t _id) {
    id = _id;
    len = 0;
    pos = 0;
  }
};

class GimbalSetup
{
private:
  boost::asio::io_service io_service_;
  serial_port_ptr port_;
  boost::mutex mutex_;

  std::string portName_;
  int baud_;

  char read_buf_raw_[SERIAL_PORT_READ_BUF_SIZE];
  std::string read_buf_str_;

  char end_of_line_char_ = '>';

  SerialCommand cmd_in, cmd_out;
  int len;
  uint8_t checksum = 0;
  enum STATE state = STATE_WAIT;
  enum TIMER_STATE timer_state = TIMER_STATE_READ_1;

  int drone_id_;

  ros::Timer timer;

  int _all_ok_cnt_ = 0;

  SBGC_cmd_read_params_t rt_data_default;

  void start();
  void async_read_some_();
  void on_receive_(const boost::system::error_code& ec, size_t bytes_transferred);
  void parseData(const std::string &data);
  void SBGC_write_params_pack(SBGC_cmd_read_params_t &p, SerialCommand &cmd);
  uint8_t SBGC_cmd_read_params_unpack(SBGC_cmd_read_params_t &p, SerialCommand &cmd);
  int write_some(const std::string &buf);
  int write_some(const char *buf, const int &size);
  void stop();
  void send_cmd();
  void gimbal_setup();
  void timerCallback(const ros::TimerEvent&);

public:

  GimbalSetup(int _argc, char** _argv)
  {
    ros::init(_argc, _argv, "Gimbal_Setup_");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<int>("drone_id", drone_id_, 1);
    pnh.param<std::string>("portName", portName_, std::string("/dev/ttyUSB0"));
    pnh.param<int>("baud", baud_, 115200);

    ROS_INFO("Gimbal Setup [%d] initialized! /n", drone_id_);

    //open serial
    if (port_) {
      std::cout << "error : port is already opened..." << std::endl;
      ros::shutdown();
    }

    start();

    timer = nh.createTimer(ros::Duration(5), &GimbalSetup::timerCallback, this);

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    stop();

    return;
  }

  // Destructor
  ~GimbalSetup(){};
};

void GimbalSetup::start()
{
  boost::system::error_code ec;

  port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
  port_->open(portName_, ec);

  while (ec) {
    std::cout << "Waiting for UART connection on port : "
      << portName_ << " ( " << ec.message().c_str() << ")" << std::endl;
      port_->open(portName_, ec);
      ros::Duration(1).sleep();
  }

  std::cout << "UART connected on port : "
    << portName_ << " ( " << ec.message().c_str() << ")" << std::endl;

  ROS_INFO("Setting up Gimbal Parameters...");

  // option settings...
  port_->set_option(boost::asio::serial_port_base::baud_rate(baud_));
  port_->set_option(boost::asio::serial_port_base::character_size(8));
  port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

  async_read_some_();

  // boost::asio::service should be launched after async_read_some so it has some work when launched preventing to return
  boost::thread t(boost::bind(&boost::asio::io_service::run, &io_service_));
}

void GimbalSetup::async_read_some_()
{
  if (port_.get() == NULL || !port_->is_open()) return;

  port_->async_read_some(
    boost::asio::buffer(read_buf_raw_, SERIAL_PORT_READ_BUF_SIZE),
    boost::bind(
      &GimbalSetup::on_receive_,
      this, boost::asio::placeholders::error,
      boost::asio::placeholders::bytes_transferred));
}

  /** \brief read loop on serial port
   *  \details this function read data on serial port and send every good frame to ds301 stack
   *  Warning : - this is a blocking function
   *            - init must be call before calling this function
   */
void GimbalSetup::on_receive_(const boost::system::error_code& ec, size_t bytes_transferred)
{
  boost::mutex::scoped_lock lock(mutex_);

  if (port_.get() == NULL || !port_->is_open()) return;
  if (ec) {
    async_read_some_();
    return;
  }

  for (unsigned int i = 0; i < bytes_transferred; ++i) {
    char c = read_buf_raw_[i];
    if (c == end_of_line_char_) {
      parseData(read_buf_str_);
      read_buf_str_.clear();
    }
    //else {
      read_buf_str_ += c;
    //}
  }

  async_read_some_();
}

void GimbalSetup::parseData(const std::string &data)
{

  if(data.size() == 0)
      return;

  uint8_t c;
  for(int i = 0; i < data.size(); i++){
    c = data.at(i);
    switch(state)
    {
      case STATE_WAIT:
        if(c == '>'){
          state = STATE_GOT_MARKER;
          ROS_DEBUG_STREAM("GOT_MARKER" << std::endl);
        }
        break;

      case STATE_GOT_MARKER:
        cmd_in.init(c);
        state = STATE_GOT_ID;
        ROS_DEBUG_STREAM("GOT_ID" << std::endl);
        break;

      case STATE_GOT_ID:
        len = c;
        state = STATE_GOT_LEN;
        ROS_DEBUG_STREAM("GOT_LEN" << std::endl);
        break;

      case STATE_GOT_LEN:
        if(c == (uint8_t)(cmd_in.id+len) && len <= sizeof(cmd_in.data)){
          checksum = 0;
          state = (len == 0) ? STATE_GOT_DATA : STATE_GOT_HEADER;
          if(len == 0)
            ROS_DEBUG_STREAM("GOT_DATA" << std::endl);
          else
            ROS_DEBUG_STREAM("GOT_HEADER" << std::endl);
        }else{
          state = STATE_WAIT;
          ROS_DEBUG_STREAM("WRONG HEADER CHECKSUM" << std::endl);
        }

        break;

      case STATE_GOT_HEADER:
        cmd_in.data[cmd_in.len++] = c;
        checksum+=c;
        if(cmd_in.len == len){
            state = STATE_GOT_DATA;
            ROS_DEBUG_STREAM("GOT_DATA" << std::endl);
          }
        break;

      case STATE_GOT_DATA:
        state = STATE_WAIT;
        if(c != checksum){
          ROS_DEBUG_STREAM("WRONG DATA CHECKSUM" << std::endl);
          return;
        }else
          ROS_DEBUG_STREAM("DATA CHECKSUM VERIFIED" << std::endl);
        break;
    }
  }

  if(state == STATE_WAIT){

	  if(cmd_in.id == CMD_READ_PARAMS_3){

      SBGC_cmd_read_params_t rt_data;
      SBGC_cmd_read_params_unpack(rt_data, cmd_in);

      _all_ok_cnt_ = 0;
      if (rt_data.profile_id == _PROFILE_ID_)
        _all_ok_cnt_++;
      if (rt_data.motor_stab_data[0].gain_P == _MOTOR_STAB_DATA_0_P_)
        _all_ok_cnt_++;
      if (rt_data.motor_stab_data[0].gain_I == _MOTOR_STAB_DATA_0_I_)
        _all_ok_cnt_++;
      if (rt_data.motor_stab_data[0].gain_D == _MOTOR_STAB_DATA_0_D_)
        _all_ok_cnt_++;
      if (rt_data.motor_stab_data[0].power == _MOTOR_STAB_DATA_0_POWER_)
        _all_ok_cnt_++;
      if (rt_data.motor_stab_data[0].invert == _MOTOR_STAB_DATA_0_INVERT_)
        _all_ok_cnt_++;
      if (rt_data.motor_stab_data[0].poles == _MOTOR_STAB_DATA_0_POLES_)
        _all_ok_cnt_++;
      if (rt_data.motor_stab_data[1].gain_P == _MOTOR_STAB_DATA_1_P_)
        _all_ok_cnt_++;
      if (rt_data.motor_stab_data[1].gain_I == _MOTOR_STAB_DATA_1_I_)
        _all_ok_cnt_++;
      if (rt_data.motor_stab_data[1].gain_D == _MOTOR_STAB_DATA_1_D_)
        _all_ok_cnt_++;
      if (rt_data.motor_stab_data[1].power == _MOTOR_STAB_DATA_1_POWER_)
        _all_ok_cnt_++;
      if (rt_data.motor_stab_data[1].invert == _MOTOR_STAB_DATA_1_INVERT_)
        _all_ok_cnt_++;
      if (rt_data.motor_stab_data[1].poles == _MOTOR_STAB_DATA_1_POLES_)
        _all_ok_cnt_++;
      if (rt_data.motor_stab_data[2].gain_P == _MOTOR_STAB_DATA_2_P_)
        _all_ok_cnt_++;
      if (rt_data.motor_stab_data[2].gain_I == _MOTOR_STAB_DATA_2_I_)
        _all_ok_cnt_++;
      if (rt_data.motor_stab_data[2].gain_D == _MOTOR_STAB_DATA_2_D_)
        _all_ok_cnt_++;
      if (rt_data.motor_stab_data[2].power == _MOTOR_STAB_DATA_2_POWER_)
        _all_ok_cnt_++;
      if (rt_data.motor_stab_data[2].invert == _MOTOR_STAB_DATA_2_INVERT_)
        _all_ok_cnt_++;
      if (rt_data.acc_limiter_all == _ACC_LIMITER_ALL_)
        _all_ok_cnt_++;
      if (rt_data.ext_fc_gain == _EXT_FC_GAIN_)
        _all_ok_cnt_++;
      if (rt_data.reserved == _RESERVED_)
        _all_ok_cnt_++;
      if (rt_data.rc_data[0].rc_min_angle == _RC_DATA_0_MIN_ANGLE_)
        _all_ok_cnt_++;
      if (rt_data.rc_data[0].rc_max_angle == _RC_DATA_0_MAX_ANGLE_)
        _all_ok_cnt_++;
      if (rt_data.rc_data[0].rc_mode == _RC_DATA_0_MODE_)
        _all_ok_cnt_++;
      if (rt_data.rc_data[0].rc_lpf == _RC_DATA_0_LPF_)
        _all_ok_cnt_++;
      if (rt_data.rc_data[0].rc_speed == _RC_DATA_0_SPEED_)
        _all_ok_cnt_++;
      if (rt_data.rc_data[0].rc_follow == _RC_DATA_0_FOLLOW_)
        _all_ok_cnt_++;
      if (rt_data.rc_data[0].rc_follow == _RC_DATA_0_FOLLOW_)
        _all_ok_cnt_++;
      if (rt_data.rc_data[1].rc_min_angle == _RC_DATA_1_MIN_ANGLE_)
        _all_ok_cnt_++;
      if (rt_data.rc_data[1].rc_max_angle == _RC_DATA_1_MAX_ANGLE_)
        _all_ok_cnt_++;
      if (rt_data.rc_data[1].rc_mode == _RC_DATA_1_MODE_)
        _all_ok_cnt_++;
      if (rt_data.rc_data[1].rc_lpf == _RC_DATA_1_LPF_)
        _all_ok_cnt_++;
      if (rt_data.rc_data[1].rc_speed == _RC_DATA_1_SPEED_)
        _all_ok_cnt_++;
      if (rt_data.rc_data[1].rc_follow == _RC_DATA_1_FOLLOW_)
        _all_ok_cnt_++;
      if (rt_data.rc_data[2].rc_min_angle == _RC_DATA_2_MIN_ANGLE_)
        _all_ok_cnt_++;
      if (rt_data.rc_data[2].rc_max_angle == _RC_DATA_2_MAX_ANGLE_)
        _all_ok_cnt_++;
      if (rt_data.rc_data[2].rc_mode == _RC_DATA_2_MODE_)
        _all_ok_cnt_++;
      if (rt_data.rc_data[2].rc_lpf == _RC_DATA_2_LPF_)
        _all_ok_cnt_++;
      if (rt_data.rc_data[2].rc_speed == _RC_DATA_2_SPEED_)
        _all_ok_cnt_++;
      if (rt_data.rc_data[2].rc_follow == _RC_DATA_2_FOLLOW_)
        _all_ok_cnt_++;
      if (rt_data.gyro_trust == _GYRO_TRUST_)
        _all_ok_cnt_++;
      if (rt_data.use_model == _USE_MODEL_)
        _all_ok_cnt_++;
      if (rt_data.pwm_freq == _PWM_FREQ_)
        _all_ok_cnt_++;
      if (rt_data.serial_speed == _SERIAL_SPEED_)
        _all_ok_cnt_++;
      if (rt_data.rc_trim[0] == _RC_TRIM_0_)
        _all_ok_cnt_++;
      if (rt_data.rc_trim[1] == _RC_TRIM_1_)
        _all_ok_cnt_++;
      if (rt_data.rc_trim[2] == _RC_TRIM_2_)
        _all_ok_cnt_++;
      if (rt_data.rc_deadband == _RC_DEADBAND_)
        _all_ok_cnt_++;
      if (rt_data.rc_expo_rate == _RC_EXPO_RATE_)
        _all_ok_cnt_++;
      if (rt_data.rc_virt_mode == _RC_VIRT_MODE_)
        _all_ok_cnt_++;
      if (rt_data.rc_map_roll == _RC_MAP_ROLL_)
        _all_ok_cnt_++;
      if (rt_data.rc_map_pitch == _RC_MAP_PITCH_)
        _all_ok_cnt_++;
      if (rt_data.rc_map_yaw == _RC_MAP_YAW_)
        _all_ok_cnt_++;
      if (rt_data.rc_map_cmd == _RC_MAP_CMD_)
        _all_ok_cnt_++;
      if (rt_data.rc_map_fc_roll == _RC_MAP_FC_ROLL_)
        _all_ok_cnt_++;
      if (rt_data.rc_map_fc_pitch == _RC_MAP_FC_PITCH_)
        _all_ok_cnt_++;
      if (rt_data.rc_mix_fc_roll == _RC_MIX_FC_ROLL_)
        _all_ok_cnt_++;
      if (rt_data.rc_mix_fc_pitch == _RC_MIX_FC_PITCH_)
        _all_ok_cnt_++;
      if (rt_data.follow_mode == _FOLLOW_MODE_)
        _all_ok_cnt_++;
      if (rt_data.follow_deadband == _FOLLOW_DEADBAND_)
        _all_ok_cnt_++;
      if (rt_data.follow_expo_rate == _FOLLOW_EXPO_RATE_)
        _all_ok_cnt_++;
      if (rt_data.follow_offset[0] == _FOLLOW_OFFSET_0_)
        _all_ok_cnt_++;
      if (rt_data.follow_offset[1] == _FOLLOW_OFFSET_1_)
        _all_ok_cnt_++;
      if (rt_data.follow_offset[2] == _FOLLOW_OFFSET_2_)
        _all_ok_cnt_++;
      if (rt_data.axis_top == _AXIS_TOP_)
        _all_ok_cnt_++;
      if (rt_data.axis_right == _AXIS_RIGHT_)
        _all_ok_cnt_++;
      if (rt_data.frame_axis_top == _FRAME_AXIS_TOP_)
        _all_ok_cnt_++;
      if (rt_data.frame_axis_right == _FRAME_AXIS_RIGHT_)
        _all_ok_cnt_++;
      if (rt_data.frame_imu_pos == _FRAME_IMU_POS_)
        _all_ok_cnt_++;
      if (rt_data.gyro_deadband == _GYRO_DEADBAND_)
        _all_ok_cnt_++;
      if (rt_data.gyro_sens == _GYRO_SENS_)
        _all_ok_cnt_++;
      if (rt_data.i2c_speed_fast == _I2C_SPEED_FAST_)
        _all_ok_cnt_++;
      if (rt_data.skip_gyro_calib == _SKIP_GYRO_CALIB_)
        _all_ok_cnt_++;
      if (rt_data.rc_cmd_low == _RC_CMD_LOW_)
        _all_ok_cnt_++;
      if (rt_data.rc_cmd_mid == _RC_CMD_MID_)
        _all_ok_cnt_++;
      if (rt_data.rc_cmd_high == _RC_CMD_HIGH_)
        _all_ok_cnt_++;
      if (rt_data.menu_cmd[0] == _MENU_CMD_0_)
        _all_ok_cnt_++;
      if (rt_data.menu_cmd[1] == _MENU_CMD_1_)
        _all_ok_cnt_++;
      if (rt_data.menu_cmd[2] == _MENU_CMD_2_)
        _all_ok_cnt_++;
      if (rt_data.menu_cmd[3] == _MENU_CMD_3_)
        _all_ok_cnt_++;
      if (rt_data.menu_cmd[4] == _MENU_CMD_4_)
        _all_ok_cnt_++;
      if (rt_data.menu_cmd_long == _MENU_CMD_LONG_)
        _all_ok_cnt_++;
      if (rt_data.motor_output[0] == _MOTOR_OUTPUT_0_)
        _all_ok_cnt_++;
      if (rt_data.motor_output[1] == _MOTOR_OUTPUT_1_)
        _all_ok_cnt_++;
      if (rt_data.motor_output[2] == _MOTOR_OUTPUT_2_)
        _all_ok_cnt_++;
      if (rt_data.bat_threshold_alarm == _BAT_THRESHOLD_ALARM_)
        _all_ok_cnt_++;
      if (rt_data.bat_threshold_motors == _BAT_THRESHOLD_MOTORS_)
        _all_ok_cnt_++;
      if (rt_data.bat_comp_ref == _BAT_COMP_REF_)
        _all_ok_cnt_++;
      if (rt_data.beeper_modes == _BEEPER_MODES_)
        _all_ok_cnt_++;
      if (rt_data.follow_roll_mix_start == _FOLLOW_ROLL_MIX_START_)
        _all_ok_cnt_++;
      if (rt_data.follow_roll_mix_range == _FOLLOW_ROLL_MIX_RANGE_)
        _all_ok_cnt_++;
      if (rt_data.booster_power[0] == _BOOSTER_POWER_0_)
        _all_ok_cnt_++;
      if (rt_data.booster_power[1] == _BOOSTER_POWER_1_)
        _all_ok_cnt_++;
      if (rt_data.booster_power[2] == _BOOSTER_POWER_2_)
        _all_ok_cnt_++;
      if (rt_data.follow_speed[0] == _FOLLOW_SPEED_0_)
        _all_ok_cnt_++;
      if (rt_data.follow_speed[1] == _FOLLOW_SPEED_1_)
        _all_ok_cnt_++;
      if (rt_data.follow_speed[2] == _FOLLOW_SPEED_2_)
        _all_ok_cnt_++;
      if (rt_data.frame_angle_from_motors == _FRAME_ANGLE_FROM_MOTORS_)
        _all_ok_cnt_++;
      if (rt_data.rc_memory[0] == _RC_MEMORY_0_)
        _all_ok_cnt_++;
      if (rt_data.rc_memory[1] == _RC_MEMORY_1_)
        _all_ok_cnt_++;
      if (rt_data.rc_memory[2] == _RC_MEMORY_2_)
        _all_ok_cnt_++;
      if (rt_data.servo_out[0] == _SERVO_OUT_0_)
        _all_ok_cnt_++;
      if (rt_data.servo_out[1] == _SERVO_OUT_1_)
        _all_ok_cnt_++;
      if (rt_data.servo_out[2] == _SERVO_OUT_2_)
        _all_ok_cnt_++;
      if (rt_data.servo_out[3] == _SERVO_OUT_3_)
        _all_ok_cnt_++;
      if (rt_data.servo_rate == _SERVO_RATE_)
        _all_ok_cnt_++;
      if (rt_data.adaptive_pid_enabled == _ADAPTIVE_PID_ENABLED_)
        _all_ok_cnt_++;
      if (rt_data.adaptive_pid_threshold == _ADAPTIVE_PID_THRESHOLD_)
        _all_ok_cnt_++;
      if (rt_data.adaptive_pid_rate == _ADAPTIVE_PID_RATE_)
        _all_ok_cnt_++;
      if (rt_data.adaptive_pid_recovery_factor == _ADAPTIVE_PID_RECOVERY_FACTOR_)
        _all_ok_cnt_++;
      if (rt_data.follow_lpf[0] == _FOLLOW_LPF_0_)
        _all_ok_cnt_++;
      if (rt_data.follow_lpf[1] == _FOLLOW_LPF_1_)
        _all_ok_cnt_++;
      if (rt_data.follow_lpf[2] == _FOLLOW_LPF_2_)
        _all_ok_cnt_++;
      if (rt_data.general_flags_1 == _GENERAL_FLAGS_1_)
        _all_ok_cnt_++;
      if (rt_data.profile_flags_1 == _PROFILE_FLAGS_1_)
        _all_ok_cnt_++;
      if (rt_data.spektrum_mode == _SPEKTRUM_MODE_)
        _all_ok_cnt_++;
      if (rt_data.order_of_axes == _ORDER_OF_AXES_)
        _all_ok_cnt_++;
      if (rt_data.euler_order == _EULER_ORDER_)
        _all_ok_cnt_++;
      if (rt_data.cur_imu == _CUR_IMU_)
        _all_ok_cnt_++;
      if (rt_data.cur_profile_id == _CUR_PROFILE_ID_)
        _all_ok_cnt_++;

      if (DEBUG_PRINT) {

        ROS_WARN("_all_ok_cnt_:");
        std::cout << _all_ok_cnt_ << '\n';

        ROS_INFO("CMD ID:                       %d", cmd_in.id);
    	  ROS_INFO("Profile ID:                   %d/%d", rt_data.profile_id,_PROFILE_ID_);
    	  ROS_INFO("Axis 1:");
    	  ROS_INFO("P:                            %d/%d",rt_data.motor_stab_data[0].gain_P,_MOTOR_STAB_DATA_0_P_);
    	  ROS_INFO("I:                            %d/%d",rt_data.motor_stab_data[0].gain_I,_MOTOR_STAB_DATA_0_I_);
    	  ROS_INFO("D:                            %d/%d",rt_data.motor_stab_data[0].gain_D,_MOTOR_STAB_DATA_0_D_);
    	  ROS_INFO("power:                        %d/%d",rt_data.motor_stab_data[0].power,_MOTOR_STAB_DATA_0_POWER_);
    	  ROS_INFO("invert:                       %d/%d",rt_data.motor_stab_data[0].invert,_MOTOR_STAB_DATA_0_INVERT_);
    	  ROS_INFO("poles:                        %d/%d",rt_data.motor_stab_data[0].poles,_MOTOR_STAB_DATA_0_POLES_);
    	  ROS_INFO("Axis 2:");
    	  ROS_INFO("P:                            %d/%d",rt_data.motor_stab_data[1].gain_P,_MOTOR_STAB_DATA_1_P_);
    	  ROS_INFO("I:                            %d/%d",rt_data.motor_stab_data[1].gain_I,_MOTOR_STAB_DATA_1_I_);
    	  ROS_INFO("D:                            %d/%d",rt_data.motor_stab_data[1].gain_D,_MOTOR_STAB_DATA_1_D_);
    	  ROS_INFO("power:                        %d/%d",rt_data.motor_stab_data[1].power,_MOTOR_STAB_DATA_1_POWER_);
    	  ROS_INFO("invert:                       %d/%d",rt_data.motor_stab_data[1].invert,_MOTOR_STAB_DATA_1_INVERT_);
    	  ROS_INFO("poles:                        %d/%d",rt_data.motor_stab_data[1].poles,_MOTOR_STAB_DATA_1_POLES_);
    	  ROS_INFO("Axis 3:");
    	  ROS_INFO("P:                            %d/%d",rt_data.motor_stab_data[2].gain_P,_MOTOR_STAB_DATA_2_P_);
    	  ROS_INFO("I:                            %d/%d",rt_data.motor_stab_data[2].gain_I,_MOTOR_STAB_DATA_2_I_);
    	  ROS_INFO("D:                            %d/%d",rt_data.motor_stab_data[2].gain_D,_MOTOR_STAB_DATA_2_D_);
    	  ROS_INFO("power:                        %d/%d",rt_data.motor_stab_data[2].power,_MOTOR_STAB_DATA_2_POWER_);
    	  ROS_INFO("invert:                       %d/%d",rt_data.motor_stab_data[2].invert,_MOTOR_STAB_DATA_2_INVERT_);
    	  ROS_INFO("poles:                        %d/%d",rt_data.motor_stab_data[2].poles,_MOTOR_STAB_DATA_2_POLES_);
    	  ROS_INFO("acc_limiter_all:              %d/%d",rt_data.acc_limiter_all,_ACC_LIMITER_ALL_);
    	  ROS_INFO("ext_fc_gain:                  %d/%d",rt_data.ext_fc_gain,_EXT_FC_GAIN_);
    	  ROS_INFO("reserved:                     %d/%d",rt_data.reserved,_RESERVED_);
    	  ROS_INFO("rc_min_angle 1:               %d/%d",rt_data.rc_data[0].rc_min_angle,_RC_DATA_0_MIN_ANGLE_);
    	  ROS_INFO("rc_max_angle:                 %d/%d",rt_data.rc_data[0].rc_max_angle,_RC_DATA_0_MAX_ANGLE_);
    	  ROS_INFO("rc_mode:                      %d/%d",rt_data.rc_data[0].rc_mode,_RC_DATA_0_MODE_);
    	  ROS_INFO("rc_lpf:                       %d/%d",rt_data.rc_data[0].rc_lpf,_RC_DATA_0_LPF_);
    	  ROS_INFO("rc_speed:                     %d/%d",rt_data.rc_data[0].rc_speed,_RC_DATA_0_SPEED_);
    	  ROS_INFO("rc_follow:                    %d/%d",rt_data.rc_data[0].rc_follow,_RC_DATA_0_FOLLOW_);
    	  ROS_INFO("rc_min_angle 2:               %d/%d",rt_data.rc_data[1].rc_min_angle,_RC_DATA_1_MIN_ANGLE_);
    	  ROS_INFO("rc_max_angle:                 %d/%d",rt_data.rc_data[1].rc_max_angle,_RC_DATA_1_MAX_ANGLE_);
    	  ROS_INFO("rc_mode:                      %d/%d",rt_data.rc_data[1].rc_mode,_RC_DATA_1_MODE_);
    	  ROS_INFO("rc_lpf:                       %d/%d",rt_data.rc_data[1].rc_lpf,_RC_DATA_1_LPF_);
    	  ROS_INFO("rc_speed:                     %d/%d",rt_data.rc_data[1].rc_speed,_RC_DATA_1_SPEED_);
    	  ROS_INFO("rc_follow:                    %d/%d",rt_data.rc_data[1].rc_follow,_RC_DATA_1_FOLLOW_);
    	  ROS_INFO("rc_min_angle 3:               %d/%d",rt_data.rc_data[2].rc_min_angle,_RC_DATA_2_MIN_ANGLE_);
    	  ROS_INFO("rc_max_angle:                 %d/%d",rt_data.rc_data[2].rc_max_angle,_RC_DATA_2_MAX_ANGLE_);
    	  ROS_INFO("rc_mode:                      %d/%d",rt_data.rc_data[2].rc_mode,_RC_DATA_2_MODE_);
    	  ROS_INFO("rc_lpf:                       %d/%d",rt_data.rc_data[2].rc_lpf,_RC_DATA_2_LPF_);
    	  ROS_INFO("rc_speed:                     %d/%d",rt_data.rc_data[2].rc_speed,_RC_DATA_2_SPEED_);
    	  ROS_INFO("rc_follow:                    %d/%d",rt_data.rc_data[2].rc_follow,_RC_DATA_2_FOLLOW_);
    	  ROS_INFO("Gyro Trust:                   %d/%d",rt_data.gyro_trust,_GYRO_TRUST_);
    	  ROS_INFO("use_model:                    %d/%d",rt_data.use_model,_USE_MODEL_);
    	  ROS_INFO("pwm_freq:                     %d/%d",rt_data.pwm_freq,_PWM_FREQ_);
    	  ROS_INFO("Serial Speed:                 %d/%d",rt_data.serial_speed,_SERIAL_SPEED_);
    	  ROS_INFO("rc_trim 1:                    %d/%d",rt_data.rc_trim[0],_RC_TRIM_0_);
    	  ROS_INFO("rc_trim 2:                    %d/%d",rt_data.rc_trim[1],_RC_TRIM_1_);
    	  ROS_INFO("rc_trim 3:                    %d/%d",rt_data.rc_trim[2],_RC_TRIM_2_);
    	  ROS_INFO("rc_deadband:                  %d/%d",rt_data.rc_deadband,_RC_DEADBAND_);
    	  ROS_INFO("rc_expo_rate:                 %d/%d",rt_data.rc_expo_rate,_RC_EXPO_RATE_);
    	  ROS_INFO("rc_virt_mode:                 %d/%d",rt_data.rc_virt_mode,_RC_VIRT_MODE_);
    	  ROS_INFO("rc_map_roll:                  %d/%d",rt_data.rc_map_roll,_RC_MAP_ROLL_);
    	  ROS_INFO("rc_map_pitch:                 %d/%d",rt_data.rc_map_pitch,_RC_MAP_PITCH_);
    	  ROS_INFO("rc_map_yaw:                   %d/%d",rt_data.rc_map_yaw,_RC_MAP_YAW_);
    	  ROS_INFO("rc_map_cmd:                   %d/%d",rt_data.rc_map_cmd,_RC_MAP_CMD_);
    	  ROS_INFO("rc_map_fc_roll:               %d/%d",rt_data.rc_map_fc_roll,_RC_MAP_FC_ROLL_);
    	  ROS_INFO("rc_map_fc_pitch:              %d/%d",rt_data.rc_map_fc_pitch,_RC_MAP_FC_PITCH_);
    	  ROS_INFO("rc_mix_fc_roll:               %d/%d",rt_data.rc_mix_fc_roll,_RC_MIX_FC_ROLL_);
    	  ROS_INFO("rc_mix_fc_pitch:              %d/%d",rt_data.rc_mix_fc_pitch,_RC_MIX_FC_PITCH_);
    	  ROS_INFO("follow_mode:                  %d/%d",rt_data.follow_mode,_FOLLOW_MODE_);
    	  ROS_INFO("follow_deadband:              %d/%d",rt_data.follow_deadband,_FOLLOW_DEADBAND_);
    	  ROS_INFO("follow_expo_rate:             %d/%d",rt_data.follow_expo_rate,_FOLLOW_EXPO_RATE_);
    	  ROS_INFO("follow_offset 1:              %d/%d",rt_data.follow_offset[0],_FOLLOW_OFFSET_0_);
    	  ROS_INFO("follow_offset 2:              %d/%d",rt_data.follow_offset[1],_FOLLOW_OFFSET_1_);
    	  ROS_INFO("follow_offset 3:              %d/%d",rt_data.follow_offset[2],_FOLLOW_OFFSET_2_);
    	  ROS_INFO("axis_top:                     %d/%d",rt_data.axis_top,_AXIS_TOP_);
    	  ROS_INFO("axis_right:                   %d/%d",rt_data.axis_right,_AXIS_RIGHT_);
    	  ROS_INFO("frame_axis_top:               %d/%d",rt_data.frame_axis_top,_FRAME_AXIS_TOP_);
    	  ROS_INFO("frame_axis_right:             %d/%d",rt_data.frame_axis_right,_FRAME_AXIS_RIGHT_);
    	  ROS_INFO("frame_imu_pos:                %d/%d",rt_data.frame_imu_pos,_FRAME_IMU_POS_);
    	  ROS_INFO("gyro_deadband:                %d/%d",rt_data.gyro_deadband,_GYRO_DEADBAND_);
    	  ROS_INFO("gyro_sens:                    %d/%d",rt_data.gyro_sens,_GYRO_SENS_);
    	  ROS_INFO("i2c_speed_fast:               %d/%d",rt_data.i2c_speed_fast,_I2C_SPEED_FAST_);
    	  ROS_INFO("skip_gyro_calib:              %d/%d",rt_data.skip_gyro_calib,_SKIP_GYRO_CALIB_);
    	  ROS_INFO("rc_cmd_low:                   %d/%d",rt_data.rc_cmd_low,_RC_CMD_LOW_);
    	  ROS_INFO("rc_cmd_mid:                   %d/%d",rt_data.rc_cmd_mid,_RC_CMD_MID_);
    	  ROS_INFO("rc_cmd_high:                  %d/%d",rt_data.rc_cmd_high,_RC_CMD_HIGH_);
    	  ROS_INFO("menu_cmd 1:                   %d/%d",rt_data.menu_cmd[0],_MENU_CMD_0_);
    	  ROS_INFO("menu_cmd 2:                   %d/%d",rt_data.menu_cmd[1],_MENU_CMD_1_);
    	  ROS_INFO("menu_cmd 3:                   %d/%d",rt_data.menu_cmd[2],_MENU_CMD_2_);
    	  ROS_INFO("menu_cmd 4:                   %d/%d",rt_data.menu_cmd[3],_MENU_CMD_3_);
    	  ROS_INFO("menu_cmd 5:                   %d/%d",rt_data.menu_cmd[4],_MENU_CMD_4_);
    	  ROS_INFO("menu_cmd_long:                %d/%d",rt_data.menu_cmd_long,_MENU_CMD_LONG_);
    	  ROS_INFO("motor_output 1:               %d/%d",rt_data.motor_output[0],_MOTOR_OUTPUT_0_);
    	  ROS_INFO("motor_output 2:               %d/%d",rt_data.motor_output[1],_MOTOR_OUTPUT_1_);
    	  ROS_INFO("motor_output 3:               %d/%d",rt_data.motor_output[2],_MOTOR_OUTPUT_2_);
    	  ROS_INFO("bat_threshold_alarm:          %d/%d",rt_data.bat_threshold_alarm,_BAT_THRESHOLD_ALARM_);
    	  ROS_INFO("bat_threshold_motors:         %d/%d",rt_data.bat_threshold_motors,_BAT_THRESHOLD_MOTORS_);
    	  ROS_INFO("bat_comp_ref:                 %d/%d",rt_data.bat_comp_ref,_BAT_COMP_REF_);
    	  ROS_INFO("beeper_modes:                 %d/%d",rt_data.beeper_modes,_BEEPER_MODES_);
    	  ROS_INFO("follow_roll_mix_start:        %d/%d",rt_data.follow_roll_mix_start,_FOLLOW_ROLL_MIX_START_);
    	  ROS_INFO("follow_roll_mix_range:        %d/%d",rt_data.follow_roll_mix_range,_FOLLOW_ROLL_MIX_RANGE_);
    	  ROS_INFO("booster_power 1:              %d/%d",rt_data.booster_power[0],_BOOSTER_POWER_0_);
    	  ROS_INFO("booster_power 2:              %d/%d",rt_data.booster_power[1],_BOOSTER_POWER_1_);
    	  ROS_INFO("booster_power 3:              %d/%d",rt_data.booster_power[2],_BOOSTER_POWER_2_);
    	  ROS_INFO("follow_speed 1:               %d/%d",rt_data.follow_speed[0],_FOLLOW_SPEED_0_);
    	  ROS_INFO("follow_speed 2:               %d/%d",rt_data.follow_speed[1],_FOLLOW_SPEED_1_);
    	  ROS_INFO("follow_speed 3:               %d/%d",rt_data.follow_speed[2],_FOLLOW_SPEED_2_);
    	  ROS_INFO("frame_angle_from_motors:      %d/%d",rt_data.frame_angle_from_motors,_FRAME_ANGLE_FROM_MOTORS_);
    	  ROS_INFO("rc_memory 1:                  %d/%d",rt_data.rc_memory[0],_RC_MEMORY_0_);
    	  ROS_INFO("rc_memory 2:                  %d/%d",rt_data.rc_memory[1],_RC_MEMORY_1_);
    	  ROS_INFO("rc_memory 3:                  %d/%d",rt_data.rc_memory[2],_RC_MEMORY_2_);
    	  ROS_INFO("servo_out 1:                  %d/%d",rt_data.servo_out[0],_SERVO_OUT_0_);
    	  ROS_INFO("servo_out 2:                  %d/%d",rt_data.servo_out[1],_SERVO_OUT_1_);
    	  ROS_INFO("servo_out 3:                  %d/%d",rt_data.servo_out[2],_SERVO_OUT_2_);
    	  ROS_INFO("servo_out 4:                  %d/%d",rt_data.servo_out[3],_SERVO_OUT_3_);
    	  ROS_INFO("servo_rate:                   %d/%d",rt_data.servo_rate,_SERVO_RATE_);
    	  ROS_INFO("adaptive_pid_enabled:         %d/%d",rt_data.adaptive_pid_enabled,_ADAPTIVE_PID_ENABLED_);
    	  ROS_INFO("adaptive_pid_threshold:       %d/%d",rt_data.adaptive_pid_threshold,_ADAPTIVE_PID_THRESHOLD_);
    	  ROS_INFO("adaptive_pid_rate:            %d/%d",rt_data.adaptive_pid_rate,_ADAPTIVE_PID_RATE_);
    	  ROS_INFO("adaptive_pid_recovery_factor: %d/%d",rt_data.adaptive_pid_recovery_factor,_ADAPTIVE_PID_RECOVERY_FACTOR_);
    	  ROS_INFO("follow_lpf 1:                 %d/%d",rt_data.follow_lpf[0],_FOLLOW_LPF_0_);
    	  ROS_INFO("follow_lpf 2:                 %d/%d",rt_data.follow_lpf[1],_FOLLOW_LPF_1_);
    	  ROS_INFO("follow_lpf 3:                 %d/%d",rt_data.follow_lpf[2],_FOLLOW_LPF_2_);
    	  ROS_INFO("general_flags_1:              %d/%d",rt_data.general_flags_1,_GENERAL_FLAGS_1_);
    	  ROS_INFO("profile_flags_1:              %d/%d",rt_data.profile_flags_1,_PROFILE_FLAGS_1_);
    	  ROS_INFO("spektrum_mode:                %d/%d",rt_data.spektrum_mode,_SPEKTRUM_MODE_);
    	  ROS_INFO("order_of_axes:                %d/%d",rt_data.order_of_axes,_ORDER_OF_AXES_);
    	  ROS_INFO("euler_order:                  %d/%d",rt_data.euler_order,_EULER_ORDER_);
    	  ROS_INFO("cur_imu:                      %d/%d",rt_data.cur_imu,_CUR_IMU_);
    	  ROS_INFO("cur_profile_id:               %d/%d",rt_data.cur_profile_id,_CUR_PROFILE_ID_);
      }
    } else{
      ROS_DEBUG("> %d\n", cmd_in.id);
    }
  }
}

/* Packs command structure to SerialCommand object */
void GimbalSetup::SBGC_write_params_pack(SBGC_cmd_read_params_t &p, SerialCommand &cmd)
{
	cmd.init(CMD_WRITE_PARAMS_3);
	memcpy(cmd.data, &p, sizeof(p));
	uint8_t tmp = sizeof(p);
	cmd.len = tmp;
}

/*
* Unpacks SerialCommand object to command structure.
* Returns 0 on success, PARSER_ERROR_XX code on fail.
*/
uint8_t GimbalSetup::SBGC_cmd_read_params_unpack(SBGC_cmd_read_params_t &p, SerialCommand &cmd)
{
    // ROS_INFO("cmd.len = %d",cmd.len);
    // ROS_INFO("sizeof(p) = %d",(int)sizeof(p));
  if(cmd.len <= sizeof(p)) {
      memcpy(&p, cmd.data, cmd.len);
      return 0;
    } else
      return 1;
}

int GimbalSetup::write_some(const std::string &buf)
{
  return write_some(buf.c_str(), buf.size());
}

int GimbalSetup::write_some(const char *buf, const int &size)
{
  boost::system::error_code ec;

  if (!port_) return -1;
  if (size == 0) return 0;

  return port_->write_some(boost::asio::buffer(buf,size),ec);
}

void GimbalSetup::stop()
{
  boost::mutex::scoped_lock lock(mutex_);

  if (port_) {
    port_->cancel();
    port_->close();
    port_.reset();
  }
  io_service_.stop();
  io_service_.reset();
}

void GimbalSetup::send_cmd()
{

  std::string data_str(">");
  data_str.push_back(cmd_out.id);
  data_str.push_back(cmd_out.len);
  data_str.push_back(cmd_out.id+cmd_out.len);

  uint8_t checksum = 0;
  for(int i = 0; i < cmd_out.len; i++){
    data_str.push_back(cmd_out.data[i]);
    checksum+=cmd_out.data[i];
  }
  data_str.push_back(checksum);

  int rep = write_some(data_str);
  if (!rep) {
    ROS_WARN("ERROR: No serial output");
  }

}

void GimbalSetup::gimbal_setup()
{

  SBGC_cmd_read_params_t rt_data;

  rt_data.profile_id                   = _PROFILE_ID_;
  rt_data.motor_stab_data[0].gain_P    = _MOTOR_STAB_DATA_0_P_;
  rt_data.motor_stab_data[0].gain_I    = _MOTOR_STAB_DATA_0_I_;
  rt_data.motor_stab_data[0].gain_D    = _MOTOR_STAB_DATA_0_D_;
  rt_data.motor_stab_data[0].power     = _MOTOR_STAB_DATA_0_POWER_;
  rt_data.motor_stab_data[0].invert    = _MOTOR_STAB_DATA_0_INVERT_;
  rt_data.motor_stab_data[0].poles     = _MOTOR_STAB_DATA_0_POLES_;
  rt_data.motor_stab_data[1].gain_P    = _MOTOR_STAB_DATA_1_P_;
  rt_data.motor_stab_data[1].gain_I    = _MOTOR_STAB_DATA_1_I_;
  rt_data.motor_stab_data[1].gain_D    = _MOTOR_STAB_DATA_1_D_;
  rt_data.motor_stab_data[1].power     = _MOTOR_STAB_DATA_1_POWER_;
  rt_data.motor_stab_data[1].invert    = _MOTOR_STAB_DATA_1_INVERT_;
  rt_data.motor_stab_data[1].poles     = _MOTOR_STAB_DATA_1_POLES_;
  rt_data.motor_stab_data[2].gain_P    = _MOTOR_STAB_DATA_2_P_;
  rt_data.motor_stab_data[2].gain_I    = _MOTOR_STAB_DATA_2_I_;
  rt_data.motor_stab_data[2].gain_D    = _MOTOR_STAB_DATA_2_D_;
  rt_data.motor_stab_data[2].power     = _MOTOR_STAB_DATA_2_POWER_;
  rt_data.motor_stab_data[2].invert    = _MOTOR_STAB_DATA_2_INVERT_;
  rt_data.motor_stab_data[2].poles     = _MOTOR_STAB_DATA_2_POLES_;
  rt_data.acc_limiter_all              = _ACC_LIMITER_ALL_;
  rt_data.ext_fc_gain                  = _EXT_FC_GAIN_;
  rt_data.reserved                     = _RESERVED_;
  rt_data.rc_data[0].rc_min_angle      = _RC_DATA_0_MIN_ANGLE_;
  rt_data.rc_data[0].rc_max_angle      = _RC_DATA_0_MAX_ANGLE_;
  rt_data.rc_data[0].rc_mode           = _RC_DATA_0_MODE_;
  rt_data.rc_data[0].rc_lpf            = _RC_DATA_0_LPF_;
  rt_data.rc_data[0].rc_speed          = _RC_DATA_0_SPEED_;
  rt_data.rc_data[0].rc_follow         = _RC_DATA_0_FOLLOW_;
  rt_data.rc_data[1].rc_min_angle      = _RC_DATA_1_MIN_ANGLE_;
  rt_data.rc_data[1].rc_max_angle      = _RC_DATA_1_MAX_ANGLE_;
  rt_data.rc_data[1].rc_mode           = _RC_DATA_1_MODE_;
  rt_data.rc_data[1].rc_lpf            = _RC_DATA_1_LPF_;
  rt_data.rc_data[1].rc_speed          = _RC_DATA_1_SPEED_;
  rt_data.rc_data[1].rc_follow         = _RC_DATA_1_FOLLOW_;
  rt_data.rc_data[2].rc_min_angle      = _RC_DATA_2_MIN_ANGLE_;
  rt_data.rc_data[2].rc_max_angle      = _RC_DATA_2_MAX_ANGLE_;
  rt_data.rc_data[2].rc_mode           = _RC_DATA_2_MODE_;
  rt_data.rc_data[2].rc_lpf            = _RC_DATA_2_LPF_;
  rt_data.rc_data[2].rc_speed          = _RC_DATA_2_SPEED_;
  rt_data.rc_data[2].rc_follow         = _RC_DATA_2_FOLLOW_;
  rt_data.gyro_trust                   = _GYRO_TRUST_;
  rt_data.use_model                    = _USE_MODEL_;
  rt_data.pwm_freq                     = _PWM_FREQ_;
  rt_data.serial_speed                 = _SERIAL_SPEED_;
  rt_data.rc_trim[0]                   = _RC_TRIM_0_;
  rt_data.rc_trim[1]                   = _RC_TRIM_1_;
  rt_data.rc_trim[2]                   = _RC_TRIM_2_;
  rt_data.rc_deadband                  = _RC_DEADBAND_;
  rt_data.rc_expo_rate                 = _RC_EXPO_RATE_;
  rt_data.rc_virt_mode                 = _RC_VIRT_MODE_;
  rt_data.rc_map_roll                  = _RC_MAP_ROLL_;
  rt_data.rc_map_pitch                 = _RC_MAP_PITCH_;
  rt_data.rc_map_yaw                   = _RC_MAP_YAW_;
  rt_data.rc_map_cmd                   = _RC_MAP_CMD_;
  rt_data.rc_map_fc_roll               = _RC_MAP_FC_ROLL_;
  rt_data.rc_map_fc_pitch              = _RC_MAP_FC_PITCH_;
  rt_data.rc_mix_fc_roll               = _RC_MIX_FC_ROLL_;
  rt_data.rc_mix_fc_pitch              = _RC_MIX_FC_PITCH_;
  rt_data.follow_mode                  = _FOLLOW_MODE_;
  rt_data.follow_deadband              = _FOLLOW_DEADBAND_;
  rt_data.follow_expo_rate             = _FOLLOW_EXPO_RATE_;
  rt_data.follow_offset[0]             = _FOLLOW_OFFSET_0_;
  rt_data.follow_offset[1]             = _FOLLOW_OFFSET_1_;
  rt_data.follow_offset[2]             = _FOLLOW_OFFSET_2_;
  rt_data.axis_top                     = _AXIS_TOP_;
  rt_data.axis_right                   = _AXIS_RIGHT_;
  rt_data.frame_axis_top               = _FRAME_AXIS_TOP_;
  rt_data.frame_axis_right             = _FRAME_AXIS_RIGHT_;
  rt_data.frame_imu_pos                = _FRAME_IMU_POS_;
  rt_data.gyro_deadband                = _GYRO_DEADBAND_;
  rt_data.gyro_sens                    = _GYRO_SENS_;
  rt_data.i2c_speed_fast               = _I2C_SPEED_FAST_;
  rt_data.skip_gyro_calib              = _SKIP_GYRO_CALIB_;
  rt_data.rc_cmd_low                   = _RC_CMD_LOW_;
  rt_data.rc_cmd_mid                   = _RC_CMD_MID_;
  rt_data.rc_cmd_high                  = _RC_CMD_HIGH_;
  rt_data.menu_cmd[0]                  = _MENU_CMD_0_;
  rt_data.menu_cmd[1]                  = _MENU_CMD_1_;
  rt_data.menu_cmd[2]                  = _MENU_CMD_2_;
  rt_data.menu_cmd[3]                  = _MENU_CMD_3_;
  rt_data.menu_cmd[4]                  = _MENU_CMD_4_;
  rt_data.menu_cmd_long                = _MENU_CMD_LONG_;
  rt_data.motor_output[0]              = _MOTOR_OUTPUT_0_;
  rt_data.motor_output[1]              = _MOTOR_OUTPUT_1_;
  rt_data.motor_output[2]              = _MOTOR_OUTPUT_2_;
  rt_data.bat_threshold_alarm          = _BAT_THRESHOLD_ALARM_;
  rt_data.bat_threshold_motors         = _BAT_THRESHOLD_MOTORS_;
  rt_data.bat_comp_ref                 = _BAT_COMP_REF_;
  rt_data.beeper_modes                 = _BEEPER_MODES_;
  rt_data.follow_roll_mix_start        = _FOLLOW_ROLL_MIX_START_;
  rt_data.follow_roll_mix_range        = _FOLLOW_ROLL_MIX_RANGE_;
  rt_data.booster_power[0]             = _BOOSTER_POWER_0_;
  rt_data.booster_power[1]             = _BOOSTER_POWER_1_;
  rt_data.booster_power[2]             = _BOOSTER_POWER_2_;
  rt_data.follow_speed[0]              = _FOLLOW_SPEED_0_;
  rt_data.follow_speed[1]              = _FOLLOW_SPEED_1_;
  rt_data.follow_speed[2]              = _FOLLOW_SPEED_2_;
  rt_data.frame_angle_from_motors      = _FRAME_ANGLE_FROM_MOTORS_;
  rt_data.rc_memory[0]                 = _RC_MEMORY_0_;
  rt_data.rc_memory[1]                 = _RC_MEMORY_1_;
  rt_data.rc_memory[2]                 = _RC_MEMORY_2_;
  rt_data.servo_out[0]                 = _SERVO_OUT_0_;
  rt_data.servo_out[1]                 = _SERVO_OUT_1_;
  rt_data.servo_out[2]                 = _SERVO_OUT_2_;
  rt_data.servo_out[3]                 = _SERVO_OUT_3_;
  rt_data.servo_rate                   = _SERVO_RATE_;
  rt_data.adaptive_pid_enabled         = _ADAPTIVE_PID_ENABLED_;
  rt_data.adaptive_pid_threshold       = _ADAPTIVE_PID_THRESHOLD_;
  rt_data.adaptive_pid_rate            = _ADAPTIVE_PID_RATE_;
  rt_data.adaptive_pid_recovery_factor = _ADAPTIVE_PID_RECOVERY_FACTOR_;
  rt_data.follow_lpf[0]                = _FOLLOW_LPF_0_;
  rt_data.follow_lpf[1]                = _FOLLOW_LPF_1_;
  rt_data.follow_lpf[2]                = _FOLLOW_LPF_2_;
  rt_data.general_flags_1              = _GENERAL_FLAGS_1_;
  rt_data.profile_flags_1              = _PROFILE_FLAGS_1_;
  rt_data.spektrum_mode                = _SPEKTRUM_MODE_;
  rt_data.order_of_axes                = _ORDER_OF_AXES_;
  rt_data.euler_order                  = _EULER_ORDER_;
  rt_data.cur_imu                      = _CUR_IMU_;
  rt_data.cur_profile_id               = _CUR_PROFILE_ID_;

  SBGC_write_params_pack(rt_data,cmd_out);
  send_cmd();
}

void GimbalSetup::timerCallback(const ros::TimerEvent&)
{
  if (DEBUG_PRINT)
    ROS_WARN("DEBUG TIMER state: %d",timer_state);
  if (_all_ok_cnt_ == 120) {
    ROS_WARN("Gimbal parameters successfully updated!");
    ros::shutdown();
  } else if (timer_state == TIMER_STATE_READ_1) {
    char data_[] = {'>', CMD_READ_PARAMS_3, 0, CMD_READ_PARAMS_3, 0, 0};
  	write_some((char*)data_, 6);
    timer_state = TIMER_STATE_READ_2;
	  if (DEBUG_PRINT)
      ROS_INFO("Read command sent!\n");
  } else if (timer_state == TIMER_STATE_READ_2) {
    char data_[] = {'>', CMD_READ_PARAMS_3, 0, CMD_READ_PARAMS_3, 0, 0};
  	write_some((char*)data_, 6);
    timer_state = TIMER_STATE_WRITE;
	  if (DEBUG_PRINT)
      ROS_INFO("Read command sent!\n");
  } else if (timer_state == TIMER_STATE_WRITE) {
    gimbal_setup();
    timer_state = TIMER_STATE_READ_1;
    if (DEBUG_PRINT)
	    ROS_INFO("Write command sent!\n");
  }
}

int main(int argc, char **argv)
{
  sleep(2);
  GimbalSetup gimbalSetup(argc, argv);
  return 0;
}
