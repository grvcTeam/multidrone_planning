#ifndef GIMBAL_INTERFACE_H
#define GIMBAL_INTERFACE_H

#include <ros/ros.h>
#include <ros/timer.h>
#include <ros/console.h>
#include <chrono>
#include <thread>
#include <boost/bind.hpp>
#include <fstream>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point32.h>
#include <multidrone_msgs/GimbalStatus.h>
#include <multidrone_msgs/CameraStatus.h>
#include <multidrone_msgs/CameraControl.h>
#include <cmath>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>

#include <boost/thread.hpp>
#include <libusbp-1/libusbp.hpp>
#include <eigen_conversions/eigen_msg.h>
#include "tf_conversions/tf_eigen.h"
#include <Eigen/Dense>
#include <signal.h>
#include <bitset>

#define SERIAL_PORT_READ_BUF_SIZE                256
#define SBGC_CMD_MAX_BYTES                       255
#define SBGC_CMD_NON_PAYLOAD_BYTES                5
#define SBGC_RC_NUM_CHANNELS                      6
#define SBGC_CMD_DATA_SIZE   (SBGC_CMD_MAX_BYTES - SBGC_CMD_NON_PAYLOAD_BYTES)
#define SBGC_IMU_TO_DEG                      0.02197265625
#define SBGC_IMU_TO_RAD              (SBGC_IMU_TO_DEG/180.0*M_PI)
#define SBGC_CONTROL_MODE_NO_CONTROL              0
#define SBGC_CONTROL_MODE_SPEED                   1
#define SBGC_CONTROL_MODE_ANGLE                   2
#define SBGC_SPEED_SCALE                  (1.0f/0.1220740379f)
#define SBGC_ANGLE_FULL_TURN                     16384
#define SBGC_DEGREE_ANGLE_SCALE    ((float)SBGC_ANGLE_FULL_TURN/360.0f)
#define SBGC_ANGLE_DEGREE_SCALE    (360.0f/(float)SBGC_ANGLE_FULL_TURN)
#define SBGC_ANGLE_SCALE                 (1.0f/SBGC_IMU_TO_DEG)
// Conversions for angle in degrees to angle in SBGC 14bit representation, and back
#define SBGC_DEGREE_TO_ANGLE(val)     ((val)*SBGC_DEGREE_ANGLE_SCALE)
#define SBGC_ANGLE_TO_DEGREE(val)     ((val)*SBGC_ANGLE_DEGREE_SCALE)
#define TO_DEG                                  180/M_PI
#define TO_RAD                                  M_PI/180

#define CMD_REALTIME_DATA_3                       23
#define CMD_REALTIME_DATA_4                       25
#define CMD_CONTROL		                            67
#define CMD_REALTIME_DATA                         68
#define CMD_DATA_STREAM_INTERVAL                  85
#define CMD_REALTIME_DATA_CUSTOM                  88
#define CMD_MOTORS_ON                             77 //turns on motor and leaves it in the desired position
#define CMD_MOTORS_OFF                           109 //turns off motor
#define CMD_CALIB_GYRO                           103
#define CMD_CALIB_MOTOR_MAG_LINK                  74
#define CMD_CALIB_MAG                             59
#define CMD_CALIB_ORIENT_CORR                     91
#define CMD_RESET                                114 //resets the board, we're npt onterested in that

#define PUB_FREQ                                30.0  // Hz
#define CHECK_BIT(var,pos)                 ((var) & (1<<(pos)))

//CAMERA

#define SBUS_WR_FREQ	100 //Hz
#define PUBLISH_STATUS_FREQ	30 //Hz

#define	PUSH_BUTTON_TIME	0.2 //seconds
#define	PUSH_HOLD_BUTTON_TIME 1 //seconds

#define	SBUS_FRAME_HEADER	0x0f
#define	SBUS_FRAME_FOOTER	0x00
#define	SBUS2_FRAME_FOOTER	0x04
#define	SBUS_PACKET_LENGTH	25
#define	SBUS_NUMBER_CHANNELS	18

/* Default SBUS values for channels without trim */
#define	SBUS_MIN_OFFSET	173
#define	SBUS_MID_OFFSET	992
#define	SBUS_MAX_OFFSET	1811

#define	DEFAULT_SBUS_RECORD_CH	13
#define	DEFAULT_SBUS_IRIS_CH	7
#define	DEFAULT_SBUS_FOCUS_CH	1
#define	DEFAULT_SBUS_AFOCUS_CH	12
#define	DEFAULT_SBUS_ZOOM_CH	3
#define	DEFAULT_SBUS_ISO_CH	9
#define	DEFAULT_SBUS_SHUTTER_CH	10
#define	DEFAULT_SBUS_WHITE_CH	11
#define	DEFAULT_SBUS_AUDIO_CH	15
#define	DEFAULT_SBUS_FRAME_CH	5
#define	DEFAULT_SBUS_CODEC_CH	6

#define	BMMCC_MIN_OFFSET	44
#define	BMMCC_MID_OFFSET	128
#define	BMMCC_MAX_OFFSET	212

#define	DEFAULT_BMMCC_RECORD	BMMCC_MID_OFFSET
#define	DEFAULT_BMMCC_IRIS	BMMCC_MIN_OFFSET
#define	DEFAULT_BMMCC_FOCUS	BMMCC_MID_OFFSET
#define	DEFAULT_BMMCC_AFOCUS	BMMCC_MID_OFFSET
#define	DEFAULT_BMMCC_ZOOM	BMMCC_MID_OFFSET
#define	DEFAULT_BMMCC_ISO	BMMCC_MID_OFFSET
#define	DEFAULT_BMMCC_SHUTTER	BMMCC_MID_OFFSET
#define	DEFAULT_BMMCC_WHITE	BMMCC_MID_OFFSET
#define	DEFAULT_BMMCC_AUDIO	BMMCC_MID_OFFSET
#define	DEFAULT_BMMCC_FRAME	95 //50FPS
#define	DEFAULT_BMMCC_CODEC	150 // VALUE_CODEC_ProRes422

enum STATE
{
  STATE_WAIT, STATE_GOT_MARKER, STATE_GOT_ID, STATE_GOT_LEN, STATE_GOT_HEADER, STATE_GOT_DATA
};

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

// CMD_CONTROL
typedef struct __attribute__((packed)) {
  // uint8_t mode;                            // legacy format: mode is common for all axes
  uint8_t mode[3];	                          // in extended format (firmware ver. 2.55b5+): mode is set independently for each axes
  int16_t speedROLL;
  int16_t angleROLL;
  int16_t speedPITCH;
  int16_t anglePITCH;
  int16_t speedYAW;
  int16_t angleYAW;
} SBGC_cmd_control_t;

// CMD_REALTIME_DATA_3, CMD_REALTIME_DATA_4
typedef struct __attribute__((aligned)) {
  struct {
    int16_t   acc_data;
    int16_t   gyro_data;
  } sensor_data[3];                               // ACC and Gyro sensor data (with calibration) for current IMU (see cur_imu field)
  
  uint16_t    serial_error_cnt;                   // counter for communication errors
  uint16_t    system_error;                       // system error flags, defined in SBGC_SYS_ERR_XX
  uint8_t     reserved1[4];
  int16_t     rc_raw_data[SBGC_RC_NUM_CHANNELS];  // RC signal in 1000..2000 range for ROLL, PITCH, YAW, CMD, EXT_ROLL, EXT_PITCH channels
  int16_t     imu_angle[3];                       // ROLL, PITCH, YAW Euler angles of a camera, 16384/360 degrees
  int16_t     frame_imu_angle[3];                 // ROLL, PITCH, YAW Euler angles of a frame, if known
  int16_t     target_angle[3];                    // ROLL, PITCH, YAW target angle
  uint16_t    cycle_time_us;                      // cycle time in us. Normally should be 800us
  uint16_t    i2c_error_count;                    // I2C errors counter
  uint8_t     reserved2;
  uint16_t    battery_voltage;                    // units 0.01 V
  uint8_t     state_flags1;                       // bit0: motor ON/OFF state;  bits1..7: reserved
  uint8_t     cur_imu;                            // actually selecteted IMU for monitoring. 1: main IMU, 2: frame IMU
  uint8_t     cur_profile;                        // active profile number starting from 0
  uint8_t     motor_power[3];                     // actual motor power for ROLL, PITCH, YAW axis, 0..255

  // Fields below are filled only for CMD_REALTIME_DATA_4 command
  int16_t     rotor_angle[3];                     // relative angle of each motor, 16384/360 degrees
  uint8_t     reserved3;
  int16_t     balance_error[3];                   // error in balance. Ranges from -512 to 512,  0 means perfect balance.
  uint16_t    current;                            // Current that gimbal takes, in mA.
  int16_t     magnetometer_data[3];               // magnetometer sensor data (with calibration)
  int8_t      imu_temp_celcius;                   // temperature measured by the main IMU sensor, in Celsius
  int8_t      frame_imu_temp_celcius;             // temperature measured by the frame IMU sensor, in Celsius
  uint8_t     IMU_G_ERR;
  uint8_t     IMU_H_ERR;
  int16_t     MOTOR_OUT[3];
  uint8_t     reserved4[30];
} SBGC_cmd_realtime_data_t;

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

class GimbalInterface  {

	struct BMMCCcmdtiming
	{
		unsigned char function;
		unsigned char value;
		ros::Time timeout;
	};
  std::vector<BMMCCcmdtiming> BMMCCcmd;
  enum{RECORD_CH, IRIS_CH, FOCUS_CH, AFOCUS_CH, ZOOM_CH, ISO_CH, SHUTTER_CH, WHITE_CH, AUDIO_CH, FRAME_CH, CODEC_CH, NFUNCTIONS};
  int BMMCCdata[NFUNCTIONS];

public:

  ros::Publisher status_pub;
  ros::Publisher euler_pub;
  ros::Publisher vel_pub;
  ros::Publisher power_pub;

  ros::Subscriber cmd_sub;
  ros::Subscriber droneInfo_sub;

  boost::asio::io_service io_service_;
  serial_port_ptr port_;
  boost::mutex mutex_;

  std::string portName_;
  // int baud_;

  char read_buf_raw_[SERIAL_PORT_READ_BUF_SIZE];
  std::string read_buf_str_;

  char end_of_line_char_ = '>';
  
  SerialCommand cmd_in, cmd_out;
  int len;
  uint8_t checksum = 0;
  enum STATE state = STATE_WAIT;

  int gimbal_id;

  ros::Timer write_to_gimbal_timer;
  ros::Timer gimbal_timer_pub;

  Eigen::Quaterniond gimbal_quat;
  geometry_msgs::Vector3 gimbal_euler;
  geometry_msgs::Vector3 gimbal_euler_;
  geometry_msgs::Vector3 diff;
  geometry_msgs::Vector3 diff_;
  std::vector<geometry_msgs::Vector3> gimbal_ang_vel = std::vector<geometry_msgs::Vector3>(4);  
  geometry_msgs::Vector3 gimbal_motor_power;
  double offset = 0;
  double gimbal_yaw_;
  double drone_yaw_;  
  Eigen::Quaterniond  drone_att_;
  bool   has_gimbal_status_;
  bool   has_gimbal_calibration = false;
  bool   motorON = true;
  double t_0 = 0;
  double tim = 0;
  int    count = 0;
  int initial_pitch = -90;

  //Camera
	ros::Publisher cstatus_publisher;
	multidrone_msgs::CameraStatus cs;
	unsigned int SBUSreadData[SBUS_NUMBER_CHANNELS];
	int SBUSch[NFUNCTIONS];
  ros::Timer camera_timer_pub;
	ros::ServiceServer camera_control_service;


  bool cameracontrolServiceCallback (multidrone_msgs::CameraControl::Request &req, multidrone_msgs::CameraControl::Response &res);
	void pushBMMCCcmd(unsigned char BMMCCfunction, unsigned char value, ros::Time timeout);
	void checkBMMCCcmd(void);
	bool send_camera_cmd(unsigned int* cameradata, int arraysize);
	void timerCallback_camera_status(const ros::TimerEvent& event);
	void timerCallback_write_camera(const ros::TimerEvent& event);
  void camera_reset();


  void init();
  void start();
  void async_read_some_();
  void on_receive_(const boost::system::error_code& ec, size_t bytes_transferred);
  void parseData(const std::string &data);
  void SBGC_cmd_control_pack(int command, SBGC_cmd_control_t &p, SerialCommand &cmd);
  uint8_t SBGC_cmd_realtime_data_unpack(SBGC_cmd_realtime_data_t &p, SerialCommand &cmd);
  int write_some(const std::string &buf);     
  int write_some(const char *buf, const int &size);
  void send_cmd();
  void timerCallback(const ros::TimerEvent&);
  void timerCallbackPub(const ros::TimerEvent&);
  void cmd_callback(const geometry_msgs::Vector3::ConstPtr& msg);
  void droneInfo_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void turnOff();
  void turnOn();
  void calibration();
  double getDiff();


// Constructor
  GimbalInterface(){};

// Destructor
  ~GimbalInterface(){};
  void initi(int _argc, char** _argv);
  void stop();
};

#endif