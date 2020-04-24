#include <gimbal_camera_interface/gimbal_camera_interface.h>
#include <interface_test/interface_test.h>
#include <boost/bind.hpp>
#include <fstream>
#include <string>
#include <iostream>

int product_id;
int vendor_id;
int baud_;

void main_with_exceptions(std::string &port_name)
{
	
    libusbp::device device = libusbp::find_device_with_vid_pid(vendor_id, product_id);
    if (device){
			libusbp::serial_port port(device);//, interface_number, composite);
	    port_name = port.get_name();
		}
}

  void GimbalInterface::initi(int _argc, char** _argv)
  {
    ros::init(_argc, _argv, "Gimbal_Control_", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // CameraInterface cam;
    pnh.param<int>("SBUS_RECORD_CH", SBUSch[RECORD_CH], DEFAULT_SBUS_RECORD_CH);
	  pnh.param<int>("SBUS_IRIS_CH", SBUSch[IRIS_CH], DEFAULT_SBUS_IRIS_CH);
	  pnh.param<int>("SBUS_FOCUS_CH", SBUSch[FOCUS_CH], DEFAULT_SBUS_FOCUS_CH);
	  pnh.param<int>("SBUS_AFOCUS_CH", SBUSch[AFOCUS_CH], DEFAULT_SBUS_AFOCUS_CH);
	  pnh.param<int>("SBUS_ZOOM_CH", SBUSch[ZOOM_CH], DEFAULT_SBUS_ZOOM_CH);
	  pnh.param<int>("SBUS_ISO_CH", SBUSch[ISO_CH], DEFAULT_SBUS_ISO_CH);
	  pnh.param<int>("SBUS_SHUTTER_ANGLE_CH", SBUSch[SHUTTER_CH], DEFAULT_SBUS_SHUTTER_CH);
	  pnh.param<int>("SBUS_WHITE_BALANCE_CH", SBUSch[WHITE_CH], DEFAULT_SBUS_WHITE_CH);
	  pnh.param<int>("SBUS_AUDIO_CH", SBUSch[AUDIO_CH], DEFAULT_SBUS_AUDIO_CH);
	  pnh.param<int>("SBUS_FRAME_RATE_CH", SBUSch[FRAME_CH], DEFAULT_SBUS_FRAME_CH);
	  pnh.param<int>("SBUS_CODEC_CH", SBUSch[CODEC_CH], DEFAULT_SBUS_CODEC_CH);
	  /*Select camera starting functions values*/
	  pnh.param<int>("BMMCC_IRIS_INIT", BMMCCdata[IRIS_CH], DEFAULT_BMMCC_IRIS);
	  pnh.param<int>("BMMCC_FOCUS_INIT", BMMCCdata[FOCUS_CH], DEFAULT_BMMCC_FOCUS);
	  pnh.param<int>("BMMCC_AUDIO_INIT", BMMCCdata[AUDIO_CH], DEFAULT_BMMCC_AUDIO);
	  pnh.param<int>("BMMCC_FRAME_RATE_INIT", BMMCCdata[FRAME_CH], DEFAULT_BMMCC_FRAME);
	  pnh.param<int>("BMMCC_CODEC_INIT", BMMCCdata[CODEC_CH], DEFAULT_BMMCC_CODEC);
    BMMCCdata[RECORD_CH]  = DEFAULT_BMMCC_RECORD;
	  BMMCCdata[AFOCUS_CH]  = DEFAULT_BMMCC_AFOCUS;
	  BMMCCdata[ZOOM_CH]    = DEFAULT_BMMCC_ZOOM;
	  BMMCCdata[ISO_CH]     = DEFAULT_BMMCC_ISO;
	  BMMCCdata[SHUTTER_CH] = DEFAULT_BMMCC_SHUTTER;
	  BMMCCdata[WHITE_CH]   = DEFAULT_BMMCC_WHITE;

    pnh.param<int>("baud", baud_, 115200);
    pnh.param<int>("vend", vendor_id, 0x10c4);
    pnh.param<int>("prod", product_id,0xea60);
    pnh.param<int>("gimbal_id", gimbal_id,0);     

    if(gimbal_id == 1) initial_pitch = 0;

    ROS_INFO("Gimbal Camera Interface initialized");

    // Publisher
    status_pub              = nh.advertise<multidrone_msgs::GimbalStatus>("gimbal/status", 1);    
    cstatus_publisher       = nh.advertise<multidrone_msgs::CameraStatus>("camera_status", 1);
    
    // Service
    camera_control_service  = nh.advertiseService("camera_control", &GimbalInterface::cameracontrolServiceCallback, this);

    // Subscribers
    cmd_sub                 = nh.subscribe("gimbal/cmd",1,&GimbalInterface::cmd_callback, this);
    droneInfo_sub           = nh.subscribe<geometry_msgs::PoseStamped>("ual/pose",1,&GimbalInterface::droneInfo_callback, this);
	  
    boost::shared_ptr<geometry_msgs::PoseStamped const> sharedPtr;
    do{
      sharedPtr  = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("ual/pose", ros::Duration(0.5));
    } while(sharedPtr == NULL);
    ROS_INFO("Got drone's orientation");

    //open serial
    if (port_) {
      std::cout << "error : port is already opened..." << std::endl;
      ros::shutdown();
    }

    for (int i = 0; i < 4; ++i) {
      gimbal_ang_vel[i].x = 0;
      gimbal_ang_vel[i].y = 0;
      gimbal_ang_vel[i].z = 0;
    }

    start();
    camera_reset();

    write_to_gimbal_timer   = nh.createTimer(ros::Duration(0.01), &GimbalInterface::timerCallback, this);
    gimbal_timer_pub        = nh.createTimer(ros::Duration(1/PUB_FREQ), &GimbalInterface::timerCallbackPub, this);

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return;
}

void GimbalInterface::start()
{
  boost::system::error_code ec;

  port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
  port_->open(portName_, ec);

  while (ec && ros::ok()) {
    main_with_exceptions(portName_);
    std::cout << "Waiting for UART connection on port : " << portName_ << " ( " << ec.message().c_str() << ")" << std::endl;
    port_->open(portName_, ec);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  std::cout << "UART connected on port : " << portName_ << " ( " << ec.message().c_str() << ")" << std::endl;

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

void GimbalInterface::async_read_some_()
{
  if (port_.get() == NULL || !port_->is_open()) return;
  port_->async_read_some(boost::asio::buffer(read_buf_raw_, SERIAL_PORT_READ_BUF_SIZE),boost::bind(&GimbalInterface::on_receive_, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

/** \brief read loop on serial port
 *  \details this function read data on serial port and send every good frame to ds301 stack
 *  Warning : - this is a blocking function
 *            - init must be call before calling this function
 */
void GimbalInterface::on_receive_(const boost::system::error_code& ec, size_t bytes_transferred)
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
    read_buf_str_ += c;
  }

  async_read_some_();
}

void GimbalInterface::parseData(const std::string &data)
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
	  if(cmd_in.id == CMD_REALTIME_DATA_3 || cmd_in.id == CMD_REALTIME_DATA_4){
      SBGC_cmd_realtime_data_t rt_data;
      SBGC_cmd_realtime_data_unpack(rt_data, cmd_in);


      if(gimbal_id == 0) {
	      gimbal_quat = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) * \
	                    Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()) * \
	                    Eigen::AngleAxisd((rt_data.imu_angle[2]*SBGC_IMU_TO_RAD - offset *TO_RAD), Eigen::Vector3d::UnitZ()) * \
	                    Eigen::AngleAxisd(rt_data.imu_angle[0]*SBGC_IMU_TO_RAD, Eigen::Vector3d::UnitX()) * \
	                    Eigen::AngleAxisd(-rt_data.imu_angle[1]*SBGC_IMU_TO_RAD, Eigen::Vector3d::UnitY());   

	      gimbal_ang_vel[0].x = -rt_data.sensor_data[0].gyro_data/32767.0*2000.0*M_PI/180.0;
	      gimbal_ang_vel[0].y = -rt_data.sensor_data[1].gyro_data/32767.0*2000.0*M_PI/180.0;
	      gimbal_ang_vel[0].z =  rt_data.sensor_data[2].gyro_data/32767.0*2000.0*M_PI/180.0;
	      
	      gimbal_euler.x = rt_data.imu_angle[0]*SBGC_IMU_TO_DEG;
	      gimbal_euler.y = -rt_data.imu_angle[1]*SBGC_IMU_TO_DEG;
	      gimbal_euler.z = rt_data.imu_angle[2]*SBGC_IMU_TO_DEG - offset;
       } else {
       	  gimbal_quat = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) * \
	                    Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()) * \
	                    Eigen::AngleAxisd((rt_data.imu_angle[2]*SBGC_IMU_TO_RAD - offset *TO_RAD), Eigen::Vector3d::UnitZ()) * \
	                    Eigen::AngleAxisd(rt_data.imu_angle[0]*SBGC_IMU_TO_RAD, Eigen::Vector3d::UnitX()) * \
	                    Eigen::AngleAxisd(-rt_data.imu_angle[1]*SBGC_IMU_TO_RAD+M_PI/2, Eigen::Vector3d::UnitY());   
	      
	      gimbal_euler.x = rt_data.imu_angle[0]*SBGC_IMU_TO_DEG;
	      gimbal_euler.y = -rt_data.imu_angle[1]*SBGC_IMU_TO_DEG + 90;
	      gimbal_euler.z = rt_data.imu_angle[2]*SBGC_IMU_TO_DEG - offset;
       }

      diff.z = gimbal_euler.z - gimbal_euler_.z;
      gimbal_euler_ = gimbal_euler;

      if (CHECK_BIT(rt_data.system_error,9)){ 
        tim = ros::Time::now().toSec() - t_0;
        if (tim > 3)
          count = 0; 
        count += 1;
        if (count > 80){
            ROS_INFO("Vibrating");
            turnOff();
            count = 0;
        }          
        t_0 = ros::Time::now().toSec();
      }
      
      std::rotate(gimbal_ang_vel.rbegin(), gimbal_ang_vel.rbegin() + 1, gimbal_ang_vel.rend());
      has_gimbal_status_ = true;
    } else
		    ROS_DEBUG("> %d\n", cmd_in.id);
  }
}

/* Packs command structure to SerialCommand object */
void GimbalInterface::SBGC_cmd_control_pack(int command, SBGC_cmd_control_t &p, SerialCommand &cmd)
{
	cmd.init(command);
	memcpy(cmd.data, &p, sizeof(p));
	uint8_t tmp = sizeof(p);
	cmd.len = tmp;
}

  /*
* Unpacks SerialCommand object to command structure.
* Returns 0 on success, PARSER_ERROR_XX code on fail.
*/
uint8_t GimbalInterface::SBGC_cmd_realtime_data_unpack(SBGC_cmd_realtime_data_t &p, SerialCommand &cmd)
{
    if(cmd.len <= sizeof(p)) {
      memcpy(&p, cmd.data, cmd.len);
      return 0;
    } else
      return 1;
}

int GimbalInterface::write_some(const std::string &buf)
{
  return write_some(buf.c_str(), buf.size());
}

int GimbalInterface::write_some(const char *buf, const int &size)
{
  boost::system::error_code ec;

  if (!port_) return -1;
  if (size == 0) return 0;
  return port_->write_some(boost::asio::buffer(buf, size), ec); //this function stalls if the serial receiver (teensy) is corrupted 
}

void GimbalInterface::send_cmd(){
  
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
  if (!rep)
    ROS_WARN("ERROR: No serial output");
}

void GimbalInterface::timerCallback(const ros::TimerEvent&)
{
  char data_[] = {'>', CMD_REALTIME_DATA_4, 0, CMD_REALTIME_DATA_4, 0, 0};
  write_some((char*)data_, 6);
}

void GimbalInterface::timerCallbackPub(const ros::TimerEvent&)
{
  if (has_gimbal_status_) {
    multidrone_msgs::GimbalStatus msg_status;
    geometry_msgs::Vector3   msg_euler;
    geometry_msgs::Vector3   msg_vel;

    msg_status.header.stamp = ros::Time::now();
    msg_status.orientation.x = gimbal_quat.x();
    msg_status.orientation.y = gimbal_quat.y();
    msg_status.orientation.z = gimbal_quat.z();
    msg_status.orientation.w = gimbal_quat.w();
    msg_status.angular_velocity = gimbal_ang_vel[0];
    msg_status.roll  = gimbal_euler.x;
    msg_status.pitch = gimbal_euler.y;
    msg_status.yaw   = gimbal_euler.z;

    msg_euler = gimbal_euler;
    msg_vel   = gimbal_ang_vel[0]; 

    if (!has_gimbal_calibration)
      calibration();

    status_pub.publish(msg_status);
  }
  has_gimbal_status_ = false;
}

bool GimbalInterface::send_camera_cmd(unsigned int* cameradata, int arraysize){  
    std::string data_str;

    for(int i = 0; i < arraysize; i++){
     data_str.push_back(cameradata[i] >> 8);
     data_str.push_back(cameradata[i]);
    }
    int rep = write_some(data_str);
if (!rep)
      ROS_WARN("ERROR: No serial output");
    else
      return true;
    return false;
  };


void GimbalInterface::pushBMMCCcmd(unsigned char BMMCCfunction, unsigned char value, ros::Time timeout)
{
	struct BMMCCcmdtiming cmd;

	cmd.function = BMMCCfunction;
	cmd.value = value;
	cmd.timeout = timeout;
	this->BMMCCcmd.push_back(cmd);
}

/*
 *Handle camera commands array
 */
void GimbalInterface::checkBMMCCcmd()
{
	if(this->BMMCCcmd.empty())
		return;

	for(auto cmd = this->BMMCCcmd.begin(); cmd != this->BMMCCcmd.end();){
		if(cmd->timeout <= ros::Time::now()){
			BMMCCdata[cmd->function] = BMMCC_MID_OFFSET;
			cmd = this->BMMCCcmd.erase(cmd);
		}
		else{
			BMMCCdata[cmd->function] = cmd->value;
			++cmd ;
		}
	}
}

bool GimbalInterface::cameracontrolServiceCallback (multidrone_msgs::CameraControl::Request &req, multidrone_msgs::CameraControl::Response &res){

	if(req.value < BMMCC_MIN_OFFSET)
		 req.value = BMMCC_MIN_OFFSET;
	else
		if(req.value > BMMCC_MAX_OFFSET)
			 req.value = BMMCC_MAX_OFFSET;

	ROS_INFO("Camera_Control: Control received %d %d %d", req.control_function, req.value, req.manual_ctrl);

	switch(req.control_function){

		// /*push-button like camera functions*/
		case(multidrone_msgs::CameraControl::Request::CONTROL_FUNCTION_RECORD): // Status: Values range in [On OFF]
		case(multidrone_msgs::CameraControl::Request::CONTROL_FUNCTION_AFOCUS): // Status: No status
		case(multidrone_msgs::CameraControl::Request::CONTROL_FUNCTION_ISO): //Status: Values range in [200 400 800 1600]
		case(multidrone_msgs::CameraControl::Request::CONTROL_FUNCTION_SHUTTER): //
		case(multidrone_msgs::CameraControl::Request::CONTROL_FUNCTION_WHITE): // Status: Values range [2500K : 8000K]

			pushBMMCCcmd(req.control_function, req.value, ros::Time::now()+ros::Duration(PUSH_BUTTON_TIME));
			break;

		/*push and hold like camera functions*/
		case(multidrone_msgs::CameraControl::Request::CONTROL_FUNCTION_ZOOM):

			pushBMMCCcmd(req.control_function, req.value, ros::Time::now()+ros::Duration(PUSH_HOLD_BUTTON_TIME));
			break;

		// /*knob like camera functions*/
		case(multidrone_msgs::CameraControl::Request::CONTROL_FUNCTION_IRIS):
		case(multidrone_msgs::CameraControl::Request::CONTROL_FUNCTION_FOCUS):
		case(multidrone_msgs::CameraControl::Request::CONTROL_FUNCTION_AUDIO): //TODO
		case(multidrone_msgs::CameraControl::Request::CONTROL_FUNCTION_FRAME):
		case(multidrone_msgs::CameraControl::Request::CONTROL_FUNCTION_CODEC):

			BMMCCdata[req.control_function] = req.value;
			break;

    case(multidrone_msgs::CameraControl::Request::CAMERA_RESET):
      camera_reset();
      break;
		default:
			ROS_WARN("camera_control: Unmatched camera funtion to control");
  }

      	/* Check if camera is to be manually controlled by the cameraman's radio */
	if (req.manual_ctrl != cs.manual_ctrl){
		cs.manual_ctrl = req.manual_ctrl;

		ROS_INFO(cs.manual_ctrl ? "camera_control: MANUAL Remote Control":"camera_control: AUTOMATIC Control");
	}

  static int write_error_cnt;
	unsigned int SBUSwriteData[SBUS_NUMBER_CHANNELS];

	/* initialize with BMMCC_MID_OFFSET << 3 */
	memset(SBUSwriteData, 1024, sizeof SBUSwriteData);

	checkBMMCCcmd();

	for(int i=0; i<NFUNCTIONS; i++)
		SBUSwriteData[SBUSch[i]-1] = (BMMCCdata[i] << 3);

  

  /* If manual control then write what was read */ /*with black magic can't read data from camera */
	// if(cs.manual_ctrl)
	// 	for(int i=0; i<SBUS_NUMBER_CHANNELS; i++)
	// 		SBUSwriteData[i] = SBUSreadData[i];

	if (send_camera_cmd(SBUSwriteData,sizeof(SBUSwriteData)/sizeof(SBUSwriteData[0]))){
		cs.trigger_record = (unsigned char) (SBUSwriteData[SBUSch[RECORD_CH]-1] >> 3);
		cs.iris           = (unsigned char) (SBUSwriteData[SBUSch[IRIS_CH]-1] >> 3);
		cs.focus          = (unsigned char) (SBUSwriteData[SBUSch[FOCUS_CH]-1] >> 3);
		cs.auto_focus     = (unsigned char) (SBUSwriteData[SBUSch[AFOCUS_CH]-1] >> 3);
		cs.zoom           = (unsigned char) (SBUSwriteData[SBUSch[ZOOM_CH]-1] >> 3);
		cs.iso            = (unsigned char) (SBUSwriteData[SBUSch[ISO_CH]-1] >> 3);
		cs.shutter_angle  = (unsigned char) (SBUSwriteData[SBUSch[SHUTTER_CH]-1] >> 3);
		cs.white_balance  = (unsigned char) (SBUSwriteData[SBUSch[WHITE_CH]-1] >> 3);
		cs.audio_levels   = (unsigned char) (SBUSwriteData[SBUSch[AUDIO_CH]-1] >> 3);
		cs.frame_rate     = (unsigned char) (SBUSwriteData[SBUSch[FRAME_CH]-1] >> 3);
		cs.codec          = (unsigned char) (SBUSwriteData[SBUSch[CODEC_CH]-1] >> 3);

		write_error_cnt = 0;
		cs.SBUS_write_error = false;
    return true;
	}
	else{
		write_error_cnt ++;
		if( write_error_cnt > SBUS_WR_FREQ) /* 1s of consecutive write error */
      cs.SBUS_write_error = true;
    return false;
	}
}

void GimbalInterface::droneInfo_callback(const geometry_msgs::PoseStamped::ConstPtr&  _msg){
  tf::quaternionMsgToEigen(_msg->pose.orientation, drone_att_);
  drone_yaw_ = atan2(2*(drone_att_.w()*drone_att_.z()+drone_att_.x()*drone_att_.y()),1-2*(drone_att_.y()*drone_att_.y()+drone_att_.z()*drone_att_.z()));
}

void GimbalInterface::cmd_callback(const geometry_msgs::Vector3::ConstPtr& _msg)
{
  if (interface_test_giae) {
    run_test("giae", interface_test_giae);
  }
  
  // std::cout << drone_yaw_*TO_DEG << std::endl;

  if (has_gimbal_calibration && has_gimbal_status_){
      int16_t speedROLL_   = _msg->x;
	    int16_t speedPITCH_  = _msg->y;
	    int16_t speedYAW_    = _msg->z;
    
	    SBGC_cmd_control_t c = {0,0,0,0,0,0,0,0,0};
    
      c.mode[0]    = SBGC_CONTROL_MODE_SPEED;
      c.mode[1]    = SBGC_CONTROL_MODE_SPEED;
      c.mode[2]    = SBGC_CONTROL_MODE_SPEED;
      c.speedROLL  = speedROLL_*SBGC_SPEED_SCALE;
      c.speedPITCH = -speedPITCH_*SBGC_SPEED_SCALE;
      c.speedYAW   = speedYAW_*SBGC_SPEED_SCALE;
      c.angleROLL  = 0;
      c.anglePITCH = 0;
      c.angleYAW   = 0;
      
      // std::cout << remainder(gimbal_euler.z +drone_yaw_*TO_DEG,360) << std::endl;
      if ((remainder(gimbal_euler.z +drone_yaw_*TO_DEG,360)>= 160 && speedYAW_>0) || (remainder(gimbal_euler.z +drone_yaw_*TO_DEG,360)<= -120 && speedYAW_<0)){
        c.speedROLL  = 0;
        c.speedPITCH = 0;
        c.speedYAW   = 0;
      }
        SBGC_cmd_control_pack(CMD_CONTROL, c, cmd_out);
        send_cmd();
  }
}

void GimbalInterface::turnOn()
{
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  std::cout << "turn on" << std::endl;
  SBGC_cmd_control_t c = {0,0,0,0,0,0,0,0,0};
  SBGC_cmd_control_pack(CMD_MOTORS_ON, c, cmd_out);
  send_cmd();
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  offset = 0;
  motorON = true;
  has_gimbal_calibration = false;
}

void GimbalInterface::turnOff()
{
  offset = 0;
  motorON = false;
  std::cout << "turn off" << std::endl;
	SBGC_cmd_control_t c = {2,0,0,0,0,0,0,0,0}; //number 2 is for slow shutdown. only works for firmware version 2.68b7
  SBGC_cmd_control_pack(CMD_MOTORS_OFF, c, cmd_out);
  send_cmd();
  turnOn();
}

void GimbalInterface::calibration()
{
std::cout <<"Calibrating";

  SBGC_cmd_control_t c = {0,0,0,0,0,0,0,0,0};

  int16_t speedROLL_   =   0;
	int16_t speedPITCH_  =   0;
	int16_t speedYAW_    =  20;

  c.mode[0]    = SBGC_CONTROL_MODE_SPEED;
  c.mode[1]    = SBGC_CONTROL_MODE_SPEED;
  c.mode[2]    = SBGC_CONTROL_MODE_SPEED;
  c.speedROLL  = speedROLL_*SBGC_SPEED_SCALE;
  c.speedPITCH = -speedPITCH_*SBGC_SPEED_SCALE;
  c.speedYAW   = speedYAW_*SBGC_SPEED_SCALE;
  c.angleROLL  = 0;
  c.anglePITCH = 0;
  c.angleYAW   = 0;

  if(!motorON)
    return;
  SBGC_cmd_control_pack(CMD_CONTROL, c, cmd_out);
  send_cmd();

  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  while (diff.z > 0.0){
    if(!motorON)
      return;   
  }

  ROS_INFO("Hit at  %f",gimbal_euler.z);
  double home  = gimbal_euler.z - 180; 

  c.mode[0]    = SBGC_CONTROL_MODE_ANGLE;
  c.mode[1]    = SBGC_CONTROL_MODE_ANGLE;
  c.mode[2]    = SBGC_CONTROL_MODE_ANGLE;
  c.speedROLL  = 0;
  c.speedPITCH = 0;
  c.speedYAW   = -0.1;
  c.angleROLL  = 0;
  c.anglePITCH = SBGC_DEGREE_TO_ANGLE(initial_pitch);
  c.angleYAW   = SBGC_DEGREE_TO_ANGLE(home);

  if(!motorON)
    return;

  SBGC_cmd_control_pack(CMD_CONTROL, c, cmd_out);
  send_cmd();

  std::this_thread::sleep_for(std::chrono::milliseconds(800));
  while (diff.z != 0) {
    if(!motorON)
    return;
  }
  
  offset = home + drone_yaw_*TO_DEG;
  if(!motorON)
    return;
  has_gimbal_calibration = true;
  ROS_INFO("Calibrated");
  ROS_INFO("Offset is %f and drone yaw is %f", home, drone_yaw_*TO_DEG); 

}

void GimbalInterface::camera_reset(){
multidrone_msgs::CameraControl *camera_control_msg_ = new multidrone_msgs::CameraControl;
    int decrease = 128;
    camera_control_msg_->request.control_function = multidrone_msgs::CameraControl::Request::CONTROL_FUNCTION_ZOOM;
    camera_control_msg_->request.manual_ctrl = false;
    camera_control_msg_->request.value = 128;

    for (int i=0; i < 25; i++){
      decrease--;
      camera_control_msg_->request.value = decrease;
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      cameracontrolServiceCallback(camera_control_msg_->request, camera_control_msg_->response);
    }
    delete camera_control_msg_;
}   

void GimbalInterface::stop()
{
  boost::mutex::scoped_lock lock(mutex_);

	int16_t speedROLL_   = 0;
	int16_t speedPITCH_  = 0;
	int16_t speedYAW_    = 0;

	SBGC_cmd_control_t c = {0,0,0,0,0,0,0,0,0};

	c.mode[0]    = SBGC_CONTROL_MODE_SPEED;
	c.mode[1]    = SBGC_CONTROL_MODE_SPEED;
	c.mode[2]    = SBGC_CONTROL_MODE_SPEED;
	c.speedROLL  = speedROLL_*SBGC_SPEED_SCALE;
	c.speedPITCH = -speedPITCH_*SBGC_SPEED_SCALE;
	c.speedYAW   = speedYAW_*SBGC_SPEED_SCALE;
	c.angleROLL  = 0;
	c.anglePITCH = 0;
	c.angleYAW   = 0;

	SBGC_cmd_control_pack(CMD_CONTROL, c, cmd_out);
	send_cmd();
  
  if (port_) {
    port_->cancel();
    port_->close();
    port_.reset();
  }
  io_service_.stop();
  io_service_.reset();
  std::cout << "Outter";
}

