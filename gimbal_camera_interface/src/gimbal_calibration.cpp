#include <gimbal_camera_interface/gimbal_camera_interface.h>
#include <boost/bind.hpp>
#include <fstream>
#include <string>
#include <iostream>


int product_id;
int vendor_id;
int baud_;

int main(int _argc, char **_argv)
{
  ROS_INFO("Calibrating gimbal");
  GimbalInterface gimbal;
  sleep(.5);

  
  gimbal.initi(_argc,_argv);
  ros::shutdown();
  return 0;
}

void main_with_exceptions(std::string &port_name)
{
    libusbp::device device = libusbp::find_device_with_vid_pid(vendor_id, product_id);
    if (device){
			libusbp::serial_port port(device);
	    port_name = port.get_name();
		}
}

  void GimbalInterface::initi(int _argc, char** _argv)
  {
    ros::init(_argc, _argv, "Gimbal_Control_");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<int>("baud", baud_, 115200);
    pnh.param<int>("vend", vendor_id, 0x16c0);// 10c4);
    pnh.param<int>("prod", product_id,0x0483);// ea60); 
    
    std::cout << std::hex << vendor_id << '\n';
    std::cout << std::hex << product_id << '\n';

    //open serial
    if (port_) {
      std::cout << "error : port is already opened..." << std::endl;
      ros::shutdown();
    }

    start();

    turnOff();
    port_->cancel();
    port_->close();
    port_.reset();
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
    }
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
  // boost::mutex::scoped_lock lock(mutex_);
  // double a = ros::Time::now().toSec();
  // static double b = 0;
  // std::cout << chamador <<"\n";// << a-b <<"   ";
  // b = a;
  // std::cout << "  " << buf.size() << "\n";
  return write_some(buf.c_str(), buf.size());
}

int GimbalInterface::write_some(const char *buf, const int &size)
{
  boost::system::error_code ec;

  if (!port_) return -1;
  if (size == 0) return 0;

  return port_->write_some(boost::asio::buffer(buf, size), ec);
}

void GimbalInterface::send_cmd()
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

void GimbalInterface::timerCallback(const ros::TimerEvent&)
{
  char data_[] = {'>', CMD_REALTIME_DATA_4, 0, CMD_REALTIME_DATA_4, 0, 0};
  write_some((char*)data_, 6);
}

void GimbalInterface::cmd_callback(const geometry_msgs::Vector3::ConstPtr& _msg)
{
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
  std::cout << "turn on" << std::endl;
  SBGC_cmd_control_t c = {0,0,0,0,0,0,0,0,0};
  SBGC_cmd_control_pack(CMD_MOTORS_ON, c, cmd_out);
  send_cmd();
}

void GimbalInterface::turnOff()
{
  std::cout << "turn off" << std::endl;
	SBGC_cmd_control_t c = {2,0,0,0,0,0,0,0,0}; //number 2 is for slow shutdown. only works for firmware version 2.68b7
  SBGC_cmd_control_pack(CMD_MOTORS_OFF, c, cmd_out);
  send_cmd();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  std::cout << "Calibrating" << std::endl;
  c = {0,0,0,0,0,0,0,0,0}; //number 2 is for slow shutdown. only works for firmware version 2.68b7
  SBGC_cmd_control_pack(CMD_CALIB_GYRO, c, cmd_out);
  send_cmd();
  std::this_thread::sleep_for(std::chrono::milliseconds(10000));
  std::cout << "Calibrated" << std::endl;
  turnOn();
}