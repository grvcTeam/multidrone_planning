/** Author: Alfonso Alcantara
 *  Email: aamarin@us.es
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>
#include <random>
#include <stdio.h>
#include <stdexcept>
#include <sdf/sdf.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float32.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>

#include <ignition/math.hh>
#include <iostream>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <multidrone_msgs/TargetStateArray.h>
#include <iostream>
#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

#include <unistd.h>
#include <termios.h>


namespace gazebo
{

/* Class definition */  //{
class dynamicTargetPlugin : public ModelPlugin {
public:
  dynamicTargetPlugin() : ModelPlugin(){
      printf("Target constructor\n");
  }
  /*! \brief Called when a Plugin is first created, and after the World has been loaded to INITIALIZE 
  *         This function read the sdf elements, initialize services to control the plugin, loadMap(), start timer, wait for a key
  *   \param _parent pointer to the model
  *   \param _sdf pointer to the sdf element of the plugin
  */
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/); 

private:
  physics::ModelPtr  model_;  /*! pointer to the model */
  ros::NodeHandle *  nh_;
  ros::ServiceServer activation_srv, reset_srv;
  ros::Subscriber    load_map_sub;
  ros::Publisher     pose_pub;   /*< target pos ros publisher */
  ros::Timer         motion_timer;
  ros::Timer         timer_pose;
  std::atomic<bool> interrupted;

  void loadMapCallback(const std_msgs::String &msg);
  bool activationCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
  bool resetCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /** \brief function to load the map from a txt file and save data in pitch_angles, yaw_angles, roll_angles, updated_trajectory, velocities
   */
  void   loadMap();
  /** \brief function that segmentate the loaded trajectory
  */
  void   segmentateMap();
  void   OnUpdate();
  /**! \brief Callback to move the dynamic target. This function call moveModel() and manage the trajectory pointer
   */
  void   motionTimerCallback(const ros::TimerEvent &event);
  /** \brief set current pose
   */
  void   moveModel();

  /**! \brief utility funciton to get euclidean distance betwenn two poses
   *   \param p_1
   *   \param p_2
   *   \return euclidean
   */
  double euclideanDistance(ignition::math::Pose3d p_1, ignition::math::Pose3d p_2);

  /**! \brief
   */
  ignition::math::Quaternion<double> convertEulerOffsetToQuaternion(double roll_offset, double pitch_offset, double yaw_offset);
  /** \brief get points between two point to adjust velocity
   *  \param p_1 from that point
   *  \param p_2 to that point
   *  \param current_angles current angles
   *  \param index_from current index of uploaded_trajectory
   *  \return return a vector with the point xyzrpy and steps between p_1 and p_2
   */
  std::vector<double>    getSegmentationSteps(ignition::math::Pose3d p_1, ignition::math::Pose3d p_2, std::vector<double> current_angles, int index_from);
  /** \brief
  */
  ignition::math::Pose3d getShiftedPose(ignition::math::Pose3d p, std::vector<double> xyzrpy_step);

  void publishPoseTimer(const ros::TimerEvent &event);

  ///////////////////
  int getch(void);
  /////////////////////

  ////////////////////////////////////
void motionTimerKeyboardCallback(const ros::TimerEvent &event);
//////////////////////////////////////////////////////

  std::string            trajectory_file;
  std::string            parent_name;  /*< model name */
  double                 update_rate;
  int                    trajectory_pointer;
  bool                   tracking_active;
  bool                   initial_on;
  bool                   loop_enabled;
  bool                   use_segmentation;
  bool                   use_directional_yaw;
  bool                   map_loaded = false;
  bool                   initialized = false;

  char key =' ';


  std::vector<ignition::math::Pose3d> uploaded_trajectory;
  std::vector<ignition::math::Pose3d> segmented_trajectory;
  std::vector<double>                 velocities;
  std::vector<double>                 yaw_angles;
  std::vector<double>                 roll_angles;
  std::vector<double>                 pitch_angles;
  ignition::math::Pose3d              current_pose;
  double                              current_yaw = 0.0;
  ignition::math::Pose3d previous_pose; /*< previous pose to calculate the velocity */

};
//}

int dynamicTargetPlugin::getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

///////////////////////////////////////////
/* Load */  //{
void dynamicTargetPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

  ROS_INFO("[%s]: Dynamic target plugin started.", ros::this_node::getName().c_str());
  model_      = _parent;
  parent_name = model_->GetName().c_str();
  std::string topic_name = "";
  bool keyboard = false;
  
  if(_sdf->HasElement("topic_name")){
    topic_name = _sdf->Get<std::string>("topic_name");
  }else
  {
    ROS_WARN("[%s][Target]: topic name not defined. Setting to default value", parent_name.c_str());
    topic_name = "/target_3d_state"; // /targets_pose
  }
  
  if (_sdf->HasElement("update_rate")) {
    update_rate = _sdf->Get<double>("update_rate");
  } else {
    ROS_WARN("[%s][Dynamic target]: Update_rate not defined. Setting to defalt value.", parent_name.c_str());
    update_rate = 30;
  }
  if(_sdf->HasElement("keyboard")){
    ROS_INFO("[%s][Target]: keyboard actived", parent_name.c_str());
    keyboard = _sdf->Get<bool>("keyboard");
  }else
  {
      ROS_WARN("[%s][Target]: keyboard not defined. Setting to default value", parent_name.c_str());
      keyboard = false;
  }
  if (_sdf->HasElement("initial_on")) {
    initial_on = _sdf->Get<bool>("initial_on");
  } else {
    ROS_WARN("[%s][Dynamic target]: Initial_on not defined. Setting to defalt value.", parent_name.c_str());
    initial_on = true;
  }
  if (_sdf->HasElement("loop_enabled")) {
    loop_enabled = _sdf->Get<bool>("loop_enabled");
  } else {
    ROS_WARN("[%s][Dynamic target]: Loop_enabled not defined. Setting to defalt value.", parent_name.c_str());
    loop_enabled = true;
  }
  if (_sdf->HasElement("trajectory_file")) {
    trajectory_file = _sdf->Get<std::string>("trajectory_file");
    ROS_INFO("[%s]: Trajectory file = %s ", ros::this_node::getName().c_str(), trajectory_file.c_str());
  } else {
    ROS_WARN("[%s][Dynamic target]: Map_path not defined. Has to be loaded by publishing on topic.", parent_name.c_str());
  }
  if (_sdf->HasElement("use_segmentation")) {
    use_segmentation = _sdf->Get<bool>("use_segmentation");
  } else {
    ROS_WARN("[%s][Dynamic target]: Use segmentation not defined. Setting to default value.", parent_name.c_str());
    use_segmentation = false;
  }
  if (_sdf->HasElement("use_directional_yaw")) {
    use_directional_yaw = _sdf->Get<bool>("use_directional_yaw");
  } else {
    ROS_WARN("[%s][Dynamic target]: Use directional yaw not defined. Setting to default value.", parent_name.c_str());
    use_directional_yaw = false;
  }

  // initialize current pose

  current_pose.Pos().X() = 0.0;
  current_pose.Pos().Y() = 0.0;
  current_pose.Pos().Z() = 0.0;
  // initialize ROS
  int    argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "dynamic_traget_plugin", ros::init_options::NoSigintHandler);
  nh_ = new ros::NodeHandle("~");

  // create ROS services to control the light
  std::stringstream ss;
  ss << "/gazebo/dynamic_target/" << parent_name << "/load_map";
  load_map_sub = nh_->subscribe(ss.str().c_str(), 1, &dynamicTargetPlugin::loadMapCallback, this);
  ss.str(std::string());
  ss << "/gazebo/dynamic_target/" << parent_name << "/activate";
  activation_srv = nh_->advertiseService(ss.str().c_str(), &dynamicTargetPlugin::activationCallback, this);
  ss.str(std::string());
  ss << "/gazebo/dynamic_target/" << parent_name << "/reset";
  reset_srv = nh_->advertiseService(ss.str().c_str(), &dynamicTargetPlugin::resetCallback, this);
  ss.str(std::string());
  ss << topic_name;


  pose_pub     = nh_->advertise<multidrone_msgs::TargetStateArray>(ss.str().c_str(), 1);

  timer_pose = nh_->createTimer(ros::Rate(update_rate), &dynamicTargetPlugin::publishPoseTimer,this);

  // start timer
  if(keyboard){ 
    motion_timer = nh_->createTimer(ros::Rate(update_rate), &dynamicTargetPlugin::motionTimerKeyboardCallback,this);

  }else{
      motion_timer = nh_->createTimer(ros::Rate(update_rate), &dynamicTargetPlugin::motionTimerCallback, this);
      if (!trajectory_file.empty()) {
        loadMap();
      }
  }

  initialized = true;
  ROS_INFO("[%s][Light]: Dynamic target plugin initialized.", parent_name.c_str());

  if (initial_on && map_loaded) {
    tracking_active = true;
    ROS_INFO("[%s]: Initial on flag set to true. Tracking activated.", ros::this_node::getName().c_str());
  }

  if(keyboard){
    std::cout<<"Use keyboard to move the target: "<<std::endl;
    std::cout<<"W -> forward"<<std::endl;
    std::cout<<"S -> Backward"<<std::endl;
    std::cout<<"A -> Left"<<std::endl;
    std::cout<<"D -> Right"<<std::endl;
    std::cout<<"Use J-K to move the orientation of the target"<<std::endl;
    std::cout<<"Press b to quit"<<std::endl;
  }

}
//}

/* convertEulerOffsetToQuaternion() //{ */
ignition::math::Quaternion<double> dynamicTargetPlugin::convertEulerOffsetToQuaternion(double roll_offset, double pitch_offset, double yaw_offset) {
  return ignition::math::Quaternion<double>(roll_offset, pitch_offset, yaw_offset);
}
//}

/* activationCallback() */  //{
bool dynamicTargetPlugin::activationCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {
  ROS_INFO("[%s][Light]: Activation callback!", parent_name.c_str());
  res.success = true;
  if (req.data) {
    if (!map_loaded) {
      ROS_WARN("[%s]: Map not loaded yet. Tracking cannot be activated.", ros::this_node::getName().c_str());
      res.success = false;
      return false;
    }
    if (tracking_active) {
      ROS_WARN("[%s]: Tracking is already active.", ros::this_node::getName().c_str());
      res.success = false;
    } else {
      tracking_active = true;
      ROS_INFO("[%s]: Tracking activated.", ros::this_node::getName().c_str());
    }
  } else {
    if (tracking_active) {
      tracking_active = false;
      ROS_INFO("[%s]: Tracking deactivated", ros::this_node::getName().c_str());
    } else {
      ROS_WARN("[%s]: Tracking already deactivated", ros::this_node::getName().c_str());
      res.success = false;
    }
  }
  return res.success;
}
//}

/* resetCallback() */  //{
bool dynamicTargetPlugin::resetCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
  ROS_INFO("[%s][Light]: Reset callback!", parent_name.c_str());
  if (tracking_active) {
    ROS_WARN("[%s]: Tracking is active, cannot be reset.", ros::this_node::getName().c_str());
    res.success = false;
  } else {
    trajectory_pointer = 0;
    current_pose       = segmented_trajectory[trajectory_pointer];
    moveModel();
    ROS_INFO("[%s]: Tracking reset.", ros::this_node::getName().c_str());
    res.success = true;
  }
  return res.success;
}
//}

void dynamicTargetPlugin::publishPoseTimer(const ros::TimerEvent &event){
  multidrone_msgs::TargetStateArray target_array;
  multidrone_msgs::TargetState target_msgs;
  target_msgs.pose.pose.position.x = current_pose.Pos().X();
  target_msgs.pose.pose.position.y = current_pose.Pos().Y();
  target_msgs.pose.pose.position.z = current_pose.Pos().Z();
  target_msgs.pose.pose.orientation.x = current_pose.Rot().X();
  target_msgs.pose.pose.orientation.y = current_pose.Rot().Y();
  target_msgs.pose.pose.orientation.z = current_pose.Rot().Z();
  target_msgs.pose.pose.orientation.w = current_pose.Rot().W();
  target_msgs.velocity.twist.linear.x = (current_pose.Pos().X()-previous_pose.Pos().X())*update_rate;
  target_msgs.velocity.twist.linear.y = (current_pose.Pos().Y()-previous_pose.Pos().Y())*update_rate;
  target_msgs.velocity.twist.linear.z = (current_pose.Pos().Z()-previous_pose.Pos().Z())*update_rate;

  target_array.targets.push_back(target_msgs);
  previous_pose =current_pose;

  try {
    pose_pub.publish(target_array);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", pose_pub.getTopic().c_str());
  }
}

//}

/* loadMapCallback() */  //{
void dynamicTargetPlugin::loadMapCallback(const std_msgs::String &msg) {
  ROS_INFO("[%s][Light]: Load map callback!", parent_name.c_str());
  trajectory_file = msg.data;
  loadMap();
}
//}

void dynamicTargetPlugin::motionTimerKeyboardCallback(const ros::TimerEvent &event){
  key = getch();
  if(key=='w'){ // up
    current_pose.Pos().X() = current_pose.Pos().X()+(cos(current_yaw)*1/update_rate);
    current_pose.Pos().Y() = current_pose.Pos().Y()+(sin(current_yaw)*1/update_rate);
    current_pose.Pos().Z() = current_pose.Pos().Z();
  }else if(key=='j'){ //orientation
    current_yaw=current_yaw+1/update_rate;
    current_pose.Rot() = convertEulerOffsetToQuaternion(0.0, 0.0, current_yaw);

  }else if(key=='k'){ //orientation
    current_yaw=current_yaw-1/update_rate;
    current_pose.Set(current_pose.Pos().X(),current_pose.Pos().Y(),current_pose.Pos().Z(),0.0,0.0,current_yaw);

  }else if(key=='d'){ //right
    current_pose.Pos().X() = current_pose.Pos().X()+(+sin(current_yaw)*1/update_rate);
    current_pose.Pos().Y() = current_pose.Pos().Y()+(-cos(current_yaw)*1/update_rate);
    current_pose.Pos().Z() = current_pose.Pos().Z();
  }else if(key == 'a'){ //lefts
    current_pose.Pos().X() = current_pose.Pos().X()+(-sin(current_yaw)*1/update_rate);
    current_pose.Pos().Y() = current_pose.Pos().Y()+(+cos(current_yaw)*1/update_rate);
    current_pose.Pos().Z() = current_pose.Pos().Z(); 
  }else if(key=='s'){ // down
    current_pose.Pos().X() = current_pose.Pos().X()+(-cos(current_yaw)*1/update_rate);
    current_pose.Pos().Y() = current_pose.Pos().Y()+(-sin(current_yaw)*1/update_rate);
    current_pose.Pos().Z() = current_pose.Pos().Z();
  }else if(key=='b'){
    std::cout<<"\nPlease, close the window and restart the simulation"<<std::endl;
    motion_timer.stop();
  }
   
   moveModel(); // move model to the current pose
}

/* motionTimerCallback */  //{
void dynamicTargetPlugin::motionTimerCallback(const ros::TimerEvent &event) {

  if (!initialized || !tracking_active) {
    return;
  }

  current_pose = segmented_trajectory[trajectory_pointer];
  moveModel();
  trajectory_pointer++;

  if (trajectory_pointer >= segmented_trajectory.size()) {
    if (loop_enabled) {
      trajectory_pointer = 0;
      ROS_INFO("[%s]: Last point of trajectory achieved. Loop enabled -> starting from the first point.", ros::this_node::getName().c_str());
    } else {
      tracking_active    = false;
      trajectory_pointer = 0;
      ROS_INFO("[%s]: Last point of trajectory achieved. Tracking deactivated.", ros::this_node::getName().c_str());
    }
  }

  //publishPose(current_pose);
}
//}

/* euclideanDistance() */  //{
double dynamicTargetPlugin::euclideanDistance(ignition::math::Pose3d p_1, ignition::math::Pose3d p_2) {
  return sqrt(pow(p_1.Pos().X() - p_2.Pos().X(), 2) + pow(p_1.Pos().Y() - p_2.Pos().Y(), 2) + pow(p_1.Pos().Z() - p_2.Pos().Z(), 2));
}
//}

/* getShiftedPose() */  //{
ignition::math::Pose3d dynamicTargetPlugin::getShiftedPose(ignition::math::Pose3d p, std::vector<double> xyzrpy_step) {
  ignition::math::Pose3d new_pose;
  new_pose.Pos().X() = p.Pos().X() + xyzrpy_step[0];
  new_pose.Pos().Y() = p.Pos().Y() + xyzrpy_step[1];
  new_pose.Pos().Z() = p.Pos().Z() + xyzrpy_step[2];
  new_pose.Rot()     = p.Rot() * convertEulerOffsetToQuaternion(xyzrpy_step[3], xyzrpy_step[4], xyzrpy_step[5]);
  return new_pose;
}
//}

/* loadMap */  //{
void dynamicTargetPlugin::loadMap() {
  std::string mypackage = ros::package::getPath("multidrone_simulator");
  std::string myfile = mypackage+"/"+trajectory_file;
  std::ifstream in(myfile);
  ROS_INFO(" Loading MAP: ");
  double x, y, z, roll, pitch, yaw, v;
  uploaded_trajectory.clear();
  velocities.clear();
  roll_angles.clear();
  pitch_angles.clear();
  yaw_angles.clear();
  if (in.is_open()) {

    while (in >> x >> y >> z >> roll >> pitch >> yaw >> v) {
      ignition::math::Pose3d pose;
      pose.Pos().X() = x;
      pose.Pos().Y() = y;
      pose.Pos().Z() = z;
      pose.Rot()     = convertEulerOffsetToQuaternion(roll, pitch, yaw);
      roll_angles.push_back(fmod(roll + 2 * M_PI, 2 * M_PI));
      pitch_angles.push_back(fmod(pitch + 2 * M_PI, 2 * M_PI));
      yaw_angles.push_back(fmod(yaw + 2 * M_PI, 2 * M_PI));
      uploaded_trajectory.push_back(pose);
      velocities.push_back(v);
    }

    in.close();
    map_loaded = true;

    if (use_directional_yaw) {
      for (int k = 0; k < yaw_angles.size(); k++) {
        yaw_angles[k] = fmod(atan2(uploaded_trajectory[(k + 1) % yaw_angles.size()].Pos().Y() - uploaded_trajectory[k].Pos().Y(),
                                   uploaded_trajectory[(k + 1) % yaw_angles.size()].Pos().X() - uploaded_trajectory[k].Pos().X()) + 2 * M_PI, 2 * M_PI);
        uploaded_trajectory[k].Rot() = convertEulerOffsetToQuaternion(roll_angles[k], pitch_angles[k], yaw_angles[k]);
      }
      yaw_angles[yaw_angles.size() - 1] = yaw_angles[0];
    }

    current_pose = uploaded_trajectory[0];
    ROS_INFO("[%s]: Map successfully loaded.", ros::this_node::getName().c_str());

    if (use_segmentation) {
      segmentateMap();
    } else {
      segmented_trajectory = uploaded_trajectory;
    }

  } else {
    ROS_ERROR("[%s]: Unable to open trajectory file.", ros::this_node::getName().c_str());
  }
}
//}

/* getSegmentationSteps() //{ */

std::vector<double> dynamicTargetPlugin::getSegmentationSteps(ignition::math::Pose3d p_from, ignition::math::Pose3d p_to, std::vector<double> current_angles,
                                                              int idx_from) {
  std::vector<double> xyzrpy_steps;
  double              n_steps, dist;  // shouldnt be int!
  dist    = euclideanDistance(p_from, p_to);
  n_steps = dist / velocities[idx_from] * update_rate;
  ignition::math::Pose3d offset_step, last_pose;
  xyzrpy_steps.push_back((p_to.Pos().X() - p_from.Pos().X()) / n_steps);
  xyzrpy_steps.push_back((p_to.Pos().Y() - p_from.Pos().Y()) / n_steps);
  xyzrpy_steps.push_back((p_to.Pos().Z() - p_from.Pos().Z()) / n_steps);

  offset_step.Pos().X() = xyzrpy_steps[0];
  offset_step.Pos().Y() = xyzrpy_steps[1];
  offset_step.Pos().Z() = xyzrpy_steps[2];
  last_pose.Pos().X()   = p_from.Pos().X() + floor(n_steps) * xyzrpy_steps[0];
  last_pose.Pos().Y()   = p_from.Pos().Y() + floor(n_steps) * xyzrpy_steps[1];
  last_pose.Pos().Z()   = p_from.Pos().Z() + floor(n_steps) * xyzrpy_steps[2];
  
  if (euclideanDistance(last_pose, p_to) > euclideanDistance(last_pose + offset_step, p_to)) {
    n_steps = ceil(n_steps);
  } else {
    n_steps = floor(n_steps);
  }

  xyzrpy_steps.push_back((roll_angles[idx_from + 1] - current_angles[0]) / n_steps);
  xyzrpy_steps.push_back((pitch_angles[idx_from + 1] - current_angles[1]) / n_steps);
  xyzrpy_steps.push_back((yaw_angles[idx_from + 1] - current_angles[2]) / n_steps);
  xyzrpy_steps.push_back(n_steps);

  if (fabs(yaw_angles[idx_from + 1] - current_angles[2]) > M_PI) {
    xyzrpy_steps[5] = (2 * M_PI - fabs(yaw_angles[idx_from + 1] - current_angles[2])) / n_steps;
    xyzrpy_steps[5] = (yaw_angles[idx_from + 1] - current_angles[2]) > 0 ? -1 * xyzrpy_steps[5] : xyzrpy_steps[5];
  }

  return xyzrpy_steps;
}

//}

/* segmentateMape() //{ */

void dynamicTargetPlugin::segmentateMap() {
  ignition::math::Pose3d current_pose, next_pose, step_pose_3d;
  double                 n_steps;
  std::vector<double>    xyzrpy_steps;
  std::vector<double>    current_angles;
  ignition::math::Pose3d current_pose_tmp;
  current_pose_tmp = uploaded_trajectory[0];
  current_angles.push_back(roll_angles[0]);
  current_angles.push_back(pitch_angles[0]);
  current_angles.push_back(yaw_angles[0]);

  for (int i = 1; i < uploaded_trajectory.size(); i++) {
    xyzrpy_steps           = getSegmentationSteps(current_pose_tmp, uploaded_trajectory[i], current_angles, i - 1);
    n_steps                = xyzrpy_steps[6];
    step_pose_3d.Pos().X() = xyzrpy_steps[0];
    step_pose_3d.Pos().Y() = xyzrpy_steps[1];
    step_pose_3d.Pos().Z() = xyzrpy_steps[2];
    step_pose_3d.Rot()     = convertEulerOffsetToQuaternion(xyzrpy_steps[3], xyzrpy_steps[4], xyzrpy_steps[5]);
    for (int k = 0; k < n_steps; k++) {
      current_pose_tmp.Pos() += step_pose_3d.Pos();
      current_pose_tmp.Rot() = step_pose_3d.Rot() * current_pose_tmp.Rot();
      segmented_trajectory.push_back(current_pose_tmp);
    }
    current_angles[0] = fmod(roll_angles[i - 1] + n_steps * xyzrpy_steps[3] + 2 * M_PI, 2 * M_PI);
    current_angles[1] = fmod(pitch_angles[i - 1] + n_steps * xyzrpy_steps[4] + 2 * M_PI, 2 * M_PI);
    current_angles[2] = fmod(yaw_angles[i - 1] + n_steps * xyzrpy_steps[5] + 2 * M_PI, 2 * M_PI);
  }

  ROS_INFO("[%s]: Trajectory segmented.", ros::this_node::getName().c_str());
}

//}

/* moveLight() //{ */
void dynamicTargetPlugin::moveModel() {
  model_->SetRelativePose(current_pose);
}
//}

GZ_REGISTER_MODEL_PLUGIN(dynamicTargetPlugin)
}  // namespace gazebo
