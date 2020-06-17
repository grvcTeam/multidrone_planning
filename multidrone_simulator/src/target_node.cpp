/** Author: Alfonso Alcantara
 *  Email: aamarin@us.es
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <multidrone_msgs/TargetStateArray.h>
#include <iostream>
#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>
#include <unistd.h>
#include <termios.h>
#include <iostream>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float32.h>
#include <ros/package.h>
#include <ignition/math.hh>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

class Target
{
public:
  Target();

private:
  std::string trajectory_file;
  double update_rate = 30;  //read by parameter
  int trajectory_pointer = 0;
  bool tracking_active;
  bool initial_on;
  bool loop_enabled;
  bool use_segmentation = true;
  bool use_directional_yaw;
  bool map_loaded = false;
  bool initialized = false;
  double current_yaw = 0.0;
  ignition::math::Pose3d previous_pose; /*< previous pose to calculate the velocity */
  ros::Subscriber load_map_sub;
  std::atomic<bool> interrupted;

  char key = ' ';

  std::vector<ignition::math::Pose3d> uploaded_trajectory;
  std::vector<ignition::math::Pose3d> segmented_trajectory;
  std::vector<double> velocities;
  std::vector<double> yaw_angles;
  std::vector<double> roll_angles;
  std::vector<double> pitch_angles;
  ignition::math::Pose3d current_pose;

  ros::Publisher pose_pub; /*< target pos ros publisher */
  ros::NodeHandle *nh_;
  ros::ServiceServer activation_srv, reset_srv;
  ros::Timer timer_pose;
  std::thread keyboard_thread;
  ros::Timer motion_timer;
  std::thread spin_thread_;

  ros::Publisher model_pose_pub_;


  /** \brief function to load the map from a txt file and save data in pitch_angles, yaw_angles, roll_angles, updated_trajectory, velocities
    */
  void loadMap();
  /** \brief function that segmentate the loaded trajectory
    */
  void segmentateMap();
  /**! \brief Callback to move the dynamic target. This function call moveModel() and manage the trajectory pointer
    */
  void motionTimerCallback(const ros::TimerEvent &event);
  /**! \brief utility funciton to get euclidean distance betwenn two poses
    *   \param p_1
    *   \param p_2
    *   \return euclidean
    */
  double euclideanDistance(ignition::math::Pose3d p_1, ignition::math::Pose3d p_2);
  /**! \brief configuration function to activate keyboard
   */
  int getch(void);
  /**! \brief
   */
  ignition::math::Quaternion<double> convertEulerOffsetToQuaternion(double roll_offset, double pitch_offset, double yaw_offset);
  bool activationCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);

  bool resetCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /** \brief get points between two point to adjust velocity
   *  \param p_1 from that point
   *  \param p_2 to that point
   *  \param current_angles current angles
   *  \param index_from current index of uploaded_trajectory
   *  \return return a vector with the point xyzrpy and steps between p_1 and p_2
   */
  std::vector<double> getSegmentationSteps(ignition::math::Pose3d p_1, ignition::math::Pose3d p_2, std::vector<double> current_angles, int index_from);
  /** \brief
  */
  ignition::math::Pose3d getShiftedPose(ignition::math::Pose3d p, std::vector<double> xyzrpy_step);

  void publishPoseTimer(const ros::TimerEvent &event);

  void motionTimerKeyboardCallback(void);

  void loadMapCallback(const std_msgs::String &msg);
  void moveModel();
};

Target::Target()
{

  // initialize ROS
  int argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "target_node", ros::init_options::NoSigintHandler);
  nh_ = new ros::NodeHandle("~");
  std::string target_topic_name = "";

  // initialize current pose from parameters
  std::vector<double> initial_target_pose;
  if(nh_->getParam("target_pose",initial_target_pose)){
    current_pose.Pos().X() = initial_target_pose[0];
    current_pose.Pos().Y() = initial_target_pose[1];
    current_pose.Pos().Z() = initial_target_pose[2];
    ignition::math::Quaternion<double> quat = convertEulerOffsetToQuaternion(initial_target_pose[3],initial_target_pose[4],initial_target_pose[5]);
    current_pose.Rot() = quat;
    current_yaw = initial_target_pose[5];
  }else{
    ROS_INFO("Target node: fail to get initial pose");
  }
  if(!nh_->getParam("trajectory_file",trajectory_file)){
    ROS_WARN("Target node: fail to get trajectory file");
  }
  bool keyboard = true;

  if(!nh_->getParam("keyboard",keyboard)){
    ROS_WARN("Target node: Fail to get keyboard mode");
  }
  if(!nh_->getParam("update_rate",update_rate)){
    ROS_WARN("Target node: Fail to get update rate");
  }
  if(!nh_->getParam("use_directional_yaw",use_directional_yaw)){
    ROS_WARN("Target node: Fail to get update rate");
  }
  if(!nh_->getParam("target_topic_name",target_topic_name)){
    ROS_WARN("Target node: fail to get target topic name");
  }

  std::string target_topic = "/gazebo/target";
  timer_pose = nh_->createTimer(ros::Rate(update_rate), &Target::publishPoseTimer, this);

  pose_pub = nh_->advertise<multidrone_msgs::TargetStateArray>(target_topic_name, 1);
  model_pose_pub_ = nh_->advertise<geometry_msgs::PoseStamped>(target_topic, 1);

  if (keyboard)
  {
    keyboard_thread = std::thread(&Target::motionTimerKeyboardCallback, this);
  }
  else
  {
    motion_timer = nh_->createTimer(ros::Rate(update_rate), &Target::motionTimerCallback, this); //trajectory
    if (!trajectory_file.empty())
    {
      loadMap();
    }
  }

  initialized = true;
  ROS_INFO("Target plugin initialized");

  if (initial_on && map_loaded)
  {
    tracking_active = true;
    ROS_INFO("[%s]: Initial on flag set to true. Tracking activated.", ros::this_node::getName().c_str());
  }

  if (keyboard)
  {
    std::cout << "Use keyboard to move the target: " << std::endl;
    std::cout << "W -> forward" << std::endl;
    std::cout << "S -> Backward" << std::endl;
    std::cout << "A -> Left" << std::endl;
    std::cout << "D -> Right" << std::endl;
    std::cout << "Use J-K to move the orientation of the target" << std::endl;
    std::cout << "Press b to quit" << std::endl;
  }
  ros::spin();
}
/* loadMap */ //{
void Target::loadMap()
{
  std::string mypackage = ros::package::getPath("multidrone_simulator");
  std::string myfile = mypackage + "/" + trajectory_file;
  std::ifstream in(myfile);
  ROS_INFO(" Loading MAP: ");
  double x, y, z, roll, pitch, yaw, v;
  uploaded_trajectory.clear();
  velocities.clear();
  roll_angles.clear();
  pitch_angles.clear();
  yaw_angles.clear();
  if (in.is_open())
  {

    while (in >> x >> y >> z >> roll >> pitch >> yaw >> v)
    {
      ignition::math::Pose3d pose;
      pose.Pos().X() = x;
      pose.Pos().Y() = y;
      pose.Pos().Z() = z;
      pose.Rot() = convertEulerOffsetToQuaternion(roll, pitch, yaw);
      roll_angles.push_back(fmod(roll + 2 * M_PI, 2 * M_PI));
      pitch_angles.push_back(fmod(pitch + 2 * M_PI, 2 * M_PI));
      yaw_angles.push_back(fmod(yaw + 2 * M_PI, 2 * M_PI));
      uploaded_trajectory.push_back(pose);
      velocities.push_back(v);
    }

    in.close();
    map_loaded = true;

    if (use_directional_yaw)
    {
      for (int k = 0; k < yaw_angles.size(); k++)
      {
        yaw_angles[k] = fmod(atan2(uploaded_trajectory[(k + 1) % yaw_angles.size()].Pos().Y() - uploaded_trajectory[k].Pos().Y(),
                                   uploaded_trajectory[(k + 1) % yaw_angles.size()].Pos().X() - uploaded_trajectory[k].Pos().X()) +
                                 2 * M_PI,
                             2 * M_PI);
        uploaded_trajectory[k].Rot() = convertEulerOffsetToQuaternion(roll_angles[k], pitch_angles[k], yaw_angles[k]);
      }
      yaw_angles[yaw_angles.size() - 1] = yaw_angles[0];
    }

    current_pose = uploaded_trajectory[0];
    ROS_INFO("[%s]: Map successfully loaded.", ros::this_node::getName().c_str());

    if (use_segmentation)
    {
      segmentateMap();
    }
    else
    {
      segmented_trajectory = uploaded_trajectory;
    }
  }
  else
  {
    ROS_ERROR("[%s]: Unable to open trajectory file.", ros::this_node::getName().c_str());
  }
}

void Target::segmentateMap()
{
  ignition::math::Pose3d current_pose, next_pose, step_pose_3d;
  double n_steps;
  std::vector<double> xyzrpy_steps;
  std::vector<double> current_angles;
  ignition::math::Pose3d current_pose_tmp;
  current_pose_tmp = uploaded_trajectory[0];
  current_angles.push_back(roll_angles[0]);
  current_angles.push_back(pitch_angles[0]);
  current_angles.push_back(yaw_angles[0]);

  for (int i = 1; i < uploaded_trajectory.size(); i++)
  {
    xyzrpy_steps = getSegmentationSteps(current_pose_tmp, uploaded_trajectory[i], current_angles, i - 1);
    n_steps = xyzrpy_steps[6];
    step_pose_3d.Pos().X() = xyzrpy_steps[0];
    step_pose_3d.Pos().Y() = xyzrpy_steps[1];
    step_pose_3d.Pos().Z() = xyzrpy_steps[2];
    step_pose_3d.Rot() = convertEulerOffsetToQuaternion(xyzrpy_steps[3], xyzrpy_steps[4], xyzrpy_steps[5]);
    for (int k = 0; k < n_steps; k++)
    {
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

void Target::motionTimerCallback(const ros::TimerEvent &event)
{

  if (!initialized || !tracking_active)
  {
    return;
  }
  current_pose = segmented_trajectory[trajectory_pointer];

  moveModel();
  trajectory_pointer++;

  if (trajectory_pointer >= segmented_trajectory.size())
  {
    if (loop_enabled)
    {
      trajectory_pointer = 0;
      ROS_INFO("[%s]: Last point of trajectory achieved. Loop enabled -> starting from the first point.", ros::this_node::getName().c_str());
    }
    else
    {
      tracking_active = false;
      trajectory_pointer = 0;
      ROS_INFO("[%s]: Last point of trajectory achieved. Tracking deactivated.", ros::this_node::getName().c_str());
    }
  }

  //publishPose(current_pose);
}

double Target::euclideanDistance(ignition::math::Pose3d p_1, ignition::math::Pose3d p_2)
{
  return sqrt(pow(p_1.Pos().X() - p_2.Pos().X(), 2) + pow(p_1.Pos().Y() - p_2.Pos().Y(), 2) + pow(p_1.Pos().Z() - p_2.Pos().Z(), 2));
}

int Target::getch(void)
{

  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;

}

/* convertEulerOffsetToQuaternion() //{ */
ignition::math::Quaternion<double> Target::convertEulerOffsetToQuaternion(double roll_offset, double pitch_offset, double yaw_offset)
{
  return ignition::math::Quaternion<double>(roll_offset, pitch_offset, yaw_offset);
}

/* activationCallback() */ //{
bool Target::activationCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res)
{
  ROS_INFO("Target: Activation callback!");
  res.success = true;
  if (req.data)
  {
    if (!map_loaded)
    {
      ROS_WARN("[%s]: Map not loaded yet. Tracking cannot be activated.", ros::this_node::getName().c_str());
      res.success = false;
      return false;
    }
    if (tracking_active)
    {
      ROS_WARN("[%s]: Tracking is already active.", ros::this_node::getName().c_str());
      res.success = false;
    }
    else
    {
      tracking_active = true;
      ROS_INFO("[%s]: Tracking activated.", ros::this_node::getName().c_str());
    }
  }
  else
  {
    if (tracking_active)
    {
      tracking_active = false;
      ROS_INFO("[%s]: Tracking deactivated", ros::this_node::getName().c_str());
    }
    else
    {
      ROS_WARN("[%s]: Tracking already deactivated", ros::this_node::getName().c_str());
      res.success = false;
    }
  }
  return res.success;
}

/* resetCallback() */ //{
bool Target::resetCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  ROS_INFO("Target: Reset callback");
  if (tracking_active)
  {
    ROS_WARN("[%s]: Tracking is active, cannot be reset.", ros::this_node::getName().c_str());
    res.success = false;
  }
  else
  {
    trajectory_pointer = 0;
    current_pose = segmented_trajectory[trajectory_pointer];
    moveModel();
    ROS_INFO("[%s]: Tracking reset.", ros::this_node::getName().c_str());
    res.success = true;
  }
  return res.success;
}

std::vector<double> Target::getSegmentationSteps(ignition::math::Pose3d p_from, ignition::math::Pose3d p_to, std::vector<double> current_angles,
                                                 int idx_from)
{
  std::vector<double> xyzrpy_steps;
  double n_steps, dist; // shouldnt be int!
  dist = euclideanDistance(p_from, p_to);
  n_steps = dist / velocities[idx_from] * update_rate;
  ignition::math::Pose3d offset_step, last_pose;
  xyzrpy_steps.push_back((p_to.Pos().X() - p_from.Pos().X()) / n_steps);
  xyzrpy_steps.push_back((p_to.Pos().Y() - p_from.Pos().Y()) / n_steps);
  xyzrpy_steps.push_back((p_to.Pos().Z() - p_from.Pos().Z()) / n_steps);

  offset_step.Pos().X() = xyzrpy_steps[0];
  offset_step.Pos().Y() = xyzrpy_steps[1];
  offset_step.Pos().Z() = xyzrpy_steps[2];
  last_pose.Pos().X() = p_from.Pos().X() + floor(n_steps) * xyzrpy_steps[0];
  last_pose.Pos().Y() = p_from.Pos().Y() + floor(n_steps) * xyzrpy_steps[1];
  last_pose.Pos().Z() = p_from.Pos().Z() + floor(n_steps) * xyzrpy_steps[2];

  if (euclideanDistance(last_pose, p_to) > euclideanDistance(last_pose + offset_step, p_to))
  {
    n_steps = ceil(n_steps);
  }
  else
  {
    n_steps = floor(n_steps);
  }

  xyzrpy_steps.push_back((roll_angles[idx_from + 1] - current_angles[0]) / n_steps);
  xyzrpy_steps.push_back((pitch_angles[idx_from + 1] - current_angles[1]) / n_steps);
  xyzrpy_steps.push_back((yaw_angles[idx_from + 1] - current_angles[2]) / n_steps);
  xyzrpy_steps.push_back(n_steps);

  if (fabs(yaw_angles[idx_from + 1] - current_angles[2]) > M_PI)
  {
    xyzrpy_steps[5] = (2 * M_PI - fabs(yaw_angles[idx_from + 1] - current_angles[2])) / n_steps;
    xyzrpy_steps[5] = (yaw_angles[idx_from + 1] - current_angles[2]) > 0 ? -1 * xyzrpy_steps[5] : xyzrpy_steps[5];
  }

  return xyzrpy_steps;
}

ignition::math::Pose3d Target::getShiftedPose(ignition::math::Pose3d p, std::vector<double> xyzrpy_step)
{
  ignition::math::Pose3d new_pose;
  new_pose.Pos().X() = p.Pos().X() + xyzrpy_step[0];
  new_pose.Pos().Y() = p.Pos().Y() + xyzrpy_step[1];
  new_pose.Pos().Z() = p.Pos().Z() + xyzrpy_step[2];
  new_pose.Rot() = p.Rot() * convertEulerOffsetToQuaternion(xyzrpy_step[3], xyzrpy_step[4], xyzrpy_step[5]);
  return new_pose;
}


void Target::motionTimerKeyboardCallback(void)
{
  while(ros::ok){
    key = getch();
    if (key == 'w')
    { // up
      current_pose.Pos().X() = current_pose.Pos().X() + (cos(current_yaw) * 1 / update_rate);
      current_pose.Pos().Y() = current_pose.Pos().Y() + (sin(current_yaw) * 1 / update_rate);
      current_pose.Pos().Z() = current_pose.Pos().Z();
    }
    else if (key == 'j')
    { //orientation
      current_yaw = current_yaw + 1 / update_rate;
      current_pose.Rot() = convertEulerOffsetToQuaternion(0.0, 0.0, current_yaw);
    }
    else if (key == 'k')
    { //orientation
      current_yaw = current_yaw - 1 / update_rate;
      current_pose.Set(current_pose.Pos().X(), current_pose.Pos().Y(), current_pose.Pos().Z(), 0.0, 0.0, current_yaw);
    }
    else if (key == 'd')
    { //right
      current_pose.Pos().X() = current_pose.Pos().X() + (+sin(current_yaw) * 1 / update_rate);
      current_pose.Pos().Y() = current_pose.Pos().Y() + (-cos(current_yaw) * 1 / update_rate);
      current_pose.Pos().Z() = current_pose.Pos().Z();
    }
    else if (key == 'a')
    { //lefts
      current_pose.Pos().X() = current_pose.Pos().X() + (-sin(current_yaw) * 1 / update_rate);
      current_pose.Pos().Y() = current_pose.Pos().Y() + (+cos(current_yaw) * 1 / update_rate);
      current_pose.Pos().Z() = current_pose.Pos().Z();
    }
    else if (key == 's')
    { // down
      current_pose.Pos().X() = current_pose.Pos().X() + (-cos(current_yaw) * 1 / update_rate);
      current_pose.Pos().Y() = current_pose.Pos().Y() + (-sin(current_yaw) * 1 / update_rate);
      current_pose.Pos().Z() = current_pose.Pos().Z();
    }
    else if (key == 'b')
    {
      std::cout << "\nPlease, close the window and restart the simulation" << std::endl;
      motion_timer.stop();
    }

    moveModel(); // move model to the current pose
  }
}

void Target::publishPoseTimer(const ros::TimerEvent &event)
{
  multidrone_msgs::TargetStateArray target_array;
  multidrone_msgs::TargetState target_msgs;
  target_msgs.target_id = 1;
  target_msgs.pose.pose.position.x = current_pose.Pos().X();
  target_msgs.pose.pose.position.y = current_pose.Pos().Y();
  target_msgs.pose.pose.position.z = current_pose.Pos().Z();
  target_msgs.pose.pose.orientation.x = current_pose.Rot().X();
  target_msgs.pose.pose.orientation.y = current_pose.Rot().Y();
  target_msgs.pose.pose.orientation.z = current_pose.Rot().Z();
  target_msgs.pose.pose.orientation.w = current_pose.Rot().W();
  target_msgs.velocity.twist.linear.x = (current_pose.Pos().X() - previous_pose.Pos().X()) * update_rate;
  target_msgs.velocity.twist.linear.y = (current_pose.Pos().Y() - previous_pose.Pos().Y()) * update_rate;
  target_msgs.velocity.twist.linear.z = (current_pose.Pos().Z() - previous_pose.Pos().Z()) * update_rate;

  target_array.targets.push_back(target_msgs);
  previous_pose = current_pose;

  try
  {
    pose_pub.publish(target_array);
  }
  catch (...)
  {
    ROS_ERROR("Exception caught during publishing topic %s.", pose_pub.getTopic().c_str());
  }
}

void Target::loadMapCallback(const std_msgs::String &msg)
{
  ROS_INFO("Target: load map callback");
  trajectory_file = msg.data;
  loadMap();
}

void Target::moveModel()
{
  geometry_msgs::PoseStamped new_pose;
  new_pose.pose.position.x = current_pose.Pos().X();
  new_pose.pose.position.y = current_pose.Pos().Y();
  new_pose.pose.position.z = current_pose.Pos().Z();
  new_pose.pose.orientation.x = current_pose.Rot().X();
  new_pose.pose.orientation.y = current_pose.Rot().Y();
  new_pose.pose.orientation.z = current_pose.Rot().Z();
  new_pose.pose.orientation.w = current_pose.Rot().W();
  model_pose_pub_.publish(new_pose);
}

int main(int argc, char *argv[])
{
  Target target;
  ros::spin();
  return 0;
}