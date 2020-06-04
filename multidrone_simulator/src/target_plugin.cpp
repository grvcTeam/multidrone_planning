/** Author: Alfonso Alcantara
 *  Email: aamarin@us.es
 */

#include <ros/ros.h>
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
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <ignition/math.hh>
#include <geometry_msgs/PoseStamped.h>


namespace gazebo
{
  /* Class definition */ //{
  class dynamicTargetPlugin : public ModelPlugin
  {
  public:
    dynamicTargetPlugin() : ModelPlugin()
    {
      printf("Target constructor\n");
    }
    /*! \brief Called when a Plugin is first created, and after the World has been loaded to INITIALIZE 
  *         This function read the sdf elements, initialize services to control the plugin, loadMap(), start timer, wait for a key
  *   \param _parent pointer to the model
  *   \param _sdf pointer to the sdf element of the plugin
  */
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

  private:
    physics::ModelPtr model_; /*! pointer to the model */
    ros::NodeHandle *nh_;
    ros::Subscriber pose_sub;
    ignition::math::Pose3d current_pose;

    /** \brief set current pose
   */
    void moveModel();
    void targetMovementCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  };

  void dynamicTargetPlugin::targetMovementCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose.Pos().X() = msg->pose.position.x;
    current_pose.Pos().Y() = msg->pose.position.y;
    current_pose.Pos().Z() = msg->pose.position.z;
    current_pose.Rot().X() = msg->pose.orientation.x;
    current_pose.Rot().Y() = msg->pose.orientation.y;
    current_pose.Rot().Z() = msg->pose.orientation.z;
    current_pose.Rot().W() = msg->pose.orientation.w;
    moveModel();
  }
  void dynamicTargetPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {

    ROS_INFO("[%s]: Dynamic target plugin started.", ros::this_node::getName().c_str());
    model_ = _parent;
    std::string parent_name = model_->GetName().c_str();
    std::string topic_name = "";
    bool keyboard = false;

    if (_sdf->HasElement("topic_name"))
    {
      topic_name = _sdf->Get<std::string>("topic_name");
    }
    else
    {
      ROS_WARN("[%s][Target]: topic name not defined. Setting to default value", parent_name.c_str());
      topic_name = "/gazebo/target"; // /target_3d_state
    }

    
    // initialize current pose
    // TODO initial postion of the target by sdf

    // initialize ROS
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "dynamic_target_plugin", ros::init_options::NoSigintHandler);
    nh_ = new ros::NodeHandle("~");

    pose_sub = nh_->subscribe(topic_name,10,&dynamicTargetPlugin::targetMovementCallback,this);
  }

  void dynamicTargetPlugin::moveModel()
  {
    model_->SetRelativePose(current_pose);
  }

  GZ_REGISTER_MODEL_PLUGIN(dynamicTargetPlugin)
} // namespace gazebo
