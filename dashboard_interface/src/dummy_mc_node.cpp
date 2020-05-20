#include <ros/ros.h>
#include <multidrone_msgs/SendXML.h>
#include <multidrone_msgs/ValidateMission.h>
#include <multidrone_msgs/SelectRole.h>
#include <multidrone_msgs/DirectorEvent.h>
#include <multidrone_msgs/MissionStatus.h>
#include <multidrone_msgs/SystemStatus.h>
#include <multidrone_msgs/DroneActionStatus.h>
#include <thread>

class DummyMC{
    private:

    ros::NodeHandle n;

    // Callbacks
    bool eventServiceCallback(multidrone_msgs::SendXML::Request &_req, multidrone_msgs::SendXML::Response &_res);
    bool missionServiceCallback(multidrone_msgs::SendXML::Request &_req, multidrone_msgs::SendXML::Response &_res);
    bool validateMissionCallback(multidrone_msgs::ValidateMission::Request &_req, multidrone_msgs::ValidateMission::Response &_res);
    bool selectRoleCallback(multidrone_msgs::SelectRole::Request &_req, multidrone_msgs::SelectRole::Response &_res);
    void updatingMissionStatus();
    bool directorEventCallback(multidrone_msgs::DirectorEvent::Request &_req, multidrone_msgs::DirectorEvent::Response &_res);

    // publishers
    ros::Publisher system_status_pub_;


    // services
    ros::ServiceServer send_event_srv_;
    ros::ServiceServer send_mission_srv_;
    ros::ServiceServer validate_mission_srv_;
    ros::ServiceServer select_role_srv_;
    ros::ServiceServer director_event_srv_;
    
    std::thread status_thread_;

    public:

    DummyMC() 
    {
        send_event_srv_ = n.advertiseService("mission_controller/send_event_xml", &DummyMC::eventServiceCallback, this);
        send_mission_srv_ = n.advertiseService("mission_controller/send_mission_xml", &DummyMC::missionServiceCallback, this);
        validate_mission_srv_ = n.advertiseService("mission_controller/validate", &DummyMC::validateMissionCallback, this);
        select_role_srv_ = n.advertiseService("mission_controller/select_role", &DummyMC::selectRoleCallback, this);
        director_event_srv_ = n.advertiseService("mission_controller/director_event", &DummyMC::directorEventCallback, this);

        system_status_pub_ = n.advertise<multidrone_msgs::SystemStatus>("mission_controller/system_status",1000);
        ROS_INFO("Dummy Mission Controller ready");

        status_thread_ = std::thread(&DummyMC::updatingMissionStatus, this);
    }

    ~DummyMC(){}
};

void DummyMC::updatingMissionStatus()
{
    multidrone_msgs::SystemStatus status;
    // event list
    status.events.push_back("B4742C5-8714-7D9C-B37F-444D61437099");
    status.events.push_back("2C5B474-7D9C-B37F-8714-6144370D61A4");

    // mission 0
    multidrone_msgs::MissionStatus mission1, mission2;
    mission1.mission_id = "E88948C8-12A4-B37F-7D9C-437099444D61";
    mission1.status = multidrone_msgs::MissionStatus::STATUS_ENROLLED;
    status.missions.push_back(mission1);
    // mission 1
    mission1.mission_id = "06695c37-49bb-499b-ac93-b66b062af445";
    mission1.status = multidrone_msgs::MissionStatus::STATUS_PLANNING;
    status.missions.push_back(mission1);
    // mission 2
    mission1.mission_id = "fdba6031-c9c5-4220-a574-d52c43a39848";
    mission1.status = multidrone_msgs::MissionStatus::STATUS_SUPERVISING;
    status.missions.push_back(mission1);
    // mission 3
    mission2.status = multidrone_msgs::MissionStatus::STATUS_REJECTED;
    mission2.mission_id = "4f3f20dc-1410-4a18-925f-ac30784bf62f";
    multidrone_msgs::MissionError error;
    error.type = multidrone_msgs::MissionError::ERROR_TYPE_PLANNER;
    error.message = "random message";
    mission2.error_list.push_back(error);
    status.missions.push_back(mission2);
    // mission 4
    mission1.mission_id = "e9d0b3f0-2aeb-4362-bb06-3076a731ce9a";
    mission1.status = multidrone_msgs::MissionStatus::STATUS_VALIDATED;
    status.missions.push_back(mission1);
    // mission 5
    mission1.mission_id = "bb62c40a-9de5-46b6-9d06-e59f56693e6a";
    mission1.status = multidrone_msgs::MissionStatus::STATUS_READY;
    status.missions.push_back(mission1);
    // mission 6
    mission1.mission_id = "c7c22710-b171-4231-8eeb-f1d74a5bd26f";
    mission1.status = multidrone_msgs::MissionStatus::STATUS_RUNNING;
    status.missions.push_back(mission1);
    // mission 7
    mission1.mission_id = "50625440-016c-4274-aff4-dc8bdf197938";
    mission1.status = multidrone_msgs::MissionStatus::STATUS_ABORTED;
    status.missions.push_back(mission1);
    // mission 8
    mission1.mission_id = "9d26d7aa-9209-44bd-8219-4be6328b8ca9";
    mission1.status = multidrone_msgs::MissionStatus::STATUS_ENDED;
    status.missions.push_back(mission1);

    // Drones status
    multidrone_msgs::DroneActionStatus drone1, drone2;
    drone1.drone_id = 1;
    drone2.drone_id = 2;
    drone1.action_id = "SA1";
    drone2.action_id = "SA2";
    drone1.status = multidrone_msgs::DroneActionStatus::AS_RUNNING;
    drone2.status = multidrone_msgs::DroneActionStatus::AS_GOING_TO_START_POSE;

    status.drones.push_back(drone1);
    status.drones.push_back(drone2);

    while (ros::ok())
    {
        system_status_pub_.publish(status);

        ros::spinOnce();
        sleep(1);
    }
}

// test service to Send Event XML
bool DummyMC::eventServiceCallback(multidrone_msgs::SendXML::Request &_req, multidrone_msgs::SendXML::Response &_res)
{
    ROS_INFO("MC: event XML received");
    ROS_INFO("%s",_req.xml.c_str());
    _res.success = true;
    return true;
}

// test service to Send Mission XML
bool DummyMC::missionServiceCallback(multidrone_msgs::SendXML::Request &_req, multidrone_msgs::SendXML::Response &_res)
{
    ROS_INFO("MC: mission received");
    ROS_INFO("%s",_req.xml.c_str());
    _res.success = true;
    return true;
}

// test service to ValidateMission.srv
bool DummyMC::validateMissionCallback(multidrone_msgs::ValidateMission::Request &_req, multidrone_msgs::ValidateMission::Response &_res)
{
    ROS_INFO("MC: validate mission received");
    ROS_INFO("%s",_req.stem_event_id.c_str());
    return true;
}

// test service to SelectRole.srv
bool DummyMC::selectRoleCallback(multidrone_msgs::SelectRole::Request &_req, multidrone_msgs::SelectRole::Response &_res)
{
    _res.success = true;
    ROS_INFO("MC: role selection received");
    ROS_INFO("mission_role: %s",_req.mission_role.c_str());
    for (auto r: _req.sas_roles) {
        ROS_INFO("mission_id: %s\tsas_role: %s",r.mission_id.c_str(),r.sas_role.c_str());
    }
    return true;
}

// test service to test the director event
bool DummyMC::directorEventCallback(multidrone_msgs::DirectorEvent::Request &_req, multidrone_msgs::DirectorEvent::Response &_res)
{
    ROS_INFO("MC: director event received: %s",_req.event_id.c_str());
    return true;
}

int main(int _argc, char** _argv) {

    ros::init(_argc, _argv, "dummy_mc_node");

    DummyMC dummy_mc;
    
    ros::spin();

    return 0;
}