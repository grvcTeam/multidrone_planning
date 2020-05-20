#include <multidrone_planning/mission_controller.h>
#include <ros/ros.h>

int main(int _argc, char** _argv) {


    ros::init(_argc, _argv, "mission_controller");


    multidrone::MissionController mission_controller;


    while (ros::ok()) {
        sleep(1); 
    }

    return 0;
}