#include <global_tracker/global_tracker.h>
#include <ros/ros.h>

int main(int _argc, char** _argv) {

    ros::init(_argc, _argv, "global_tracker");

    multidrone::GlobalTracker global_tracker;

    while (ros::ok())
    {
        sleep(1);
    }

    return 0;
}