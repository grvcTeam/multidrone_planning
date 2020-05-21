#include <visualizer.h>

/** Main function
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualizer_node");

    Drone drone1(2);
    Drone drone2(1);
    Drone drone3(3);
    Drone drone4(4);
    Drone drone5(5);
    Drone drone6(6);

    Target target(1);
    NoFlyZones noflyzone1;

    while (ros::ok())
    {
        ros::spinOnce();

        drone1.publishMarker();
        drone2.publishMarker();
        drone3.publishMarker();
        drone4.publishMarker();
        drone5.publishMarker();
        drone6.publishMarker();
        target.publishMarker();


        ros::Duration(0.1).sleep();
    }
}
