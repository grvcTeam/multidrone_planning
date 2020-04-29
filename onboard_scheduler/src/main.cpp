/**
 * 
 */

#include <onboard_scheduler.h>

int main(int _argc, char** _argv)
{

  ros::init(_argc, _argv, "scheduler_node");

  OnBoardScheduler scheduler;
  
  while (ros::ok()) { sleep(1); }

  return 0;
}