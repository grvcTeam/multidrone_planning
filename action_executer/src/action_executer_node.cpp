#include <action_executer/action_executer.h>

int main(int _argc, char **_argv)
{
  ROS_INFO("Setting up Executer");
  sleep(2);
  Executer executer(_argc,_argv);
  return 0;
}