#include <gimbal_camera_interface/gimbal_camera_interface.h>


GimbalInterface gimbal;

void killer(int s){
      gimbal.stop();
      exit(1);
    }

int main(int _argc, char **_argv)
{

  ROS_INFO("Setting up Gimbal Control node");
  sleep(2);

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = killer;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    
    gimbal.initi(_argc,_argv);
  return 0;
}