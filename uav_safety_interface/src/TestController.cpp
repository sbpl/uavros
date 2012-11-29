#include <SafetyInterface.h>

int main(int argc, char** argv)
{
  printf("Initializing test controller...");
  ros::init(argc,argv,"uav_test_controller");
  ros::NodeHandle nh;
  uavTestController* tctrl = new uavTestController(nh, "/cmd_in");
  tctrl->run();
  return 0;
}


