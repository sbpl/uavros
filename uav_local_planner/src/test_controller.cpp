#include <ros/ros.h>
#include <uav_local_planner/controller.h>

int main(int argc, char** argv)
{
  ros::init(argc,argv,"uav_local_planner");

  HexaController hexa_controller;
  ros::spin();

}
