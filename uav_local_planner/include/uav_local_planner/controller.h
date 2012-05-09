#include <ros/ros.h>
#include <uav_msgs/ControllerCommand.h>

//Controller Gains and Variables (Copied over from Jon's MATLAB version)

struct{


  // Roll and Pitch Gains
  float RPkp = 20;       //10 150
  float RPkd = 2.5*8/8; //6*10/8;        
  float RPki = 0.6*2/40; //12/50;

  // Yaw Gains
  float Ykp = 6;
  float Ykd = 6*15/8;
  float Yki = 12/580;

  // Thrust Gains
  
  float Tkp = 10;
  float Tkd = 6;
  floatTki = 0.1;

  //Roll and Pitch Gains for Position
  
  Posekp = 8;
  Posekd = 8*6/8;
  Poseki = 0;

  // Initialize the integral terms
  
  RI = 0;
  YI = 0;
