#ifndef SERVO_NODE_H
#define SERVO_NODE_H

#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

using namespace std;
using namespace ros;

class ServoNode{
  public:
    ServoNode();
    ~ServoNode();
    int initialize();
    int updateServo();
    ros::Publisher angle_pub;
    float position;
  private:
    ros::NodeHandle nh;

    char* dev;
    int baudRate;
    int moduleId;

    int state;
    double minAngle;  //degrees
    double maxAngle;  //degrees
    double vel;   //degrees per second
    double desAngle;
    int dir;
    bool scan;
    double reversePoint; //degrees away from the goal to start reversing

    Dynamixel dynamixel;
    Upenn::Timer cmdTimer;

};

#endif

