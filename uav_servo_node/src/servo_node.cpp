#include <uav_servo_node/Dynamixel.h>
#include <iostream>
#include <uav_servo_node/Timer.h>
#include <uav_servo_node/ErrorMessage.h>
#include <uav_servo_node/servo_node.h>
#include <ros/ros.h>

#define PI 3.14159

//states for the built-in mini-state machine
enum { DYNAMIXEL_STATE_UNINITIALIZED, 
       DYNAMIXEL_STATE_INITIALIZED,
       DYNAMIXEL_STATE_MOVE_CMD_SENT,    
       DYNAMIXEL_STATE_STOPPED
     };

ServoNode::ServoNode() : nh("~"){
  dev = (char*)"/dev/ttyUSB0";
  baudRate = 57600;

  nh.param("servoID", moduleId, 0);
  nh.param("minAngle", minAngle, 0.0);
  nh.param("maxAngle", maxAngle, 0.0);
  nh.param("velocity", vel, 50.0);
  ROS_INFO("[servo] Loaded parameters...");

  reversePoint = 5; //degrees away from the goal to start reversing
  desAngle = 0;
  dir = 1;
  scan = minAngle != maxAngle;
  state = DYNAMIXEL_STATE_INITIALIZED;
  position = 0;
  angle_pub = nh.advertise<sensor_msgs::JointState>("/servoAngle", 3);
}

int ServoNode::initialize(){
  ROS_INFO("Begin init...");
  //open the serial port
  if (dynamixel.Connect(dev,baudRate,moduleId)){
    ROS_ERROR("servo_node: module could not connect\n");
    return -1;
  }
  
  //check to see if the servo is hooked up
  if (dynamixel.StartDevice()){
    ROS_ERROR("servo_node: module could not get status\n");
    return -1;
  }

  ROS_INFO("servo_node: module connected\n");

  if (dynamixel.MoveToPos(0,50)){
    ROS_ERROR("servo_node: module could not send move cmd\n");
    return -1;
  }
  ROS_INFO("servo_node: module sent move cmd\n");
  ROS_INFO("Init complete!");
  return 0;
}


ServoNode::~ServoNode(){}

int ServoNode::updateServo(){
  if(scan){
    //servo back and forth
    switch (state){

      case DYNAMIXEL_STATE_INITIALIZED:

      case DYNAMIXEL_STATE_STOPPED:
ROS_INFO("stopped");
      
        desAngle = dir > 0 ? maxAngle : minAngle;
      
        if (dynamixel.MoveToPos(desAngle,vel)){
          ROS_ERROR("servo_node: module could not send MoveToPos command\n");
          return -1;
        }
      
        state = DYNAMIXEL_STATE_MOVE_CMD_SENT;
        cmdTimer.Tic();
        
        //fall through here to get the position

      case DYNAMIXEL_STATE_MOVE_CMD_SENT:
ROS_INFO("sent");

        if (dynamixel.GetPosition(position) == 0){
          ROS_INFO("module: angle = %f\n",position);
          //if we are close to getting to the desired angle, switch the direction without getting there
          if ((dir>0) && (position > ( desAngle - reversePoint))){
            state = DYNAMIXEL_STATE_STOPPED;
            dir*=-1;
          }

          else if ((dir<0) && (position < (desAngle + reversePoint))){
            state = DYNAMIXEL_STATE_STOPPED;
            dir*=-1;
          }
          break;
        }
        else{
          ROS_ERROR( "servo_node: module could not get position from the servo\n");
          return -1;
        }

      default:
        ROS_ERROR("servo_node: module unknown state\n");
        return -1;
    }
  }
  else{
    switch(state){
      case DYNAMIXEL_STATE_INITIALIZED:
        if (dynamixel.MoveToPos(maxAngle,vel)){
          ROS_ERROR("servo_node: module could not send MoveToPos command\n");
          return -1;
        }
        state = DYNAMIXEL_STATE_MOVE_CMD_SENT;
        cmdTimer.Tic();
        //fall through here to get the position

      case DYNAMIXEL_STATE_MOVE_CMD_SENT:
        if (dynamixel.GetPosition(position) == 0){
          ROS_INFO("module: angle = %f\n",position);
          break;
        }
        else{
          ROS_ERROR( "servo_node: module could not get position from the servo\n");
          return -1;
        }

      default:
        ROS_ERROR("servo_node: module unknown state\n");
        return -1;
    }
  }

  sensor_msgs::JointState msg;
  msg.header.stamp = ros::Time::now();
  msg.name.push_back("panning_laser_to_body");
  msg.position.push_back(position*PI/180);
  angle_pub.publish(msg);

  return 0;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "servo_node");
  ServoNode s;
  if(s.initialize()){
    ROS_ERROR("Failed to initialize servo. Publishing a 0 joint state...");
    ros::Rate r(50.0);
    while(ros::ok()){
      sensor_msgs::JointState msg;
      msg.header.stamp = ros::Time::now();
      msg.name.push_back("panning_laser_to_body");
      msg.position.push_back(0);
      s.angle_pub.publish(msg);
      r.sleep();
    }
    return 1;
  }

  //set up periodic call
  while(ros::ok()){
    if(s.updateServo())
      return 1;
    //ros::spinOnce();
  }

  return 0;
}

