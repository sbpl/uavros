#ifndef UAVGOALINTERPOLATE_H
#define UAVGOALINTERPOLATE_H
#include <ros/ros.h>
#include <vector>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "boost/thread/mutex.hpp"



class UAVGoalInterpolate {
public:
  UAVGoalInterpolate();

ros::Subscriber goal_sub_;
ros::Subscriber pose_sub_;
ros::Publisher path_pub_;

void PoseCallback(nav_msgs::OdometryConstPtr state);
void GoalCallback(geometry_msgs::PoseStampedConstPtr goal);

private:
  std::string goal_sub_topic_, pose_sub_topic_, path_pub_topic_, map_frame_topic_;
  double DivSize_;

  boost::mutex pose_mutex_;
  nav_msgs::Odometry pose_;

};

#endif