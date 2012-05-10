/* Test function illustrating the usage of HexaController class */

#include <ros/ros.h>
#include <uav_local_planner/controller.h>

int main(int argc, char** argv)
{
  ros::init(argc,argv,"uav_local_planner");
  ros::NodeHandle nh;
  HexaController* hexa_controller = new HexaController();

  // Dynamic reconfigure stuff
  dynamic_reconfigure::Server<uav_local_planner::UAVControllerConfig> server;
  dynamic_reconfigure::Server<uav_local_planner::UAVControllerConfig>::CallbackType f;
  f = boost::bind(&HexaController::dynamic_reconfigure_callback, hexa_controller, _1, _2);
  server.setCallback(f);

  geometry_msgs::PoseStamped goal_pose;
  goal_pose.pose.position.x = 0;
  goal_pose.pose.position.y = 0;
  goal_pose.pose.position.z = 0;
  goal_pose.pose.orientation.x = 0;
  goal_pose.pose.orientation.y = 0;
  goal_pose.pose.orientation.z = 1;
  goal_pose.pose.orientation.w = 1;
   

  geometry_msgs::PoseStamped current_pose;
  current_pose.pose.position.x = -0.5;
  current_pose.pose.position.y = -0.5;
  current_pose.pose.position.z = -0.5;
  current_pose.pose.orientation.x = 0;
  current_pose.pose.orientation.y = 0;
  current_pose.pose.orientation.z = 1;
  current_pose.pose.orientation.w = 1;

  geometry_msgs::TwistStamped current_velocities;
  current_velocities.twist.linear.x = 0;
  current_velocities.twist.linear.y = 0;
  current_velocities.twist.linear.z = 0;
  current_velocities.twist.angular.x = 0.3;
  current_velocities.twist.angular.y = 0.2;
  current_velocities.twist.angular.z = 0;

  uav_msgs::ControllerCommand u;
  u = hexa_controller->Controller(current_pose, current_velocities, goal_pose);

  ros::spin();
  return 0;
}
