#include <ros/ros.h>
#include<interactive_markers/menu_handler.h>
#include<interactive_markers/interactive_marker_server.h>
#include<geometry_msgs/PoseStamped.h>

interactive_markers::MenuHandler menu_handler;
ros::Publisher goal_pose_pub;


void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM( feedback->marker_name << " is now at "
      << feedback->pose.position.x << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z );

  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT)
  {
    ROS_INFO_STREAM("Received new goal !" );
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.stamp = ros::Time::now();
    goal_pose.header.frame_id = "/map";
    goal_pose.pose = feedback->pose;
    goal_pose_pub.publish(goal_pose);
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uav_set_goal");
  ros::NodeHandle nh;
  goal_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal_pose",1);

  // create an interactive marker server on the topic namespace simple_marker
  interactive_markers::InteractiveMarkerServer server("uav_set_goal");

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "/base_link";
  int_marker.name = "goal_cube";
  int_marker.description = "Simple 4-DOF Control (X,Y,Z,Yaw)";

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = 0.45;
  box_marker.scale.y = 0.45;
  box_marker.scale.z = 0.45;
  box_marker.color.r = 1;
  box_marker.color.g = 0;
  box_marker.color.b = 0;
  box_marker.color.a = 1.0;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control, control, menu_control;
  box_control.always_visible = true;
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  int_marker.controls.push_back( box_control );

  // Menu Stuff
  menu_handler.insert( "Send Goal !", &processFeedback );
  menu_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  menu_control.description="Goal Options";
  menu_control.name = "menu_only_control";
  menu_control.always_visible = true;
  int_marker.controls.push_back(menu_control);
  
  
  
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);


  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server.insert(int_marker, &processFeedback);

  
  menu_handler.apply(server, int_marker.name );
  // 'commit' changes and send to all clients
  server.applyChanges();

  // start the ROS main loop
  ros::spin();
}

