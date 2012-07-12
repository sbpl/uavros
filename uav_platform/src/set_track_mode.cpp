#include <ros/ros.h>
#include <interactive_markers/menu_handler.h>
#include <uav_msgs/mode_msg.h>

#define ALIGN_FRONT     1
#define ALIGN_TOP       2
#define ROTATE          3
#define LAND            4
#define CHANGE_FRONT    5
#define CHANGE_BOTTOM   6   

interactive_markers::MenuHandler menu_handler;
ros::Publisher track_mode_pub;

/* Callback from a change of the interactive marker */
void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) {
        if(feedback->menu_entry_id == ALIGN_FRONT) {
            ROS_INFO("Aligning in front of platform");
            uav_msgs::mode_msg msg;
            msg.mode = ALIGN_FRONT;
            track_mode_pub.publish(msg);
        } else if(feedback->menu_entry_id == ALIGN_TOP) {
            ROS_INFO("Aligning on top of platform");
            uav_msgs::mode_msg msg;
            msg.mode = ALIGN_TOP;
            track_mode_pub.publish(msg);
        } else if(feedback->menu_entry_id == ROTATE) {
            ROS_INFO("Rotate 180 over the platform");
            uav_msgs::mode_msg msg;
            msg.mode = ROTATE;
            track_mode_pub.publish(msg);
        } else if(feedback->menu_entry_id == LAND) {
            ROS_INFO("Land on platform");
            uav_msgs::mode_msg msg;
            msg.mode = LAND;
            track_mode_pub.publish(msg);
        } else if(feedback->menu_entry_id == CHANGE_FRONT) {
            ROS_INFO("Change to front camera");
            uav_msgs::mode_msg msg;
            msg.mode = CHANGE_FRONT;
            track_mode_pub.publish(msg);
        } else if(feedback->menu_entry_id == CHANGE_BOTTOM) {
            ROS_INFO("Change to bottom camera");
            uav_msgs::mode_msg msg;
            msg.mode = CHANGE_BOTTOM;
            track_mode_pub.publish(msg);
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "set_track_mode");
    ros::NodeHandle nh;

    track_mode_pub = nh.advertise<uav_msgs::mode_msg>("/track_mode",1);

    /* Create Interactive Marker Server */
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
    server.reset(new interactive_markers::InteractiveMarkerServer("set_track_mode", "", false));

    /* Labels need to be inserted in the order of the define values */
    menu_handler.insert("Align in Front", &processFeedback);
    menu_handler.insert("Align on Top", &processFeedback);
    menu_handler.insert("Rotate over Marker", &processFeedback);
    menu_handler.insert("Land on Marker", &processFeedback);
    menu_handler.insert("Change to front camera", &processFeedback);
    menu_handler.insert("Change to bottom camera", &processFeedback);

    /* Create Interactive Marker */
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "/usb_cam";
    int_marker.pose.position.y = -1;
    int_marker.scale = 1;
    int_marker.name = "track_mode_marker";
    int_marker.description = "Menu";

    /* Customize interactive Marker */
    visualization_msgs::InteractiveMarkerControl menu_control;
    menu_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
    menu_control.name = "track_mode_menu";
    menu_control.description = "Track Mode Options";
    menu_control.always_visible = true;
    int_marker.controls.push_back(menu_control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
    menu_handler.apply(*server, int_marker.name);

    server->applyChanges();

    ros::spin();

    server.reset();
}

