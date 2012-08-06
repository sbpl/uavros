#include <ros/ros.h>
#include <interactive_markers/menu_handler.h>
#include <uav_msgs/mode_msg.h>

#define LAND			0
#define APPROACH_POINT	1
#define LAND_POINT		2

/* Callback from a change of the interactive marker */
void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
		uav_msgs::mode_msg msg;
		msg.marker = -1;
		msg.mode = -1;

		if(feedback->menu_entry_id == 5) {
			msg.marker = 1;
			msg.mode = APPROACH_POINT;
			ROS_ERROR("M1 Approach");
		}
		if(feedback->menu_entry_id == 6) {
			msg.marker = 1;
			msg.mode = LAND_POINT;
			ROS_ERROR("M1 Land");
		}
		if(feedback->menu_entry_id == 7) {
			msg.marker = 2;
			msg.mode = APPROACH_POINT;
			ROS_ERROR("M2 Approach");
		}
		if(feedback->menu_entry_id == 8) {
			msg.marker = 2;
			msg.mode = LAND_POINT;
			ROS_ERROR("M2 Land");
		}
		if(feedback->menu_entry_id == 9) {
			msg.marker = 3;
			msg.mode = APPROACH_POINT;
			ROS_ERROR("M3 Approach");
		}
		if(feedback->menu_entry_id == 10) {
			msg.marker = 3;
			msg.mode = LAND_POINT;
			ROS_ERROR("M3 Land");
		}
		if(feedback->menu_entry_id == 11) {
			msg.marker = 4;
			msg.mode = APPROACH_POINT;
			ROS_ERROR("M4 Approach");
		}
		if(feedback->menu_entry_id == 12) {
			msg.marker = 4;
			msg.mode = LAND_POINT;
			ROS_ERROR("M4 Land");
		}
		if(feedback->menu_entry_id == 13) {
			msg.marker = LAND;
			msg.mode = LAND;
			ROS_ERROR("Land");
		}
//	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "align_node");
	ros::NodeHandle nh;

	ros::Publisher align_mode_pub = nh.advertise<uav_msgs::mode_msg>("/align_mode", 1);

	interactive_markers::MenuHandler menu_handler;

	/* Create Interactive Marker Server */
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
    server.reset(new interactive_markers::InteractiveMarkerServer("align_node", "", false));
	
	/* Create submenus entries */
	interactive_markers::MenuHandler::EntryHandle sub_menu1 = 
										menu_handler.insert("Marker 1");
	interactive_markers::MenuHandler::EntryHandle sub_menu2 = 
										menu_handler.insert("Marker 2");
	interactive_markers::MenuHandler::EntryHandle sub_menu3 = 
										menu_handler.insert("Marker 3");
	interactive_markers::MenuHandler::EntryHandle sub_menu4 = 
										menu_handler.insert("Marker 4");
	
	/* Create submenu entries */
	menu_handler.insert(sub_menu1, "Approach Point", &processFeedback);
	menu_handler.insert(sub_menu1, "Landing Point", &processFeedback);
	menu_handler.insert(sub_menu2, "Approach Point", &processFeedback);
	menu_handler.insert(sub_menu2, "Landing Point", &processFeedback);
	menu_handler.insert(sub_menu3, "Approach Point", &processFeedback);
	menu_handler.insert(sub_menu3, "Landing Point", &processFeedback);
	menu_handler.insert(sub_menu4, "Approach Point", &processFeedback);
	menu_handler.insert(sub_menu4, "Landing Point", &processFeedback);

	menu_handler.insert("Land", &processFeedback);

    /* Create Interactive Marker */
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "/map";
    int_marker.pose.position.z = 2;
    int_marker.scale = 1;
    int_marker.name = "align_mode_marker";
    int_marker.description = "Menu";

    /* Customize interactive Marker */
    visualization_msgs::InteractiveMarkerControl menu_control;
    menu_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
    menu_control.name = "align_mode_menu";
    menu_control.description = "Align Options";
    menu_control.always_visible = true;
    int_marker.controls.push_back(menu_control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
    menu_handler.apply(*server, int_marker.name);

    server->applyChanges();

    ros::spin();

    server.reset();
}


