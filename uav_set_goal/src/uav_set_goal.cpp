#include "uav_set_goal.h"

UAV_SET_GOAL_C::UAV_SET_GOAL_C():server("uav_set_goal") {
  ros::NodeHandle nh;

  nh.param<std::string>("goal_pub_topic",goal_pub_topic_,"/goal_pose");
  nh.param<std::string>("flt_mode_req_topic",flt_mode_req_topic_,"flight_mode_request");
  nh.param<std::string>("flt_mode_stat_topic",flt_mode_stat_topic_,"flight_mode_status");
  nh.param<std::string>("map_topic",map_topic_,"/map");
  nh.param<std::string>("goal_marker_name",goal_marker_name_,"UAV Goal Marker");

  goal_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(goal_pub_topic_,1);
  flight_request_pub = nh.advertise<uav_msgs::FlightModeRequest>(flt_mode_req_topic_,1);
  flight_status_sub = nh.subscribe(flt_mode_stat_topic_, 1, &UAV_SET_GOAL_C::FlightModeStatusCallback,this);

  lastTime = ros::Time::now();

  // create an interactive marker for our server
  int_marker.header.frame_id = map_topic_;
  int_marker.name = goal_marker_name_;
  int_marker.description = "";

  // create a grey box marker
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = 0.1;
  box_marker.scale.y = 0.1;
  box_marker.scale.z = 0.1;
  box_marker.color.r = 1;
  box_marker.color.g = 0;
  box_marker.color.b = 0;
  box_marker.color.a = 1.0;

  // create a non-interactive control which contains the box
  box_control.always_visible = true;
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  int_marker.controls.push_back( box_control );

  // Menu Stuff
  menu_handler.insert( "Send Goal", boost::bind(&UAV_SET_GOAL_C::procFeedback, this, _1) );
  menu_handler.insert( "Hover", boost::bind(&UAV_SET_GOAL_C::procFeedback, this, _1) );
  menu_handler.insert( "Land", boost::bind(&UAV_SET_GOAL_C::procFeedback, this, _1) );
  menu_handler.insert( "Take off", boost::bind(&UAV_SET_GOAL_C::procFeedback, this, _1) );
  menu_handler.insert( "Square Test", boost::bind(&UAV_SET_GOAL_C::procFeedback, this, _1) );
  menu_handler.insert( "Auto Flight", boost::bind(&UAV_SET_GOAL_C::procFeedback, this, _1) );
  menu_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  menu_control.description="UAV Control Options";
  menu_control.name = "menu_only_control";
  menu_control.always_visible = true;
  int_marker.controls.push_back(menu_control);


  //TODO: these commands don't make sense
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
  server.insert(int_marker, boost::bind(&UAV_SET_GOAL_C::procFeedback, this, _1) );

  menu_handler.apply(server, int_marker.name );
  // 'commit' changes and send to all clients
  server.applyChanges();

  //force landing as initial flight mode
  uav_msgs::FlightModeRequest mode_msg;
  mode_msg.mode = uav_msgs::FlightModeRequest::LAND;
  flight_request_pub.publish(mode_msg);
  SquareTest = false;
}

UAV_SET_GOAL_C::~UAV_SET_GOAL_C() {

}

void UAV_SET_GOAL_C::FlightModeStatusCallback(uav_msgs::FlightModeStatusConstPtr status) {
  if (AutoFlight) {
	if (ros::Time::now().toSec() - lastTime.toSec() > 0.2) {
	  lastTime = ros::Time::now();
	  ROS_WARN("Auto Flight is in realm: %d\n", AutoFlightRealm);
	  switch (AutoFlightRealm) {
		case -1:  // just called for first time
			if (status->mode == uav_msgs::FlightModeStatus::LANDED) {
			  AutoFlightRealm = 1;  // only go to 1 if initially on the ground
			}
			break;
		case 1:  // take off
			if (status->mode == uav_msgs::FlightModeStatus::LANDED) {
			  ROS_INFO_STREAM("AF sending take off request");
			  uav_msgs::FlightModeRequest mode_msg;
			  mode_msg.mode = uav_msgs::FlightModeRequest::TAKE_OFF;
			  flight_request_pub.publish(mode_msg);
			}
			else if (status->mode == uav_msgs::FlightModeStatus::HOVER) {
			  AutoFlightRealm = 2;
			}
			break;
		case 2:  // initial goto point
		  ROS_INFO_STREAM("AF sending new goal !" );
		  goal_pose.header.stamp = ros::Time::now();
		  goal_pose_pub.publish(goal_pose);
		  AutoFlightRealm = 3;
		  break;
		case 3: // waiting for acknowledgment to follow path
		  if (status->mode == uav_msgs::FlightModeStatus::FOLLOWING) {
			AutoFlightRealm = 4;
		  }
		  break;
		case 4:  // done following - land and stop Auto Follow
			if (status->mode == uav_msgs::FlightModeStatus::HOVER) {
			  AutoFlightRealm = 0; AutoFlight = false;

			  ROS_INFO_STREAM("AF sending land request");
			  uav_msgs::FlightModeRequest mode_msg;
			  mode_msg.mode = uav_msgs::FlightModeRequest::LAND;
			  flight_request_pub.publish(mode_msg);
			}
			break;
	  }
	}
  }


  if (SquareTest) {
    if (ros::Time::now().toSec() - lastTime.toSec() > 0.2) {
      lastTime = ros::Time::now();
      ROS_WARN("Square test is at pt %d\n", SqTPt);
      switch (SqTPt) {
        case -2:    // wasn't at start, was landing, once landed, commence test
        if (status->mode == uav_msgs::FlightModeStatus::LANDED) {
          SqTPt = 0;
        }
        break;
        case -1:  // wasn't at the start, was moving, if hovering, then land
        if (status->mode == uav_msgs::FlightModeStatus::HOVER) {
          uav_msgs::FlightModeRequest mode_msg;
          mode_msg.mode = uav_msgs::FlightModeRequest::LAND;
          flight_request_pub.publish(mode_msg);
          SqTPt = -2;
        }
        break;
        case 0:
          if (status->mode != uav_msgs::FlightModeStatus::LANDED) {
            // send to (0,0,0)
            geometry_msgs::PoseStamped goal_pose;
            goal_pose.header.stamp = ros::Time::now();
            goal_pose.header.frame_id = map_topic_;
            goal_pose.pose.position.x = 0;
            goal_pose.pose.position.y = 0;
            goal_pose.pose.position.z = 1.0;
            goal_pose.pose.orientation.w = 1;
            goal_pose.pose.orientation.x = 0;
            goal_pose.pose.orientation.y = 0;
            goal_pose.pose.orientation.z = 0;
            goal_pose_pub.publish(goal_pose);
          }
          else {
            uav_msgs::FlightModeRequest mode_msg;
            mode_msg.mode = uav_msgs::FlightModeRequest::TAKE_OFF;
            flight_request_pub.publish(mode_msg);
            SqTPt = 1;
          }
          break;
        case 1:
          if (status->mode == uav_msgs::FlightModeStatus::HOVER) {
            geometry_msgs::PoseStamped goal_pose;
            goal_pose.header.stamp = ros::Time::now();
            goal_pose.header.frame_id = map_topic_;
            goal_pose.pose.position.x = 1;
            goal_pose.pose.position.y = 0;
            goal_pose.pose.position.z = 1.0;
            goal_pose.pose.orientation.w = sqrt(0.5);
            goal_pose.pose.orientation.x = 0;
            goal_pose.pose.orientation.y = 0;
            goal_pose.pose.orientation.z = sqrt(0.5);
            goal_pose_pub.publish(goal_pose);
            SqTPt = 2;
          }
          break;
        case 2:
          if (status->mode == uav_msgs::FlightModeStatus::HOVER) {
            geometry_msgs::PoseStamped goal_pose;
            goal_pose.header.stamp = ros::Time::now();
            goal_pose.header.frame_id = map_topic_;
            goal_pose.pose.position.x = 1;
            goal_pose.pose.position.y = 1;
            goal_pose.pose.position.z = 1.4;
            goal_pose.pose.orientation.w = 0;
            goal_pose.pose.orientation.x = 0;
            goal_pose.pose.orientation.y = 0;
            goal_pose.pose.orientation.z = 1;
            goal_pose_pub.publish(goal_pose);
            SqTPt = 3;
          }
          break;
        case 3:
          if (status->mode == uav_msgs::FlightModeStatus::HOVER) {
            geometry_msgs::PoseStamped goal_pose;
            goal_pose.header.stamp = ros::Time::now();
            goal_pose.header.frame_id = map_topic_;
            goal_pose.pose.position.x = 0;
            goal_pose.pose.position.y = 1;
            goal_pose.pose.position.z = 0.8;
            goal_pose.pose.orientation.w = sqrt(0.5);
            goal_pose.pose.orientation.x = 0;
            goal_pose.pose.orientation.y = 0;
            goal_pose.pose.orientation.z = -sqrt(0.5);
            goal_pose_pub.publish(goal_pose);
            SqTPt = 4;
          }
          break;
        case 4:
          if (status->mode == uav_msgs::FlightModeStatus::HOVER) {
            geometry_msgs::PoseStamped goal_pose;
            goal_pose.header.stamp = ros::Time::now();
            goal_pose.header.frame_id = map_topic_;
            goal_pose.pose.position.x = 0;
            goal_pose.pose.position.y = 0;
            goal_pose.pose.position.z = 1.0;
            goal_pose.pose.orientation.w = 1;
            goal_pose.pose.orientation.x = 0;
            goal_pose.pose.orientation.y = 0;
            goal_pose.pose.orientation.z = 0;
            goal_pose_pub.publish(goal_pose);
            SqTPt = 5;
          }
          break;
        case 5:  // back at the start now land
        if (status->mode == uav_msgs::FlightModeStatus::HOVER) {
          uav_msgs::FlightModeRequest mode_msg;
          mode_msg.mode = uav_msgs::FlightModeRequest::LAND;
          flight_request_pub.publish(mode_msg);
          SqTPt = 6;
        }
        break;
        case 6:  //landing order given
        if (status->mode == uav_msgs::FlightModeStatus::LANDED) {
          SqTPt =0;
          SquareTest = false;
        }
        break;
        default:
          SquareTest = false;
          break;
      }
    }
  }
}

void UAV_SET_GOAL_C::procFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM( feedback->marker_name << " is now at "
  << feedback->pose.position.x << ", " << feedback->pose.position.y
  << ", " << feedback->pose.position.z );

  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT)
  {
    if(feedback->menu_entry_id == MENU_ENTRY_NEW_GOAL)
    {
      ROS_INFO_STREAM("Sending new goal !" );
      //geometry_msgs::PoseStamped goal_pose;
      goal_pose.header.stamp = ros::Time::now();
      goal_pose.header.frame_id = map_topic_;
      goal_pose.pose = feedback->pose;
      goal_pose_pub.publish(goal_pose);
	  SquareTest = false;AutoFlight = false;
    }
    else if(feedback->menu_entry_id == MENU_ENTRY_HOVER)
    {
      ROS_INFO_STREAM("Sending hover request");
      uav_msgs::FlightModeRequest mode_msg;
      mode_msg.mode = uav_msgs::FlightModeRequest::HOVER;
      flight_request_pub.publish(mode_msg);
	  SquareTest = false;AutoFlight = false;
    }
    else if(feedback->menu_entry_id == MENU_ENTRY_LAND)
    {
      ROS_INFO_STREAM("Sending land request");
      uav_msgs::FlightModeRequest mode_msg;
      mode_msg.mode = uav_msgs::FlightModeRequest::LAND;
      flight_request_pub.publish(mode_msg);
	  SquareTest = false;AutoFlight = false;
    }
    else if(feedback->menu_entry_id == MENU_ENTRY_TAKE_OFF)
    {
      ROS_INFO_STREAM("Sending take off request");
      uav_msgs::FlightModeRequest mode_msg;
      mode_msg.mode = uav_msgs::FlightModeRequest::TAKE_OFF;
      flight_request_pub.publish(mode_msg);
	  SquareTest = false;AutoFlight = false;
    }
    else if(feedback->menu_entry_id == MENU_ENTRY_SQUARE_TEST)
    {
      ROS_INFO_STREAM("Starting Square Flight Test");
	  SquareTest = true;
	  AutoFlight = false;
      SqTPt = 0;
    }
    else if(feedback->menu_entry_id == MENU_ENTRY_AUTO_FLIGHT)
	{
	  ROS_INFO_STREAM("Starting Square Flight Test");
	  AutoFlight = true;
	  SquareTest = false;
	  AutoFlightRealm = -1;

	  // save the goal location
	  goal_pose.header.stamp = ros::Time::now();
	  goal_pose.header.frame_id = map_topic_;
	  goal_pose.pose = feedback->pose;
	}
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uav_set_goal");


  UAV_SET_GOAL_C uav_set_goal;

  ros::spin();
}

