#include <ros/ros.h>
#include <string>
#include <interactive_markers/menu_handler.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/PoseStamped.h>
#include "uav_msgs/FlightModeRequest.h"
#include "uav_msgs/FlightModeStatus.h"
#include "boost/thread/mutex.hpp"

#define MENU_ENTRY_NEW_GOAL 1
#define MENU_ENTRY_HOVER 2
#define MENU_ENTRY_LAND 3
#define MENU_ENTRY_TAKE_OFF 4
#define MENU_ENTRY_SQUARE_TEST 5
#define MENU_ENTRY_AUTO_FLIGHT 6

class UAV_SET_GOAL_C
{
public:

    UAV_SET_GOAL_C();
    ~UAV_SET_GOAL_C();

private:

    interactive_markers::MenuHandler menu_handler;

    visualization_msgs::InteractiveMarkerControl box_control;
    visualization_msgs::InteractiveMarkerControl control;
    visualization_msgs::InteractiveMarkerControl menu_control;

    visualization_msgs::InteractiveMarker int_marker;
    visualization_msgs::Marker box_marker;

    interactive_markers::InteractiveMarkerServer server;

    ros::Publisher goal_pose_pub;
    ros::Publisher flight_request_pub;
    ros::Subscriber flight_status_sub;

    bool SquareTest;
    bool AutoFlight;
    int SqTPt;
    int AutoFlightRealm;

    geometry_msgs::PoseStamped goal_pose;

    ros::Time lastTime;
    std::string goal_pub_topic_;
    std::string flt_mode_req_topic_;
    std::string flt_mode_stat_topic_;
    std::string map_topic_;
    std::string goal_marker_name_;

    std::string tf_prefix_;

    void procFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    void FlightModeStatusCallback(uav_msgs::FlightModeStatusConstPtr status);

};
