
#ifndef UAV_LOCAL_PLANNER_H
#define UAV_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <uav_msgs/FlightModeRequest.h>
#include <uav_msgs/FlightModeStatus.h>
#include <uav_msgs/ControllerCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
// #include <nav_msgs/OccupancyGrid.h>
#include <arm_navigation_msgs/CollisionMap.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <uav_local_planner/controller.h>
#include <string>

// enum UAVControllerState {
//   LANDED,
//   LANDING,
//   TAKE_OFF,ss
//   HOVER,
//   FOLLOWING
// };

class UAVLocalPlanner{
  public:
    UAVLocalPlanner();
    ~UAVLocalPlanner();

    void controllerThread();

    uav_msgs::ControllerCommand land(geometry_msgs::PoseStamped, geometry_msgs::TwistStamped, uav_msgs::FlightModeStatus&);
    uav_msgs::ControllerCommand takeOff(geometry_msgs::PoseStamped, geometry_msgs::TwistStamped, uav_msgs::FlightModeStatus&);
    uav_msgs::ControllerCommand hover(geometry_msgs::PoseStamped, geometry_msgs::TwistStamped);
    uav_msgs::ControllerCommand followPath(geometry_msgs::PoseStamped, geometry_msgs::TwistStamped, uav_msgs::FlightModeStatus&, bool isNewPath);


  private:
    bool updateCollisionMap();
    bool updatePath(uav_msgs::FlightModeStatus&);
    void getFlightMode(uav_msgs::FlightModeStatus &state);
    void getRobotPose(geometry_msgs::PoseStamped& pose, geometry_msgs::TwistStamped& velocity);

    void collisionMapCallback(arm_navigation_msgs::CollisionMapConstPtr cm);
    void pathCallback(nav_msgs::PathConstPtr path);
    void goalCallback(geometry_msgs::PoseStampedConstPtr goal);
    void stateCallback(nav_msgs::OdometryConstPtr state);
    void flightModeCallback(uav_msgs::FlightModeRequestConstPtr req);
    void visualizeTargetPose(geometry_msgs::PoseStamped p);

    UAVController controller;
    dynamic_reconfigure::Server<uav_local_planner::UAVControllerConfig> dynamic_reconfigure_server_;

    double sizex_, sizey_, sizez_, resolution_;
    std::string flt_mode_req_topic_, flt_mode_stat_topic_, ctrl_cmd_topic_, goal_pub_topic_, goal_sub_topic_, next_waypoint_topic_, local_collision_topic_, uav_state_topic_, path_topic_;

    ros::Publisher waypoint_vis_pub_;
    ros::Publisher command_pub_;
    ros::Publisher RPYT_pub_;
    ros::Publisher status_pub_;
    ros::Publisher goal_pub_;
    ros::Subscriber collision_map_sub_;
    ros::Subscriber path_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber flight_mode_sub_;

    //UAVCollisionSpace* cspace_;
//     OccupancyGrid* controller_grid_;
//     OccupancyGrid* latest_grid_;
//     OccupancyGrid* callback_grid_;
    nav_msgs::Path* controller_path_;
    nav_msgs::Path* latest_path_;
    nav_msgs::Path* callback_path_;
    uav_msgs::FlightModeRequest flight_mode_;
    geometry_msgs::PoseStamped latest_goal_;
    nav_msgs::Odometry latest_state_;

    boost::thread* controller_thread_;
    boost::mutex grid_mutex_;
    boost::mutex path_mutex_;
    boost::mutex goal_mutex_;
    boost::mutex state_mutex_;
    boost::mutex flight_mode_mutex_;
    bool new_grid_, new_path_;

    double controller_frequency_;
    double collision_map_tolerance_, pose_tolerance_;
    double landing_height_, nominal_height_, nominal_linear_velocity_, nominal_angular_velocity_, landing_z_;


    uav_msgs::ControllerCommand last_u_;
    geometry_msgs::PoseStamped hover_pose_;
    uav_msgs::FlightModeStatus last_state_;
    unsigned int path_idx_;
};

#endif
