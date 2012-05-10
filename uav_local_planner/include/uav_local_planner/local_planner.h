
#ifndef UAV_LOCAL_PLANNER_H
#define UAV_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <uav_msgs/FlightModeRequest.h>
#include <uav_msgs/ControllerCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <arm_navigation_msgs/CollisionMap.h>
#include <tf/transform_listener.h>
#include <uav_collision_checking/uav_collision_space.h>

enum UAVControllerState {
  LANDED,
  LANDING,
  TAKE_OFF,
  HOVER,
  FOLLOWING
};

class UAVLocalPlanner{
  public:
    UAVLocalPlanner();
    ~UAVLocalPlanner();

    void controllerThread();

    uav_msgs::ControllerCommand land(geometry_msgs::PoseStamped, geometry_msgs::TwistStamped);
    uav_msgs::ControllerCommand takeOff(geometry_msgs::PoseStamped, geometry_msgs::TwistStamped);
    uav_msgs::ControllerCommand hover(geometry_msgs::PoseStamped, geometry_msgs::TwistStamped);
    uav_msgs::ControllerCommand followPath(geometry_msgs::PoseStamped, geometry_msgs::TwistStamped, bool isNewPath);


  private:
    bool updateCollisionMap();
    bool updatePath(UAVControllerState& state);
    void getFlightMode(UAVControllerState& state);
    void getRobotPose(geometry_msgs::PoseStamped& pose, geometry_msgs::TwistStamped& velocity);
  
    void collisionMapCallback(arm_navigation_msgs::CollisionMapConstPtr cm);
    void pathCallback(nav_msgs::PathConstPtr path);
    void goalCallback(geometry_msgs::PoseStampedConstPtr goal);
    void flightModeCallback(uav_msgs::FlightModeRequestConstPtr req);

    double sizex_, sizey_, sizez_, resolution_;

    ros::Publisher waypoint_vis_pub_;
    ros::Publisher command_pub_;
    ros::Publisher goal_pub_;
    ros::Subscriber collision_map_sub_;
    ros::Subscriber path_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber flight_mode_sub_;
    
    UAVCollisionSpace* cspace_;
    OccupancyGrid* controller_grid_;
    OccupancyGrid* latest_grid_;
    OccupancyGrid* callback_grid_;
    nav_msgs::Path* controller_path_;
    nav_msgs::Path* latest_path_;
    nav_msgs::Path* callback_path_;
    uav_msgs::FlightModeRequest flight_mode_;
    geometry_msgs::PoseStamped latest_goal_;

    boost::thread* controller_thread_;
    boost::mutex grid_mutex_;
    boost::mutex path_mutex_;
    boost::mutex flight_mode_mutex_;
    bool new_grid_, new_path_;

    double controller_frequency_;
    double collision_map_tolerance_, pose_tolerance_;

    tf::TransformListener tf_;
};

#endif
