#ifndef uav_local_planner_TwistLocalPlanner_h
#define uav_local_planner_TwistLocalPlanner_h

#include <stdint.h>
#include <unordered_map>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <uav_msgs/FlightModeRequest.h>
#include <uav_msgs/FlightModeStatus.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <uav_msgs/ControllerCommand.h>

namespace uav_local_planner {

class TwistLocalPlanner
{
public:

    TwistLocalPlanner();

    bool init();
    int run();

private:

    struct TrackingContext
    {
        geometry_msgs::PoseStamped pose;
        geometry_msgs::TwistStamped vel;

        // indicate whether a new path was swapped into the controller path
        bool new_path;

        uav_msgs::FlightModeRequest flight_mode_request;

        TrackingContext() :
            pose(),
            vel(),
            new_path(false),
            flight_mode_request()
        {
            flight_mode_request.mode = uav_msgs::FlightModeRequest::NONE;
        }
    };

    struct State
    {
        typedef void (TwistLocalPlanner::*EnterStateCallback)(
            const TrackingContext&, int8_t);
        typedef void (TwistLocalPlanner::*ExitStateCallback)(
            const TrackingContext&, int8_t);
        typedef int8_t (TwistLocalPlanner::*OnStateCallback)(
            const TrackingContext&, geometry_msgs::Twist&);

        EnterStateCallback enter;
        OnStateCallback on;
        ExitStateCallback exit;
    };

    ros::NodeHandle m_nh;
    ros::NodeHandle m_ph;

    // Publishers
    ros::Publisher m_waypoint_vis_pub;
    ros::Publisher m_command_pub;
    ros::Publisher m_status_pub;
    ros::Publisher m_goal_pub;

    // Subscribers
    ros::Subscriber m_path_sub;
    ros::Subscriber m_goal_sub;
    ros::Subscriber m_state_sub;
    ros::Subscriber m_flight_mode_sub;

    // Parameters
    double m_controller_frequency;
    double m_landing_height;
    double m_nominal_height;
    double m_nominal_linear_velocity;
    double m_nominal_angular_velocity;

    nav_msgs::Path* m_controller_path;
    nav_msgs::Path* m_latest_path;
    nav_msgs::Path* m_callback_path;

    geometry_msgs::PoseStamped m_latest_goal;
    nav_msgs::Odometry::ConstPtr m_latest_state;

    double m_landing_z;
    geometry_msgs::PoseStamped m_hover_pose;

    bool m_new_path; // set to true if a path is received this iteration
    // set to non-none if a flight mode request is received this iteration
    uav_msgs::FlightModeRequest m_flight_mode;

    unsigned int m_path_idx;

    uav_msgs::FlightModeStatus m_prev_state;
    uav_msgs::FlightModeStatus m_curr_state;
    uav_msgs::FlightModeStatus m_next_state;

    std::unordered_map<int, State> m_states; // state machine

    geometry_msgs::Twist m_prev_cmd;

    geometry_msgs::Twist control(
        const geometry_msgs::Pose& pose,
        const geometry_msgs::Twist& vel,
        const geometry_msgs::Pose& target);

    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void stateCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void flightModeCallback(const uav_msgs::FlightModeRequest::ConstPtr& msg);

    bool updatePath();
    uav_msgs::FlightModeRequest getFlightMode();
    bool getRobotPose(
        geometry_msgs::PoseStamped& pose,
        geometry_msgs::TwistStamped& vel);

    void onEnter_Landed(
        const TrackingContext& context,
        int8_t prev_status);
    int8_t on_Landed(
        const TrackingContext& context,
        geometry_msgs::Twist& cmd);
    void onExit_Landed(
        const TrackingContext& context,
        int8_t next_status);

    void onEnter_Landing(
        const TrackingContext& context,
        int8_t prev_status);
    int8_t on_Landing(
        const TrackingContext& context,
        geometry_msgs::Twist& cmd);
    void onExit_Landing(
        const TrackingContext& context,
        int8_t next_status);

    void onEnter_TakeOff(
        const TrackingContext& context,
        int8_t prev_status);
    int8_t on_TakeOff(
        const TrackingContext& context,
        geometry_msgs::Twist& cmd);
    void onExit_TakeOff(
        const TrackingContext& context,
        int8_t next_status);

    void onEnter_Hover(
        const TrackingContext& context,
        int8_t prev_status);
    int8_t on_Hover(
        const TrackingContext& context,
        geometry_msgs::Twist& cmd);
    void onExit_Hover(
        const TrackingContext& context,
        int8_t next_status);

    void onEnter_Following(
        const TrackingContext& context,
        int8_t prev_status);
    int8_t on_Following(
        const TrackingContext& context,
        geometry_msgs::Twist& cmd);
    void onExit_Following(
        const TrackingContext& context,
        int8_t next_status);
};

} // nanmespace uav_local_planner

#endif
