#include "uav_goal_interpolate.h"

UAVGoalInterpolate::UAVGoalInterpolate()
{
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    goal_sub_topic_ = "goal_pose";
    pose_sub_topic_ = "uav_state";
    path_pub_topic_ = "path";

    ph.param<std::string>("map_frame_topic", map_frame_topic_, "map");
    ph.param("Size_of_divisions", DivSize_, 0.1);

    path_pub_ = nh.advertise<nav_msgs::Path>(path_pub_topic_, 1);

    goal_sub_ = nh.subscribe(goal_sub_topic_, 1, &UAVGoalInterpolate::GoalCallback, this);
    pose_sub_ = nh.subscribe(pose_sub_topic_, 1, &UAVGoalInterpolate::PoseCallback, this);
}

void UAVGoalInterpolate::PoseCallback(nav_msgs::OdometryConstPtr state)
{
    boost::unique_lock<boost::mutex> lock(pose_mutex_);
    pose_ = *state;
    lock.unlock();
}

void UAVGoalInterpolate::GoalCallback(geometry_msgs::PoseStampedConstPtr goal)
{
    float x, y, z;
    boost::unique_lock<boost::mutex> lock(pose_mutex_);
    x = pose_.pose.pose.position.x;
    y = pose_.pose.pose.position.y;
    z = pose_.pose.pose.position.z;
    lock.unlock();

    float gx, gy, gz;
    gx = goal->pose.position.x;
    gy = goal->pose.position.y;
    gz = goal->pose.position.z;

    float norm;
    norm = sqrt((gx - x) * (gx - x) + (gy - y) * (gy - y) + (gz - z) * (gz - z));

    int divnorm = norm / DivSize_ + 1;

    float dx, dy, dz;
    dx = x;
    dy = y;
    dz = z;

    std::vector<geometry_msgs::PoseStamped> pathpts;
    geometry_msgs::PoseStamped pt;
    pathpts.resize(divnorm - 1);

    for (int idx = 0; idx < divnorm - 1; idx++) {
        pathpts[idx].header.frame_id = map_frame_topic_;
        pathpts[idx].pose.position.x = dx + (gx - x) / divnorm;
        pathpts[idx].pose.position.y = dy + (gy - y) / divnorm;
        pathpts[idx].pose.position.z = dz + (gz - z) / divnorm;
        pathpts[idx].pose.orientation = goal->pose.orientation;

        dx = dx + (gx - x) / divnorm;
        dy = dy + (gy - y) / divnorm;
        dz = dz + (gz - z) / divnorm;
    }

    pathpts.push_back(*goal);

    nav_msgs::Path path;
    path.poses = pathpts;
    path.header.frame_id = map_frame_topic_;
    path.header.stamp = ros::Time::now();

    path_pub_.publish(path);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uav_goal_interpolate");
    UAVGoalInterpolate Goal_interpolate;
    ros::spin();
    return 0;
}
