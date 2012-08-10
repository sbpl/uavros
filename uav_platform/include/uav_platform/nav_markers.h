#ifndef PLATFORM_CONTROLLER_H
#define PLATFORM_CONTROLLER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <uav_msgs/mode_msg.h>
#include <uav_msgs/camera_msg.h>
#include <string.h>
#include <math.h>

#define PI  3.14159265

#define LAND            0
#define APPROACH_POINT  1
#define LAND_POINT      2

#define IN_RANGE_DIST	0.20	//in meters
#define IN_RANGE_RAD	PI/6	//in radians

const ros::Duration IN_RANGE_TIME(5.0); 	//in seconds
const ros::Duration GOAL_FREQUENCY(0.1);	//in seconds
const ros::Duration MARKER_FREQUENCY(0.5);	//in seconds

typedef struct {
	double x;
	double y;
	double z;
	double theta;
} goal;

typedef struct {
	double approach[3];
	double land[3];
	double height;
} marker_goal;

class nav_markers {
	public:
		nav_markers(ros::NodeHandle &n);
		~nav_markers();

	private:
		void init();
		void get_params();
		void create_publishers();
		void create_subscribers();
		
		void mode_callback(const uav_msgs::mode_msg msg);
		void align_done_callback(const ros::TimerEvent&);
		void transform_callback(tf::tfMessageConstPtr msg);

		void align(tf::tfMessageConstPtr msg, double goal_from_marker[3]);
		void land();

		void check_time();
		bool check_in_range();

		void update_goal(goal goal_calculated);
		void publish_goal(double x, double y, double z, double theta);
		bool get_transform(std::string parent, std::string child,
						   tf::StampedTransform &transform);
		void get_pose_from_msg(tf::tfMessageConstPtr msg, btVector3 &p,
                               btQuaternion &q);
		void get_pose_from_tf(double pos[3], double quat[4], 
					 		  tf::StampedTransform transform);
		void quat_to_euler(double q[4], double r[3]);

		ros::NodeHandle n_;
		ros::Publisher goal_pose_pub_;
		ros::Subscriber tf_sub_;
		ros::Subscriber mode_sub_;

		ros::Time old_goal_ts_;
		ros::Time last_marker_ts_;
		ros::Timer timer_;

		int align_marker_;
		int align_mode_;

		geometry_msgs::PoseStamped goal_pose_;
		goal goal_;
		marker_goal marker_goals_[4];
};

#endif
