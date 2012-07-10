#ifndef PLATFORM_CONTROLLER_H
#define PLATFORM_CONTROLLER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <platform/mode_msg.h>
#include <camera/camera_msg.h>
#include <string.h>
#include <math.h>

#define PI  3.14159265

#define DISTANCE_FROM_PLATFORM  2       //in meters
#define MARKER_ANGLE            0       //angle of the marker on respect to the plataform
#define INTERVALS_ANGLE         15      //angle to mark for each zone
#define HOVER_ABOVE_PLATFORM    0.5     //in meters
#define PLATFORM_ANGLE          180     //In angles

/* Track modes */
#define ALIGN_FRONT 	1
#define ALIGN_TOP   	2
#define ROTATE      	3
#define LAND        	4
#define CHANGE_FRONT 	5
#define CHANGE_BOTTOM	6

#define FRONT_CAMERA    0
#define BOTTOM_CAMERA   1

class platform_controller {
	public:
		platform_controller(ros::NodeHandle &n);
		~platform_controller();

	private:
		void init();
		void get_params();
		void create_publishers();
		void create_subscribers();
		
		void transform_callback(const tf::tfMessageConstPtr msg);
		void mode_callback(const platform::mode_msg msg);

		void align_front(tf::StampedTransform transform);
		void align_top(tf::StampedTransform transform, bool bottom_camera);
		void rotate();
		void land();

		void get_transform(std::string parent, std::string child,
						   tf::StampedTransform &transform);
		void get_pose(double pos[3], double quat[4], 
					  tf::StampedTransform transform);
		void quat_to_euler(double q[4], double r[3]);

		ros::NodeHandle n_;
		ros::Publisher change_res_pub_;
		ros::Subscriber tf_sub_;
		ros::Subscriber track_sub_;

		int track_mode_;
		std::string align_front_;
		std::string align_top_;
};

#endif
