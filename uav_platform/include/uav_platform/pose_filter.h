#ifndef POSE_FILTER_H
#define POSE_FILTER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <string.h>

class pose_filter{
	public:
		pose_filter(ros::NodeHandle &n);

	private:
		void init();
		void get_params();
		void create_subscribers();
		
		void tf_callback(const tf::tfMessageConstPtr msg);
		void filter(const tf::tfMessageConstPtr msg);
		void publish_filtered();

		ros::NodeHandle n_;
		ros::Subscriber tf_sub_;

		double filter_ratio_;
		std::string frame_id_; 
		std::string child_frame_id_;

		btQuaternion filtered_quat_; 
		btVector3 filtered_pos_;
};

#endif
