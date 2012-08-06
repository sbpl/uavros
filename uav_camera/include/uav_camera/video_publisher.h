#ifndef VIDEO_PUBLISHER_H
#define VIDEO_PUBLISHER_H

#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <uav_msgs/camera_msg.h>
#include <stdio.h>
#include <boost/thread.hpp>
#include <dynamic_reconfigure/server.h>
#include <uav_camera/thresholdConfig.h>

typedef struct {
	int width;
	int height;
} resolution;

typedef struct{
	double d[8];
	boost::array<double, 9ul> k;
	boost::array<double, 9ul> r;
	boost::array<double, 12ul> p;
} ar_params;

class video_publisher {
	public:
		video_publisher(ros::NodeHandle &n);
		~video_publisher();

	private:
		void init();
		void get_params();
		void create_publishers();
		void create_subscribers();
		void set_camera_calibration();
		void binarize(cv::Mat &source, cv::Mat &dest);
		void reconf(uav_camera::thresholdConfig &config, uint32_t level);
		
		void main_loop();
		boost::thread* main_loop_;

		ros::NodeHandle n_;
		image_transport::ImageTransport it_;

		cv::VideoCapture capture;
		image_transport::Publisher image_pub_, image_pub2_;	
		ros::Publisher info_pub_, info_pub2_;	

		int camera_number;
		bool camera_on_;
		std::string camera_id;
		resolution resolution_;
		ar_params ar_params_;
		cv::Mat camera_matrix_;
		cv::Mat distorted_coefficients_;
		int threshold_;
};

#endif

