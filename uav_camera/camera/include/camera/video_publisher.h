#ifndef VIDEO_PUBLISHER_H
#define VIDEO_PUBLISHER_H

#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera/camera_msg.h>
#include <stdio.h>
#include <boost/thread.hpp>

typedef struct {
	int width_high;
	int height_high;
	int width_low;
	int height_low;
	int current_width;
	int current_height;
} resolution;

typedef struct{
	double d[5];
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
		
		void decrease_resolution_callback(const camera::camera_msg msg);
		void increase_resolution_callback(const camera::camera_msg msg);

		void main_loop();
		boost::thread* main_loop_;

		ros::NodeHandle n_;
		image_transport::ImageTransport it_;

		cv::VideoCapture capture;
		image_transport::Publisher image_pub_;	
		ros::Publisher info_pub_;	
		ros::Publisher res_ready_pub_;
		ros::Subscriber dec_res_sub_;
		ros::Subscriber inc_res_sub_;

		int camera_number;
		bool camera_on_;
		resolution resolution_;
		ar_params ar_params_;
		cv::Mat camera_matrix_;
		cv::Mat distorted_coefficients_;
};

#endif
