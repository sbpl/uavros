#include <uav_camera/video_publisher.h>
#include <iostream>

/* Create a node handle and a video_publisher */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "video_publisher");
	ros::NodeHandle n;
	video_publisher vid_pub(n);
	ros::spin();
	return 0;
}

/* Constructor */
video_publisher::video_publisher(ros::NodeHandle &n): n_ (n), it_ (n_)
{
	get_params();
	create_publishers();
	create_subscribers();
	capture.open(camera_number);
	set_camera_calibration();

  dynamic_reconfigure::Server<uav_camera::thresholdConfig> server;
  dynamic_reconfigure::Server<uav_camera::thresholdConfig>::CallbackType f;

	f = boost::bind(&video_publisher::reconf, this, _1, _2);
  server.setCallback(f);

	main_loop();
}

/* Destructor */
video_publisher::~video_publisher()
{

}

void video_publisher::reconf(uav_camera::thresholdConfig &config, uint32_t level)
{
	threshold_ = config.t;
	ROS_INFO("New threshold: %d", threshold_);	
}

void video_publisher::main_loop() 
{
	cv::Mat frame;
	ros::Time ts;
	ros::Time last_ts = ros::Time::now();
	double fps;
	
	ros::Rate loop_rate(50);
	while(n_.ok()) {
		/* Grab a new frame */
		capture >> frame;
		ts = ros::Time::now();
	
		/* Get the undistort image with the camera parameters */
		cv::Mat undistorted, binary_image;
		undistort(frame, undistorted, camera_matrix_, distorted_coefficients_);

//		binarize(undistorted, binary_image);

		/* Create message to send the image */
		cv_bridge::CvImage cvi;
		cvi.header.stamp = ts;
		cvi.header.frame_id = camera_id;
		cvi.encoding = "bgr8";
		cvi.image = undistorted;

		/* Publish the message of the image */
		sensor_msgs::Image image_msg;
		cvi.toImageMsg(image_msg);
		image_pub_.publish(image_msg);

		/* Create the CameraInfo message */
		sensor_msgs::CameraInfo info_msg;
		info_msg.header.stamp = ts;
		info_msg.header.frame_id = camera_id;
		info_msg.width = resolution_.width;
		info_msg.height = resolution_.height;
		info_msg.distortion_model = "plumb_bob";
		info_msg.D = cv::vector<double>(ar_params_.d, ar_params_.d + 
						sizeof(ar_params_.d)/sizeof(*ar_params_.d));
		info_msg.K = ar_params_.k;
		info_msg.R = ar_params_.r;
		info_msg.P = ar_params_.p;

		/* Publish the message of the CameraInfo */
		info_pub_.publish(info_msg);
		info_msg.header.stamp = ros::Time::now();
		info_msg.header.frame_id = "raw";

		/* Count fps */
/*		if(ts - last_ts > ros::Duration(1.0)) {
			ROS_ERROR("%f", fps);
			fps = 0;
			last_ts = ts;
		} else {
			fps++;
		}
*/
		ros::spinOnce();
		loop_rate.sleep();
	}
}

/* Binary image from RGB8 to RGB8 with given threshold */
void video_publisher::binarize(cv::Mat &source, cv::Mat &dest)
{
	dest = cv::Mat(source.rows, source.cols, source.type());
	int sum;

	for(int y = 0; y < source.rows; ++y) {
		uchar *dest_ptr = dest.ptr(y);
		const uchar *source_ptr = source.ptr(y);
		for(int x = 0; x < source.cols; ++x) {
			sum = source_ptr[3*x] + source_ptr[3*x+1] + source_ptr[3*x+2];
			if(sum < threshold_ * 3) {
				dest_ptr[3*x] = 0;
				dest_ptr[3*x+1] = 0;
				dest_ptr[3*x+2] = 0;
			} else {
				dest_ptr[3*x] = 255;
				dest_ptr[3*x+1] = 255;
				dest_ptr[3*x+2] = 255;
			}
		}
	}			
}

/* Get the parameters, if none given, assume defaults */
void video_publisher::get_params()
{
	ros::NodeHandle n_param("~");

	if(!n_param.getParam("camera_number", camera_number)) {
		camera_number = 0;
	}
	if(!n_param.getParam("width", resolution_.width)) {
		resolution_.width= 1280;
	}
	if(!n_param.getParam("height", resolution_.height)) {
		resolution_.height= 960;
	}
}

/* Create the publishers to advertise in the corresponding topics */
void video_publisher::create_publishers()
{
	/* Get the string of the msg topics with the camera_num parameter */
	char cam_info[40], cam_image[40], cam_id[30];
	int n = sprintf(cam_image, "usb_cam%d/image_raw", camera_number);
	n = sprintf(cam_info, "usb_cam%d/camera_info", camera_number);
	n = sprintf(cam_id, "/usb_cam%d", camera_number);
	camera_id = cam_id;

	image_pub_ = it_.advertise(cam_image, 1);
	image_pub2_ = it_.advertise("/raw/image_raw", 1);
	info_pub_ = n_.advertise<sensor_msgs::CameraInfo>(cam_info, 1);
	info_pub2_ = n_.advertise<sensor_msgs::CameraInfo>("/raw/camera_info", 1);
}

/* Create the subscribers to the corresponding topcis */
void video_publisher::create_subscribers()
{

}

/* Set camera calibration values and ar_pose values */
void video_publisher::set_camera_calibration()
{
	capture.set(CV_CAP_PROP_BRIGHTNESS, 0.6);
	capture.set(CV_CAP_PROP_CONTRAST, 0.2);
	capture.set(CV_CAP_PROP_SATURATION, 0.2);

	capture.set(CV_CAP_PROP_FRAME_WIDTH, resolution_.width);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, resolution_.height);

	//Only for resolution of 1280, 960
	//Calibrate aug 1  1280 x 960
	camera_matrix_ = (cv::Mat_<double>(3,3) << 
									1635.281066, 0.000000, 607.644423,
									0.000000, 1625.623556, 480.528544,
									0.000000, 0.000000, 1.000000);

	distorted_coefficients_ = (cv::Mat_<double>(1,5) <<
						-1.477884, 1.031956, 0.026532, 0.017903, 0.000000);

	double temp_d[] = {-0.05, 0.0, 0.0, 0.0, 0.0};
	*ar_params_.d = *temp_d;
	ar_params_.k = {{ 730.0, 0.0, 640.0,
					  0.0, 730.0, 480.0,
					  0.0, 0.0, 1.0 }};
   ar_params_.r = { {1.0, 0.0, 0.0,
                     0.0, 1.0, 0.0,
                     0.0, 0.0, 1.0} };
	ar_params_.p = {{ 730.0, 0.0, 625.0, 0.0,
					  0.0, 730.0, 480.0, 0.0,
					  0.0, 0.0, 1.0, 0.0 }};
}


