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
//	frame = cv::imread("~/home/eduardo/Desktop/pic_3.jpeg", 1);
//	ROS_INFO("%d, %d", frame.cols, frame.rows);

//	ros::Rate loop_rate(0.5);
	ros::Rate loop_rate(50);
	while(n_.ok()) {
		/* Grab a new frame */
		capture >> frame;
	
		/* Get the undistort image with the camera parameters */
		cv::Mat undistorted, binary_image;
		undistort(frame, undistorted, camera_matrix_, distorted_coefficients_);

		binarize(undistorted, binary_image);

//		cv::imwrite("pic.jpeg", undistorted);
//		ROS_INFO("Image written");

		/* Create message to send the image */
		cv_bridge::CvImage cvi;
		cvi.header.stamp = ros::Time::now();
		cvi.header.frame_id = camera_id;
		cvi.encoding = "bgr8";
		cvi.image = binary_image;
//		cvi.image = undistorted;
//		cvi.image = frame;

		/* Publish the message of the image */
		sensor_msgs::Image image_msg;
		cvi.toImageMsg(image_msg);
		image_pub_.publish(image_msg);

		cvi.image = undistorted;
		cvi.toImageMsg(image_msg);
		image_pub2_.publish(image_msg);

		/* Create the CameraInfo message */
		sensor_msgs::CameraInfo info_msg;
		info_msg.header.stamp = ros::Time::now();
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
		info_pub2_.publish(info_msg);

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
		resolution_.width= 640;
	}
	if(!n_param.getParam("height", resolution_.height)) {
		resolution_.height= 480;
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

//	if(resolution_.width == 640) {
	    /* Parameters for camera with fish-eye lens */
/*   	 	camera_matrix_ = (cv::Mat_<double>(3,3) << 1408.831918, 0.0, 294.668141,
	   	                                           0.0, 1389.801615, 237.959610,
                                         		   0.0, 0.0, 1.0);
	    distorted_coefficients_ = (cv::Mat_<double>(1,5) << 
								-3.3458, 10.736563, 0.077535, 0.003666, 0.0);
*/
        /* Parameters so ARToolkit think we have a normal   *
         * camera, and see a rectify image                  */
/*		double temp_d[] = {-0.363638, 0.095521, 0.002198, 0.002716, 0.000000};
        *ar_params_.d = *temp_d;
        ar_params_.k = { {450.724360, 0.000000, 317.891495,
                          0.000000, 444.956819, 233.815601,
                          0.0, 0.0, 1.0} };
        ar_params_.r = { {1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0} };
        ar_params_.p = { {450.72436, 0.000000, 317.891495, 0.0,
                          0.000000, 444.956819, 233.815601, 0.0,
                          0.0, 0.0, 1.0, 0.0} };
*/		
		//Calibration July 30  640 x 480
/*		camera_matrix_ = (cv::Mat_<double>(3,3) << 1395.791709, 0.0, 287.558558,
												   0.0, 1407.777487, 236.280335,
												   0.0, 0.0, 1.0);

		distorted_coefficients_ = (cv::Mat_<double>(1,5) <<
							-3.632066, 8.878626, 0.017618, 0.032021, 0.000000);
		
		double temp_d[] = {-0.024614, 0.017706, -0.005636, 0.018934, 0.000000};
		*ar_params_.d = *temp_d;
		ar_params_.k = { {428.897061, 0.000000, 320.725215,
						  0.000000, 424.474553, 242.396429,
						  0.000000, 0.000000, 1.000000} };
        ar_params_.r = { {1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0} };
		ar_params_.p = { {425.922150, 0.000000, 336.015247, 0.000000,
						  0.000000, 431.503204, 238.508429, 0.000000,
 						  0.000000, 0.000000, 1.000000, 0.000000} };
*/
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

/*		double temp_d[] = {-0.024614, 0.017706, -0.005636, 0.018934, 0.000000};
		*ar_params_.d = *temp_d;
		ar_params_.k = { {2*428.897061, 0.000000, 2*320.725215,
						  0.000000, 2*424.474553, 2*242.396429,
						  0.000000, 0.000000, 1.000000} };
        ar_params_.r = { {1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0} };
		ar_params_.p = { {2*425.922150, 0.000000, 2*336.015247, 0.000000,
						  0.000000, 2*431.503204, 2*238.508429, 0.000000,
 						  0.000000, 0.000000, 1.000000, 0.000000} };
*/
/*
		double temp_d[] = {-1.427884, 1.231956, 0.016532, 0.017903, 0.000000};
		*ar_params_.d = *temp_d;
		ar_params_.k = {{1645.281066, 0.000000, 617.644423,
						0.000000, 1635.623556, 467.528544,
						0.000000, 0.000000, 1.000000}};
        ar_params_.r = { {1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0} };
		ar_params_.p = {{1645.281066, 0.000000, 617.644423, 0.0,
						0.000000, 1635.623556, 467.528544, 0.0,
						0.000000, 0.000000, 1.000000, 0.0}};
*/	
/*		CvMat* new_matrix = cvCreateMat(3, 3, CV_32FC1);
		CvSize size;
		size.width = 640;
		size.height = 480;
		CvSize new_size;	
		CvMat cam_matrix = camera_matrix_;
		CvMat dist = distorted_coefficients_;
	
		cvGetOptimalNewCameraMatrix(&cam_matrix, &dist, size, 0.0, new_matrix, new_size);	
	
		for(int i=0; i<3; i++) {
			for(int j=0; j<3; j++) {
				CvScalar scal = cvGet2D(new_matrix, i, j);
				ROS_INFO("%d: %f", i*4+j, scal.val[0]);
				ar_params_.k[i*3+j] = scal.val[0];
				ar_params_.p[i*4+j] = scal.val[0];
			}
			ar_params_.p[i*4+3] = 0.0;
		}					 
*/
//		camera_matrix_ = (cv::Mat_<double>(3,3) << 
//129.590942, 0.0, 280.704895, 0.0, 1228.940918, 238.314178, 0.0, 0.0, 1.0);
		
//		cv::Mat amera_matrix_(cam_matrix);

//		double temp_d2[] = {0.0, 0.0, 0.0, 0.0, 0.0};
//		*ar_params_.d = *temp_d2;

//		} else {

		/* For bottom camera */
/*    	camera_matrix_ = (cv::Mat_<double>(3,3) << 214.651930, 0.0, 158.156551,
												   0.0, 214.041442, 112.968173,
												   0.0, 0.0, 1.0);

	    distorted_coefficients_ = (cv::Mat_<double>(1,5) << 
							-0.361626, 0.096834, -0.002050, -0.001804, 0.0000);	

    	double temp_d[] = {-0.022142, 0.030375, 0.000896, 0.002747, 0.000000};
		*ar_params_.d = *temp_d;
	    ar_params_.k = { {215.904518, 0.000000, 162.810658,
						  0.000000, 214.999713, 114.332141,
						  0.000000, 0.000000, 1.000000} };
	    ar_params_.r = { {1.0, 0.0, 0.0,
	                      0.0, 1.0, 0.0,
	                      0.0, 0.0, 1.0} };
	    ar_params_.p = { {215.904518, 0.000000, 162.810658, 0.0,
						  0.000000, 214.999713, 114.332141, 0.0,
						  0.000000, 0.000000, 1.000000, 0.0} };
	}

*/
}


