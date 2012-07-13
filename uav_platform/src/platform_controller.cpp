#include <platform/platform_controller.h>

/* Creates a node handle and a platform controller */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "platform_controller");
	ros::NodeHandle n;
	platform_controller ptf_ctrl(n);
	ros::spin();
	return 0;
}

/* Constructor */
platform_controller::platform_controller(ros::NodeHandle &n): n_ (n)
{
	get_params();
	create_publishers();
	create_subscribers();
	ros::spin();
}

/* Destructor */
platform_controller::~platform_controller()
{

}

/* Get the parameters, if none given, assume defaults */
void platform_controller::get_params()
{
	ros::NodeHandle n_param("~");

	if(!n_param.getParam("align_front", align_front_)) {
		align_front_ = "/align_front";
	}
	if(!n_param.getParam("align_top", align_top_)) {
		align_top_ = "/align_top";
	}
}

/* Create publishers to advertis in the corresponding topics */
void platform_controller::create_publishers()
{
	change_res_pub_ = n_.advertise<uav_msgs::camera_msg>("dec_res", 1);
	goal_pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>("/goal_pose",1);
}

/* Create the subscribers to the corresponding topics */
void platform_controller::create_subscribers()
{
	tf_sub_ = n_.subscribe("/tf", 1000, 
						  &platform_controller::transform_callback, this);
	track_sub_ = n_.subscribe("/track_mode", 1000, 
							 &platform_controller::mode_callback, this);
}

/* Function call every time tf send a message */
void platform_controller::transform_callback(tf::tfMessageConstPtr msg)
{
    tf::StampedTransform transform;

    /* Check wether we are seeing a marker and take action  *
     * depending on the last track mode send                */
    if(msg->transforms[0].child_frame_id == "/marker1") {
        if(track_mode_ == ALIGN_FRONT) {
				//get_transform("/usb_cam", align_front_, transform);
				//align_front(transform);
			align_front(msg);
        } else if(track_mode_ == ALIGN_TOP) {
            //get_transform("/usb_cam", align_top_, transform);
            //align_top(transform, FRONT_CAMERA);
			align_top(msg, FRONT_CAMERA);
        }
    } else if(msg->transforms[0].child_frame_id == "/marker2") {
        if(track_mode_ == ALIGN_TOP) {
            //get_transform("/usb_cam", "/align_top", transform);
            //align_top(transform, BOTTOM_CAMERA);
			align_top(msg, BOTTOM_CAMERA);
        } else if(track_mode_ == ROTATE) {
            rotate();
        } else if(track_mode_ == LAND) {
            land();
        }
    }
}

/* Callback when the track mode is changed */
void platform_controller::mode_callback(uav_msgs::mode_msg msg)
{
    track_mode_ = msg.mode;
	if(track_mode_ == CHANGE_FRONT) {
		uav_msgs::camera_msg change_msg;
		change_msg.change_res = BOTTOM_CAMERA;
		change_res_pub_.publish(change_msg);
	} else if(track_mode_ == CHANGE_BOTTOM) {
		uav_msgs::camera_msg change_msg;
		change_msg.change_res = FRONT_CAMERA;
		change_res_pub_.publish(change_msg);
	}
}

/* Function to analize the transform from the camera to the align goal, *
 * the goal is to move in front of the plataform but, not rotating      *
 * always looking to the marker, but a mid point specified in the       *
 * auxiliar front frame                                                 */
//void platform_controller::align_front(tf::StampedTransform transform)
void platform_controller::align_front(tf::tfMessageConstPtr msg)
{
    double pos[3], quat[4];
	//get_pose(pos, quat, transform);
	get_pose2(msg);

    /* Get Euler angles with the quaternion */
	//double euler_rad[3], theta;
	//quat_to_euler(quat, euler_rad);
	//theta = euler_rad[1];
	double theta = pose_.rot.y;

    /* Assume theta is positive */
    double theta_sign = 1;

    /* If theta is negative, adjust */
    if(theta < 0) {
        theta_sign = -1;
    }

    /* Get the current zone depending on the *
     * angle range of each zone              */
    double interval_rad = INTERVALS_ANGLE * PI / 180;
    int current_zone = floor(fabs(theta) / interval_rad);

    /* Get goal angle */
    double marker_rad = MARKER_ANGLE * PI / 180;
    double goal_theta = theta_sign * (current_zone) * interval_rad + marker_rad;

    /* Get goal position */
	//double goal_x = pos[0] + DISTANCE_FROM_PLATFORM * cos(theta);
	//double goal_y = pos[1] + DISTANCE_FROM_PLATFORM * sin(theta);
    double goal_x = pose_.pos.x;// + DISTANCE_FROM_PLATFORM * cos(theta);
    double goal_y = pose_.pos.y;// + DISTANCE_FROM_PLATFORM * sin(theta);
	double goal_z = pose_.pos.z;

	ROS_INFO("Goal: %f, %f, %f", goal_x, goal_y, goal_theta * 180 / PI);
    
	tf::StampedTransform transform;
    get_transform("/map", "/usb_cam0", transform);
    get_pose(pos, quat, transform);
	double euler_rad[3];
    quat_to_euler(quat, euler_rad);
    
    goal_x += pos[1];
    goal_y += pos[2];
	goal_z += pos[0];

	/* With zones */
	//goal_theta += euler_rad[1];

	/* Without zones */
	goal_theta = theta + euler_rad[1];
    
    ROS_INFO("Map Frame Goal: %f, %f, %f", goal_x, goal_y, goal_theta * 180 / PI);
    
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.stamp = ros::Time::now();
    goal_pose.header.frame_id = "/map";
    goal_pose.pose.position.x = goal_z - DISTANCE_FROM_PLATFORM;
    goal_pose.pose.position.y = goal_x;
    goal_pose.pose.position.z = HOVER_ABOVE_PLATFORM - goal_y;
    goal_pose.pose.orientation.w = cos(goal_theta/2);
    goal_pose.pose.orientation.x = 0;
    goal_pose.pose.orientation.y = 0;
    goal_pose.pose.orientation.z = 0;
    goal_pose_pub_.publish(goal_pose);
    
}

/* Align on top of the marker */
//void platform_controller::align_top(tf::StampedTransform transform, 
//									bool bottom_camera)
void platform_controller::align_top(tf::tfMessageConstPtr msg, int camera)
{
    double pos[3], quat[4];
    get_pose2(msg);
	tf::StampedTransform transform;

    double goal_x_w, goal_y_w, goal_z_w;
    if(camera == FRONT_CAMERA) {
		goal_x_w = pose_.pos.z + WIDTH_PLATFORM;
        goal_y_w = pose_.pos.x;
		goal_z_w = pose_.pos.y;
    	get_transform("/map", "/usb_cam0", transform);
    } else {
        goal_x_w = pose_.pos.x;
        goal_y_w = pose_.pos.y;
        goal_z_w = pose_.pos.z;
    	get_transform("/map", "/usb_cam1", transform);
    }

    /* Get Euler angles with the quaternion */
    double euler_rad[3], theta;
    quat_to_euler(quat, euler_rad);
    theta = euler_rad[2];

    get_pose(pos, quat, transform);

	goal_x_w += pos[0];
	goal_y_w += pos[1];
	goal_z_w = pos[2] - goal_z_w;

    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.stamp = ros::Time::now();
    goal_pose.header.frame_id = "/map";
    goal_pose.pose.position.x = goal_x_w;
    goal_pose.pose.position.y = goal_y_w;
    goal_pose.pose.position.z = HOVER_ABOVE_PLATFORM + goal_z_w;
    goal_pose.pose.orientation.w = 0;//cos(goal_theta/2);
    goal_pose.pose.orientation.x = 0;
    goal_pose.pose.orientation.y = 0;
    goal_pose.pose.orientation.z = 0;
    goal_pose_pub_.publish(goal_pose);
}

/* Rotate the angle desired above marker */
void platform_controller::rotate() 
{
    tf::StampedTransform transform;
    get_transform("/usb_cam", "/marker2", transform);

    double pos[3], quat[4];
    get_pose(pos, quat, transform);

    /* Get Euler angles with the quaternion */
    double euler_rad[3], theta;
    quat_to_euler(quat, euler_rad);
    theta = euler_rad[0] + PLATFORM_ANGLE * 180 / PI;

    ROS_INFO("Goal: %f, %f, %f", pos[0], pos[1], theta);
}

/* Land on marker */
void platform_controller::land() 
{
    ROS_INFO("Need to land");
}

/* Get transform between the two given frames */
void platform_controller::get_transform(std::string parent, std::string child,
					                    tf::StampedTransform &transform)
{
    tf::TransformListener listener;
    try {
        listener.waitForTransform(parent, child, ros::Time(), 
								  ros::Duration(3.0));
        listener.lookupTransform(parent, child, ros::Time(), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }
}


void platform_controller::get_pose2(tf::tfMessageConstPtr msg)
{
    pose_.pos.x = msg->transforms[0].transform.translation.x;
    pose_.pos.y = msg->transforms[0].transform.translation.y;
    pose_.pos.z = msg->transforms[0].transform.translation.z;

	double quat[4], euler_rad[3];
    quat[0] = msg->transforms[0].transform.rotation.x;
    quat[1] = msg->transforms[0].transform.rotation.y;
    quat[2] = msg->transforms[0].transform.rotation.z;
    quat[3] = msg->transforms[0].transform.rotation.w;
	quat_to_euler(quat, euler_rad);

	pose_.rot.x = euler_rad[0];
	pose_.rot.y = euler_rad[1];
	pose_.rot.z = euler_rad[2];
}
	

/* Get position translation and rotation of the transform */
void platform_controller::get_pose(double pos[3], double quat[4], 
								   tf::StampedTransform transform)
{
    pos[0] = transform.getOrigin().x();
    pos[1] = transform.getOrigin().y();
    pos[2] = transform.getOrigin().z();

    quat[0] = transform.getRotation().x();
    quat[1] = transform.getRotation().y();
    quat[2] = transform.getRotation().z();
    quat[3] = transform.getRotation().w();
}

/* Return the Euler angles with the Quaternion */
void platform_controller::quat_to_euler(double q[4], double r[3]) 
{
    r[0] = atan2(2 * (q[0] * q[1] + q[2] * q[3]),
                 1 - 2 * ((q[0] * q[0]) + (q[1] * q[1])));
    r[1] = asin(2 * (q[0] * q[2] - q[3] * q[1]));
    r[2] = atan2(2 * (q[0] * q[3] + q[1] * q[2]),
                 1 - 2 * ((q[2] * q[2]) + (q[3] * q[3])));
}

