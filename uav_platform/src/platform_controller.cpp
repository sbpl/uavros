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
	goal_.x = 0;
	goal_.y = 0;
	goal_.z = 0;
	goal_.theta = 0;
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
}

/* Create publishers to advertis in the corresponding topics */
void platform_controller::create_publishers()
{
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

/* Callback when the track mode is changed */
void platform_controller::mode_callback(uav_msgs::mode_msg msg)
{
	track_mode_ = msg.mode;
}

/* Callback called when aligned in front was in range for the stablished time */
void platform_controller::align_done_callback(const ros::TimerEvent&)
{
	track_mode_ = ALIGN_TOP;
}

/* Function call every time tf send a message */
void platform_controller::transform_callback(tf::tfMessageConstPtr msg)
{
    tf::StampedTransform transform;

    /* Check wether we are seeing a marker and take action  *
     * depending on the last track mode send                */
    if(msg->transforms[0].child_frame_id == "/marker1") {
        if(track_mode_ == ALIGN_FRONT) {
			align_front(msg);
        } else if(track_mode_ == ALIGN_TOP) {
			align_top(msg, FRONT_CAMERA, false);
        } else if(track_mode_ == ROTATE) {
			align_top(msg, FRONT_CAMERA, true);
        }
    } else if(msg->transforms[0].child_frame_id == "/marker2") {
        if(track_mode_ == ALIGN_TOP) {
		//	align_top(msg, BOTTOM_CAMERA, false);
        } else if(track_mode_ == ROTATE) {
	//		align_top(msg, FRONT_CAMERA, true);
        } else if(track_mode_ == LAND) {
            land(msg);
        }
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
	double theta, theta_sign;
	double interval_rad, marker_rad, goal_theta;
	double goal_x, goal_y, goal_z;
	int current_zone;
	tf::StampedTransform transform;

	/* Update struct pose_ */
	get_pose_from_msg(msg);

	/* Get the angle in map's z axis */
	theta = pose_.rot.y;

	ROS_INFO("Theta: %f", theta);

    /* Assume theta is positive */
    theta_sign = 1;

    /* If theta is negative, adjust */
    if(theta < 0) {
        theta_sign = -1;
    }

    /* Get the current zone depending on the angle range of each zone */
    interval_rad = INTERVALS_ANGLE * PI / 180;
    current_zone = floor(fabs(theta) / interval_rad);

    /* Get goal angle */
    marker_rad = MARKER_ANGLE * PI / 180;
    goal_theta = theta_sign * (current_zone) * interval_rad + marker_rad;

    /* Get goal position */
	goal_x = pose_.pos.z - DISTANCE_FROM_PLATFORM * cos(theta);
	goal_y = pose_.pos.x - DISTANCE_FROM_PLATFORM * sin(theta);
	goal_z = pose_.pos.y;

	/* Get transform from the camera to the map */
    get_transform("/map", "/usb_cam0", transform);
    get_pose_from_tf(pos, quat, transform);
    
	/* Update the goal considering the position of the camera */
	goal_x += pos[0];// - DISTANCE_FROM_PLATFORM;
    goal_y += pos[1];
	goal_z = pos[2] - goal_z + HOVER_ABOVE_PLATFORM;

	/* Update goal */
	update_goal(goal_x, goal_y, goal_z, theta);
}

/* Align on top of the marker */
void platform_controller::align_top(tf::tfMessageConstPtr msg, int camera,
									bool rotate)
{
    double pos[3], quat[4];
    double goal_theta;
    double goal_x, goal_y, goal_z;
	tf::StampedTransform transform;

	/* Update struct pose_ */
    get_pose_from_msg(msg);

	/* Update goal depending if the front or bottom camera	*
	 * detected the marker 									*/
    if(camera == FRONT_CAMERA) {
		goal_x= (pose_.pos.z + WIDTH_PLATFORM) * cos(pose_.rot.y);
        goal_y= pose_.pos.x - pose_.pos.x * sin(pose_.rot.y);
		goal_z= pose_.pos.y;
    	get_transform("/map", "/usb_cam0", transform);
    } else {
        goal_x= pose_.pos.x;
        goal_y= pose_.pos.y;
        goal_z= pose_.pos.z;
    	get_transform("/map", "/usb_cam1", transform);
    }

	/* Get the theta desired depending the state stablished */
	if(rotate) {
		goal_theta = pose_.rot.z + PI; 
	} else {
		goal_theta = pose_.rot.z;
	}

	/* Get transform from map to the camera */
    get_pose_from_tf(pos, quat, transform);

	/* Taking in count where is the camera, update the goal */
	goal_x += pos[0];
	goal_y += pos[1];
	goal_z = pos[2] - goal_z + HOVER_ABOVE_PLATFORM;

	/* Update Goal */
	update_goal(goal_x, goal_y, goal_z, goal_theta);
}

/* Land on marker */
void platform_controller::land(tf::tfMessageConstPtr msg) 
{
    ROS_INFO("Need to land");
}


/* Update the goal in the class */
void platform_controller::update_goal(double x, double y, double z, 
									  double theta)
{
	goal_.x = (goal_.x + x) / 2;
	goal_.y = (goal_.y + y) / 2;
	goal_.z = (goal_.z + z) / 2;
	goal_.theta = (goal_.theta + theta) / 2;

	/* Publish the goal */
	publish_goal(goal_.x, goal_.y, goal_.z, goal_.theta);
}

/* Publish the goal */
void platform_controller::publish_goal(double x, double y, double z, 
									  double theta)
{
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.stamp = ros::Time::now();
    goal_pose.header.frame_id = "/map";
    goal_pose.pose.position.x = x;
    goal_pose.pose.position.y = y;
    goal_pose.pose.position.z = z;
    goal_pose.pose.orientation.x = 0;
    goal_pose.pose.orientation.y = 0;
    goal_pose.pose.orientation.z = sin(theta/2);
    goal_pose.pose.orientation.w = cos(theta/2);
    goal_pose_pub_.publish(goal_pose);
}

/* Check wether it has been near the goal for the stablished time */
void platform_controller::check_time(tf::tfMessageConstPtr msg)
{
	if(timer_.isValid()) {
		if(!check_in_range(msg)) {

			/* If not in range reset timer */
			timer_.stop();
			timer_.start();
		}
	} else {
		if(check_in_range(msg)) {
			timer_ = n_.createTimer(IN_RANGE_TIME, 
							&platform_controller::align_done_callback, this);
		}
	}
}

/* Check wether the transform in the message is in certain range */
bool platform_controller::check_in_range(tf::tfMessageConstPtr msg)
{
	if(abs(msg->transforms[0].transform.translation.x) < IN_RANGE_DIST) {
		if(abs(msg->transforms[0].transform.translation.y) < IN_RANGE_DIST) {
			return true;
		}
	}
	return false;
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


void platform_controller::get_pose_from_msg(tf::tfMessageConstPtr msg)
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
void platform_controller::get_pose_from_tf(double pos[3], double quat[4], 
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

