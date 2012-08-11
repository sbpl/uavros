#include <uav_platform/nav_markers.h>

/* Creates a node handle and a platform controller */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "navigate");
	ros::NodeHandle n;
	nav_markers ptf_ctrl(n);
	ros::spin();
	return 0;
}

/* Constructor */
nav_markers::nav_markers(ros::NodeHandle &n): n_ (n)
{
	get_params();
	create_publishers();
	create_subscribers();
	goal_.x = 0;
	goal_.y = 0;
	goal_.z = 0;
	goal_.theta = 0;
	old_goal_ts_ = ros::Time::now();
	ros::spin();
}

/* Destructor */
nav_markers::~nav_markers()
{

}

/* Get the parameters, if none given, assume defaults */
void nav_markers::get_params()
{
	ros::NodeHandle n_param("~");
	
	n_param.getParam("m1_a_x", marker_goals_[0].approach[0]);
	n_param.getParam("m1_a_y", marker_goals_[0].approach[1]);
	n_param.getParam("m1_a_z", marker_goals_[0].approach[2]);
	n_param.getParam("m1_l_x", marker_goals_[0].land[0]);
	n_param.getParam("m1_l_y", marker_goals_[0].land[1]);
	n_param.getParam("m1_l_z", marker_goals_[0].land[2]);
	n_param.getParam("m1_height", marker_goals_[0].height);
	
	n_param.getParam("m2_a_x", marker_goals_[1].approach[0]);
	n_param.getParam("m2_a_y", marker_goals_[1].approach[1]);
	n_param.getParam("m2_a_z", marker_goals_[1].approach[2]);
	n_param.getParam("m2_l_x", marker_goals_[1].land[0]);
	n_param.getParam("m2_l_y", marker_goals_[1].land[1]);
	n_param.getParam("m2_l_z", marker_goals_[1].land[2]);
	n_param.getParam("m2_height", marker_goals_[1].height);
	
	n_param.getParam("m3_a_x", marker_goals_[2].approach[0]);
	n_param.getParam("m3_a_y", marker_goals_[2].approach[1]);
	n_param.getParam("m3_a_z", marker_goals_[2].approach[2]);
	n_param.getParam("m3_l_x", marker_goals_[2].land[0]);
	n_param.getParam("m3_l_y", marker_goals_[2].land[1]);
	n_param.getParam("m3_l_z", marker_goals_[2].land[2]);
	n_param.getParam("m3_height", marker_goals_[2].height);
	
	n_param.getParam("m4_a_x", marker_goals_[3].approach[0]);
	n_param.getParam("m4_a_y", marker_goals_[3].approach[1]);
	n_param.getParam("m4_a_z", marker_goals_[3].approach[2]);
	n_param.getParam("m4_l_x", marker_goals_[3].land[0]);
	n_param.getParam("m4_l_y", marker_goals_[3].land[1]);
	n_param.getParam("m4_l_z", marker_goals_[3].land[2]);
	n_param.getParam("m4_height", marker_goals_[3].height);
}

/* Create publishers to advertis in the corresponding topics */
void nav_markers::create_publishers()
{
	goal_pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>("/goal_pose",1);
}

/* Create the subscribers to the corresponding topics */
void nav_markers::create_subscribers()
{
	tf_sub_ = n_.subscribe("/tf", 1000, &nav_markers::transform_callback, this);
	mode_sub_ = n_.subscribe("/align_mode", 1, 
							 &nav_markers::mode_callback, this);
}

/* Callback when the track mode is changed */
void nav_markers::mode_callback(uav_msgs::mode_msg msg)
{
	align_marker_ = msg.marker;
	align_mode_ = msg.mode;
}

/* Function call every time tf send a message */
void nav_markers::transform_callback(tf::tfMessageConstPtr msg)
{

    /* Check wether we are seeing a marker and take action  *
     * depending on the last track mode send                */
    if(msg->transforms[0].child_frame_id == "/m1" && align_marker_ == 1) {
		if(align_mode_ == APPROACH_POINT) {
			align(msg, marker_goals_[0].approach);
		} else if (align_mode_ == LAND_POINT) {
			align(msg, marker_goals_[0].land);
		}
    } else if(msg->transforms[0].child_frame_id == "/m2" 
			  && align_marker_ == 2) {
		if(align_mode_ == APPROACH_POINT) {
			align(msg, marker_goals_[1].approach);
		} else if (align_mode_ == LAND_POINT) {
			align(msg, marker_goals_[1].land);
		}
    } else if(msg->transforms[0].child_frame_id == "/m3" 
			  && align_marker_ == 3) {
		if(align_mode_ == APPROACH_POINT) {
			align(msg, marker_goals_[2].approach);
		} else if (align_mode_ == LAND_POINT) {
			align(msg, marker_goals_[2].land);
		}
    } else if(msg->transforms[0].child_frame_id == "/m4" 
			  && align_marker_ == 4) {
		if(align_mode_ == APPROACH_POINT) {
			align(msg, marker_goals_[3].approach);
		} else if (align_mode_ == LAND_POINT) {
			align(msg, marker_goals_[3].land);
		}
	}
}

/* Calculate goal to go to desired position */
void nav_markers::align(tf::tfMessageConstPtr msg, double goal_from_marker[3])
{
	btVector3 pos;
	btQuaternion quat;
	goal calculated_goal;
	double r, p, y;
	tf::StampedTransform transform;

	get_pose_from_msg(msg, pos, quat);
	tf::quaternionMsgToTF( msg->transforms[0].transform.rotation, quat);
	btMatrix3x3(quat).getRPY(r, p, y);

	calculated_goal.x = pos.getX() - goal_from_marker[0] * cos(p);
	calculated_goal.y = pos.getY() + goal_from_marker[0] * sin(p);
	calculated_goal.z = pos.getZ();//goal_from_marker[2];
	calculated_goal.theta = -p;

	update_goal(calculated_goal);
}

/* Land on marker */
void nav_markers::land() 
{
    ROS_INFO("Need to land");
}

/* When timer is done, for aligning this callback is called */
void nav_markers::align_done_callback(const ros::TimerEvent&)
{

}

/* Update the goal in the class */
void nav_markers::update_goal(goal goal_calculated)
{
//	goal_.x = goal_.x * OLD_GOAL_RATIO + x * NEW_GOAL_RATIO;
//	goal_.y = goal_.y * OLD_GOAL_RATIO + y * NEW_GOAL_RATIO;
//	goal_.z = goal_.z * OLD_GOAL_RATIO + z * NEW_GOAL_RATIO;
//	goal_.theta = goal_.theta * OLD_GOAL_RATIO + theta * NEW_GOAL_RATIO;
//
	goal_.x = goal_calculated.x;
	goal_.y = goal_calculated.y;
	goal_.z = goal_calculated.z;
	goal_.theta = goal_calculated.theta;

	if(ros::Time::now() - old_goal_ts_ > GOAL_FREQUENCY) {
	
		/* Publish the goal */
		publish_goal(goal_.x, goal_.y, goal_.z, goal_.theta);

		old_goal_ts_ = ros::Time::now();
	} 
}

/* Publish the goal */
void nav_markers::publish_goal(double x, double y, double z, 
									  double theta)
{
    goal_pose_.header.stamp = ros::Time::now();
    goal_pose_.header.frame_id = "/map";
    goal_pose_.pose.position.x = x;
    goal_pose_.pose.position.y = y;
    goal_pose_.pose.position.z = z;
    goal_pose_.pose.orientation.x = 0;
    goal_pose_.pose.orientation.y = 0;
    goal_pose_.pose.orientation.z = sin(theta/2);
    goal_pose_.pose.orientation.w = cos(theta/2);
    goal_pose_pub_.publish(goal_pose_);
}

/* Check wether it has been near the goal for the stablished time */
void nav_markers::check_time()
{
	if(check_in_range()) {
		if(!timer_.isValid()) {
			timer_ = n_.createTimer(IN_RANGE_TIME, 
							&nav_markers::align_done_callback, this);
			timer_.start();
				ROS_INFO("Start timer");
		} else {
			if(ros::Time::now() - last_marker_ts_ > MARKER_FREQUENCY) {
				ROS_INFO("Delay between marker frames is high");
				timer_.stop();
			} else {
				timer_.start();
			}
		}
		last_marker_ts_ = ros::Time::now();		
	} else {
		timer_.stop();
		ROS_INFO("Stoping timer");
	}
}

/* Check wether the transform in the message is in certain range */
bool nav_markers::check_in_range()
{
	tf::StampedTransform current_pose;
	get_transform("/map", "/body_frame", current_pose);
	if(fabs(current_pose.getOrigin().x() - goal_pose_.pose.position.x) 
			< IN_RANGE_DIST) {
		if(fabs(current_pose.getOrigin().y() - goal_pose_.pose.position.y) 
				< IN_RANGE_DIST) {
			if(fabs(current_pose.getOrigin().z() - goal_pose_.pose.position.z) 
					< IN_RANGE_DIST) {
				if(fabs(current_pose.getRotation().z() - 
					goal_pose_.pose.orientation.z) < IN_RANGE_RAD) {
					ROS_WARN("In range");
					return true;
				}
			}
		}
	}
	ROS_WARN("Not in range from goal");
	return false;
}

/* Get transform between the two given frames */
bool nav_markers::get_transform(std::string parent, std::string child,
					                    tf::StampedTransform &transform)
{
    tf::TransformListener listener;
    try {
        listener.waitForTransform(parent, child, ros::Time(), 
								  ros::Duration(0.5));
        listener.lookupTransform(parent, child, ros::Time(), transform);
    } catch (tf::TransformException ex) {
		return false;
        ROS_ERROR("%s",ex.what());
    }
	return true;
}

/* Get pose from a tf message */
void nav_markers::get_pose_from_msg(tf::tfMessageConstPtr msg, btVector3 &p,
									btQuaternion &q)
{
	p.setValue(msg->transforms[0].transform.translation.x,
	  msg->transforms[0].transform.translation.y,
	  msg->transforms[0].transform.translation.z);

	btVector3 temp(msg->transforms[0].transform.rotation.x,
				   msg->transforms[0].transform.rotation.y,
				   msg->transforms[0].transform.rotation.z);
	btScalar scalar = msg->transforms[0].transform.rotation.w;
	q.setRotation(temp, scalar);
}

/* Get position translation and rotation of the transform */
void nav_markers::get_pose_from_tf(double pos[3], double quat[4], 
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
void nav_markers::quat_to_euler(double q[4], double r[3]) 
{
    r[0] = atan2(2 * (q[0] * q[1] + q[2] * q[3]),
                 1 - 2 * ((q[0] * q[0]) + (q[1] * q[1])));
    r[1] = asin(2 * (q[0] * q[2] - q[3] * q[1]));
    r[2] = atan2(2 * (q[0] * q[3] + q[1] * q[2]),
                 1 - 2 * ((q[2] * q[2]) + (q[3] * q[3])));
}

