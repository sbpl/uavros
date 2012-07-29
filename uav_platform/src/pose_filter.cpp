#include <uav_platform/pose_filter.h>

/* Creates a node handle and a pose filter */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "pose_filter");
	ros::NodeHandle n;
	pose_filter pose_filter_(n);
	ros::spin();
	return 0;
}

/* Main loop that looks for transform desired */
void pose_filter::main_loop()
{
	tf::TransformListener listener;

	ros::Rate rate(10.0);
	while(n_.ok()) {
		tf::StampedTransform transform;
		try {
			listener.waitForTransform(frame_id_, child_frame_id_, ros::Time(),
									 ros::Duration(0.5));
			listener.lookupTransform(frame_id_, child_frame_id_, ros::Time(), 
									 transform);
		} catch (tf::TransformException ex) {
			ROS_DEBUG("%s", ex.what());
			tf_found_ = false;
		}

		if(tf_found_) {
			filter(transform);
			publish_filtered();
		} else {
			tf_found_ = true;
		}

		rate.sleep();
	}
}			

/* Constructor */
pose_filter::pose_filter(ros::NodeHandle &n): n_ (n)
{
	init();
	get_params();
//	create_subscribers();
	main_loop();
}

/* Initialize values in 0 */
void pose_filter::init()
{
	tf_found_ = true;

	filtered_pos_.setValue(0.0, 0.0, 0.0);
	btVector3 temp(0.001,0.001,0.001);
	double x, y, z, w;
	x = temp.getX();
	y = temp.getY();
	z = temp.getZ();
	btScalar sca = 1.0;
	filtered_quat_.setRotation(temp, sca);
	btVector3 vec = filtered_quat_.getAxis();
	x = vec.getX();
	y = vec.getY();
	z = vec.getZ();
	w = filtered_quat_.getW();
}

/* Get the parameters */
void pose_filter::get_params()
{
	ros::NodeHandle n_param("~");

	if(!n_param.getParam("filter_ratio", filter_ratio_)) {
		filter_ratio_ = 0.5;
	} 
	ROS_INFO("Filter ratio: %f", filter_ratio_);

	if(!n_param.getParam("frame_id", frame_id_)) {
		frame_id_ = "/marker";
	}
	ROS_INFO("Frame id: %s", frame_id_.c_str());

	if(!n_param.getParam("child_frame_id", child_frame_id_)) {
		child_frame_id_ = "/usb_cam";
	}
	ROS_INFO("Child frame id: %s", child_frame_id_.c_str());

	if(!n_param.getParam("filtered_frame", filtered_frame_)) {
		filtered_frame_ = frame_id_; 
	}
	ROS_INFO("Filtered frame id: %s", filtered_frame_.c_str());

	if(!n_param.getParam("filtered_child_frame", filtered_child_frame_)) {
		filtered_child_frame_ = "/marker_filtered";
	}
	ROS_INFO("Filtered child frame id: %s", filtered_child_frame_.c_str());
}

/* Create the subscribers to the the corresponding topcis */
void pose_filter::create_subscribers()
{
	tf_sub_ = n_.subscribe("/tf", 1000, &pose_filter::tf_callback, this);
}

/* Callback when a tf message is received */
void pose_filter::tf_callback(tf::tfMessageConstPtr msg)
{

    /* Check wether corresponds to the frame and child specified */
//    if(msg->transforms[0].child_frame_id == child_frame_id_ && 
//		msg->transforms[0].header.frame_id == frame_id_) {
    if(msg->transforms[0].child_frame_id == child_frame_id_ && 
		msg->transforms[0].header.frame_id == frame_id_) {
		filter(msg);
		publish_filtered();
    }
}

void pose_filter::filter(tf::StampedTransform transform) 
{
	static tf::TransformBroadcaster br;

	btVector3 current_pos(transform.getOrigin().x(),
						  transform.getOrigin().y(),
						  transform.getOrigin().z());

	btVector3 current_rot_vec(transform.getRotation().x(),
							  transform.getRotation().y(),
							  transform.getRotation().z());

	btScalar current_rot_scale(transform.getRotation().w());
	
	btQuaternion current_rot(current_rot_vec, current_rot_scale);

	/* Filter postion and rotation */
	filtered_pos_ = filtered_pos_.lerp(current_pos, filter_ratio_);
	filtered_quat_ = filtered_quat_.slerp(current_rot, filter_ratio_);

	double x,y,z,w;
	x = filtered_pos_.getX();
	y = filtered_pos_.getY();
	z = filtered_pos_.getZ();
	ROS_INFO("Pos:%f,%f,%f",x,y,z);
	btVector3 vec = filtered_quat_.getAxis();
	x = vec.getX();
	y = vec.getY();
	z = vec.getZ();
	w = filtered_quat_.getW();
	ROS_INFO("Rot:%f,%f,%f,%f",x,y,z,w);
}	
	

/* Filter the position, with the past ones */
void pose_filter::filter(tf::tfMessageConstPtr msg)
{
	static tf::TransformBroadcaster br;

	btVector3 current_pos(msg->transforms[0].transform.translation.x,
						  msg->transforms[0].transform.translation.y,	
						  msg->transforms[0].transform.translation.z);	

	btVector3 current_rot_vec(msg->transforms[0].transform.rotation.x,
							 msg->transforms[0].transform.rotation.y,
							 msg->transforms[0].transform.rotation.z);

	btScalar current_rot_scale(msg->transforms[0].transform.rotation.w);

	btQuaternion current_rot(current_rot_vec, current_rot_scale);

	/* Filter postion and rotation */
	filtered_pos_ = filtered_pos_.lerp(current_pos, filter_ratio_);
	filtered_quat_ = filtered_quat_.slerp(current_rot, filter_ratio_);
	double x, y, z, w;
	btVector3 vec = filtered_quat_.getAxis();
	x = vec.getX();
	y = vec.getY();
	z = vec.getZ();
	ROS_INFO("%f,%f,%f",x,y,z);
}	

/* Publish the filtered pose to tf */
void pose_filter::publish_filtered()
{
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(filtered_pos_);
	transform.setRotation(filtered_quat_);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
									filtered_frame_, filtered_child_frame_));
}
