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

/* Constructor */
pose_filter::pose_filter(ros::NodeHandle &n): n_ (n)
{
	init();
	get_params();
	main_loop();
}

/* Initialize values in 0 */
void pose_filter::init()
{
	filtered_pos_.setValue(0.0, 0.0, 0.0);
	tf::Vector3 temp(0.001,0.001,0.001);
	double sca = 1.0;
	filtered_quat_.setRotation(temp, sca);
}

/* Get the parameters */
void pose_filter::get_params()
{
	ros::NodeHandle n_param("~");

	if(!n_param.getParam("filter_ratio", filter_ratio_)) {
		filter_ratio_ = 0.5;
	}
	ROS_DEBUG("Filter ratio: %f", filter_ratio_);

	if(!n_param.getParam("frame_id", frame_id_)) {
		frame_id_ = "/map";
	}
	ROS_DEBUG("Frame id: %s", frame_id_.c_str());

	if(!n_param.getParam("child_frame_id", child_frame_id_)) {
		child_frame_id_ = "/marker";
	}
	ROS_DEBUG("Child frame id: %s", child_frame_id_.c_str());

	if(!n_param.getParam("filtered_frame", filtered_frame_)) {
		filtered_frame_ = frame_id_;
	}
	ROS_DEBUG("Filtered frame id: %s", filtered_frame_.c_str());

	if(!n_param.getParam("filtered_child_frame", filtered_child_frame_)) {
		filtered_child_frame_ = "/marker_filtered";
	}
	ROS_DEBUG("Filtered child frame id: %s", filtered_child_frame_.c_str());
}

/* Main loop that looks for transform desired */
void pose_filter::main_loop()
{
	tf::TransformListener listener;
	bool tf_available;
	last_ts_ = ros::Time::now();
	ros::Rate rate(50.0);
	while(n_.ok()) {
		tf_available = true;

		tf::StampedTransform transform, transform2;
		try {
			listener.lookupTransform(frame_id_, child_frame_id_, ros::Time(0),
									 transform2);
			ts_ = transform2.stamp_;
			listener.lookupTransform(frame_id_, child_frame_id_, ts_ - ros::Duration(0.10),
									 transform);
		} catch (tf::TransformException ex) {
			ROS_DEBUG("%s", ex.what());
			tf_available = false;
		}

		if(last_ts_ != transform.stamp_ && tf_available) {
			filter(transform);
			publish_filtered();
		}
		last_ts_ = transform.stamp_;

		rate.sleep();
	}
}

/* Filter the pose with the past poses */
void pose_filter::filter(tf::StampedTransform transform)
{
	static bool first_tf = true;
	tf::Vector3 current_pos(transform.getOrigin().x(),
						  transform.getOrigin().y(),
						  transform.getOrigin().z());

	tf::Quaternion current_rot(transform.getRotation().x(),
							 transform.getRotation().y(),
							 transform.getRotation().z(),
							 transform.getRotation().w());

	if(!first_tf) {
		/* Filter postion and rotation */
		filtered_pos_ = filtered_pos_.lerp(current_pos, filter_ratio_);
		filtered_quat_ = filtered_quat_.slerp(current_rot, filter_ratio_);
	} else {
		filtered_pos_ = current_pos;
		filtered_quat_ = current_rot;
		first_tf = false;
	}
}

/* Publish the filtered pose to tf */
void pose_filter::publish_filtered()
{
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(filtered_pos_);
	transform.setRotation(filtered_quat_);
//	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
	br.sendTransform(tf::StampedTransform(transform, ts_,
									filtered_frame_, filtered_child_frame_));
}
