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
	get_params();
	create_subscribers();
	ros::spin();
}

/* Initialize values in 0 */
void pose_filter::init()
{
	filtered_pos_.setValue(0.0, 0.0, 0.0);
	filtered_quat_.setRotation(btVector3(0.0, 0.0, 0.0), 1.0);
}

/* Get the parameters */
void pose_filter::get_params()
{
	ros::NodeHandle n_param("~");

	n_param.getParam("filter_ratio", filter_ratio_);
	n_param.getParam("frame_id", frame_id_);
	n_param.getParam("child_frame_id", child_frame_id_);
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
    if(msg->transforms[0].child_frame_id == child_frame_id_ && 
		msg->transforms[0].header.frame_id == frame_id_) {
		filter(msg);
		publish_filtered();
    }
}

/* Filter the position, with the past ones */
void pose_filter::filter(tf::tfMessageConstPtr msg)
{
	static tf::TransformBroadcaster br;

	btVector3 current_pos(msg->transforms[0].transform.translation.x,
						  msg->transforms[0].transform.translation.y,	
						  msg->transforms[0].transform.translation.z);	

	btQuaternion current_rot(msg->transforms[0].transform.rotation.x,
							 msg->transforms[0].transform.rotation.y,
							 msg->transforms[0].transform.rotation.z,
							 msg->transforms[0].transform.rotation.w);

	/* Filter postion and rotation */
	filtered_pos_.lerp(current_pos, filter_ratio_);
	filtered_quat_.slerp(current_rot, filter_ratio_);
}	

/* Publish the filtered pose to tf */
void pose_filter::publish_filtered()
{
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(filtered_pos_);
	transform.setRotation(filtered_quat_);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
									frame_id_, child_frame_id_ + "_filtered"));
}
