#include <SafetyInterface.h>

uavSafetyInterface::uavSafetyInterface(ros::NodeHandle n, std::string in_cmd_topic, std::string out_cmd_topic, std::string laser_topic) : 
	n_()
{
	ROS_INFO("SafetyInterface!");
	printf("Subscribing to laser topic %s...", laser_topic.c_str());
	laser_sub_ = n_.subscribe(laser_topic, 1, &uavSafetyInterface::scanCallback, this);
	printf("done!\n");
	printf("Subscribing to cmd_in topic %s...", in_cmd_topic.c_str());
	cmd_reader_ = n_.subscribe(in_cmd_topic, 10, &uavSafetyInterface::cmdReceived,this);
	printf("done!\n");
	printf("Advertising cmd_out topic %s...", out_cmd_topic.c_str());
	cmd_writer_ = n_.advertise<uav_msgs::ControllerCommand>(out_cmd_topic, 1);
	printf("done!\n");
	
	marker_publisher_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 500);
	
	laser_data_received = false;
}

void uavSafetyInterface::scanCallback (const sensor_msgs::LaserScan &scan_in)
{
	if(!laser_mutex_.try_lock()) {
		ROS_WARN("uavSafetyInterface::scanCallback could not get mutex lock! skipping this laser scan!");
		return;
	}
	laser_data_received = false;
	double ang_range = scan_in.angle_max - scan_in.angle_min;
	#ifdef UAVSAFETY_VERBOSE
	ROS_INFO("received point cloud in %s frame", scan_in.header.frame_id.c_str());
	ROS_INFO("samples: %d", (int)scan_in.ranges.size());
	ROS_INFO("angle range: (%.3f, %.3f) :: %.3f (%.3f degrees)", scan_in.angle_min, scan_in.angle_max, ang_range, ang_range * 57.2957795); //57.2957795 degrees per radian
	#endif
	min_range = 10.0;
	min_angle = 0;
	if(las_ranges.size() < scan_in.ranges.size()) las_ranges.resize(scan_in.ranges.size());
	if(las_angles.size() < scan_in.ranges.size()) las_angles.resize(scan_in.ranges.size());
	for(int i = 0; i < (int)scan_in.ranges.size(); i++){
		double dist = scan_in.ranges[i];
		if(dist < 0.50) dist = 10.0;
		double angle = scan_in.angle_min + ang_range * i / (double)scan_in.ranges.size();
		//store angle and distance info
		las_ranges[i] = dist;
		las_angles[i] = angle;
		//also store min distance and corresponding angle for quick access
		if(dist < min_range){
			min_range = dist;
			min_angle = angle;
		}
	}
	#ifdef UAVSAFETY_VERBOSE
	ROS_INFO("nearest point at distance (%.3f) at angle (%.3f)", min_range, min_angle);
	ROS_INFO("==============");
	#endif
	laser_data_received = true;
	laser_mutex_.unlock();
}

void uavSafetyInterface::cmdReceived(const uav_msgs::ControllerCommand &cmd_in){
	visualizeCmd(cmd_in, "cmd_in", 240);
	uav_msgs::ControllerCommand cmd_out;
	cmd_out.header.stamp = ros::Time::now();
	filterCommand(cmd_in, &cmd_out);
	visualizeCmd(cmd_out, "cmd_out", 30);
	
	//enforce max/min here!!!
	cmd_out.roll = max(-1.0, min(1.0, cmd_out.roll));
	cmd_out.pitch = max(-1.0, min(1.0, cmd_out.pitch));
	
	cmd_writer_.publish(cmd_out);
	ROS_INFO("command received! in RPYT [%.3f, %.3f, %.3f, %.3f] out RPYT [%.3f, %.3f, %.3f, %.3f]", cmd_in.roll, cmd_in.pitch, cmd_in.yaw, cmd_in.thrust, cmd_out.roll, cmd_out.pitch, cmd_out.yaw, cmd_out.thrust);
}

double uavSafetyInterface::estimate_heading(double pitch, double roll){
	//A = <r, p>
	//B = <1, 0>
	//A . B = p
	//|A| = sqrtf(r*r + p*p)
	//|B| = 1
	//theta = acos(A . B / |A|*|B|)
	if(pitch < 0.001 && roll < 0.001) return 0.0;
	double theta = acos(pitch / sqrtf(pitch*pitch+roll*roll));
	if(roll < 0.0){
		theta = 2.0 * 3.14159265359 - theta;
	}
	return theta;
}

double uavSafetyInterface::angle_normalize(double th1){
	if(th1 >= 2.0 * 3.14159265359){
		do {
			//keep subtracting two PI until you get to th < two PI
			th1 -= 2.0 * 3.14159265359;
		} while(th1 >= 2.0 * 3.14159265359);
		return th1;
	}
	if(th1 < 0){
		do {
			//keep adding two PI until you get to th > 0
			th1 += 2.0 * 3.14159265359;
		} while(th1 < 0);
		return th1;
	}
	return th1;
}

double uavSafetyInterface::angle_diff(double th1, double th2){
	double t1 = angle_normalize(th1);
	double t2 = angle_normalize(th2);
	double diff = t1 - t2;
	if(diff < 0) diff = 2.0 * 3.14159265359 - diff;
	return angle_normalize(diff);
}

//useful info: 
//pitch+ : nose down
//pitch- : nose up
//roll+ : right wing down
//roll- : left wing down
bool uavSafetyInterface::filterCommand(const uav_msgs::ControllerCommand &cmd_in, uav_msgs::ControllerCommand *cmd_out){

	double pitch_gain = 5.0;
	double roll_gain = 5.0;

	if(laser_data_received){
		std::vector<double> weights;
		cmd_out->thrust = cmd_in.thrust;
		cmd_out->yaw = cmd_in.yaw;
		double heading_est = estimate_heading(cmd_in.pitch, -cmd_in.roll); //estimate heading direction based on roll and pitch of cmd_in
		visualizeHeading(heading_est, "heading_est", 60);
		double pitch = 0;
		double roll = 0;
		for(unsigned int i = 0; i < las_ranges.size(); i++){
			double dist_coeff = max(0.0, 10.0 / (las_ranges[i]) - 5.0);
			double pitch_component = cos(las_angles[i]);
			double roll_component = sin(las_angles[i]);
			double angl_dif = fabs(angle_diff(heading_est, las_angles[i]));
			double angle_coeff = cos(angl_dif);
			if(angl_dif > 0.5*3.14159265359 && angl_dif < 1.5 * 3.14159265359) angle_coeff = 0.0;
			pitch -= angle_coeff*dist_coeff * pitch_component;
			roll -= angle_coeff*dist_coeff * roll_component;
			//weights.push_back(120.0 - dist_coeff*8.0);
			weights.push_back((120.0 - 8.0*angle_coeff*dist_coeff));
		}
		cmd_out->pitch = cmd_in.pitch + pitch_gain * pitch / (double) las_ranges.size();
		cmd_out->roll = -(-cmd_in.roll + roll_gain * roll / (double) las_ranges.size());
		visualizeRanges(las_ranges, las_angles, "weights", weights);
		return true;
	} else {
		ROS_WARN("Laser data not fully received!");
		cmd_out->thrust = cmd_in.thrust;
		cmd_out->yaw = cmd_in.yaw;
		cmd_out->pitch = 0.0;
		cmd_out->roll = 0.0;
		return false;
	}
}

int main(int argc, char** argv)
{
  printf("Initializing safety interface...");
  ros::init(argc,argv,"uav_safety_interface");
  ros::NodeHandle nh;
 // uavTestController* tctrl = new uavTestController(nh, "/cmd_in");
  std::string high_ctrl_cmd_topic, ctrl_cmd_topic, laser_topic;
  nh.param<std::string>("high_level_controller_cmd",high_ctrl_cmd_topic,"/high_level_controller_cmd");
  nh.param<std::string>("controller_cmd",ctrl_cmd_topic,"/controller_cmd");
  nh.param<std::string>("fixe_laser_topic",laser_topic,"/fixed_laser");
  

  uavSafetyInterface* safety = new uavSafetyInterface(nh, high_ctrl_cmd_topic, ctrl_cmd_topic, laser_topic);
  //tctrl->run();
  ros::spin();
  return 0;
}


