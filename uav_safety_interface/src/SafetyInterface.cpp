#include <SafetyInterface.h>

uavSafetyInterface::uavSafetyInterface(ros::NodeHandle n, std::string in_cmd_topic, std::string out_cmd_topic, std::string laser_topic, std::string imu_topic) : 
	n_()
{
	printf("\tSubscribing to laser topic %s...", laser_topic.c_str());
	laser_sub_ = n_.subscribe(laser_topic, 1, &uavSafetyInterface::scanCallback, this);
	printf("done!\n");
	printf("\tSubscribing to cmd_in topic %s...", in_cmd_topic.c_str());
	cmd_reader_ = n_.subscribe(in_cmd_topic, 10, &uavSafetyInterface::cmdReceived,this);
	printf("done!\n");
	printf("\tSubscribing to imu topic %s...", imu_topic.c_str());
	imu_sub_ = n_.subscribe(imu_topic, 1, &uavSafetyInterface::imuCallback, this);
	printf("done!\n");
	printf("\tAdvertising cmd_out topic %s...", out_cmd_topic.c_str());
	cmd_writer_ = n_.advertise<uav_msgs::ControllerCommand>(out_cmd_topic, 1);
	printf("done!\n");
	#ifdef UAVSAFETY_VISUALS
	marker_publisher_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 500);
	#endif
	imu_linear_acc.resize(3,0.0);
	imu_linear_acc[0] = 0.0;
	imu_linear_acc[1] = 0.0;
	imu_linear_acc[2] = -9.8;
	roll_gain = 5.0;
	pitch_gain = 5.0;
	laser_data_received = false;
	imu_data_received = false;
}

void uavSafetyInterface::imuCallback( const sensor_msgs::Imu &imu_in)
{
	if(!imu_mutex_.try_lock()){
		ROS_WARN("uavSafetyInterface::imuCallback could not get mutex lock! skipping");
		return;
	}
	imu_linear_acc[0] = imu_in.linear_acceleration.x;
	imu_linear_acc[1] = imu_in.linear_acceleration.y;
	imu_linear_acc[2] = imu_in.linear_acceleration.z;
	
	roll_estimate  = atan2(imu_linear_acc[1], imu_linear_acc[2]);
	pitch_estimate = atan2(imu_linear_acc[0], imu_linear_acc[2]);
	
	uav_msgs::ControllerCommand cmd;
	cmd.thrust = 0.0;
	cmd.yaw = 0.0;
	cmd.roll = roll_estimate;
	cmd.pitch = pitch_estimate;
	visualizeCmd(cmd, "imu roll pitch estimate", 330);
	visualizeVector(imu_linear_acc, "imu linear acc", 0); 
	imu_data_received = true;
	imu_mutex_.unlock();
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
	#ifdef UAVSAFETY_VISUALS
	visualizeCmd(cmd_in, "cmd_in", 240);
	#endif
	uav_msgs::ControllerCommand cmd_out;
	cmd_out.header.stamp = ros::Time::now();
	filterCommand(cmd_in, &cmd_out);
	#ifdef UAVSAFETY_VISUALS
	visualizeCmd(cmd_out, "cmd_out", 30);
	#endif
	
	//enforce max/min here!!!
	cmd_out.roll = max(roll_min, min(roll_max, cmd_out.roll));
	cmd_out.pitch = max(pitch_min, min(pitch_max, cmd_out.pitch));
	
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
	if(laser_data_received){
		std::vector<double> weights;
		cmd_out->thrust = cmd_in.thrust;
		cmd_out->yaw = cmd_in.yaw;
		double heading_est = estimate_heading(cmd_in.pitch, -cmd_in.roll); //estimate heading direction based on roll and pitch of cmd_in
		#ifdef UAVSAFETY_VISUALS
		visualizeHeading(heading_est, "heading_est", 60);
		#endif
		double pitch = 0;
		double roll = 0;
		double max_coeff = 0;
		double max_hdg = 0;
		for(unsigned int i = 0; i < las_ranges.size(); i++){
			double dist_coeff = max(0.0, 10.0 / (las_ranges[i]) - 5.0);
			double pitch_component = cos(las_angles[i]);
			double roll_component = sin(las_angles[i]);
			double angl_dif = fabs(angle_diff(heading_est, las_angles[i]));
			double angle_coeff = cos(angl_dif);
			if(angl_dif > 0.5*3.14159265359 && angl_dif < 1.5 * 3.14159265359) angle_coeff = 0.0;
			
			#ifdef UAV_SAFETY_USEALLPOINTS			
			pitch -= angle_coeff*dist_coeff * pitch_component / (double) las_ranges.size();
			roll -= angle_coeff*dist_coeff * roll_component / (double) las_ranges.size();
			#else
			if(max_coeff < angle_coeff*dist_coeff){
				max_hdg = las_angles[i];
				max_coeff = angle_coeff*dist_coeff;
				pitch = angle_coeff*dist_coeff * pitch_component;
				roll = angle_coeff*dist_coeff * pitch_component;
			}
			#endif
			//weights.push_back(120.0 - dist_coeff*8.0);
			weights.push_back((120.0 - 8.0*angle_coeff*dist_coeff));
		}
		cmd_out->pitch = cmd_in.pitch + pitch_gain * pitch;
		cmd_out->roll = -(-cmd_in.roll + roll_gain * roll);
		#ifdef UAVSAFETY_VISUALS
		visualizeHeading(max_hdg, "danger", 0);
		visualizeRanges(las_ranges, las_angles, "weights", weights);
		#endif
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
  printf("Initializing safety interface...\n");
  ros::init(argc,argv,"uav_safety_interface");
  ros::NodeHandle nh;
  std::string high_ctrl_cmd_topic, ctrl_cmd_topic, laser_topic, imu_topic;
  double roll_gain, pitch_gain;
  nh.param<std::string>("high_level_controller_cmd",high_ctrl_cmd_topic,"/high_level_controller_cmd");
  nh.param<std::string>("controller_cmd",ctrl_cmd_topic,"/controller_cmd");
  nh.param<std::string>("fixed_laser_topic",laser_topic,"/fixed_laser");
  nh.param<std::string>("imu_data", imu_topic, "/imu_data");
  if(!nh.getParam("safety_roll_gain", roll_gain)){
  	roll_gain = 5.0;
  }
  if(!nh.getParam("safety_pitch_gain", pitch_gain)){
  	pitch_gain = 5.0;
  }
  printf("\tRoll Gain: %.3f\n", roll_gain);
  printf("\tPitch Gain: %.3f\n", pitch_gain);
  uavSafetyInterface* safety = new uavSafetyInterface(nh, high_ctrl_cmd_topic, ctrl_cmd_topic, laser_topic, imu_topic);
  safety->setRollGain(roll_gain);
  safety->setPitchGain(pitch_gain);
  printf("Safety interface running!\n");
  ros::spin();
  return 0;
}


