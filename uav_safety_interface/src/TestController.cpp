#include <SafetyInterface.h>

class uavTestController {
public:
	ros::NodeHandle nh_;
	std::string cmd_topic_;
	ros::Publisher cmd_writer_;
	
	inline uavTestController(ros::NodeHandle n, std::string cmd_topic) : 
		nh_(n),
		cmd_topic_(cmd_topic) {
		printf("Advertising controller topic %s...", cmd_topic_.c_str());
		cmd_writer_ = nh_.advertise<uav_msgs::ControllerCommand>(cmd_topic_, 1);
		printf("done!\n");
	}
	
	inline double unirand(double min, double max){
		double zeroone = ((double) rand()) / ((double) RAND_MAX); //random between 0 and 1
		return min + zeroone*(max - min);
	}

	inline void run(){
		uav_msgs::ControllerCommand cmd;
		cmd.thrust = 0;
		cmd.yaw = 0;
		cmd.pitch = 0;
		cmd.roll = 0;
		while(ros::ok()){
			printf("."); fflush(stdout);
			cmd.header.seq = 0;
			cmd.header.stamp = ros::Time::now();
			cmd.pitch = unirand(-10.0, 10.0);
			cmd.roll = unirand(-10.0, 10.0);
			cmd_writer_.publish(cmd);
			ros::Rate(1.0).sleep();
		}
	}
};

int main(int argc, char** argv)
{
  printf("Initializing test controller...");
  ros::init(argc,argv,"uav_test_controller");
  ros::NodeHandle nh;
  uavTestController* tctrl = new uavTestController(nh, "/high_level_controller_cmd");
  tctrl->run();
  return 0;
}


