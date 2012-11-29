#ifndef _uavSafetyInterface_H_
#define _uavSafetyInterface_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <boost/lexical_cast.hpp>
#include <boost/thread/mutex.hpp>
#include <uav_msgs/ControllerCommand.h>
#include <message_filters/subscriber.h>

#define UAVSAFETY_MIN_RANGE 2.0 //only consider points 2m or closer

#define UAVSAFETY_ENFORCE_LIMITS true
#define UAVSAFETY_ROLL_MIN -0.785398163
#define UAVSAFETY_ROLL_MAX 0.785398163
#define UAVSAFETY_PITCH_MIN -0.785398163
#define UAVSAFETY_PITCH_MAX 0.785398163

//#define UAVSAFETY_VERBOSE

double max(double a, double b){
	return a>b?a:b;
}

double min(double a, double b){
	return a<b?a:b;
}

void HSVtoRGB( double *r, double *g, double *b, double h, double s, double v )
{
	int i;
	double f, p, q, t;
	if( s == 0 ) {
		// achromatic (grey)
		*r = *g = *b = v;
		return;
	}
	h /= 60;        // sector 0 to 5
	i = floor(h);
	f = h - i;			// factorial part of h
	p = v * ( 1 - s );
	q = v * ( 1 - s * f );
	t = v * ( 1 - s * ( 1 - f ) );
	switch( i ) {
		case 0:
			*r = v;
			*g = t;
			*b = p;
			break;
		case 1:
			*r = q;
			*g = v;
			*b = p;
			break;
		case 2:
			*r = p;
			*g = v;
			*b = t;
			break;
		case 3:
			*r = p;
			*g = q;
			*b = v;
			break;
		case 4:
			*r = t;
			*g = p;
			*b = v;
			break;
    default:
			*r = v;
			*g = p;
			*b = q;
			break;
	}
}

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

class uavSafetyInterface {

public: 
	uavSafetyInterface(ros::NodeHandle n, std::string in_cmd_topic, std::string out_cmd_topic, std::string laser_topic);
	
	void scanCallback (const sensor_msgs::LaserScan &scan_in);
	
	void cmdReceived(const uav_msgs::ControllerCommand &cmd_in);
	
private:
	ros::NodeHandle n_;
	
	ros::Subscriber laser_sub_;
	ros::Subscriber cmd_reader_;
	ros::Publisher cmd_writer_;
	
	ros::Publisher marker_publisher_;
	
	std::vector<double> las_ranges;
	std::vector<double> las_angles;
	
	double min_range;
	double min_angle;
	
	bool laser_data_received;
	boost::mutex laser_mutex_;
	
	bool filterCommand(const uav_msgs::ControllerCommand &cmd_in, uav_msgs::ControllerCommand *cmd_out);
	
	double estimate_heading(double pitch, double roll);

	double angle_normalize(double th1);

	double angle_diff(double th1, double th2);
	
	inline void visualizeCmd(const uav_msgs::ControllerCommand &cmd, std::string ns, double color){
		visualization_msgs::Marker marker;
		double r=0,g=0,b=0;
		HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

		marker.header.stamp = ros::Time::now();
		marker.header.frame_id = "base_link"; //reference_frame_;
		marker.ns = ns;
		marker.id = 1;
		marker.type = visualization_msgs::Marker::LINE_LIST;
		marker.action = visualization_msgs::Marker::ADD;
		marker.points.resize(4);
		marker.points[0].x = 0.0;
		marker.points[0].y = 0.0;
		marker.points[0].z = 0.0;
		marker.points[1].x = 0.5*cmd.pitch;
		marker.points[1].y = 0.0;
		marker.points[1].z = 0.0;
		marker.points[2].x = 0.0;
		marker.points[2].y = 0.0;
		marker.points[2].z = 0.0;
		marker.points[3].x = 0.0;
		marker.points[3].y = 0.5*cmd.roll;
		marker.points[3].z = 0.0;
 		marker.scale.x = 0.05;
		marker.color.r = r;
		marker.color.g = g;
		marker.color.b = b;
		marker.color.a = 1.0;
		marker.lifetime = ros::Duration(180.0);
		marker_publisher_.publish(marker);
		marker_publisher_.publish(marker);
	}
	
	inline void visualizeHeading(double h, std::string ns, double color){
		visualization_msgs::Marker marker;
		double r=0,g=0,b=0;
		HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

		marker.header.stamp = ros::Time::now();
		marker.header.frame_id = "base_link"; //reference_frame_;
		marker.ns = ns;
		marker.id = 1;
		marker.type = visualization_msgs::Marker::LINE_LIST;
		marker.action = visualization_msgs::Marker::ADD;
		marker.points.resize(2);
		marker.points[0].x = 0.0;
		marker.points[0].y = 0.0;
		marker.points[0].z = 0.0;
		marker.points[1].x = cos(h);
		marker.points[1].y = sin(h);
		marker.points[1].z = 0.0;
 		marker.scale.x = 0.05;
		marker.color.r = r;
		marker.color.g = g;
		marker.color.b = b;
		marker.color.a = 1.0;
		marker.lifetime = ros::Duration(180.0);
		marker_publisher_.publish(marker);
		marker_publisher_.publish(marker);
	}
	
	inline void visualizeRanges(const std::vector<double> &rngs, const std::vector<double> &angles, std::string ns, const std::vector<double> &colors){
		visualization_msgs::Marker marker;
		double r=0,g=0,b=0;

		marker.header.stamp = ros::Time::now();
		marker.header.frame_id = "base_link"; //reference_frame_;
		marker.ns = ns;
		marker.id = 1;
		marker.type = visualization_msgs::Marker::POINTS;
		marker.action = visualization_msgs::Marker::ADD;
		marker.points.resize(rngs.size());
		marker.colors.resize(rngs.size());
		for(int i = 0; i < (int) rngs.size(); i++){
			marker.points[i].x = rngs[i] * cos(angles[i]);
			marker.points[i].y = rngs[i] * sin(angles[i]);
			marker.points[i].z = 0.0;
			HSVtoRGB(&r, &g, &b, colors[i], 1.0, 1.0);
			marker.colors[i].r = r;
			marker.colors[i].g = g;
			marker.colors[i].b = b;
		}
 		marker.scale.x = 0.025;
 		marker.scale.y = 0.025;
 		marker.scale.z = 0.025;
 		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 1.0;
		marker.color.a = 1.0;
		marker.lifetime = ros::Duration(180.0);
		marker_publisher_.publish(marker);
		marker_publisher_.publish(marker);
	}
};

#endif

