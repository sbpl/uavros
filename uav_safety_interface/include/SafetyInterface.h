#ifndef _uavSafetyInterface_H_
#define _uavSafetyInterface_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
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
#define UAVSAFETY_VISUALS
//#define UAVSAFETY_TESTCONTROLLER

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

class uavSafetyInterface {

public: 
	uavSafetyInterface(ros::NodeHandle n, std::string in_cmd_topic, std::string out_cmd_topic, std::string laser_topic, std::string imu_topic);
	
	void scanCallback (const sensor_msgs::LaserScan &scan_in);
	
	void imuCallback(const sensor_msgs::Imu &imu_in);
	
	void cmdReceived(const uav_msgs::ControllerCommand &cmd_in);
	
	inline void setPitchGain(double pg){
		pitch_gain = pg;
	}
	
	inline void setRollGain(double rg){
		roll_gain = rg;
	}
	
	inline void setRollLimits(double rmin, double rmax){
		roll_min = rmin;
		roll_max = rmax;
	}
	
	inline void setPitchLimits(double pmin, double pmax){
		pitch_max = pmax;
		pitch_min = pmin;
	}
	
private:
	ros::NodeHandle n_;
	
	ros::Subscriber laser_sub_;
	ros::Subscriber imu_sub_;
	ros::Subscriber cmd_reader_;
	ros::Publisher cmd_writer_;
	
	ros::Publisher marker_publisher_;
	
	std::vector<double> las_ranges;
	std::vector<double> las_angles;
	
	std::vector<double> imu_linear_acc;
	double pitch_estimate;
	double roll_estimate;
	
	double min_range;
	double min_angle;
	
	double pitch_gain, pitch_min, pitch_max;
	double roll_gain, roll_min, roll_max;
	
	bool laser_data_received, imu_data_received;
	boost::mutex laser_mutex_, imu_mutex_;
	
	bool filterCommand(const uav_msgs::ControllerCommand &cmd_in, uav_msgs::ControllerCommand *cmd_out);
	
	double estimate_heading(double pitch, double roll);

	double angle_normalize(double th1);

	double angle_diff(double th1, double th2);
	
	inline void visualizeCmd(const uav_msgs::ControllerCommand &cmd, std::string ns, double color){
		visualization_msgs::Marker marker;
		double r=0,g=0,b=0;
		HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

		marker.header.stamp = ros::Time::now();
		marker.header.frame_id = "body_frame"; //reference_frame_;
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
		marker.points[3].y = -0.5*cmd.roll;
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
		marker.header.frame_id = "body_frame"; //reference_frame_;
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
		marker.header.frame_id = "body_frame"; //reference_frame_;
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
	
	inline void visualizeVector(const std::vector<double> v, std::string ns, double color){
		visualization_msgs::Marker marker;
		double r=0,g=0,b=0;
		HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

		marker.header.stamp = ros::Time::now();
		marker.header.frame_id = "body_frame"; //reference_frame_;
		marker.ns = ns;
		marker.id = 1;
		marker.type = visualization_msgs::Marker::LINE_LIST;
		marker.action = visualization_msgs::Marker::ADD;
		marker.points.resize(2);
		marker.points[0].x = 0.0;
		marker.points[0].y = 0.0;
		marker.points[0].z = 0.0;
		marker.points[1].x = v[0];
		marker.points[1].y = v[1];
		marker.points[1].z = v[2];
 		marker.scale.x = 0.05;
		marker.color.r = r;
		marker.color.g = g;
		marker.color.b = b;
		marker.color.a = 1.0;
		marker.lifetime = ros::Duration(180.0);
		marker_publisher_.publish(marker);
		marker_publisher_.publish(marker);
	}
};

#endif

