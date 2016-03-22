#ifndef UAV_STATE_PUBLISHER_H
#define UAV_STATE_PUBLISHER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <string>
#include <list>
#include <map>
#include <uav_msgs/FlightModeStatus.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class UAVStatePublisher
{

public:
	UAVStatePublisher();
	~UAVStatePublisher()
	{
	}
	;

private:
	class FIFO
	{
	public:
		FIFO(int s)
		{
			size_ = s;
			head_ = 0;
			tail_ = 0;
			elements_ = 0;
			q_ = new double[s];
		}

		~FIFO()
		{
			delete[] q_;
		}

		void insert(double x)
		{
			*(q_ + head_) = x;
			head_++;
			if (head_ == size_)
				head_ = 0;
			if (elements_ < size_)
				elements_++;
		}

		double operator[](int i)
		{
			int j = tail_ + i;
			if (j >= size_)
				j -= size_;
			return *(q_ + j);
		}

		int size()
		{
			return elements_;
		}

	private:
		int size_, head_, tail_, elements_;
		double* q_;
	};

	class velo_list
	{

	public:

		velo_list(int s)
		{
			count = 0;
			size = s;
		}

		~velo_list()
		{
		}

		void add_value(double val)
		{
			if (count == size)
			{
				my_list.pop_front();
			}
			else
			{
				count++;
			}
			my_list.push_back(val);
		}
		double get_last()
		{
			return my_list.front();
		}

	private:
		std::list<double> my_list;
		int count;
		int size;
	};

	struct reading
	{

		double value;
		ros::Time time;

	};

	class integrated_accel
	{

	public:

		integrated_accel(int s)
		{
			size = s;
			count = 0;
			offset_set = false;
			offset = 0;
			sum = 0;
		}

		~integrated_accel()
		{
		}

		void set_value(double val, ros::Time time)
		{
			if (!offset_set)
			{
				offset_set = true;
				offset = val;
			}

			if (count == size)
			{
				my_list.pop_front();
			}
			else
			{
				count++;
			}
			reading r;
			r.value = val - offset;
			dummy = r.value;
			r.time = time;
			my_list.push_back(r);
		}
		double get_value()
		{
			return dummy;
		}

		double get_total_sum()
		{
			return sum;
		}

		double get_list_sum()
		{
			double s = 0;
			int index = 0;
			reading pr;
			for (std::list<reading>::iterator it = my_list.begin(); it != my_list.end(); ++it)
			{
				if (index != 0)
				{
					reading r = *it;
					ros::Duration dt = r.time - pr.time;
					double val = (pr.value + r.value) / 2;
					s += (val * dt.toSec());
					//ROS_ERROR("index %d ds %f dt %f d %f sum %f", index, val, dt.toSec(), val * dt.toSec(), s);
				}
				pr = *it;
				index++;
			}
			return s;
		}

	private:
		std::list<reading> my_list;
		int count;
		int size;
		double sum;
		bool offset_set;
		double offset;
		double dummy;
	};

	void ekfCallback(nav_msgs::OdometryConstPtr p);
	void lidarCallback(sensor_msgs::LaserScanConstPtr scan);
	void slamCallback(geometry_msgs::PoseStampedConstPtr slam_msg);
	bool estimateInitialHeight(sensor_msgs::LaserScanConstPtr  scan, double& ret_height);
    void flightModeCallback( uav_msgs::FlightModeStatusConstPtr msg);
    void rawImuCallback(sensor_msgs::Imu imu);
    bool get_closest_elevation(std::map<std::pair<double, double>, double> elevation_map_, double current_x, double current_y, 
    							 double &min_dist, double &ret_x, double &ret_y, double &ret_elevation);


    // topic names
	std::string state_pub_topic_;
    std::string z_laser_topic_;
    std::string z_laser_median_topic_;
    std::string position_sub_topic_;
    std::string vertical_laser_data_topic_;
    std::string slam_topic_;
    std::string imu_topic_;
    std::string rpy_pub_topic_;

    // tf frame names
    std::string tf_prefix_;
    std::string vertical_laser_frame_topic_;
    std::string map_topic_;
    std::string odom_topic_;
    std::string body_topic_;
    std::string body_map_aligned_topic_;
    std::string body_stabilized_topic_;
    std::string base_footprint_topic_;

	nav_msgs::Odometry state_;
	ros::Publisher state_pub_;
	ros::Publisher pointCloud_pub_;
	ros::Publisher point_pub_;
	ros::Publisher pos_pub_;
	ros::Publisher vel_pub_;
	ros::Publisher ac_pub_;
	ros::Publisher slam_vel_pub_;
	ros::Subscriber ekf_sub_;
	ros::Subscriber slam_sub_;
	ros::Subscriber lidar_sub_;
	ros::Publisher rpy_pub_;
    ros::Subscriber flight_mode_sub_;
    ros::Publisher elv_marker_pub_;
    ros::Publisher m_max_z_pub;

	double saved_yaw_;

	tf::TransformListener tf_;
	tf::TransformBroadcaster tf_broadcaster;

	double min_lidar_angle_, max_lidar_angle_;

	double height_filter_deviation_max_;

	FIFO z_fifo_;
	FIFO z_time_fifo_;

	integrated_accel* x_integrated_;
	integrated_accel* y_integrated_;
	integrated_accel* z_integrated_;
	integrated_accel* z_pos_int_;
	velo_list* x_velo_;
	velo_list* y_velo_;
	velo_list* z_velo_;
	velo_list* z_pos_;
	
	ros::Time l_t_;
    uav_msgs::FlightModeStatus last_state_;
    std::string flt_mode_stat_topic_;

    tf::StampedTransform m_T_body_vlaser;
    tf::Transform T_map_odom;
    ros::Time m_last_scan_analysis_time;
    ros::Time m_prev_scan_time;
    ros::Subscriber raw_imu_sub_;
    visualization_msgs::MarkerArray elv_marker_array;
    
    int m_num_scans_processed;
    
    double m_filtered_z;
    double m_prev_filtered_z;
    double current_zs;
    double prev_zs;
    double current_elevation;
    double prev_elevation;
    
    double elz;
    double scale;

    double min_dist;
    double ret_x;
    double ret_y;
    double ret_elevation;

    double check_x;
    double check_y;

    typedef std::pair<double, double> key;
	std::map<std::pair<double, double> , double> elevation_map_;

    bool transform_to_map;
    bool update_map;

};

#endif

