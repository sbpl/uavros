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
	~UAVStatePublisher() { }

private:

	class FIFO
	{
	public:

		FIFO(int s);
		~FIFO();

		void insert(double x);

		double operator[](int i);

		int size() { return elements_; }

	private:

		int size_, head_, tail_, elements_;
		double* q_;
	};

	class velo_list
	{
	public:

		velo_list(int s);

		void add_value(double val);

		double get_last() { return my_list.front(); }

	private:

		std::list<double> my_list;
		int count;
		int size;
	};

	class integrated_accel
	{
	public:

        /// \param s maximum length of buffer
		integrated_accel(int s);

        /// \param val
		void set_value(double val, ros::Time time);

        /// \return the number of seconds spanned by the buffered readings
        double get_timespan() const;

		double get_integrated();

<<<<<<< HEAD
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
=======
        void clear();
        int size() const { return m_count; }
        int max_size() const { return m_size; }
        bool full() const { return m_count == m_size; }

	private:

        struct reading
        {
            double value;
            ros::Time time;
        };

		std::list<reading> m_list;   // reading buffer

        // number of readings in the buffer
		int m_count;

        // maximum length of reading buffer
		int m_size;

        // flag indicating whether an initial reading has been received to use
        // as an offset
		bool m_offset_set;

        // the stored offset from the initial reading
		double m_offset;
>>>>>>> e60c2f6a109b73167f162705cda6410f3246c066
	};

	void ekfCallback(nav_msgs::OdometryConstPtr p);
	void lidarCallback(sensor_msgs::LaserScanConstPtr scan);
	void slamCallback(geometry_msgs::PoseStampedConstPtr slam_msg);
	bool estimateInitialHeight(sensor_msgs::LaserScanConstPtr  scan, double& ret_height);
<<<<<<< HEAD
    void flightModeCallback( uav_msgs::FlightModeStatusConstPtr msg);
    void rawImuCallback(sensor_msgs::Imu imu);
    bool get_closest_elevation(std::map<std::pair<double, double>, double> elevation_map_, double current_x, double current_y, 
    							 double &min_dist, double &ret_x, double &ret_y, double &ret_elevation);

=======
    void flightModeCallback(uav_msgs::FlightModeStatusConstPtr msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
>>>>>>> e60c2f6a109b73167f162705cda6410f3246c066

    // topic names
	std::string state_pub_topic_;
    std::string z_laser_topic_;
    std::string z_laser_median_topic_;
    std::string position_sub_topic_;
    std::string vertical_laser_data_topic_;
    std::string slam_topic_;
    std::string imu_topic_;
    std::string rpy_pub_topic_;
    std::string flt_mode_stat_topic_;

    // tf frame names
    std::string tf_prefix_;
<<<<<<< HEAD
    std::string vertical_laser_frame_topic_;
    std::string map_topic_;
    std::string odom_topic_;
    std::string body_topic_;
    std::string body_map_aligned_topic_;
    std::string body_stabilized_topic_;
    std::string base_footprint_topic_;
=======
    std::string vertical_laser_frameid_;
    std::string map_frameid_;
    std::string odom_frameid_;
    std::string body_frameid_;
    std::string body_map_aligned_frameid_;
    std::string body_stabilized_frameid_;
>>>>>>> e60c2f6a109b73167f162705cda6410f3246c066

	nav_msgs::Odometry state_;

	ros::Publisher state_pub_;
	ros::Publisher pointCloud_pub_;
	ros::Publisher point_pub_;
	ros::Publisher pos_pub_;
	ros::Publisher vel_pub_;
	ros::Publisher ac_pub_;
	ros::Publisher slam_vel_pub_;
    ros::Publisher rpy_pub_;
    ros::Publisher max_z_pub_;

	ros::Subscriber ekf_sub_;
	ros::Subscriber slam_sub_;
	ros::Subscriber lidar_sub_;
    ros::Subscriber flight_mode_sub_;
<<<<<<< HEAD
    ros::Publisher elv_marker_pub_;
    ros::Publisher m_max_z_pub;
=======
    ros::Subscriber imu_sub_;
>>>>>>> e60c2f6a109b73167f162705cda6410f3246c066

	double saved_yaw_;

	tf::TransformListener tf_listener_;
	tf::TransformBroadcaster tf_broadcaster_;

	double min_lidar_angle_;
    double max_lidar_angle_;

	double height_filter_deviation_max_;

	FIFO z_fifo_;
	FIFO z_time_fifo_;

<<<<<<< HEAD
	integrated_accel* x_integrated_;
	integrated_accel* y_integrated_;
	integrated_accel* z_integrated_;
	integrated_accel* z_pos_int_;
	velo_list* x_velo_;
	velo_list* y_velo_;
	velo_list* z_velo_;
	velo_list* z_pos_;
	
	ros::Time l_t_;
=======
    // world-frame velocities determined by integrating imu measurements
	integrated_accel x_integrated_;
	integrated_accel y_integrated_;
    double vel_x_integrated_;
    double vel_y_integrated_;

    // world-frame velocities determined by differentiating slam measurements
    ros::Time vel_update_time_; // when the last velocity update was computed
    double prev_pose_x_;
    double prev_pose_y_;
    double vel_x_derived_;
    double vel_y_derived_;

	velo_list x_velo_;
	velo_list y_velo_;

>>>>>>> e60c2f6a109b73167f162705cda6410f3246c066
    uav_msgs::FlightModeStatus last_state_;

<<<<<<< HEAD
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

=======
    /// \name Height Estimation
    ///@{
    tf::StampedTransform T_body_vlaser_;
    ros::Time last_scan_analysis_time_;
    ros::Time prev_scan_time_;
    int num_scans_processed_;
    double filtered_z_;
    double prev_filtered_z_;
    ///@}
>>>>>>> e60c2f6a109b73167f162705cda6410f3246c066
};

#endif
