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
#include <uav_msgs/FlightModeStatus.h>

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

        int size() const { return m_count; }
        int max_size() const { return m_size; }

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
	};

	void ekfCallback(nav_msgs::OdometryConstPtr p);
	void lidarCallback(sensor_msgs::LaserScanConstPtr scan);
	void slamCallback(geometry_msgs::PoseStampedConstPtr slam_msg);
	bool estimateInitialHeight(sensor_msgs::LaserScanConstPtr  scan, double& ret_height);
    void flightModeCallback(uav_msgs::FlightModeStatusConstPtr msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

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

	nav_msgs::Odometry state_;
	ros::Publisher state_pub_;
	ros::Publisher pointCloud_pub_;
	ros::Publisher point_pub_;
	ros::Publisher vel_pub_;
	ros::Publisher ac_pub_;
	ros::Publisher slam_vel_pub_;
	ros::Subscriber ekf_sub_;
	ros::Subscriber slam_sub_;
	ros::Subscriber lidar_sub_;
	ros::Publisher rpy_pub_;
    ros::Subscriber flight_mode_sub_;
    ros::Subscriber m_imu_sub;

	double saved_yaw_;

	tf::TransformListener tf_;
	tf::TransformBroadcaster tf_broadcaster;

	double min_lidar_angle_, max_lidar_angle_;

	double height_filter_deviation_max_;

	FIFO z_fifo_;
	FIFO z_time_fifo_;

    sensor_msgs::Imu m_last_imu;
	integrated_accel* x_integrated_;
	integrated_accel* y_integrated_;
	velo_list* x_velo_;
	velo_list* y_velo_;

	ros::Time l_t_;
    uav_msgs::FlightModeStatus last_state_;
    std::string flt_mode_stat_topic_;

    /// \name Height Estimation
    ///@{
    tf::StampedTransform m_T_body_vlaser;
    ros::Time m_last_scan_analysis_time;
    ros::Time m_prev_scan_time;
    int m_num_scans_processed;
    double m_filtered_z;
    double m_prev_filtered_z;
    ros::Publisher m_max_z_pub;
    ///@}
};

#endif

