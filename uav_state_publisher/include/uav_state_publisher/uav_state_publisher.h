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
    std::string flt_mode_stat_topic_;

    // tf frame names
    std::string tf_prefix_;
    std::string vertical_laser_frameid_;
    std::string map_frameid_;
    std::string odom_frameid_;
    std::string body_frameid_;
    std::string body_map_aligned_frameid_;
    std::string body_stabilized_frameid_;

	nav_msgs::Odometry state_;

	ros::Publisher state_pub_;
	ros::Publisher pointCloud_pub_;
	ros::Publisher point_pub_;
	ros::Publisher vel_pub_;
	ros::Publisher ac_pub_;
	ros::Publisher slam_vel_pub_;
    ros::Publisher rpy_pub_;
    ros::Publisher max_z_pub_;

	ros::Subscriber ekf_sub_;
	ros::Subscriber slam_sub_;
	ros::Subscriber lidar_sub_;
    ros::Subscriber flight_mode_sub_;
    ros::Subscriber imu_sub_;

	double saved_yaw_;

	tf::TransformListener tf_listener_;
	tf::TransformBroadcaster tf_broadcaster_;

	double min_lidar_angle_;
    double max_lidar_angle_;

	double height_filter_deviation_max_;

	FIFO z_fifo_;
	FIFO z_time_fifo_;

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

    uav_msgs::FlightModeStatus last_state_;

    /// \name Height Estimation
    ///@{
    tf::StampedTransform T_body_vlaser_;
    ros::Time last_scan_analysis_time_;
    ros::Time prev_scan_time_;
    int num_scans_processed_;
    double filtered_z_;
    double prev_filtered_z_;
    ///@}
};

#endif
