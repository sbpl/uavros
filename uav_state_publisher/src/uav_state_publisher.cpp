#include <uav_state_publisher/uav_state_publisher.h>

#include <vector>

#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>

#define PRINT_VARIABLE(x) std::cout << #x << ": " << x << std::endl;

UAVStatePublisher::UAVStatePublisher() :
	tf_(ros::NodeHandle(), ros::Duration(10), true),
    z_fifo_(5),
    z_time_fifo_(5),
    m_last_scan_analysis_time(),
    m_prev_scan_time(),
    m_num_scans_processed(0),
    m_filtered_z(0.0),
    m_prev_filtered_z(0.0)
{
	ros::NodeHandle nh;
	ros::NodeHandle ph("~");

	ph.param<std::string>("state_pub_topic", state_pub_topic_, "uav_state");
	ph.param<std::string>("Z_laser_topic", z_laser_topic_, "HeightLaser");
	ph.param<std::string>("Z_laser_median_topic", z_laser_median_topic_, "HeightLaserMedian");
	ph.param<std::string>("position_sub_topic", position_sub_topic_, "ekf_state");
	ph.param<std::string>("vertical_laser_data_topic", vertical_laser_data_topic_, "panning_laser");
	ph.param<std::string>("vertical_laser_frame_topic", vertical_laser_frame_topic_, "/panning_laser_frame");
	ph.param<std::string>("slam_topic", slam_topic_, "slam_out_pose");
	ph.param<std::string>("map_topic", map_topic_, "/map");
	ph.param<std::string>("body_topic", body_topic_, "/body_frame");
	ph.param<std::string>("body_map_aligned_topic", body_map_aligned_topic_, "/body_frame_map_aligned");
	ph.param<std::string>("body_stabilized_topic", body_stabilized_topic_, "/body_frame_stabilized");
	ph.param<std::string>("imu_topic", imu_topic_, "/raw_imu");
    ph.param<std::string>("rpy_pub_topic", rpy_pub_topic_, "/rpy_with_acc4");
    ph.param<std::string>("flt_mode_stat_topic",flt_mode_stat_topic_,"/flight_mode_status");
    ph.param("min_lidar_angle", min_lidar_angle_, -3.14159);
    ph.param("max_lidar_angle", max_lidar_angle_, 3.14159);
    ph.param("height_filter_deviation_max", height_filter_deviation_max_, 0.2);

    //publish an odometry message (it's the only message with all the state
    //variables we want)

	state_pub_ = nh.advertise<nav_msgs::Odometry>(state_pub_topic_, 1);
	pointCloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(z_laser_topic_, 1);
	point_pub_ = nh.advertise<sensor_msgs::PointCloud2>(z_laser_median_topic_, 1);
	vel_pub_ = nh.advertise<geometry_msgs::PointStamped>("/acel_integrated", 1);
	ac_pub_ = nh.advertise<geometry_msgs::PointStamped>("/acel_trans", 1);
	slam_vel_pub_ = nh.advertise<geometry_msgs::PointStamped>("/slam_vel", 1);
	rpy_pub_ = nh.advertise<geometry_msgs::PointStamped>(rpy_pub_topic_, 1);
    m_max_z_pub = nh.advertise<std_msgs::Float32>("max_z", 10);

	//subscribe to the SLAM pose from hector_mapping, the EKF pose from hector_localization, and the vertical lidar
	ekf_sub_ = nh.subscribe(position_sub_topic_, 3, &UAVStatePublisher::ekfCallback, this);
	lidar_sub_ = nh.subscribe(vertical_laser_data_topic_, 3, &UAVStatePublisher::lidarCallback, this);
	slam_sub_ = nh.subscribe(slam_topic_, 3, &UAVStatePublisher::slamCallback, this);
    flight_mode_sub_ = nh.subscribe(flt_mode_stat_topic_, 1, &UAVStatePublisher::flightModeCallback, this);

	x_integrated_ = new integrated_accel(40);
	y_integrated_ = new integrated_accel(40);
	x_velo_ = new velo_list(41);
	y_velo_ = new velo_list(41);
	saved_yaw_ = 0;

}

/*void UAVStatePublisher::rawImuCallback(sensor_msgs::Imu imu)
 {
 ros::Time start_ = ros::Time::now();

 //determine gravity compensated accelerations
 tf::Point p(imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z);
 tf::StampedTransform transform;

 try {
 tf_.lookupTransform( body_map_aligned_topic_, body_topic_, ros::Time(0), transform);
 }
 catch (tf::TransformException ex){
 return;
 }

 tf::Point pout = transform * p;
 double accel_x = pout[0];
 double accel_y = pout[1];
 double accel_z = pout[2];

 ROS_ERROR("BEFORE %f %f %f", p[0], p[1], p[2]);
 ROS_ERROR("AFTER %f %f %f\n", pout[0],pout[1],pout[2]);

 x_integrated_->set_value(accel_x,imu.header.stamp);
 double total_sum_x = x_integrated_->get_total_sum();
 double part_sum_x = x_integrated_->get_list_sum() + x_velo_->get_last();

 y_integrated_->set_value(accel_y,imu.header.stamp);
 double total_sum_y = y_integrated_->get_total_sum();
 double part_sum_y = y_integrated_->get_list_sum() + y_velo_->get_last();

 ROS_ERROR("VALUES X %f Y %f", imu.linear_acceleration.x, imu.linear_acceleration.y);
 ROS_ERROR("TOTAL X_SUM %f Y_SUM %f ", total_sum_x, total_sum_y);
 ROS_ERROR("PARTIAL X_SUM %f Y_SUM %f", part_sum_x, part_sum_y);
 ROS_ERROR("VELO X %f Y %f\n", x_velo_->get_last(), y_velo_->get_last());

 geometry_msgs::PointStamped accel_int;
 accel_int.header.stamp = start_;
 accel_int.point.x = part_sum_x;
 accel_int.point.y = part_sum_y;
 vel_pub_.publish(accel_int);

 geometry_msgs::PointStamped acc_trans;
 acc_trans.header.stamp = start_;
 acc_trans.point.x = accel_x/40;
 acc_trans.point.y = accel_y/40;
 acc_trans.point.z = accel_z/40;
 ac_pub_.publish(acc_trans);

 }*/

void UAVStatePublisher::flightModeCallback(uav_msgs::FlightModeStatusConstPtr msg)
{
  last_state_ = *msg;
  ROS_DEBUG("flight_status is %d\n", (int) last_state_.mode);
}

void UAVStatePublisher::slamCallback(geometry_msgs::PoseStampedConstPtr slam_msg)
{
	ros::Time start_ = ros::Time::now();

	saved_yaw_ = tf::getYaw(slam_msg->pose.orientation);

	state_.pose.pose.position.x = slam_msg->pose.position.x;
	state_.pose.pose.position.y = slam_msg->pose.position.y;
	// printf("############################################got the slam stuff....\n");
	ros::Time stop_ = ros::Time::now();
	ROS_DEBUG("[state_pub] slam callback %f %f = %f", start_.toSec(), stop_.toSec(), stop_.toSec() - start_.toSec());

}

// on the order of 50Hz
void UAVStatePublisher::ekfCallback(nav_msgs::OdometryConstPtr p)
{
	ros::Time start_ = ros::Time::now();

	x_velo_->add_value(p->twist.twist.linear.x);
	y_velo_->add_value(p->twist.twist.linear.y);

	// get angular velocities
	state_.twist.twist.angular = p->twist.twist.angular;

	//get the orientation
	double roll, pitch, yaw;
	tf::Quaternion q;
	tf::quaternionMsgToTF(p->pose.pose.orientation, q);
	tf::Matrix3x3(q).getEulerZYX(yaw, pitch, roll);
	//Publish RPY for debugging (in degrees)
	geometry_msgs::PointStamped rpy;
	rpy.header.stamp = p->header.stamp;
	rpy.point.x = roll * 180 / M_PI;
	rpy.point.y = pitch * 180 / M_PI;
	rpy.point.z = yaw * 180 / M_PI;
	rpy_pub_.publish(rpy);

	state_.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, saved_yaw_);

	// get the x and y velocities
	state_.twist.twist.linear.x = p->twist.twist.linear.x;
	state_.twist.twist.linear.y = p->twist.twist.linear.y;

    if (z_fifo_.size() < 1) {
        ROS_WARN_THROTTLE(1, "State is incomplete");
        return;
    }

	// compute the z velocity
	double dz = 0.0;
    if (z_fifo_.size() > 0) {
        for (int i = 1; i < z_fifo_.size(); i++) {
            dz += (z_fifo_[i] - z_fifo_[i - 1]) / (z_time_fifo_[i] - z_time_fifo_[i - 1]);
        }
        dz /= z_fifo_.size();
    }
	state_.twist.twist.linear.z = dz;

	//publish the map to base_link transform
	geometry_msgs::TransformStamped trans;
	trans.header.stamp = p->header.stamp;
	trans.header.frame_id = map_topic_;
	trans.transform.translation.x = state_.pose.pose.position.x;
	trans.transform.translation.y = state_.pose.pose.position.y;
	trans.transform.translation.z = state_.pose.pose.position.z;

    // publish the map -> odom transform

    const tf::Transform T_odom_body(
            tf::Quaternion(
                    p->pose.pose.orientation.x,
                    p->pose.pose.orientation.y,
                    p->pose.pose.orientation.z,
                    p->pose.pose.orientation.w),
            tf::Vector3(
                    p->pose.pose.position.x,
                    p->pose.pose.position.y,
                    p->pose.pose.position.z));
    const tf::Transform T_map_body(
            tf::Quaternion(
                    state_.pose.pose.orientation.x,
                    state_.pose.pose.orientation.y,
                    state_.pose.pose.orientation.z,
                    state_.pose.pose.orientation.w),
            tf::Vector3(
                state_.pose.pose.position.x,
                state_.pose.pose.position.y,
                state_.pose.pose.position.z));

    const tf::Transform T_map_odom = T_map_body * T_odom_body.inverse();
    tf::transformTFToMsg(T_map_odom, trans.transform);

	//ROS_WARN("height is %f\n", trans.transform.translation.z);
	trans.child_frame_id = "odom"; // TODO: configurate frame name //body_topic_;
	tf_broadcaster.sendTransform(trans);

    // publish the map -> body_frame_map_aligned transform
    trans.child_frame_id = body_map_aligned_topic_;
    trans.transform.rotation.w = 1.0;
    trans.transform.rotation.x = 0.0;
    trans.transform.rotation.y = 0.0;
    trans.transform.rotation.z = 0.0;
    tf_broadcaster.sendTransform(trans);

	// publish the state
	state_.header.stamp = ros::Time::now();
	state_pub_.publish(state_);
	ros::Time stop_ = ros::Time::now();
	ROS_DEBUG("[state_pub] ekf callback %f %f = %f", start_.toSec(), stop_.toSec(), stop_.toSec() - start_.toSec());
}

//on the order of 40Hz
void UAVStatePublisher::lidarCallback(sensor_msgs::LaserScanConstPtr scan)
{
    const ros::Time now = ros::Time::now();

    // log scan count over the last period
    const ros::Duration scan_analysis_period(1.0);
    if (now > m_last_scan_analysis_time + scan_analysis_period) {
        // log how many scans we've processed in the last period
        const ros::Duration span = (now - m_last_scan_analysis_time);
        ROS_DEBUG("Processed %d scans in the last %0.3f seconds (%0.3f Hz)",
                m_num_scans_processed,
                span.toSec(),
                (double)m_num_scans_processed / (span.toSec()));
        m_num_scans_processed = 0;
        m_last_scan_analysis_time = now;
    }

    ++m_num_scans_processed;
    const ros::Duration dt = now - m_prev_scan_time;

    double prev_height = state_.pose.pose.position.z;

    /////////////////////////////////////////////////
    // estimate initial height on first invocation //
    /////////////////////////////////////////////////

    if (z_fifo_.size() < 1) {
        double est_height;
        bool suc = estimateInitialHeight(scan, est_height);
        if (!suc) {
            return;
        }
        ROS_INFO("uav_state_publisher: estimated initial height is %f", est_height);
        state_.pose.pose.position.z = est_height;
        m_filtered_z = est_height;
    }

    //////////////////////////////////////////////////////////////////
    // lookup transforms necessary for transforming points into the //
    // body_stabilized frame                                        //
    //////////////////////////////////////////////////////////////////

    // need to be able to transform laser points into the body frame...
    bool have_bodymapaligned_to_laser = false;
    bool have_body_to_laser = false;
    bool have_bodymapaligned_to_body = false;

    tf::StampedTransform T_bodystable_vlaser;
    // first try the master transform
    try {
        tf_.lookupTransform(
                body_stabilized_topic_,
                scan->header.frame_id,
                ros::Time(0),
                T_bodystable_vlaser);

        have_bodymapaligned_to_laser = true;
    }
    catch (const tf::TransformException& ex) {
        ROS_ERROR("Failed to lookup transform (%s)", ex.what());
    }

    // then try the two intermediate transforms
    tf::StampedTransform T_bodymapaligned_body;
    try {
        tf_.lookupTransform(
            body_stabilized_topic_,
            body_topic_,
            ros::Time(0),
            T_bodymapaligned_body);

        have_bodymapaligned_to_body = true;

        tf_.lookupTransform(
            body_topic_,
            vertical_laser_frame_topic_,
            ros::Time(0),
            m_T_body_vlaser);

        have_body_to_laser = true;
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("Look of partial transforms failed (%s)", ex.what());
    }

    // ensure we can transform the laser scan into the map aligned frame
    if (!have_bodymapaligned_to_laser &&
        !(have_bodymapaligned_to_body && have_body_to_laser))
    {
        ROS_ERROR("Failed to transform laser scan into z-aligned frame");
        return;
    }

    std::vector<double> zs;
    zs.reserve(scan->ranges.size());
    std::vector<geometry_msgs::Point> voxels;
    double max_z = 0;

    for (int i = 0; i < scan->ranges.size(); i++) {
        const float ang = scan->angle_min + i * scan->angle_increment;

        // skip rays below the min angle
        if (ang < min_lidar_angle_) {
            continue;
        }

        // Leave when designated max angle reached
        if (ang > max_lidar_angle_) {
            break;
        }

        // Skip ray if distance not within message provided thresholds
        if (scan->ranges[i] < scan->range_min ||
            scan->ranges[i] > scan->range_max)
        {
            continue;
        }

        // Generate ray projection (a point) with respect in the laser frame
        geometry_msgs::PointStamped pin;
        pin.header.seq = 0;
        pin.header.stamp = ros::Time(0);
        pin.header.frame_id = scan->header.frame_id;
        pin.point.x = scan->ranges[i] * cos(ang);
        pin.point.y = scan->ranges[i] * sin(ang);
        pin.point.z = 0;

        // transform the point into the body_stabilized frame
        geometry_msgs::PointStamped pout;
        pout.header.seq = 0;
        pout.header.stamp = ros::Time(0);
        pout.header.frame_id = body_stabilized_topic_;

        tf::Point ptf(pin.point.x, pin.point.y, pin.point.z);
        if (have_bodymapaligned_to_laser) {
            tf::Point ptfout(T_bodystable_vlaser * ptf);
            pout.point.x = ptfout.getX();
            pout.point.y = ptfout.getY();
            pout.point.z = ptfout.getZ();
        }
        else if (have_bodymapaligned_to_body && have_body_to_laser) {
            tf::Point ptfout(T_bodymapaligned_body * m_T_body_vlaser * ptf);
            pout.point.x = ptfout.getX();
            pout.point.y = ptfout.getY();
            pout.point.z = ptfout.getZ();
        }
        else {
            ROS_ERROR("Why am I here?");
        }

        // laser point in body stabilized frame -> body position in map frame
        pout.point.z = -pout.point.z;

        // update max_z; only calculate max_z using rays "directly under the
        // hexacopter"
        if (fabs(ang - M_PI / 2) < (10.0 * M_PI / 180.0)) {
            max_z = std::max(max_z, pout.point.z);
        }

        // only accept height estimates that are close to previous height
        if ((pout.point.z < m_filtered_z + height_filter_deviation_max_) &&
            (pout.point.z > m_filtered_z - height_filter_deviation_max_))
        {
            zs.push_back(pout.point.z);
            voxels.push_back(pout.point);
        }
    }

    const double height_cut_off = 2.0; // TODO: configurate
    max_z = std::min(max_z, height_cut_off);

    // TODO: do something smarter that will filter out tables

    // publish max_z for debugging
    std_msgs::Float32 max_z_msg;
    max_z_msg.data = max_z;
    m_max_z_pub.publish(max_z_msg);

    // publish point cloud of considered ray points
    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    for (size_t i = 0; i < voxels.size(); i++) {
        pclCloud.push_back(pcl::PointXYZ(voxels[i].x, voxels[i].y, voxels[i].z));
    }
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg(pclCloud, cloud);
    cloud.header.frame_id = body_map_aligned_topic_;
    cloud.header.stamp = ros::Time::now();
    pointCloud_pub_.publish(cloud);

    // Only estimate height afterwards when robot is in air
    if (last_state_.mode != uav_msgs::FlightModeStatus::LANDED)
    {
        // set height to previous filtered height
        state_.pose.pose.position.z = m_prev_filtered_z;

        // get z by taking the median
        if (zs.size() > 6) {
            std::sort(zs.begin(), zs.end());
            m_filtered_z = zs[zs.size() / 2];
            state_.pose.pose.position.z = m_filtered_z;
        }

        if (height_cut_off <= max_z) {
            // take max of filtered and low bigger than cutout height
            // state_.pose.pose.position.z = max(m_filtered_z, max_z);
            const double alpha = 0.9;
            state_.pose.pose.position.z = alpha * m_filtered_z + (1.0 - alpha) * max_z;
        }

        // update filtered z with weighted average
        const double beta = 0.99;

        // arbitrary number, possibly related to takeoff height, and something
        // about
        const double SOMETHING = 1.2;
        if (max_z > SOMETHING) {
            m_filtered_z = (beta * m_filtered_z) + ((1.0 - beta) * max_z);
        }
        m_prev_filtered_z = m_filtered_z;
    }
    else {
        ROS_DEBUG("landed");
    }

    z_fifo_.insert(state_.pose.pose.position.z);
    z_time_fifo_.insert(scan->header.stamp.toSec());

    // publish the position as a point cloud for visualization
    pcl::PointCloud<pcl::PointXYZ> pclmedianpt;
    pclmedianpt.push_back(pcl::PointXYZ(
            state_.pose.pose.position.x,
            state_.pose.pose.position.y,
            state_.pose.pose.position.z));
    sensor_msgs::PointCloud2 medianpt;
    pcl::toROSMsg(pclmedianpt, medianpt);
    medianpt.header.frame_id = map_topic_;
    medianpt.header.stamp = ros::Time::now();
    point_pub_.publish(medianpt);

    double ds = abs(state_.pose.pose.position.z - prev_height);
    ROS_DEBUG_THROTTLE(1, "height: %0.3f, maxz: %0.3f filteredz %f with dt %f ds %f time %f",
            state_.pose.pose.position.z,
            max_z,
            m_filtered_z,
            dt.toSec(),
            ds,
            now.toSec());

    m_prev_scan_time = now;
    ros::Time stop_ = ros::Time::now();
    ROS_DEBUG("[state_pub] lidar callback %f %f = %f", now.toSec(), stop_.toSec(), stop_.toSec() - now.toSec());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "uav_state_publisher");
	UAVStatePublisher sp;
    ros::Rate loop_rate(80);
    while(ros::ok())
    {
	   ros::spinOnce();
       loop_rate.sleep();
    }

	return 0;
}

bool UAVStatePublisher::estimateInitialHeight(
    sensor_msgs::LaserScanConstPtr scan,
    double& ret_height)
{
    std::vector<tf::Point> points;
    points.reserve(scan->ranges.size());
    float ang = scan->angle_min;

    // get points from lidar frame between desginated lidar angles and ranges
    for (size_t i = 0; i < scan->ranges.size(); i++) {
        bool valid = true;
        if (scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max) {
            valid = false;
        }

        if (ang <= min_lidar_angle_ || ang >= max_lidar_angle_) {
            valid = false;
        }

        if (valid) {
            tf::Point p(scan->ranges[i] * cos(ang), scan->ranges[i] * sin(ang), 0);
            points.push_back(p);
        }

        ang += scan->angle_increment;
    }

    // get transformation from body frame map aligned
    tf::StampedTransform T_bodystable_vlaser;
    try {
        tf_.lookupTransform(
                body_stabilized_topic_,
                vertical_laser_frame_topic_,
                ros::Time(0),
                T_bodystable_vlaser);
    }
    catch (tf::TransformException& ex) {
        ROS_ERROR_THROTTLE(1, "Failed to lookup transform (%s)", ex.what());
        return false;
    }

    // transform all these points and store their heights
    ROS_INFO("Points:");
    std::vector<double> zs(points.size());
    for (size_t i = 0; i < points.size(); i++) {
        tf::Point& p = points.at(i);
        p = T_bodystable_vlaser * p;
        zs[i] = p.getZ();
    }

    // grab 10 of the smallest points and return the median of those as height
    int range = std::min(10, (int)zs.size());
    double smallest_z = 0;
    if (!zs.empty()) {
        std::sort(zs.begin(), zs.end());
        std::vector<double> min_heights(range);
        std::copy(zs.begin(), zs.begin() + range, min_heights.begin());
        smallest_z = min_heights.at(range / 2);
    }

    ret_height = -smallest_z;
    return true;
}
