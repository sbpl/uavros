#include <uav_state_publisher/uav_state_publisher_new.h>

#include <vector>

#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>

UAVStatePublisher::UAVStatePublisher() :
    state_pub_topic_("uav_state"),
    z_laser_topic_("HeightLaser"),
    z_laser_median_topic_("HeightLaserMedian"),
    position_sub_topic_("ekf_state"),
    vertical_laser_data_topic_("panning_laser"),
    slam_topic_("slam_out_pose"),
    imu_topic_("raw_imu"),
    rpy_pub_topic_("rpy_with_acc4"),
    flt_mode_stat_topic_("flight_mode_status"),
    tf_prefix_(),
    vertical_laser_frameid_("panning_laser_frame"),
    map_frameid_("map"),
    odom_frameid_("odom"),
    body_frameid_("body_frame"),
    body_stabilized_frameid_("body_frame_stabilized"),
    state_(),
    state_pub_(),
    pointCloud_pub_(),
    point_pub_(),
    vel_pub_(),
    ac_pub_(),
    slam_vel_pub_(),
    rpy_pub_(),
    max_z_pub_(),
    ekf_sub_(),
    slam_sub_(),
    lidar_sub_(),
    flight_mode_sub_(),
    imu_sub_(),
    saved_yaw_(0.0),
    tf_listener_(ros::NodeHandle(), ros::Duration(10), true),
    tf_broadcaster_(),
    min_lidar_angle_(-3.14159),
    max_lidar_angle_(3.14159),
    height_filter_deviation_max_(0.2),
    z_fifo_(5),
    z_time_fifo_(5),
    x_integrated_(40),
    y_integrated_(40),
    vel_x_integrated_(0.0),
    vel_y_integrated_(0.0),
    vel_update_time_(0),
    prev_pose_x_(std::numeric_limits<double>::quiet_NaN()),
    prev_pose_y_(std::numeric_limits<double>::quiet_NaN()),
    vel_x_derived_(0.0),
    vel_y_derived_(0.0),
    x_velo_(41),
    y_velo_(41),
    last_state_(),
    T_body_vlaser_(),
    last_scan_analysis_time_(),
    prev_scan_time_(),
    num_scans_processed_(0),
    filtered_z_(0.0),
    another_filtered_z_(0.0),
    prev_filtered_z_(0.0),
    current_elevation_(0.0),
    prev_elevation_(0.0),
    current_zs_(0.0),
    prev_zs_(0.0),
    init_est(5, 0),
    elevation_noise_(0.25),
    check_x_(0.0),
    check_y_(0.0),
    scale_(0.001),
    update_map_(true),
    bag_test_(false)
    {
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    if(!bag_test_)
        body_map_aligned_frameid_ = "body_frame_map_aligned";
    else
        body_map_aligned_frameid_ = "body_frame_map_aligned_new";

    tf_prefix_ = tf::getPrefixParam(ph);
    if (!tf_prefix_.empty()) {
        ROS_INFO("Using tf_prefix '%s'", tf_prefix_.c_str());
    }

    ph.param<std::string>("vertical_laser_frame_topic", vertical_laser_frameid_, "panning_laser_frame");
    ph.param<std::string>("map_topic", map_frameid_, "map");
    ph.param<std::string>("odom_topic", odom_frameid_, "odom");
    ph.param<std::string>("body_topic", body_frameid_, "body_frame");
    ph.param<std::string>("body_map_aligned_topic", body_map_aligned_frameid_, "body_frame_map_aligned");
    ph.param<std::string>("body_stabilized_topic", body_stabilized_frameid_, "body_frame_stabilized");

    vertical_laser_frameid_ = tf::resolve(tf_prefix_, vertical_laser_frameid_);
    map_frameid_ = tf::resolve(tf_prefix_, map_frameid_);
    odom_frameid_ = tf::resolve(tf_prefix_, odom_frameid_);
    body_frameid_ = tf::resolve(tf_prefix_, body_frameid_);
    body_map_aligned_frameid_ = tf::resolve(tf_prefix_, body_map_aligned_frameid_);
    body_stabilized_frameid_ = tf::resolve(tf_prefix_, body_stabilized_frameid_);

    ph.param("min_lidar_angle", min_lidar_angle_, -3.14159);
    ph.param("max_lidar_angle", max_lidar_angle_, 3.14159);
    ph.param("height_filter_deviation_max", height_filter_deviation_max_, 0.2);

    //publish an odometry message (it's the only message with all the state
    //variables we want)

    state_pub_ = nh.advertise<nav_msgs::Odometry>(state_pub_topic_, 1);
    pointCloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(z_laser_topic_, 1);
    point_pub_ = nh.advertise<sensor_msgs::PointCloud2>(z_laser_median_topic_, 1);
    vel_pub_ = nh.advertise<geometry_msgs::PointStamped>("acel_integrated", 1);
    ac_pub_ = nh.advertise<geometry_msgs::PointStamped>("acel_trans", 1);
    slam_vel_pub_ = nh.advertise<geometry_msgs::PointStamped>("slam_vel", 1);
    rpy_pub_ = nh.advertise<geometry_msgs::PointStamped>(rpy_pub_topic_, 1);
    max_z_pub_ = nh.advertise<std_msgs::Float32>("max_z", 10);
    elv_marker_pub_ = nh.advertise<visualization_msgs::Marker>("elevation_marker", 10);

    //subscribe to the SLAM pose from hector_mapping, the EKF pose from hector_localization, and the vertical lidar
    ekf_sub_ = nh.subscribe(position_sub_topic_, 3, &UAVStatePublisher::ekfCallback, this);
    lidar_sub_ = nh.subscribe(vertical_laser_data_topic_, 3, &UAVStatePublisher::lidarCallback, this);
    slam_sub_ = nh.subscribe(slam_topic_, 3, &UAVStatePublisher::slamCallback, this);
    flight_mode_sub_ = nh.subscribe(flt_mode_stat_topic_, 1, &UAVStatePublisher::flightModeCallback, this);
    imu_sub_ = nh.subscribe(imu_topic_, 10, &UAVStatePublisher::imuCallback, this);

}

void UAVStatePublisher::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    ros::Time start_ = ros::Time::now();

    // determine gravity compensated accelerations

    // input accelerations are generally in the body frame; transform imu
    // acceleration into the map frame (note: assumes origins of both frames are
    // coincident...this should extract and apply just the rotation)
    tf::Point p(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z);

    tf::StampedTransform transform;
    try {
        tf_listener_.lookupTransform(body_map_aligned_frameid_, msg->header.frame_id, ros::Time(0), transform);
    }
    catch (const tf::TransformException& ex) {
        ROS_ERROR_THROTTLE(1.0, "Failed to transform IMU data (%s)", ex.what());
        return;
    }

    tf::Point pout = transform * p;
    const double accel_x = pout[0];
    const double accel_y = pout[1];
    const double accel_z = pout[2];

    ROS_DEBUG("accel_body %f %f %f -> accel_world %f %f %f",
            p[0], p[1], p[2], pout[0], pout[1], pout[2]);

    // add world-aligned acceleration reading
    x_integrated_.set_value(accel_x, msg->header.stamp);
    y_integrated_.set_value(accel_y, msg->header.stamp);

    if (x_integrated_.size() < 2 || y_integrated_.size() < 2) {
        return;
    }

    ROS_DEBUG("delta velocity (imu) = (%f, %f)",
            x_integrated_.get_integrated(), y_integrated_.get_integrated());

    ROS_DEBUG("velocity (etc) = (%f, %f)",
            x_velo_.get_last(), y_velo_.get_last());

    if (x_integrated_.full() && y_integrated_.full()) {
        vel_x_integrated_ += x_integrated_.get_integrated();
        vel_y_integrated_ += y_integrated_.get_integrated();
        x_integrated_.clear();
        y_integrated_.clear();

        ROS_DEBUG("velocity (imu) = (%f, %f)", vel_x_integrated_, vel_y_integrated_);
    }

    double part_sum_x = x_integrated_.get_integrated() + x_velo_.get_last();
    double part_sum_y = y_integrated_.get_integrated() + y_velo_.get_last();

    geometry_msgs::PointStamped accel_int;
    accel_int.header.stamp = start_;
    accel_int.point.x = part_sum_x;
    accel_int.point.y = part_sum_y;
    vel_pub_.publish(accel_int);

    geometry_msgs::PointStamped acc_trans;
    acc_trans.header.stamp = start_;
    acc_trans.point.x = accel_x / 40;
    acc_trans.point.y = accel_y / 40;
    acc_trans.point.z = accel_z / 40;
    ac_pub_.publish(acc_trans);
}

void UAVStatePublisher::flightModeCallback(uav_msgs::FlightModeStatusConstPtr msg)
{
    last_state_ = *msg;
    ROS_DEBUG("flight_status is %d\n", (int ) last_state_.mode);
}

void UAVStatePublisher::slamCallback(geometry_msgs::PoseStampedConstPtr slam_msg)
{
    ros::Time start_ = ros::Time::now();

    saved_yaw_ = tf::getYaw(slam_msg->pose.orientation);

    state_.pose.pose.position.x = slam_msg->pose.position.x;
    state_.pose.pose.position.y = slam_msg->pose.position.y;

    const ros::Rate vel_update_rate(10.0);
    if (vel_update_time_ + vel_update_rate.expectedCycleTime() < start_) {
        // update velocity if we have a previous pose
        if (prev_pose_x_ == prev_pose_x_ && prev_pose_y_ == prev_pose_y_) {
            const double dx = slam_msg->pose.position.x - prev_pose_x_;
            const double dy = slam_msg->pose.position.y - prev_pose_y_;
            const double dt = (start_ - vel_update_time_).toSec();
            vel_x_derived_ = dx / dt;
            vel_y_derived_ = dy / dt;
            ROS_DEBUG("velocity (slam): (%f, %f) = (%f, %f) / %f",
                    vel_x_derived_, vel_y_derived_,
                    dx, dy, dt);
        }

        // record position and time of slam measurement
        prev_pose_x_ = slam_msg->pose.position.x;
        prev_pose_y_ = slam_msg->pose.position.y;
        vel_update_time_ = start_;
    }

    ros::Time stop_ = ros::Time::now();
    ROS_DEBUG("[state_pub] slam callback %f %f = %f", start_.toSec(), stop_.toSec(), stop_.toSec() - start_.toSec());
}

// on the order of 50Hz
void UAVStatePublisher::ekfCallback(nav_msgs::OdometryConstPtr p)
{
    ros::Time start_ = ros::Time::now();

    try {
        // align odom-frame velocities with map frame (note comment in imu
        // callback)
        tf::Point vel_odom(
                p->twist.twist.linear.x,
                p->twist.twist.linear.y,
                p->twist.twist.linear.z);

        tf::StampedTransform T_map_odom;
        tf_listener_.lookupTransform(
                body_map_aligned_frameid_,
                p->header.frame_id,
                ros::Time(0),
                T_map_odom);

        tf::Point vel_map = T_map_odom * vel_odom;

        x_velo_.add_value(vel_map.x());
        y_velo_.add_value(vel_map.y());
    }
    catch (const tf::TransformException& ex) {
        ROS_ERROR_THROTTLE(1.0, "Failed to transform EKF velocity (%s)", ex.what());
    }

    if (z_fifo_.size() < 1) {
        ROS_WARN_THROTTLE(1, "State is incomplete");
        return;
    }

    ///////////////////////
    // State Aggregation //
    ///////////////////////

    // retrieve orientation from EKF
    double roll, pitch, yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(p->pose.pose.orientation, q);
    tf::Matrix3x3(q).getEulerZYX(yaw, pitch, roll);
    // Publish RPY for debugging (in degrees)
    geometry_msgs::PointStamped rpy;
    rpy.header.stamp = p->header.stamp;
    rpy.point.x = roll * 180 / M_PI;
    rpy.point.y = pitch * 180 / M_PI;
    rpy.point.z = yaw * 180 / M_PI;
    rpy_pub_.publish(rpy);

    state_.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, saved_yaw_);

    // compute x and y linear velocities
//    state_.twist.twist.linear.x = p->twist.twist.linear.x;
//    state_.twist.twist.linear.y = p->twist.twist.linear.y;

//    if (x_integrated_.size() > 1 && y_integrated_.size() > 1) {
//        // TODO: combine integrated accelerations pose deltas from SLAM to get
//        // the final linear velocities?
//        state_.twist.twist.linear.x = vel_x_integrated_;
//        state_.twist.twist.linear.y = vel_y_integrated_;
//    }

    state_.twist.twist.linear.x = vel_x_derived_;
    state_.twist.twist.linear.y = vel_y_derived_;

    // compute the linear z velocity
    double dz = 0.0;
    if (z_fifo_.size() > 0) {
        for (int i = 1; i < z_fifo_.size(); i++) {
            dz += (z_fifo_[i] - z_fifo_[i - 1]) / (z_time_fifo_[i] - z_time_fifo_[i - 1]);
        }
        dz /= z_fifo_.size();
    }
    state_.twist.twist.linear.z = dz;

    // retrieve angular velocities from EKF
    state_.twist.twist.angular = p->twist.twist.angular;

    ////////////////////////
    // Publish Transforms //
    ////////////////////////

    // publish the map -> odom transform

    geometry_msgs::TransformStamped trans;
    trans.header.stamp = p->header.stamp;
    trans.header.frame_id = map_frameid_;

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

    trans.child_frame_id = odom_frameid_;
    if(!bag_test_)
        tf_broadcaster_.sendTransform(trans);

    // publish the map -> body_frame_map_aligned transform
    trans.child_frame_id = body_map_aligned_frameid_;
    trans.transform.rotation.w = 1.0;
    trans.transform.rotation.x = 0.0;
    trans.transform.rotation.y = 0.0;
    trans.transform.rotation.z = 0.0;
    tf_broadcaster_.sendTransform(trans);

    // publish the state
    state_.header.stamp = ros::Time::now();
    state_pub_.publish(state_);
    ros::Time stop_ = ros::Time::now();
    ROS_DEBUG("[state_pub] ekf callback %f %f = %f", start_.toSec(), stop_.toSec(), stop_.toSec() - start_.toSec());
}

//on the order of 40Hz
void UAVStatePublisher::lidarCallback(sensor_msgs::LaserScanConstPtr scan)
{

    static int init_est_count = 0;

    const ros::Time now = ros::Time::now();

    // log scan count over the last period
    const ros::Duration scan_analysis_period(1.0);
    if (now > last_scan_analysis_time_ + scan_analysis_period) {
        // log how many scans we've processed in the last period
        const ros::Duration span = (now - last_scan_analysis_time_);
        ROS_DEBUG("Processed %d scans in the last %0.3f seconds (%0.3f Hz)", num_scans_processed_, span.toSec(), (double )num_scans_processed_ / (span.toSec()));
        num_scans_processed_ = 0;
        last_scan_analysis_time_ = now;
    }

    ++num_scans_processed_;
    const ros::Duration dt = now - prev_scan_time_;

    double prev_height = state_.pose.pose.position.z;

    /////////////////////////////////////////////////
    // estimate initial height on first invocation //
    /////////////////////////////////////////////////

    if (z_fifo_.size() < 5) {
        double est_height;
        bool suc = estimateInitialHeight(scan, init_est[init_est_count]);
        init_est_count++;
        if (!suc) {
            return;
        }
        std::sort(init_est.begin(), init_est.end());
        est_height = *std::max_element(init_est.begin(), init_est.end());
        ROS_INFO("uav_state_publisher: estimated initial height is %f", est_height);
        state_.pose.pose.position.z = est_height;
        filtered_z_ = est_height;
        prev_filtered_z_ = est_height;
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
        tf_listener_.lookupTransform(body_stabilized_frameid_, scan->header.frame_id, ros::Time(0), T_bodystable_vlaser);

        have_bodymapaligned_to_laser = true;
    }
    catch (const tf::TransformException& ex) {
        ROS_ERROR("Failed to lookup transform (%s)", ex.what());
    }

    // then try the two intermediate transforms
    tf::StampedTransform T_bodymapaligned_body;
    try {
        tf_listener_.lookupTransform(body_stabilized_frameid_, body_frameid_, ros::Time(0), T_bodymapaligned_body);

        have_bodymapaligned_to_body = true;

        tf_listener_.lookupTransform(body_frameid_, vertical_laser_frameid_, ros::Time(0), T_body_vlaser_);

        have_body_to_laser = true;
    }
    catch (const tf::TransformException& ex) {
        ROS_ERROR("Look of partial transforms failed (%s)", ex.what());
    }

    // ensure we can transform the laser scan into the map aligned frame
    if (!have_bodymapaligned_to_laser && !(have_bodymapaligned_to_body && have_body_to_laser)) {
        ROS_ERROR("Failed to transform laser scan into z-aligned frame");
        return;
    }

    // // vertical laser to map transform
    // tf::StampedTransform T_map_vlaser;
    // tf::StampedTransform T_odom_base_footprint;
    // tf::StampedTransform T_base_footprint_body_stabilized;
    // try {

    //     tf_listener_.lookupTransform(odom_frameid_, base_footprint_topic_, ros::Time(0), T_odom_base_footprint);

    //     tf_listener_.lookupTransform(base_footprint_topic_, body_stabilized_frameid_, ros::Time(0), T_base_footprint_body_stabilized);

    //     have_map_to_vlaser = true;
    // }
    // catch (const tf::TransformException& ex) {
    //     ROS_ERROR("Look of partial transforms failed (%s)", ex.what());
    // }

    //Current state vector points
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> zs;
    std::vector<double> zero_points;
    std::vector<double> not_zero_points;
    std::vector<double> filt_vect;

    xs.reserve(scan->ranges.size());
    ys.reserve(scan->ranges.size());
    zs.reserve(scan->ranges.size());

    //Elevation map vector points
    std::vector<double> xel;
    std::vector<double> yel;
    std::vector<double> zel;

    xel.reserve(scan->ranges.size());
    yel.reserve(scan->ranges.size());
    zel.reserve(scan->ranges.size());

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
        if (scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max) {
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
        pout.header.frame_id = body_stabilized_frameid_;

        tf::Point ptf(pin.point.x, pin.point.y, pin.point.z);
        if (have_bodymapaligned_to_laser) {
            tf::Point ptfout(T_bodystable_vlaser * ptf);
            pout.point.x = ptfout.getX();
            pout.point.y = ptfout.getY();
            pout.point.z = ptfout.getZ();
        }
        else if (have_bodymapaligned_to_body && have_body_to_laser) {
            tf::Point ptfout(T_bodymapaligned_body * T_body_vlaser_ * ptf);
            pout.point.x = ptfout.getX();
            pout.point.y = ptfout.getY();
            pout.point.z = ptfout.getZ();
        }
        else {
            ROS_ERROR("Why am I here?");
        }

        //transform all scans to map frame
        geometry_msgs::PointStamped el_pout;
        el_pout.header.seq = 0;
        el_pout.header.stamp = ros::Time(0);
        el_pout.header.frame_id = map_frameid_;

        //if (have_map_to_vlaser) {

            // tf::Point scanout;

            // if (have_bodymapaligned_to_laser){
            //     scanout = T_map_odom_ * T_odom_base_footprint * T_base_footprint_body_stabilized * T_bodystable_vlaser * ptf;
            // }else{
            //     scanout = T_map_odom_ * T_odom_base_footprint * T_base_footprint_body_stabilized * T_bodymapaligned_body * T_body_vlaser_ * ptf;
            // }
            // el_pout.point.x = scanout.getX();
            // el_pout.point.y = scanout.getY();
            // el_pout.point.z = scanout.getZ();
        //}
        //else
        //    return;

        // laser point in body stabilized frame -> body position in map frame
        pout.point.z = -pout.point.z;

        // update max_z; only calculate max_z using rays "directly under the
        // hexacopter"
        if (fabs(ang - M_PI / 2) < (65.0 * M_PI / 180.0)) {
            max_z = std::max(max_z, pout.point.z);
        }

        //Push back points for current state
        if (pout.point.z > 0)
        {
            //Push back points for elevation map
            xel.push_back(pout.point.x);
            yel.push_back(pout.point.y);
            zel.push_back(pout.point.z);

            //if(fabs(ang - M_PI / 2) < (10.0 * M_PI / 180.0))
            //if (pout.point.z < prev_height + 0.5 && pout.point.z > prev_height - 0.5)
            {
                xs.push_back(pout.point.x);
                ys.push_back(pout.point.y);
                zs.push_back(pout.point.z);
                voxels.push_back(pout.point);
            }

        }
        else
            continue;

    }

    const double height_cut_off = 2.0; // TODO: configurate
    max_z = std::min(max_z, height_cut_off);

    // TODO: do something smarter that will filter out tables

    // publish max_z for debugging
    std_msgs::Float32 max_z_msg;
    max_z_msg.data = max_z;
    max_z_pub_.publish(max_z_msg);

    // publish point cloud of considered ray points
    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    for (size_t i = 0; i < voxels.size(); i++) {
        pclCloud.push_back(pcl::PointXYZ(voxels[i].x, voxels[i].y, voxels[i].z));
    }
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg(pclCloud, cloud);
    cloud.header.frame_id = body_map_aligned_frameid_;
    cloud.header.stamp = ros::Time::now();
    pointCloud_pub_.publish(cloud);

    // Only estimate height afterwards when robot is in air
    if (last_state_.mode != uav_msgs::FlightModeStatus::LANDED) {
        // set height to previous filtered height
        state_.pose.pose.position.z = prev_filtered_z_;

        //elevation map update set to true
        update_map_ = true;

        // get z by taking the median
        if (zs.size() > 6) {

            for(size_t i = 0; i < xs.size(); i++)
            {
                //state to chck in map
                check_x_ = (int)((state_.pose.pose.position.x + xs[i])/scale_)*scale_;
                check_y_ = (int)((state_.pose.pose.position.y + ys[i])/scale_)*scale_;

                //current_elevation_ = prev_elevation_;

                auto search = elevation_map_.find(make_pair(check_x_, check_y_));

                if(search != elevation_map_.end())
                    current_elevation_ = (search->second);
                else
                    continue;

                current_zs_ = zs[i];

                if(current_elevation_ == 0.0)
                    zero_points.push_back(current_zs_);

                not_zero_points.push_back(current_zs_ + current_elevation_);

            }

            std::sort(zero_points.begin(), zero_points.end());
            std::sort(not_zero_points.begin(), not_zero_points.end());

            if(zero_points.size() > 0)
                filtered_z_ = zero_points[zero_points.size()/2];
            else if(not_zero_points.size() > 0)
                filtered_z_ = not_zero_points[not_zero_points.size()/2];
            else{
                filtered_z_ = prev_filtered_z_;
                update_map_ = false;
            }
            //printf("the actual elevation is %f, actual zs is %f and height calculated is %f\n", current_elevation, current_zs, m_filtered_z);

            //The map lookup might not be exact at borders for obtacles and hence we must look for some edge cases
            if(filtered_z_ > (prev_filtered_z_ + height_filter_deviation_max_) || filtered_z_ < (prev_filtered_z_ - height_filter_deviation_max_) )
            {
                    filtered_z_ = prev_filtered_z_;
                    update_map_ = false;
            }

            state_.pose.pose.position.z = filtered_z_;

            //printf("The current elevation is %f, current zs is %f and total height is %f\n", current_elevation, current_zs, m_filtered_z);

            prev_zs_ = current_zs_;
            prev_elevation_ = current_elevation_;
        }

        prev_filtered_z_ = filtered_z_;
    }
    else {
        ROS_DEBUG("landed");
    }

    // //clear elevation map after regular intervals to
    // //account for segbot movements(roughly 0.1s)
    // if(elevation_map_.size() > 2000 && update_map_)
    // {
    //     elevation_map_.clear();
    // }

    // Dont update elevation map when landing
    //TODO(Karthik) : Replace const 1 below with actual message for while landing
    if (last_state_.mode != 1 && update_map_) {

        double elz = 0;

        //Update elevation Map
        for(size_t i = 0; i < zel.size(); i++)
        {
            double approx_x, approx_y;

            elz = (filtered_z_ - zel[i]);

            approx_x = (int)((state_.pose.pose.position.x + xel[i])/scale_)*scale_;
            approx_y = (int)((state_.pose.pose.position.y + yel[i])/scale_)*scale_;

            //Zero out points below certain height to avoid ground creeping up
            if (elz < elevation_noise_)
                elz = 0;

            key_ k (approx_x, approx_y);

            if(elz >= 0 && elz <= 0.72){
                if ( (elevation_map_.find(k) == elevation_map_.end()) || (elz - elevation_map_[k] > elevation_noise_))
                {
                    elevation_map_[k] = elz;
                    //printf("x = %f y = %f the filtered_z is %f, zel is %f and elz is %f\n", approx_x, approx_y, filtered_z_, zel[i], elz);
                    //printf("Updated map at x = %f y = %f with elevation = %f\n", approx_x, approx_y, elevation_map_[k]);
                }

                //Elevation map marker(Not functional)
                visualization_msgs::Marker elv_marker;
                elv_marker.header.frame_id = map_frameid_;
                elv_marker.header.stamp = ros::Time();
                elv_marker.id = rand();
                elv_marker.type = visualization_msgs::Marker::CUBE;
                elv_marker.pose.position.x = approx_x;
                elv_marker.pose.position.y = approx_y;
                elv_marker.pose.position.z = elevation_map_[k];
                elv_marker.pose.orientation.x = 0.0;
                elv_marker.pose.orientation.y = 0.0;
                elv_marker.pose.orientation.z = 0.0;
                elv_marker.pose.orientation.w = 1.0;
                elv_marker.color.b = 1.0;
                elv_marker.color.a = 1.0;
                elv_marker.scale.x = 0.01;
                elv_marker.scale.y = 0.01;
                elv_marker.scale.z = 0.01;
                elv_marker.lifetime = ros::Duration();

                elv_marker_pub_.publish(elv_marker);

            }
        }
    }

    z_fifo_.insert(state_.pose.pose.position.z);
    z_time_fifo_.insert(scan->header.stamp.toSec());

    // publish the position as a point cloud for visualization
    pcl::PointCloud<pcl::PointXYZ> pclmedianpt;
    pclmedianpt.push_back(pcl::PointXYZ(state_.pose.pose.position.x, state_.pose.pose.position.y, state_.pose.pose.position.z));
    sensor_msgs::PointCloud2 medianpt;
    pcl::toROSMsg(pclmedianpt, medianpt);
    medianpt.header.frame_id = map_frameid_;
    medianpt.header.stamp = ros::Time::now();
    point_pub_.publish(medianpt);

    double ds = abs(state_.pose.pose.position.z - prev_height);
    ROS_DEBUG_THROTTLE(1, "height: %0.3f, maxz: %0.3f filteredz %f with dt %f ds %f time %f", state_.pose.pose.position.z, max_z, filtered_z_, dt.toSec(), ds, now.toSec());

    prev_scan_time_ = now;
    ros::Time stop_ = ros::Time::now();
    ROS_DEBUG("[state_pub] lidar callback %f %f = %f", now.toSec(), stop_.toSec(), stop_.toSec() - now.toSec());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_state_publisher_new");
    UAVStatePublisher sp;
    ros::Rate loop_rate(80);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

bool UAVStatePublisher::estimateInitialHeight(sensor_msgs::LaserScanConstPtr scan, double& ret_height)
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
        tf_listener_.lookupTransform(body_stabilized_frameid_, vertical_laser_frameid_, ros::Time(0), T_bodystable_vlaser);
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

UAVStatePublisher::FIFO::FIFO(int s) :
    size_(s),
    head_(0),
    tail_(0),
    elements_(0),
    q_(nullptr)
{
    q_ = new double[s];
}

UAVStatePublisher::FIFO::~FIFO()
{
    delete[] q_;
}

void UAVStatePublisher::FIFO::insert(double x)
{
    *(q_ + head_) = x;
    head_++;
    if (head_ == size_) {
        head_ = 0;
    }
    if (elements_ < size_) {
        elements_++;
    }
}

double UAVStatePublisher::FIFO::operator[](int i)
{
    int j = tail_ + i;
    if (j >= size_) {
        j -= size_;
    }
    return *(q_ + j);
}

UAVStatePublisher::velo_list::velo_list(int s) :
    my_list(),
    count(0),
    size(s)
{
}

void UAVStatePublisher::velo_list::add_value(double val)
{
    if (count == size) {
        my_list.pop_front();
    }
    else {
        count++;
    }
    my_list.push_back(val);
}

UAVStatePublisher::integrated_accel::integrated_accel(int s) :
    m_list(),
    m_size(s),
    m_count(0),
    m_offset_set(false),
    m_offset(0)
{
}

void UAVStatePublisher::integrated_accel::set_value(double val, ros::Time time)
{
    if (!m_offset_set) {
        m_offset_set = true;
        m_offset = val;
    }

    if (m_count == m_size) {
        m_list.pop_front();
    }
    else {
        m_count++;
    }
    reading r;
    r.value = val - m_offset;
    r.time = time;
    m_list.push_back(r);
}

double UAVStatePublisher::integrated_accel::get_timespan() const
{
    if (m_list.empty()) {
        return 0;
    }
    else {
        return m_list.back().time.toSec() - m_list.front().time.toSec();
    }
}

double UAVStatePublisher::integrated_accel::get_integrated()
{
    double s = 0;
    int index = 0;
    reading pr;
    for (auto it = m_list.begin(); it != m_list.end(); ++it) {
        if (index != 0) {
            reading r = *it;
            ros::Duration dt = r.time - pr.time;
            double val = (pr.value + r.value) / 2;
            s += (val * dt.toSec());
        }
        pr = *it;
        index++;
    }
    return s;
}

void UAVStatePublisher::integrated_accel::clear()
{
    m_count = 0;
    m_list.clear();
    // NOTE: should this clear the offset value too?
}
