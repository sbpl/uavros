#include <uav_state_publisher/uav_state_publisher.h>

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
    body_map_aligned_frameid_("body_frame_map_aligned"),
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
    x_velo_(41),
    y_velo_(41),
    last_state_(),
    T_body_vlaser_(),
    last_scan_analysis_time_(),
    prev_scan_time_(),
    num_scans_processed_(0),
    filtered_z_(0.0),
    prev_filtered_z_(0.0)
{
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    // Lookup tf_prefix and resolve frame ids

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
        ROS_ERROR("Failed to transform IMU data (%s)", ex.what());
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

    ROS_DEBUG("velocity (imu) = (%f, %f)",
            x_integrated_.get_integrated(), y_integrated_.get_integrated());
    ROS_DEBUG("velocity (etc) = (%f, %f)",
            x_velo_.get_last(), y_velo_.get_last());

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
    // printf("############################################got the slam stuff....\n");
    ros::Time stop_ = ros::Time::now();
    ROS_DEBUG("[state_pub] slam callback %f %f = %f", start_.toSec(), stop_.toSec(), stop_.toSec() - start_.toSec());
}

// on the order of 50Hz
void UAVStatePublisher::ekfCallback(nav_msgs::OdometryConstPtr p)
{
    ros::Time start_ = ros::Time::now();

    try {
        // align odom-frame velocities with map frame
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
        ROS_ERROR("Failed to transform EKF velocity (%s)", ex.what());
    }

    if (x_integrated_.size() > 1 && y_integrated_.size() > 1) {
        // TODO: combine integrated accelerations and ekf velocities to form
        // world frame velocity
    }

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
    trans.header.frame_id = map_frameid_;
    trans.transform.translation.x = state_.pose.pose.position.x;
    trans.transform.translation.y = state_.pose.pose.position.y;
    trans.transform.translation.z = state_.pose.pose.position.z;

    // publish the map -> odom transform

    const tf::Transform T_odom_body(
            tf::Quaternion(p->pose.pose.orientation.x, p->pose.pose.orientation.y, p->pose.pose.orientation.z,
                    p->pose.pose.orientation.w),
            tf::Vector3(p->pose.pose.position.x, p->pose.pose.position.y, p->pose.pose.position.z));
    const tf::Transform T_map_body(
            tf::Quaternion(state_.pose.pose.orientation.x, state_.pose.pose.orientation.y,
                    state_.pose.pose.orientation.z, state_.pose.pose.orientation.w),
            tf::Vector3(state_.pose.pose.position.x, state_.pose.pose.position.y, state_.pose.pose.position.z));

    const tf::Transform T_map_odom = T_map_body * T_odom_body.inverse();
    tf::transformTFToMsg(T_map_odom, trans.transform);

    //ROS_WARN("height is %f\n", trans.transform.translation.z);
    trans.child_frame_id = odom_frameid_;
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

    if (z_fifo_.size() < 1) {
        double est_height;
        bool suc = estimateInitialHeight(scan, est_height);
        if (!suc) {
            return;
        }
        ROS_INFO("uav_state_publisher: estimated initial height is %f", est_height);
        state_.pose.pose.position.z = est_height;
        filtered_z_ = est_height;
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

        // laser point in body stabilized frame -> body position in map frame
        pout.point.z = -pout.point.z;

        // update max_z; only calculate max_z using rays "directly under the
        // hexacopter"
        if (fabs(ang - M_PI / 2) < (10.0 * M_PI / 180.0)) {
            max_z = std::max(max_z, pout.point.z);
        }

        // only accept height estimates that are close to previous height
        if ((pout.point.z < filtered_z_ + height_filter_deviation_max_) &&
            (pout.point.z > filtered_z_ - height_filter_deviation_max_))
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

        // get z by taking the median
        if (zs.size() > 6) {
            std::sort(zs.begin(), zs.end());
            filtered_z_ = zs[zs.size() / 2];
            state_.pose.pose.position.z = filtered_z_;
        }

        if (height_cut_off <= max_z) {
            // take max of filtered and low bigger than cutout height
            // state_.pose.pose.position.z = max(filtered_z_, max_z);
            const double alpha = 0.9;
            state_.pose.pose.position.z = alpha * filtered_z_ + (1.0 - alpha) * max_z;
        }

        // update filtered z with weighted average
        const double beta = 0.99;

        // arbitrary number, possibly related to takeoff height, and something
        // about
        const double SOMETHING = 1.2;
        if (max_z > SOMETHING) {
            filtered_z_ = (beta * filtered_z_) + ((1.0 - beta) * max_z);
        }
        prev_filtered_z_ = filtered_z_;
    }
    else {
        ROS_DEBUG("landed");
    }

    z_fifo_.insert(state_.pose.pose.position.z);
    z_time_fifo_.insert(scan->header.stamp.toSec());

    // publish the position as a point cloud for visualization
    pcl::PointCloud<pcl::PointXYZ> pclmedianpt;
    pclmedianpt.push_back(
            pcl::PointXYZ(state_.pose.pose.position.x, state_.pose.pose.position.y, state_.pose.pose.position.z));
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_state_publisher");
    UAVStatePublisher sp;
    ros::Rate loop_rate(80);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
