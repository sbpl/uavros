#include <uav_state_publisher/uav_state_publisher.h>
#include <vector>


using namespace std;

UAVStatePublisher::UAVStatePublisher()
: tf_(ros::NodeHandle(), ros::Duration(10), true),z_fifo_(5),z_time_fifo_(5) {
  ros::NodeHandle nh;
  ros::NodeHandle ph;

  ph.param("min_lidar_angle",min_lidar_angle_,80.0*M_PI/180.0);
  ph.param("max_lidar_angle",max_lidar_angle_,100.0*M_PI/180.0);

  //publish an odometry message (it's the only message with all the state variables we want)
  state_pub_ = nh.advertise<nav_msgs::Odometry>("uav_state", 1);
  pointCloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("HeightLaser", 1);
  point_pub_ = nh.advertise<sensor_msgs::PointCloud2>("HeightLaserMedian", 1);

  //subscribe to the SLAM pose from hector_mapping, the EKF pose from hector_localization, and the vertical lidar
  ekf_sub_ = nh.subscribe("ekf_state", 3, &UAVStatePublisher::ekfCallback,this);
  lidar_sub_ = nh.subscribe("pan_scan", 1, &UAVStatePublisher::lidarCallback,this);
  slam_sub_ = nh.subscribe("slam_out_pose", 3, &UAVStatePublisher::slamCallback,this);


}

void UAVStatePublisher::slamCallback(geometry_msgs::PoseStampedConstPtr slam_msg) {
  saved_yaw_ = tf::getYaw(slam_msg->pose.orientation);

  state_.pose.pose.position.x = slam_msg->pose.position.x;
  state_.pose.pose.position.y = slam_msg->pose.position.y;
  printf("############################################got the slam stuff....\n");
}



//on the order of 50Hz
void UAVStatePublisher::ekfCallback(nav_msgs::OdometryConstPtr p){
  //get angular velocities
  state_.twist.twist.angular = p->twist.twist.angular;

  //get the orientation
double roll, pitch, yaw;
btQuaternion q;
tf::quaternionMsgToTF(p->pose.pose.orientation, q);
btMatrix3x3(q).getEulerZYX(yaw, pitch, roll);
state_.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, saved_yaw_);

  //get the x and y
 // state_.pose.pose.position.x = p->pose.pose.position.x;
 // state_.pose.pose.position.y = p->pose.pose.position.y;

  //get the x and y velocities
  state_.twist.twist.linear.x = p->twist.twist.linear.x;
  state_.twist.twist.linear.y = p->twist.twist.linear.y;

  //TODO: do a smarter computation of linear velocities using slam and the ekf
  //      5 x from slam, 4 dx, avg, assign the velocity to the 3rd x
  //      store dx from ekf, take difference between the corresponding time above and most recent
  //compute dx, dy, and dz
  double dz = 0;
  for(int i=1; i<z_fifo_.size(); i++){
    dz += (z_fifo_[i]-z_fifo_[i-1])/(z_time_fifo_[i]-z_time_fifo_[i-1]);
  }
  dz /= z_fifo_.size();
  state_.twist.twist.linear.z = dz;

  //publish the map to base_link transform
  geometry_msgs::TransformStamped trans;
  trans.header.stamp = p->header.stamp;
  trans.header.frame_id = "map";
  trans.transform.translation.x = state_.pose.pose.position.x;
  trans.transform.translation.y = state_.pose.pose.position.y;
  trans.transform.translation.z = state_.pose.pose.position.z;
  //ROS_ERROR("pose is %f %f %f\n", state_.pose.pose.position.x, state_.pose.pose.position.y, state_.pose.pose.position.z);

  trans.child_frame_id = "body_frame_map_aligned";
  trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
  tf_broadcaster.sendTransform(trans);

  trans.child_frame_id = "body_frame_stabilized";
  trans.transform.rotation = tf::createQuaternionMsgFromYaw(saved_yaw_);
  tf_broadcaster.sendTransform(trans);

  //ROS_WARN("height is %f\n", trans.transform.translation.z);
  trans.child_frame_id = "body_frame";
  trans.transform.rotation = state_.pose.pose.orientation;
  tf_broadcaster.sendTransform(trans);
  //publish the state
  state_.header.stamp = ros::Time::now();
  state_pub_.publish(state_);
}

//on the order of 40Hz
void UAVStatePublisher::lidarCallback(sensor_msgs::LaserScanConstPtr scan){
  //transform the scan into the map frame (according to tf)
  vector<double> zs;
  int num_rays = scan->ranges.size();
  zs.reserve(num_rays);
  float ang = scan->angle_min;
  int i;
  vector<geometry_msgs::Point> voxels;

  for(i=0; i<num_rays; i++){
    if(ang>=min_lidar_angle_)
      break;
    ang += scan->angle_increment;
  }
  for(; i<num_rays; i++){
    if(ang>max_lidar_angle_)
      break;
    if(scan->ranges[i]<scan->range_min || scan->ranges[i]>scan->range_max){
      ang += scan->angle_increment;
      continue;
    }
    geometry_msgs::PointStamped p;
    geometry_msgs::PointStamped pout;
    p.header.stamp = ros::Time(0);
    p.header.frame_id = scan->header.frame_id;
    p.point.x = scan->ranges[i]*cos(ang);
    p.point.y = scan->ranges[i]*sin(ang);
    p.point.z = 0;
    try{
      tf_.transformPoint("/body_frame_map_aligned", p, pout);  // TODO: make all this hard coded crap into parameters
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    zs.push_back(pout.point.z);
    voxels.push_back(pout.point);
    ang += scan->angle_increment;
  }
  // ROS_ERROR("size: %d first: %f\n", zs.size(),zs[0]);
  //TODO: do something smarter that will filter out tables
  //get z by taking the median

  pcl::PointCloud<pcl::PointXYZ> pclCloud;
  for(unsigned int i=0; i<voxels.size(); i++)
    pclCloud.push_back(pcl::PointXYZ(voxels[i].x, voxels[i].y, voxels[i].z));
  sensor_msgs::PointCloud2 cloud;
  pcl::toROSMsg (pclCloud, cloud);
  cloud.header.frame_id = "/body_frame_map_aligned";
  cloud.header.stamp = ros::Time::now();
  pointCloud_pub_.publish(cloud);

  sort(zs.begin(),zs.end());
  state_.pose.pose.position.z = -zs[zs.size()/2];
  //ROS_ERROR("LC z: %f\n", state_.pose.pose.position.z);
  z_fifo_.insert(state_.pose.pose.position.z);
  z_time_fifo_.insert(scan->header.stamp.toSec());

  pcl::PointCloud<pcl::PointXYZ> pclmedianpt;
  pclmedianpt.push_back(pcl::PointXYZ(state_.pose.pose.position.x, state_.pose.pose.position.y, state_.pose.pose.position.z));
  sensor_msgs::PointCloud2 medianpt;
  pcl::toROSMsg(pclmedianpt, medianpt);

  medianpt.header.frame_id = "/map";
  medianpt.header.stamp = ros::Time::now();

  point_pub_.publish(medianpt);

}

int main(int argc, char **argv){
  ros::init(argc, argv, "uav_state_publisher");
  UAVStatePublisher sp;

  ros::spin();

  return 0;
}
