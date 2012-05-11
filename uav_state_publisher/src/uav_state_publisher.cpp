#include<uav_state_publisher/uav_state_publisher.h>

UAVStatePublisher::UAVStatePublisher(){
  ros::NodeHandle nh;

  //publish an odometry message (it's the only message with all the state variables we want)
  state_pub_ = nh.advertise<nav_msgs::Odometry>("uav_state", 1);

  
  //subscribe to the SLAM pose from hector_mapping, the EKF pose from hector_localization, and the vertical lidar
  slam_sub_ = nh.subscribe("slam_out_pose", 1, &UAVStatePublisher::slamCallback,this);
  ekf_sub_ = nh.subscribe("ekf_state", 1, &UAVStatePublisher::ekfCallback,this);
  lidar_sub_ = nh.subscribe("pan_scan", 1, &UAVStatePublisher::lidarCallback,this);
}

void UAVStatePublisher::publishState(){

}
//5 x from slam, 4 dx, avg, assign the velocity to the 3rd x
//store dx from ekf, take difference between the corresponding time above and most recent

void UAVStatePublisher::slamCallback(geometry_msgs::PoseStampedConstPtr p){
  //get the x,y
  state_.pose.position.x = p->pose.position.x;
  state_.pose.position.y = p->pose.position.y;
  x_fifo_.insert(state_.pose.position.x);
  y_fifo_.insert(state_.pose.position.y);
  xy_time_fifo_.insert(p->header.stamp.toSec());

  //get the yaw
  btQuaternion q;
  tf::quaternionMsgToTF(state_.pose.orientation, q);
  double roll, pitch, yaw, dummy1, dummy2;
  btMatrix3x3(q).getEulerZYX(dummy1, pitch, roll);

  tf::quaternionMsgToTF(p->pose.orientation, q);
  btMatrix3x3(q).getEulerZYX(yaw, dummy1, dummy2);

  q.setEulerZYX(yaw,pitch,roll);
  state_.pose.orientation.x = q.getX();
  state_.pose.orientation.y = q.getY();
  state_.pose.orientation.z = q.getZ();
  state_.pose.orientation.w = q.getW();
}

void UAVStatePublisher::ekfCallback(nav_msgs::OdometryConstPtr p){
  //get angular velocities
  state_.twist.twist.angular = p->twist.twist.angular;

  //get the roll and pitch
  btQuaternion q;
  tf::quaternionMsgToTF(state_.pose.orientation, q);
  double roll, pitch, yaw, dummy1, dummy2;
  btMatrix3x3(q).getEulerZYX(yaw, dummy1, dummy2);

  tf::quaternionMsgToTF(p->pose.orientation, q);
  btMatrix3x3(q).getEulerZYX(dummy1, pitch, roll);

  q.setEulerZYX(yaw,pitch,roll);
  state_.pose.orientation.x = q.getX();
  state_.pose.orientation.y = q.getY();
  state_.pose.orientation.z = q.getZ();
  state_.pose.orientation.w = q.getW();

  //compute dx, dy, and dz

  //publish the state
  state_pub_.publish(state_);
}

void UAVStatePublisher::lidarCallback(sensor_msgs::LaserScanConstPtr scan){
  //get z
  state_.pose.position.z = 
  z_fifo_.insert(state_.pose.position.z);
  z_time_fifo_.insert(scan->header.stamp.toSec());
}

int main(int argc, char **argv){
  ros::init(argc, argv, "uav_state_publisher");
  UAVStatePublisher sp;

  ros::spin();

  return 0;
}
