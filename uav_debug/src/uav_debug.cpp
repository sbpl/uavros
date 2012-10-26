#include <uav_debug/uav_debug.h>
#include <vector>


using namespace std;

UAVDebug::UAVDebug()
: tf_(ros::NodeHandle(), ros::Duration(10), true),z_fifo_(5),z_time_fifo_(5) {
  ros::NodeHandle nh;
  ros::NodeHandle ph;

  ph.param<std::string>("state_pub_topic",state_pub_topic_,"uav_state");
  ph.param<std::string>("Z_laser_topic",z_laser_topic_,"HeightLaser");
  ph.param<std::string>("Z_laser_median_topic", z_laser_median_topic_,"HeightLaserMedian");
  ph.param<std::string>("position_sub_topic",position_sub_topic_,"ekf_state");
  ph.param<std::string>("vertical_laser_data_topic",vertical_laser_data_topic_,"panning_laser");
  ph.param<std::string>("vertical_laser_frame_topic",vertical_laser_frame_topic_,"/panning_laser_frame");
  ph.param<std::string>("slam_topic",slam_topic_,"slam_out_pose");
  ph.param<std::string>("map_topic",map_topic_,"/map");
  ph.param<std::string>("body_topic",body_topic_,"/body_frame");
  ph.param<std::string>("body_map_aligned_topic",body_map_aligned_topic_,"/body_frame_map_aligned");
  ph.param<std::string>("body_stabilized_topic",body_stabilized_topic_,"/body_frame_stabilized");
  ph.param<std::string>("imu_topic",imu_topic_,"/raw_imu");

  vel_pub_ = nh.advertise<geometry_msgs::PointStamped>("/acel_integrated",1);
  ac_pub_ = nh.advertise<geometry_msgs::PointStamped>("/acel_trans",1);
  slam_vel_pub_ = nh.advertise<geometry_msgs::PointStamped>("/slam_vel",1);
  last_vel_pub_ = nh.advertise<geometry_msgs::PointStamped>("/last_vel",1);
  slam_acc_pub_ = nh.advertise<geometry_msgs::PointStamped>("/slam_acc",1);

  ekf_sub_ = nh.subscribe(position_sub_topic_, 3, &UAVDebug::ekfCallback,this);
  imu_sub_ = nh.subscribe(imu_topic_, 1, &UAVDebug::rawImuCallback,this);
  slam_sub_ = nh.subscribe(slam_topic_, 3, &UAVDebug::slamCallback,this);
  
  x_integrated_ = new integrated_accel(40);
  y_integrated_ = new integrated_accel(40);
  x_velo_ = new velo_list(41);
  y_velo_ = new velo_list(41);

}

void UAVDebug::rawImuCallback(sensor_msgs::Imu imu)
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
  
  ROS_ERROR("VALUES X %f Y %f", imu.linear_acceleration.x, imu.linear_acceleration.y);
  
  x_integrated_->set_value(accel_x,imu.header.stamp);
  double part_sum_x = x_integrated_->get_list_sum() + x_velo_->get_last();
  y_integrated_->set_value(accel_y,imu.header.stamp);
  double part_sum_y = y_integrated_->get_list_sum() + y_velo_->get_last();
  
  ROS_ERROR("VELO X %f Y %f\n", x_velo_->get_last(), y_velo_->get_last());
  ROS_ERROR("X_SUM %f Y_SUM %f", part_sum_x, part_sum_y);
  
  geometry_msgs::PointStamped accel_int;
  accel_int.header.stamp = imu.header.stamp;
  accel_int.point.x = part_sum_x;
  accel_int.point.y = part_sum_y;
  vel_pub_.publish(accel_int);
  
  geometry_msgs::PointStamped acc_trans;
  acc_trans.header.stamp = imu.header.stamp;
  acc_trans.point.x = accel_x/40;
  acc_trans.point.y = accel_y/40;
  acc_trans.point.z = accel_z/40;
  ac_pub_.publish(acc_trans);
  
  geometry_msgs::PointStamped last_vel;
  last_vel.header.stamp = imu.header.stamp;
  last_vel.point.x = x_velo_->get_last();
  last_vel.point.y = y_velo_->get_last();
  last_vel_pub_.publish(last_vel);
  
  

}

void UAVDebug::slamCallback(geometry_msgs::PoseStampedConstPtr slam_msg) {
  ros::Time start_ = ros::Time::now();

  // current position
  current_slam_pos_.header.stamp = slam_msg->header.stamp;
  current_slam_pos_.point.x = slam_msg->pose.position.x;
  current_slam_pos_.point.y = slam_msg->pose.position.y;
  
  // current velocity
  current_slam_vel_.header.stamp = current_slam_pos_.header.stamp;
  ros::Duration p_dt = current_slam_pos_.header.stamp -  last_slam_pos_.header.stamp;
  double p_dx = current_slam_pos_.point.x - last_slam_pos_.point.x;
  double p_dy = current_slam_pos_.point.y - last_slam_pos_.point.y;
  current_slam_vel_.point.x = p_dx / p_dt.toSec();
  current_slam_vel_.point.y = p_dy / p_dt.toSec();
  
  //ROS_ERROR("\nslam pos %f %f ds %f dt %f val %f\n", current_slam_pos_.point.x,last_slam_pos_.point.x,p_dx,p_dt.toSec(),p_dx / p_dt.toSec());
  
  // acceleration
  current_slam_acc_.header.stamp = current_slam_vel_.header.stamp;
  ros::Duration v_dt = current_slam_vel_.header.stamp -  last_slam_vel_.header.stamp;
  double v_dx = current_slam_vel_.point.x - last_slam_vel_.point.x;
  double v_dy = current_slam_vel_.point.y - last_slam_vel_.point.y;
  current_slam_acc_.point.x = (v_dx / v_dt.toSec())/40;
  current_slam_acc_.point.y = (v_dy / v_dt.toSec())/40;
  
  // last position
  last_slam_pos_.header.stamp = current_slam_pos_.header.stamp;
  last_slam_pos_.point.x = current_slam_pos_.point.x;
  last_slam_pos_.point.y = current_slam_pos_.point.y;
  
  // last velocity
  last_slam_vel_.header.stamp = current_slam_vel_.header.stamp;
  last_slam_vel_.point.x = current_slam_vel_.point.x;
  last_slam_vel_.point.y = current_slam_vel_.point.y;
  
  slam_vel_pub_.publish(current_slam_vel_);
  slam_acc_pub_.publish(current_slam_acc_);
  
  
  // printf("############################################got the slam stuff....\n");
  
  ros::Time stop_ = ros::Time::now();
  ROS_DEBUG("[state_pub] slam callback %f %f = %f", start_.toSec(), stop_.toSec(), stop_.toSec()-start_.toSec() );

}

//on the order of 50Hz
void UAVDebug::ekfCallback(nav_msgs::OdometryConstPtr p){
  ros::Time start_ = ros::Time::now();
  
  x_velo_->add_value(p->twist.twist.linear.x);
  y_velo_->add_value(p->twist.twist.linear.y);
  
  
  
//    // current velocity
//   current_slam_vel_.header.stamp = p->header.stamp;
//   current_slam_vel_.point.x =p->twist.twist.linear.x;
//   current_slam_vel_.point.y =p->twist.twist.linear.y;
//   
//    // acceleration
//   current_slam_acc_.header.stamp = current_slam_vel_.header.stamp;
//   ros::Duration v_dt = current_slam_vel_.header.stamp -  last_slam_vel_.header.stamp;
//   double v_dx = current_slam_vel_.point.x - last_slam_vel_.point.x;
//   double v_dy = current_slam_vel_.point.y - last_slam_vel_.point.y;
//   current_slam_acc_.point.x = (v_dx / v_dt.toSec())/40;
//   current_slam_acc_.point.y = (v_dy / v_dt.toSec())/40;
//   
//   ROS_ERROR("\nslam vel %f %f ds %f dt %f val %f\n", current_slam_vel_.point.x,last_slam_vel_.point.x,v_dx,v_dt.toSec(),v_dx / v_dt.toSec());
//  
//   
//    // last velocity
//   last_slam_vel_.header.stamp = current_slam_vel_.header.stamp;
//   last_slam_vel_.point.x = current_slam_vel_.point.x;
//   last_slam_vel_.point.y = current_slam_vel_.point.y;
//   
//   slam_acc_pub_.publish(current_slam_acc_);
  
  
  

}

int main(int argc, char **argv){
  ros::init(argc, argv, "uav_state_publisher");
  UAVDebug sp;

  ros::spin();

  return 0;
}
