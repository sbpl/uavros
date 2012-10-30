#ifndef UAV_STATE_PUBLISHER_H
#define UAV_STATE_PUBLISHER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <string>

class UAVStatePublisher{

public:
  UAVStatePublisher();
  ~UAVStatePublisher(){};

private:
  class FIFO{
  public:
    FIFO(int s){
      size_ = s;
      head_ = 0;
      tail_ = 0;
      elements_ = 0;
      q_ = new double[s];
    }

    ~FIFO(){
      delete [] q_;
    }

    void insert(double x){
      *(q_+head_) = x;
      head_++;
      if(head_==size_)
        head_ = 0;
      if(elements_ < size_)
        elements_++;
    }

    double operator[](int i){
      int j=tail_+i;
      if(j>=size_)
        j -= size_;
      return *(q_+j);
    }

    int size(){
      return elements_;
    }

  private:
    int size_, head_, tail_, elements_;
    double* q_;
  };

  void ekfCallback(nav_msgs::OdometryConstPtr p);
  void lidarCallback(sensor_msgs::LaserScanConstPtr scan);
  void slamCallback(geometry_msgs::PoseStampedConstPtr slam_msg);

  std::string state_pub_topic_, z_laser_topic_, z_laser_median_topic_, position_sub_topic_, vertical_laser_data_topic_, vertical_laser_frame_topic_, slam_topic_, map_topic_, body_topic_, body_map_aligned_topic_, body_stabilized_topic_, rpy_pub_topic_;

  nav_msgs::Odometry state_;
  ros::Publisher state_pub_;
  ros::Publisher pointCloud_pub_;
  ros::Publisher point_pub_;
  ros::Publisher rpy_pub_;
  ros::Subscriber ekf_sub_;
  ros::Subscriber slam_sub_;
  ros::Subscriber lidar_sub_;

  tf::StampedTransform Pan2BodyTransform_;


  double saved_yaw_;

  tf::TransformListener tf_;
  tf::TransformBroadcaster tf_broadcaster;

  double min_lidar_angle_, max_lidar_angle_;

  FIFO z_fifo_;
  FIFO z_time_fifo_;

};

#endif
