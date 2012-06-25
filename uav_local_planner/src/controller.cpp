#include <uav_local_planner/controller.h>
#include <uav_local_planner/UAVControllerConfig.h>
#include <tf/tf.h>
#include <cmath>

#define min(a,b) a < b ? a : b


// Constructor
HexaController::HexaController() : tf_(ros::NodeHandle(), ros::Duration(10), true)
{
  ros::NodeHandle nh("~");
  HexaController::InitializeGains();
  HexaController::InitializeDynamics(nh);
  PID_pub_ = nh.advertise<geometry_msgs::PointStamped>("PID_out", 1);
}

// Destructor
HexaController::~HexaController()
{
}

void HexaController::InitializeGains()
{
  // Roll and Pitch Gains    TODO: read in from param server
  CONT.RPkp = 20/10;//20;       //10 150
  CONT.RPkd = 0;//2.5*8/80; //6*10/8;
  CONT.RPki = 0;//0.6*2/400; //12/50;

  // Yaw Gains
  CONT.Ykp = 1;//6;
  CONT.Ykd = 0;//6*15/800;
  CONT.Yki = 0;//12/580;

  // Thrust Gains

  CONT.Tkp = 2;//10/5;   //divide by 5
  CONT.Tkd = 6;
  CONT.Tki = 0.1;

  //Roll and Pitch Gains for Position

  CONT.Posekp = .8;
  CONT.Posekd = 0;//8*6/80;
  CONT.Poseki = 0;

  // Initialize the integral terms

  CONT.RI = 0;
  CONT.YI = 0;
  CONT.PI = 0;
  CONT.TI = 0;
  CONT.RposeI = 0;
  CONT.PposeI = 0;

  // Windup limits

  CONT.windupRP = 5;
  CONT.windupY = 5;
  CONT.windupPose = 5;

}

void HexaController::InitializeDynamics(ros::NodeHandle nh)
{
  //   //TODO: these should come in as parameters
  // Weight in kg
  HEXA.mass = 5.0;

  // N based on 2 kg max force per blade
  HEXA.maxF = 12.0;

  // N for which which we really can't issue a control
  HEXA.minF = 4.0;

// Maximum allowable position error for position ctrl and alt ctrl
  HEXA.maxError = 0.5; //TODO: make a param

  // Gravity Vector
  HEXA.g << 0, 0, -9.81;

  // TODO: Description for this   WTF is this?
  HEXA.F.setZero(6);

  //TODO:  remove this
  // Initialize transforms to identity
  //   HEXA.ANG.R_B2W.setIdentity();
  //   HEXA.ANG.R_W2B.setIdentity();
}

void HexaController::dynamic_reconfigure_callback(uav_local_planner::UAVControllerConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", config.RPkp, config.RPkd, config.RPki, config.Ykp, config.Ykd, config.Yki, config.Tkp, config.Tkd, config.Tki, config.Posekp, config.Posekd, config.Poseki);
  CONT.RPkp = config.RPkp;
  CONT.RPkd = config.RPkd;
  CONT.RPki = config.RPki;

  CONT.Ykp = config.Ykp;
  CONT.Ykd = config.Ykd;
  CONT.Yki = config.Yki;

  CONT.Tkp = config.Tkp;
  CONT.Tkd = config.Tkd;
  CONT.Tki = config.Tki;

  CONT.Posekp = config.Posekp;
  CONT.Posekd = config.Posekd;
  CONT.Poseki = config.Poseki;
}

Eigen::VectorXf HexaController::SetDesState(const geometry_msgs::PoseStamped goal_pose)
{

  Eigen::VectorXf des_state;
  des_state.setZero(12);
  // X,Y,Z in world frame
  des_state[0] = goal_pose.pose.position.x;
  des_state[1] = goal_pose.pose.position.y;
  des_state[2] = goal_pose.pose.position.z;


  double roll, pitch, yaw;
  btQuaternion q;
  tf::quaternionMsgToTF(goal_pose.pose.orientation, q);
  btMatrix3x3(q).getRPY(roll, pitch, yaw); //TODO: Check if this is right
  // ROS_INFO("Roll, Pitch, Yaw from Quaternion:  %f %f %f",roll,pitch,yaw);

  // Roll, Pitch, Yaw in world frame
  des_state[3] = roll;
  des_state[4] = pitch;
  des_state[5] = yaw;
  //  printf("$$$$$$$$ des yaw %f\n", yaw);
  return des_state;
}

Eigen::VectorXf HexaController::SetCurrState(const geometry_msgs::PoseStamped current_pose, const geometry_msgs::TwistStamped current_velocities)
{
  Eigen::VectorXf current_state;
  current_state.setZero(12);
  // X,Y,Z in world frame
  current_state[0] = current_pose.pose.position.x;
  current_state[1] = current_pose.pose.position.y;
  current_state[2] = current_pose.pose.position.z;

  double roll, pitch, yaw;
  btQuaternion q;
  tf::quaternionMsgToTF(current_pose.pose.orientation, q);
  btMatrix3x3(q).getRPY(roll, pitch, yaw);

  // Roll, Pitch, Yaw in world frame
  current_state[3] = roll;
  current_state[4] = pitch;
  current_state[5] = yaw;
  // printf("$$$$$$$$ cur yaw %f\n", yaw);

  // Linear velocities in world frame
  current_state[6] = current_velocities.twist.linear.x;
  current_state[7] = current_velocities.twist.linear.y;
  current_state[8] = current_velocities.twist.linear.z;

  // Angular velocities in the body frame
  current_state[9] = current_velocities.twist.angular.x;
  current_state[10] = current_velocities.twist.angular.y;
  current_state[11] = current_velocities.twist.angular.z;
  return current_state;
}


Eigen::Vector3f HexaController::AttitudeCtrl(Eigen::VectorXf X, Eigen::VectorXf DesX)
{
  Eigen::Vector3f RPY;
  // Limit integral windup
  if(abs(CONT.RI)>CONT.windupRP)
    CONT.RI = copysign(1,CONT.RI)*CONT.windupRP;
  if(abs(CONT.PI)>CONT.windupRP)
    CONT.PI = copysign(1,CONT.PI)*CONT.windupRP;
  if(abs(CONT.YI)>CONT.windupY)
    CONT.YI = copysign(1,CONT.YI)*CONT.windupY;

  // Calculate errors in angular velocity and angle
    Eigen::VectorXf err;
    err.setZero(6);
    err.segment(0,3) = DesX.segment(3,3) - X.segment(3,3);
    err.segment(3,3) =  - X.segment(9,3);

    for(int i=0;i<3;i++)
    {
      if(err[i] > M_PI)
        err[i] -= 2*M_PI;
      else if(err[i] < -M_PI)
        err[i] += 2*M_PI;
    }

    RPY[0] = CONT.RPkp*err[0] + CONT.RPkd*err[3] + CONT.RI*CONT.RPki;
    RPY[1] = CONT.RPkp*err[1] + CONT.RPkd*err[4] + CONT.PI*CONT.RPki;
    RPY[2] = CONT.Ykp*err[2] + CONT.Ykd*err[5] + CONT.YI*CONT.Yki;
    // ROS_WARN("yaw errors are P:%1.3f I:%1.3f D:%1.3f - P:%1.3f I:%1.3f D:%1.3f\n", err[2], CONT.YI, err[5], CONT.Ykp*err[2], CONT.YI*CONT.Yki, CONT.Ykd*err[5]);

    CONT.RI += err[0];
    CONT.PI += err[1];
    CONT.YI += err[2];
    //printf("%%%%%% RPY  R:%f P:%f Y:%f goal R:%f P:%f Y:%f\n", X(3), X(4), X(5), DesX(3), DesX(4), DesX(5));
    //  ROS_INFO("Attitude controller returned %f %f 2.3%f",RPY[0],RPY[1],RPY[2]);
    return RPY;
    //TODO: Error Logging
}

Eigen::Vector2f HexaController::PositionCtrl(Eigen::VectorXf X, Eigen::VectorXf DesX)
{

  Eigen::Vector2f RP_Pose;
  if(abs(CONT.RposeI)>CONT.windupPose)
    CONT.RposeI = copysign(1,CONT.RposeI)*CONT.windupPose;
  if(abs(CONT.PposeI)>CONT.windupPose)
    CONT.PposeI = copysign(1,CONT.PposeI)*CONT.windupPose;

  tf::StampedTransform transform;
  this->tf_.lookupTransform("body_frame", "body_frame_map_aligned", ros::Time(0), transform);
  tf::Point err_p, err_v, temp;

  //position
  temp.setX(DesX(0) - X(0));
  temp.setY(DesX(1) - X(1));
  temp.setZ(DesX(2) - X(2));

  err_p = transform * temp;
  for(int idx=0; idx<3; idx++) {
    if (err_p[idx] > HEXA.maxError){
      err_p[idx] = HEXA.maxError;
      ROS_ERROR("Above max error %d\n", idx);
    }
    else if (err_p[idx] < -HEXA.maxError) {
      err_p[idx] = -HEXA.maxError;
      ROS_ERROR("Below -max error %d\n", idx);
    }
  }

  //velocity
  temp.setX(-X(6));
  temp.setY(-X(7));
  temp.setZ(-X(8));

  err_v = transform * temp;
  // printf("vel in map frame xdot:%2.3f ydot:%2.3f zdot:%2.3f\n", X[6], X[7], X[8]);

  //  printf("vel in body frame xdot:%2.3f ydot:%2.3f zdot:%2.3f\n", err_v[0], err_v[1], err_v[2]);

  // ROS_ERROR("The goal is at x%f y%f relative\n", err_p[0], err_p[1]);
  //   Eigen::VectorXf err;
  //   Eigen::Vector3f temp;
  //   err.setZero(6);
  //   temp = DesX.segment(0,3) - X.segment(0,3);
  //   err.segment(0,3) = tf_
  //   err.segment(3,3) = HEXA.ANG.R_W2B*(DesX.segment(3,3) - X.segment(3,3));

  //roll
  RP_Pose[0] = -err_p[1]*CONT.Posekp + -err_v[1]*CONT.Posekd + -CONT.RposeI*CONT.Poseki;
  CONT.RposeI += err_p[1];

  //pitch
  RP_Pose[1] = err_p[0]*CONT.Posekp + err_v[0]*CONT.Posekd + CONT.PposeI*CONT.Poseki;
  CONT.PposeI += err_p[0];


  printf("###### roll- y:%f v:%f      pitch- x:%f v:%f\n", err_p[1], err_v[1], err_p[0], err_v[0]);
  // ROS_INFO("Position controller returned %f %f",RP_Pose[0],RP_Pose[1]);
  // TODO: Error Logging
  return  RP_Pose;
}

float HexaController:: AltitudeCtrl(Eigen::VectorXf X, Eigen::VectorXf DesX)
{
  float T;
  // Limit Integral windup
  if(abs(CONT.TI)>CONT.windupY)
    CONT.TI = copysign(1,CONT.TI)*CONT.windupY;

  Eigen::Vector2f err;
  err[0] = DesX[2]-X[2];
  err[1] = DesX[5]-X[5];

  if (err[0] > HEXA.maxError) {
    err[0] = HEXA.maxError;
    ROS_ERROR("Above max error thrust\n");
  }
  else if (err[0] < -HEXA.maxError) {
    err[0] = -HEXA.maxError;
    ROS_ERROR("Below -max error thrust\n");
  }

  tf::StampedTransform transform;
  this->tf_.lookupTransform("body_frame", "body_frame_stabilized", ros::Time(0), transform);

  btMatrix3x3 rot;
  rot = transform.getBasis();
  double rot22;
  rot22 = rot[2].getZ();

  // This accounts for gravity, should also account for other controls
  float FF = -HEXA.g[2]*HEXA.mass/(rot22*6);  //TODO: change 6 to num of props from param server
  FF = min(FF,HEXA.maxF);
  T = err[0]*CONT.Tkp + err[1]*CONT.Tkd + CONT.TI*CONT.Tki + 9.5;

  CONT.TI += err[0];

geometry_msgs::PointStamped PID_pt;
PID_pt.header.stamp = ros::Time::now();
PID_pt.point.x = err[0]*CONT.Tkp;
PID_pt.point.y = CONT.TI*CONT.Tki;
PID_pt.point.z = err[1]*CONT.Tkd;
PID_pub_.publish(PID_pt);


  printf("Altitude error is %f (v:%f) with alt at %f v:%f\n", err[0],err[1], X[2], X[5]);
  printf("Altitude Controller returning %f thrust with %f FF\n", T, FF);
  // ROS_INFO("Altitude controller returned %f",T);
  return T;


  //TODO: Error Logging
}

uav_msgs::ControllerCommand HexaController::Controller(const geometry_msgs::PoseStamped current_pose, const geometry_msgs::TwistStamped current_velocities, const geometry_msgs::PoseStamped goal_pose)
{
  Eigen::VectorXf current_state, des_state;
  current_state = SetCurrState(current_pose, current_velocities);
  des_state = SetDesState(goal_pose);
  /*UpdateTransforms(current_state);*/

  Eigen::Vector4f F;
  Eigen::Vector3f RPY_f, cRPY_f;
  Eigen::Vector2f Pose_f;
  float T_f;


  RPY_f = AttitudeCtrl(current_state, des_state);
  Pose_f = PositionCtrl(current_state, des_state);
  T_f = AltitudeCtrl(current_state, des_state);
  printf("ctrl out - ATT: r%f p%f y%f POS: r%f p:%f\n", RPY_f[0], RPY_f[1], RPY_f[2], Pose_f[0], Pose_f[1]);
  cRPY_f[0] = Pose_f[0] + RPY_f[0];
  cRPY_f[1] = Pose_f[1] + RPY_f[1];
  cRPY_f[2] = RPY_f[2];
  // cRPY_f = cRPY_f + RPY_f;

  F[0] = cRPY_f[0];
  F[1] = cRPY_f[1];
  F[2] = cRPY_f[2];
  F[3] = T_f;

  if (F[3] > HEXA.maxF) {
    ROS_ERROR("Thrust force above allowable value!!\n");
    F[3] = HEXA.maxF;
  }
  else if (F[3] < HEXA.minF) {
    ROS_ERROR("Thrust force below minimum value!!\n");
    F[3] = HEXA.minF;
  }


  uav_msgs::ControllerCommand ctrl_cmd;
  ctrl_cmd.header.stamp = ros::Time::now();
  ctrl_cmd.roll = static_cast<float>(F[0]);  //was u
  ctrl_cmd.pitch = static_cast<float>(F[1]);
  ctrl_cmd.yaw = static_cast<float>(F[2]);
  ctrl_cmd.thrust = static_cast<float>(F[3]);
  //ROS_INFO("Controller Command RPYT: %f %f %f %f", ctrl_cmd.roll, ctrl_cmd.pitch, ctrl_cmd.yaw, ctrl_cmd.thrust);
  return ctrl_cmd;

}
