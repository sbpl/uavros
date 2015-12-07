#include <uav_local_planner/controller.h>
#include <uav_local_planner/UAVControllerConfig.h>

#define min(a,b) a < b ? a : b


// Constructor
UAVController::UAVController() : tf_(ros::NodeHandle(), ros::Duration(10), true)
{
  ros::NodeHandle nh("~");
  UAVController::InitializeGains();
  Pos_err = nh.advertise<geometry_msgs::PointStamped>("Pos_error",1);
  Or_err = nh.advertise<geometry_msgs::PointStamped>("Or_error",1);
  Vel_err = nh.advertise<geometry_msgs::PointStamped>("Vel_error",1);
  A_Vel_err = nh.advertise<geometry_msgs::PointStamped>("A_Vel_error",1);
  Roll_p_gain = nh.advertise<geometry_msgs::PointStamped>("Roll_Pos_Gains",1);
  Pitch_p_gain = nh.advertise<geometry_msgs::PointStamped>("Pitch_Pos_Gains",1);
  Roll_o_gain = nh.advertise<geometry_msgs::PointStamped>("Roll_Or_Gains",1);
  Pitch_o_gain = nh.advertise<geometry_msgs::PointStamped>("Pitch_Or_Gains",1);

  PID_pub_ = nh.advertise<geometry_msgs::PointStamped>("PID_altitude", 1);

  ROS_WARN("[controller] did I get here end?");

}

// Destructor
UAVController::~UAVController()
{
}

void UAVController::InitializeGains()
{
    // TODO: ROS electric and earlier do not support params of type "float", so
    // we have to add a hack to get them...  header changed to double for all
    // robot parameters

    waitForParam("/UAV/Mass");
    readParam("/UAV/Mass", CONT.mass, 5.0, "kg");

    // number of props
    waitForParam("/UAV/NumProps");
    readParam("/UAV/NumProps", CONT.numProps, 6);

    // N based on 2 kg max force per blade
    waitForParam("/UAV/MaxForce");
    readParam("/UAV/MaxForce", CONT.maxF, 14.0, "N");

    // N for which which we really can't issue a control
    waitForParam("/UAV/MinForce");
    readParam("/UAV/MinForce", CONT.minF, 4.0, "N");

    // Maximum allowable position error for position ctrl and alt ctrl
    waitForParam("/UAV/MaxError");
    readParam("/UAV/MaxError", CONT.maxError, 0.5, "m");

    // Gravity Vector
    //TODO: load list for gravity vector
    CONT.g << 0, 0, -9.81;

    // Default thrust for hovering
    waitForParam("/UAV/DefaultThrust");
    readParam("/UAV/DefaultThrust", CONT.defaultThrust, 9.5, "N");

    // Roll and Pitch Gains
    waitForParam("/UAV/RPkp");
    readParam("/UAV/RPkp", CONT.RPkp, 2.0);
    waitForParam("/UAV/RPki");
    readParam("/UAV/RPki", CONT.RPki, 0.0);
    waitForParam("/UAV/RPkd");
    readParam("/UAV/RPkd", CONT.RPkd, 0.0);

    // Yaw Gains
    waitForParam("/UAV/Ykp");
    readParam("/UAV/Ykp", CONT.Ykp, 1.0);

    waitForParam("/UAV/Yki");
    readParam("/UAV/Yki", CONT.Yki, 0.0);

    waitForParam("/UAV/Ykd");
    readParam("/UAV/Ykd", CONT.Ykd, 0.0);

    // Thrust Gains
    waitForParam("/UAV/Tkp");
    readParam("/UAV/Tkp", CONT.Tkp, 2.0);

    waitForParam("/UAV/Tki");
    readParam("/UAV/Tki", CONT.Tki, 0.1);

    waitForParam("/UAV/Tkd");
    readParam("/UAV/Tkd", CONT.Tkd, 6.0);

    // Roll and Pitch Gains for Position
    waitForParam("/UAV/Posekp");
    readParam("/UAV/Posekp", CONT.Posekp, 0.8);

    waitForParam("/UAV/Poseki");
    readParam("/UAV/Poseki", CONT.Poseki, 0.0);

    waitForParam("/UAV/Posekd");
    readParam("/UAV/Posekd", CONT.Posekd, 0.0);

    // Initialize the integral terms
    CONT.RI = 0;
    CONT.YI = 0;
    CONT.PI = 0;
    CONT.TI = 0;

    waitForParam("/UAV/RposeI");
    readParam("/UAV/RposeI", CONT.RposeI, 0.0);

    waitForParam("/UAV/PposeI");
    readParam("/UAV/PposeI", CONT.PposeI, 0.0);

    // Windup limits
    waitForParam("/UAV/windupRP");
    readParam("/UAV/windupRP", CONT.windupRP, 5.0);

    waitForParam("/UAV/windupY");
    readParam("/UAV/windupY", CONT.windupY, 5.0);

    waitForParam("/UAV/windupPose");
    readParam("/UAV/windupPose", CONT.windupPose, 5.0);
}

void UAVController::dynamic_reconfigure_callback(uav_local_planner::UAVControllerConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", config.RPkp, config.RPkd, config.RPki, config.Ykp, config.Ykd, config.Yki, config.Tkp, config.Tkd, config.Tki, config.Posekp, config.Posekd, config.Poseki, config.HovT);

  CONT.defaultThrust = config.HovT;

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

  //CONT.mass = config.mass;
 // CONT.maxF = config.maxF;
 //CONT.minF = config.minF;
 // CONT.maxError = config.maxError;
  //TODO:  Make node for broadcast params (mass, etc) then each affected node subscribe to broadcast param topic/service

}

Eigen::VectorXf UAVController::SetDesState(const geometry_msgs::PoseStamped goal_pose)
{

  Eigen::VectorXf des_state;
  des_state.setZero(12);
  // X,Y,Z in world frame
  des_state[0] = goal_pose.pose.position.x;
  des_state[1] = goal_pose.pose.position.y;
  des_state[2] = goal_pose.pose.position.z;


  double roll, pitch, yaw;
  tf::Quaternion q;
  tf::quaternionMsgToTF(goal_pose.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  // ROS_INFO("Roll, Pitch, Yaw from Quaternion:  %f %f %f",roll,pitch,yaw);

  // Roll, Pitch, Yaw in world frame
  des_state[3] = roll;
  des_state[4] = pitch;
  des_state[5] = yaw;
  //  printf("$$$$$$$$ des yaw %f\n", yaw);
  return des_state;
}

Eigen::VectorXf UAVController::SetCurrState(const geometry_msgs::PoseStamped current_pose, const geometry_msgs::TwistStamped current_velocities)
{
  Eigen::VectorXf current_state;
  current_state.setZero(12);
  // X,Y,Z in world frame
  current_state[0] = current_pose.pose.position.x;
  current_state[1] = current_pose.pose.position.y;
  current_state[2] = current_pose.pose.position.z;

  double roll, pitch, yaw;
  tf::Quaternion q;
  tf::quaternionMsgToTF(current_pose.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

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


Eigen::Vector3f UAVController::AttitudeCtrl(Eigen::VectorXf X, Eigen::VectorXf DesX)
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

    if(err[2] > M_PI / 3)
    {
       err[2] = M_PI / 3;
    }
    if(err[2] < -M_PI /3)
    {
       err[2] = -M_PI /3;
    }

    RPY[0] = CONT.RPkp*err[0] + CONT.RPkd*err[3] + CONT.RI*CONT.RPki;
    RPY[1] = CONT.RPkp*err[1] + CONT.RPkd*err[4] + CONT.PI*CONT.RPki;
    RPY[2] = CONT.Ykp*err[2] + CONT.Ykd*err[5] + CONT.YI*CONT.Yki;

    CONT.RI += err[0];
    CONT.PI += err[1];
    CONT.YI += err[2];

    geometry_msgs::PointStamped Or_e;
    Or_e.header.stamp = ros::Time::now();
    Or_e.point.x = err[0];
    Or_e.point.y = err[1];
    Or_e.point.z = err[2];
    Or_err.publish(Or_e);

    geometry_msgs::PointStamped A_Vel_e;
    A_Vel_e.header.stamp = ros::Time::now();
    A_Vel_e.point.x = err[3];
    A_Vel_e.point.y = err[4];
    A_Vel_e.point.z = err[5];
    A_Vel_err.publish(A_Vel_e);

    geometry_msgs::PointStamped Roll_o_g;
    Roll_o_g.header.stamp = ros::Time::now();
    Roll_o_g.point.x = CONT.RPkp*err[0];
    Roll_o_g.point.y = CONT.RPkd*err[3];
    Roll_o_g.point.z = CONT.RI*CONT.RPki;
    Roll_o_gain.publish(Roll_o_g);

    geometry_msgs::PointStamped Pitch_o_g;
    Pitch_o_g.header.stamp = ros::Time::now();
    Pitch_o_g.point.x = CONT.RPkp*err[1];
    Pitch_o_g.point.y = CONT.RPkd*err[4];
    Pitch_o_g.point.z = CONT.PI*CONT.RPki;
    Pitch_o_gain.publish(Pitch_o_g);

    return RPY;
    //TODO: Error Logging  --- meaning what????
}

Eigen::Vector2f UAVController::PositionCtrl(Eigen::VectorXf X, Eigen::VectorXf DesX)
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
    if (err_p[idx] > CONT.maxError){
      err_p[idx] = CONT.maxError;
      ROS_ERROR("Above max error %d\n", idx);
    }
    else if (err_p[idx] < -CONT.maxError) {
      err_p[idx] = -CONT.maxError;
      ROS_ERROR("Below -max error %d\n", idx);
    }
  }

  //velocity
  temp.setX(-X(6));
  temp.setY(-X(7));
  temp.setZ(-X(8));

  err_v = transform * temp;

  //roll
  RP_Pose[0] = -err_p[1]*CONT.Posekp + -err_v[1]*CONT.Posekd + -CONT.RposeI*CONT.Poseki;
  CONT.RposeI += err_p[1];

  //pitch
  RP_Pose[1] = err_p[0]*CONT.Posekp + err_v[0]*CONT.Posekd + CONT.PposeI*CONT.Poseki;
  CONT.PposeI += err_p[0];

  //HACK REMOVE LATER
  double pitch_limit = abs(0.15 / CONT.Poseki);
  CONT.PposeI = std::max(-pitch_limit,min(pitch_limit,CONT.PposeI));

  geometry_msgs::PointStamped Pos_e;
  Pos_e.header.stamp = ros::Time::now();
  Pos_e.point.x = err_p[0];
  Pos_e.point.y = err_p[1];
  Pos_e.point.z = err_p[2];
  Pos_err.publish(Pos_e);

  geometry_msgs::PointStamped Vel_e;
  Vel_e.header.stamp = ros::Time::now();
  Vel_e.point.x = err_v[0];
  Vel_e.point.y = err_v[1];
  Vel_e.point.z = err_v[2];
  Vel_err.publish(Vel_e);

  geometry_msgs::PointStamped Roll_p_g;
  Roll_p_g.header.stamp = ros::Time::now();
  Roll_p_g.point.x = -err_p[1]*CONT.Posekp;
  Roll_p_g.point.y = -err_v[1]*CONT.Posekd;
  Roll_p_g.point.z = -CONT.RposeI*CONT.Poseki;
  Roll_p_gain.publish(Roll_p_g);

  geometry_msgs::PointStamped Pitch_p_g;
  Pitch_p_g.header.stamp = ros::Time::now();
  Pitch_p_g.point.x = err_p[0]*CONT.Posekp;
  Pitch_p_g.point.y = err_v[0]*CONT.Posekd;
  Pitch_p_g.point.z = CONT.PposeI*CONT.Poseki;
  Pitch_p_gain.publish(Pitch_p_g);

  return  RP_Pose;
}

float UAVController:: AltitudeCtrl(Eigen::VectorXf X, Eigen::VectorXf DesX)
{
  float T;
  // Limit Integral windup
  if(abs(CONT.TI)>CONT.windupY)
    CONT.TI = copysign(1,CONT.TI)*CONT.windupY;

  Eigen::Vector2f err;
  err[0] = DesX[2]-X[2];
  err[1] = -X[8];

  // the hope is that this is in meters
  // todo: make 'something' a parameter
  const double something = 3.0;
  if (err[1] >= something) {
    err[1] = something;
  }
  else if (err[1] <= -something) {
    err[1] = -something;
  }

  if (err[0] > CONT.maxError) {
    err[0] = CONT.maxError;
    ROS_ERROR("Above max error thrust\n");
  }
  else if (err[0] < -CONT.maxError) {
    err[0] = -CONT.maxError;
    ROS_ERROR("Below -max error thrust\n");
  }

  tf::StampedTransform transform;
  this->tf_.lookupTransform("body_frame", "body_frame_stabilized", ros::Time(0), transform);

  tf::Matrix3x3 rot;
  rot = transform.getBasis();
  double rot22;
  rot22 = rot[2].getZ();

  // This accounts for gravity, should also account for other controls
  float FF = -CONT.g[2]*CONT.mass/(rot22*CONT.numProps);
  FF = min(FF,CONT.maxF);
  T = err[0]*CONT.Tkp + err[1]*CONT.Tkd + CONT.TI*CONT.Tki + CONT.defaultThrust;

  CONT.TI += err[0];

  geometry_msgs::PointStamped PID_pt;
  PID_pt.header.stamp = ros::Time::now();
  PID_pt.point.x = err[0]*CONT.Tkp;
  PID_pt.point.y = CONT.TI*CONT.Tki;
  PID_pt.point.z = err[1]*CONT.Tkd;
  PID_pub_.publish(PID_pt);

  return T;

  //TODO: Error Logging
}

uav_msgs::ControllerCommand UAVController::Controller(
    const geometry_msgs::PoseStamped current_pose,
    const geometry_msgs::TwistStamped current_velocities,
    const geometry_msgs::PoseStamped goal_pose)
{
  Eigen::VectorXf current_state, des_state;
  current_state = SetCurrState(current_pose, current_velocities);
  des_state = SetDesState(goal_pose);

  Eigen::Vector4f F;
  Eigen::Vector3f RPY_f;
  Eigen::Vector2f Pose_f;
  float T_f;

  RPY_f = AttitudeCtrl(current_state, des_state);
  Pose_f = PositionCtrl(current_state, des_state);
  T_f = AltitudeCtrl(current_state, des_state);

  F[0] = Pose_f[0] + RPY_f[0];
  F[1] = Pose_f[1] + RPY_f[1];
  F[2] = RPY_f[2];
  F[3] = T_f;

  if (F[3] > CONT.maxF) {
    ROS_ERROR("Thrust force above allowable value!!\n");
    F[3] = CONT.maxF;
  }
  else if (F[3] < CONT.minF) {
    ROS_ERROR("Thrust force below minimum value!!\n");
    F[3] = CONT.minF;
  }

  uav_msgs::ControllerCommand ctrl_cmd;
  ctrl_cmd.header.stamp = ros::Time::now();
  ctrl_cmd.roll = static_cast<float>(F[0]);
  ctrl_cmd.pitch = static_cast<float>(F[1]);
  ctrl_cmd.yaw = static_cast<float>(F[2]);
  ctrl_cmd.thrust = static_cast<float>(F[3]);
  //ROS_INFO("Controller Command RPYT: %f %f %f %f", ctrl_cmd.roll, ctrl_cmd.pitch, ctrl_cmd.yaw, ctrl_cmd.thrust);
  return ctrl_cmd;
}

void UAVController::waitForParam(const std::string& name) const
{
    while (!ros::param::has(name)) {
        ros::Duration(0.1).sleep();
    }
}

template <typename T>
void UAVController::readParam(
    const std::string& name,
    T& value,
    const T& errval,
    const std::string& units)
{
    if (!ros::param::get(name, value)) {
        ROS_ERROR_STREAM("Missing '" << name << "' parameter, setting to " << errval << units);
        value = errval;
    }
}
