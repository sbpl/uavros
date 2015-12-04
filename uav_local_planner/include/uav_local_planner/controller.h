#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <uav_msgs/ControllerCommand.h>
#include <uav_local_planner/UAVControllerConfig.h>
#include <dynamic_reconfigure/server.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/LU>

//TODO: combine this with the local planner in a more coherent fashion

typedef struct{

  double defaultThrust;

  // Roll and Pitch Gains
  double RPkp;
  double RPkd;
  double RPki;

  // Yaw Gains
  double Ykp;
  double Ykd;
  double Yki;

  // Thrust Gains
  double Tkp;
  double Tkd;
  double Tki;

  //Roll and Pitch Gains for Position
  double Posekp;
  double Posekd;
  double Poseki;

  // Initialize the integral terms
  double RI;
  double YI;
  double PI;
  double TI;
  double RposeI;
  double PposeI;

  // Windup limits
  double windupRP;
  double windupY;
  double windupPose;

  //other variables
  double mass;
  int numProps;
  double maxF;
  double minF;
  double maxError;
  Eigen::Vector3f g;
 // Eigen::VectorXf F;
}CONT_t;

class UAVController {

  /**
   * Controller gains and dynamics parameters
   */

  CONT_t CONT;

  public:


  /**
   * Constructor - Initializes controller gains and dynamics params
   */

  UAVController();
  /**
   * Destructor
   */

  ~UAVController();

  /**
   * @brief Initialize hexarotor dynamics parameters
   */
  void InitializeGains();
  /**
   * @brief The main controller function which calls the attitude, altitude and position controller
   * @param X Current Observed State
   * @param DesX Desired Goal State
   * @return 4x1 vector - [thurst, roll, pitch, yaw]' , all numbers in the range 0-255
   */
  uav_msgs::ControllerCommand Controller(const geometry_msgs::PoseStamped, const geometry_msgs::TwistStamped, const geometry_msgs::PoseStamped);

  /**
   * @brief Attitude controller
   * @param X Current Observed State
   * @param DesX Desired Goal State
   * @return A 3x1 vector of floats for Roll,Pitch and Yaw thrusts
   */
  Eigen::Vector3f AttitudeCtrl(Eigen::VectorXf X, Eigen::VectorXf DesX);

  /**
   * @brief Position controller
   * @param X Current Observed State
   * @param DesX Desired Goal State
   * @return A 2x1 vector of floats for Roll and Pitch thrusts
   */
  Eigen::Vector2f PositionCtrl(Eigen::VectorXf X, Eigen::VectorXf DesX);

  /**
   * @brief Altitude controller
   * @param X Current Observed State
   * @param DesX Desired Goal State
   * @return A single float for the average thrust
   */
  float AltitudeCtrl(Eigen::VectorXf X, Eigen::VectorXf DesX);

  /**
   * @brief Set the desired state (X,Y,Z,roll,pitch,yaw)
   * @param goal_pose The goal pose published on the /goal_pose topic (from RVIZ)
   */
  Eigen::VectorXf SetDesState(const geometry_msgs::PoseStamped goal_pose);

  /**
   * @brief Set the current state (X,Y,Z,roll,pitch,yaw,X_dot,Y_dot,Z_dot,p,q,r)
   * @param current_pose (X,Y,Z,quaternion) from the observer (EKF)
   * @param current_velocities (X_dot,Y_dot,Z_dot,p,q,r) from the observer (EKF)
   */
  Eigen::VectorXf SetCurrState(const geometry_msgs::PoseStamped current_pose, const geometry_msgs::TwistStamped current_velocities);

  /**
   * @brief Callback for the dynamic reconfigure GUI
   * @param config Callback message from GUI
   * @param level Callback level from GUI (Not used)
   */
  void dynamic_reconfigure_callback(uav_local_planner::UAVControllerConfig &config, uint32_t level);

private:
  tf::TransformListener tf_;
  ros::Publisher PID_pub_;
  ros::Publisher Pos_err ;
  ros::Publisher Vel_err;
  ros::Publisher A_Vel_err;
  ros::Publisher Or_err ;
  ros::Publisher Roll_p_gain ;
  ros::Publisher Pitch_p_gain ;
  ros::Publisher Roll_o_gain ;
  ros::Publisher Pitch_o_gain ;

  void waitForParam(const std::string& name) const;

  template <typename T>
  void readParam(
        const std::string& name,
        T& value,
        const T& errval,
        const std::string& units = "");
};
