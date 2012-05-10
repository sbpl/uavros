#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <uav_msgs/ControllerCommand.h>
#include <uav_local_planner/hexa_dynamics.h>

//Controller Gains and Variables (Copied over from Jon's MATLAB version)

typedef struct{

  // Roll and Pitch Gains
  float RPkp;       
  float RPkd;    
  float RPki;

  // Yaw Gains
  float Ykp;
  float Ykd;
  float Yki;

  // Thrust Gains

  float Tkp;
  float Tkd;
  float Tki;

  //Roll and Pitch Gains for Position

  float Posekp;
  float Posekd;
  float Poseki;

  // Initialize the integral terms

  float RI;
  float YI;
  float PI;
  float TI;
  float RposeI;
  float PposeI;

  // Windup limits

  float windupRP;
  float windupY;
  float windupPose;
}CONT_t;

class HexaController{


  /**
   * Current state (observed state) and desired goal state
   */
    Eigen::VectorXf des_state, current_state;
    

    /**
     * Controller gains and dynamics parameters
     */

    CONT_t CONT;
    HEXA_t HEXA;

  public:


    /**
     * Constructor - Initializes controller gains and dynamics params
     */

    HexaController();
    /**
     * Destructor 
     */

    ~HexaController();
    /**
     * @brief Initialize gains for the controller
     */
    void InitializeDynamics();

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
    Eigen::Vector4i Controller(const geometry_msgs::PoseStamped, const geometry_msgs::TwistStamped, const geometry_msgs::PoseStamped);

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
   void SetDesState(const geometry_msgs::PoseStamped goal_pose);

   /**
    * @brief Set the current state (X,Y,Z,roll,pitch,yaw,X_dot,Y_dot,Z_dot,p,q,r)
    * @param current_pose (X,Y,Z,quaternion) from the observer (EKF)
    * @param current_velocities (X_dot,Y_dot,Z_dot,p,q,r) from the observer (EKF)
    */
   void SetCurrState(const geometry_msgs::PoseStamped current_pose, const geometry_msgs::TwistStamped current_velocities);

/**
 * @brief Updates the transforms between the world and body frames based on current observation
 * @param X The current observed state
 */
   void UpdateTransforms(Eigen::VectorXf X);

};
