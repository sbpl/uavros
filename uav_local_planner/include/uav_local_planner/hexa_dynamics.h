#include <Eigen/Core>
#include <Eigen/LU>

// Hexarotor dynamics parameters
typedef struct{

  // Motor coefs that must be read in from a config file
  Eigen::VectorXf FV2U_coef, UV2F_coef, Fex, Vex, Uex;

  // Arm Length
  float L;

  // Blade Radius
  float bladeL;

  // Weight in kg
  float mass;

  // TODO: This is pure crap... (description from Jon's MATLAB version)
  float Moment2Fratio;

  // N based on 2 kg max force per blade
  float maxF;

  // N for which which we really can't issue a control
  float minF;

  // Transform from motor forces to angular rates on the helo
  Eigen::Matrix<float,3,6> F_select;

  // Inertia Matrix for the Hexarotor
  Eigen::Matrix3f I;

  // Inverse of the Inertia Matrix
  Eigen::Matrix3f I_inv;

  // Gravity Vector
  Eigen::Vector3f g;

  // Generalized  Force vector ?
  Eigen::VectorXf F;

  // ANG contains body to world and world to body rotation matrices
  struct{
    Eigen::Matrix3f R_B2W, R_W2B;
  }ANG;

  // This should be read in from the system
  float V; // in tenths of volts

}HEXA_t;
