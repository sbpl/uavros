#include <Eigen/Core>
#include <Eigen/LU>

// Hexarotor dynamics parameters
typedef struct{
  // Arm Length
  float L;

  // Blade Radius
  float bladeL;

  // Weight in kg
  float mass;

  // N based on 2 kg max force per blade
  float maxF;

  // N for which which we really can't issue a control
  float minF;

  // Gravity Vector
  Eigen::Vector3f g;

  // Generalized  Force vector ?
  Eigen::VectorXf F;

}HEXA_t;
