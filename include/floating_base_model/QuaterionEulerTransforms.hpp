#ifndef _QUATERION_TRANSFORMS_HPP_
#define _QUATERION_TRANSFORMS_HPP_

#include <Eigen/Core>
#include <array>
#include <cmath>
#include <ocs2_core/Types.h>
#include <ocs2_robotic_tools/common/SkewSymmetricMatrix.h>

// CppAD
#include <ocs2_core/automatic_differentiation/Types.h>

namespace quaterion_euler_transforms
{

  /**
   * Computes mapping from ZYX euler angles to quaterion
   *
   * @param eulerAnglesZyx: euler angles
   * @return quaterion
   *
   */
  template <typename SCALAR_T>
  Eigen::Quaternion<SCALAR_T> getQuaternionFromEulerAnglesZyx(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesZyx);

  /**
   * Computes mapping from ZYX euler angles to quaterion derivative
   * with respect to ZYX euler angles
   *
   * @param eulerAnglesZyx: euler angles
   * @return derivative of quaterion with respect to euler angles
   *
   */
  template <typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, 4, 3> getQuaternionFromEulerAnglesZyxGradient(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesZyx);

  /**
   * Computes derivatives of rotation matrix
   * with respect to quaterion dRdq
   *
   * @param quaterion: quaterion
   * @return array of rotation matrix partial derivatives
   *
   */
  template <typename SCALAR_T>
  std::array<Eigen::Matrix<SCALAR_T, 3, 3>, 4> getRotationMatrixQuaterionGradient(const Eigen::Quaternion<SCALAR_T>& quaterion);

  /**
  * Compute the matrix that maps derivatives 
  * of local angular velocities to ZYX-Euler angles derivatives
  *
  * @param [in] eulerAngles: ZYX-Euler angles
  * @return 3x3 matrix mapping
  */
  template<typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, 3, 3> getMappingFromLocalAngularVelocitytoEulerAnglesDerivative(const Eigen::Matrix<SCALAR_T, 3, 1> &eulerAngles);

  /**
  * Compute the matrix that maps derivatives 
  * of local angular velocities to ZYX-Euler angles derivatives 
  * gradient with respect to euler angles
  *
  * @param [in] eulerAngles: ZYX-Euler angles
  * @return array of mapping partial derivatives
  */
  template<typename SCALAR_T>
  std::array<Eigen::Matrix<SCALAR_T, 3, 3>, 3> getMappingFromLocalAngularVelocitytoEulerAnglesDerivativeZyxGradient(const Eigen::Matrix<SCALAR_T, 3, 1> &eulerAngles);

  /**
  * Compute the matrix that maps derivatives 
  * of local angular velocities to ZYX-Euler angles derivatives
  *
  * @param [in] quaterion: quaterion
  * @return 3x3 matrix mapping
  */
  template<typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, 3, 3> getMappingFromLocalAngularVelocitytoEulerAnglesDerivative(const Eigen::Quaternion<SCALAR_T>& quaterion);

  /**
   * Computes derivatives of operation -> rotation_matrix * vector (rotated vector)
   * with respect to quaterion
   *
   * @param quaterion: quaterion
   * @return rotated_vector derivative 
   * with respect to quaterion
   *
   */
  template<typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, 3, 4> getRotatedVectorQuaterionGraient(const Eigen::Quaternion<SCALAR_T>& quaterion, const Eigen::Matrix<SCALAR_T, 3, 1> & vector);

  /**
  * Computes the matrix that maps derivatives 
  * of local angular velocities to ZYX-Euler angles derivatives 
  * gradient with respect to quaterions
  *
  * @param [in] quaterions: quaterions
  * @return array of mapping partial derivatives
  */
  template<typename SCALAR_T>
  std::array<Eigen::Matrix<SCALAR_T, 3, 3>, 4> getMappingFromLocalAngularVelocitytoEulerAnglesDerivativeQuaterionGradient(const Eigen::Quaternion<SCALAR_T>& quaterion);



  /*
      TESTING FUNCTIONS
  */
  // JEST GIT !!!!
  template <typename SCALAR_T> 
  Eigen::Matrix<SCALAR_T, 1, 4> test_func_derivative(const Eigen::Quaternion<SCALAR_T>& quaterion)
  {
    Eigen::Matrix<SCALAR_T, 1, 4> return_val;
    return_val << 1, 1, 0, 0;
    return return_val;
  };
  // JEST GIT !!!!
  template <typename SCALAR_T> 
  Eigen::Matrix<SCALAR_T, 1, 3> test_func_derivative(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesZyx)
  {
    const SCALAR_T half = 0.5;
    const SCALAR_T z = eulerAnglesZyx[0] * half;
    const SCALAR_T y = eulerAnglesZyx[1] * half;
    const SCALAR_T x = eulerAnglesZyx[2] * half;

    const SCALAR_T cz = cos(z);
    const SCALAR_T cy = cos(y);
    const SCALAR_T cx = cos(x);
    const SCALAR_T sz = sin(z);
    const SCALAR_T sy = sin(y);
    const SCALAR_T sx = sin(x);

    const SCALAR_T sxcysz = sx * cy * sz;
    const SCALAR_T cxsycz = cx * sy * cz;
    const SCALAR_T cxsysz = cx * sy * sz;
    const SCALAR_T sxcycz = sx * cy * cz;
    const SCALAR_T cxcycz = cx * cy * cz;
    const SCALAR_T sxsysz = sx * sy * sz;
    const SCALAR_T cxcysz = cx * cy * sz;
    const SCALAR_T sxsycz = sx * sy * cz;

    Eigen::Matrix<SCALAR_T, 1, 3> funtionDerivativeMatrix;

    funtionDerivativeMatrix << -half * (sxcysz + cxsycz) - half * (cxsysz - sxcycz),
                               -half * (sxsycz + cxcysz) + half * (cxcycz - sxsysz),
                                half * (cxcycz + sxsysz) - half * (sxsycz - cxcysz);
    
    return funtionDerivativeMatrix;

  };

};




#endif