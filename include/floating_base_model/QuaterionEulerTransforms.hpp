#ifndef __QUATERION_TRANSFORMS_HPP__
#define __QUATERION_TRANSFORMS_HPP__

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
   */
  template <typename SCALAR_T>
  Eigen::Quaternion<SCALAR_T> getQuaternionFromEulerAnglesZyx(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesZyx);

  /**
   * Computes mapping from rotation matrxi to ZYX euler angles
   *
   * @param rotationMatrix: rotation matrix
   * @return quaterion
   */
  template <typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, 3, 1> getEulerAnglesFromRotationMatrix(const Eigen::Matrix<SCALAR_T, 3, 3>& rotationMatrix);

  /**
   * Computes mapping from ZYX euler angles to quaterion derivative
   * with respect to ZYX euler angles
   *
   * @param eulerAnglesZyx: euler angles
   * @return derivative of quaterion with respect to euler angles
   */
  template <typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, 4, 3> getQuaternionFromEulerAnglesZyxGradient(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesZyx);

  /**
   * Computes derivatives of rotation matrix
   * with respect to quaterion dRdq
   *
   * @param quaterion: quaterion
   * @return array of rotation matrix partial derivatives
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
   * @return rotated_vector derivative with respect to quaterion
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


  /* Explicit template instantiation for scalar_t and ad_scalar_t */
  extern template Eigen::Quaternion<ocs2::scalar_t> getQuaternionFromEulerAnglesZyx(const Eigen::Matrix<ocs2::scalar_t, 3, 1>& eulerAnglesZyx);
  extern template Eigen::Quaternion<ocs2::ad_scalar_t> getQuaternionFromEulerAnglesZyx(const Eigen::Matrix<ocs2::ad_scalar_t, 3, 1>& eulerAnglesZyx);

  extern template Eigen::Matrix<ocs2::scalar_t, 3, 1> getEulerAnglesFromRotationMatrix(const Eigen::Matrix<ocs2::scalar_t, 3, 3>& rotationMatrix);
  extern template Eigen::Matrix<ocs2::ad_scalar_t, 3, 1> getEulerAnglesFromRotationMatrix(const Eigen::Matrix<ocs2::ad_scalar_t, 3, 3>& rotationMatrix);

  extern template Eigen::Matrix<ocs2::scalar_t, 4, 3> getQuaternionFromEulerAnglesZyxGradient(const Eigen::Matrix<ocs2::scalar_t, 3, 1>& eulerAnglesZyx);
  extern template Eigen::Matrix<ocs2::ad_scalar_t, 4, 3> getQuaternionFromEulerAnglesZyxGradient(const Eigen::Matrix<ocs2::ad_scalar_t, 3, 1>& eulerAnglesZyx);

  extern template std::array<Eigen::Matrix<ocs2::scalar_t, 3, 3>, 4> getRotationMatrixQuaterionGradient(const Eigen::Quaternion<ocs2::scalar_t>& quaterion);
  extern template std::array<Eigen::Matrix<ocs2::ad_scalar_t, 3, 3>, 4> getRotationMatrixQuaterionGradient(const Eigen::Quaternion<ocs2::ad_scalar_t>& quaterion);

  extern template Eigen::Matrix<ocs2::scalar_t, 3, 3> getMappingFromLocalAngularVelocitytoEulerAnglesDerivative(const Eigen::Matrix<ocs2::scalar_t, 3, 1> &eulerAngles);
  extern template Eigen::Matrix<ocs2::ad_scalar_t, 3, 3> getMappingFromLocalAngularVelocitytoEulerAnglesDerivative(const Eigen::Matrix<ocs2::ad_scalar_t, 3, 1> &eulerAngles);

  extern template std::array<Eigen::Matrix<ocs2::scalar_t, 3, 3>, 3> getMappingFromLocalAngularVelocitytoEulerAnglesDerivativeZyxGradient(const Eigen::Matrix<ocs2::scalar_t, 3, 1> &eulerAngles);
  extern template std::array<Eigen::Matrix<ocs2::ad_scalar_t, 3, 3>, 3> getMappingFromLocalAngularVelocitytoEulerAnglesDerivativeZyxGradient(const Eigen::Matrix<ocs2::ad_scalar_t, 3, 1> &eulerAngles); 

  extern template Eigen::Matrix<ocs2::scalar_t, 3, 3> getMappingFromLocalAngularVelocitytoEulerAnglesDerivative(const Eigen::Quaternion<ocs2::scalar_t>& quaterion);
  extern template Eigen::Matrix<ocs2::ad_scalar_t, 3, 3> getMappingFromLocalAngularVelocitytoEulerAnglesDerivative(const Eigen::Quaternion<ocs2::ad_scalar_t>& quaterion);  

  extern template Eigen::Matrix<ocs2::scalar_t, 3, 4> getRotatedVectorQuaterionGraient(const Eigen::Quaternion<ocs2::scalar_t>& quaterion, const Eigen::Matrix<ocs2::scalar_t, 3, 1> & vector);
  extern template Eigen::Matrix<ocs2::ad_scalar_t, 3, 4> getRotatedVectorQuaterionGraient(const Eigen::Quaternion<ocs2::ad_scalar_t>& quaterion, const Eigen::Matrix<ocs2::ad_scalar_t, 3, 1> & vector);

  extern template std::array<Eigen::Matrix<ocs2::scalar_t, 3, 3>, 4> getMappingFromLocalAngularVelocitytoEulerAnglesDerivativeQuaterionGradient(const Eigen::Quaternion<ocs2::scalar_t>& quaterion); 
  extern template std::array<Eigen::Matrix<ocs2::ad_scalar_t, 3, 3>, 4> getMappingFromLocalAngularVelocitytoEulerAnglesDerivativeQuaterionGradient(const Eigen::Quaternion<ocs2::ad_scalar_t>& quaterion);
};

#endif