#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <floating_base_model/QuaterionEulerTransforms.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

using namespace quaterion_euler_transforms;

/*
  TESTING FUNCTIONS FOR MAPPINGS
*/
  
template <typename SCALAR_T> 
Eigen::Matrix<SCALAR_T, 1, 4> test_func_derivative(const Eigen::Quaternion<SCALAR_T>& quaterion)
{
  Eigen::Matrix<SCALAR_T, 1, 4> return_val;
  return_val << 1, 1, 0, 0;
  return return_val;
};

template <typename SCALAR_T> 
Eigen::Matrix<SCALAR_T, 1, 3> test_func_derivative(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesZyx)
{
  const SCALAR_T half = SCALAR_T(0.5);
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

TEST(QuaterionEulerTransformsTest, QuaterionAndEulerConversions)
{
  Eigen::Vector3d eulerAngles = Eigen::Vector3d::Random();
  const auto quaterion = getQuaternionFromEulerAnglesZyx(eulerAngles);
  const auto rotationMatrixEuler = ocs2::getRotationMatrixFromZyxEulerAngles(eulerAngles);
  const auto rotationMatrixQuaterion = quaterion.toRotationMatrix();
  const auto norm = (rotationMatrixQuaterion - rotationMatrixEuler).norm();
  ASSERT_NEAR(norm, 0.0, 1e-6);
};


TEST(QuaterionEulerTransformsTest, RotationMatrixEulerConversions)
{
  Eigen::Vector3d eulerAngles = Eigen::Vector3d::Random();
  const auto rotationMatrixEuler = ocs2::getRotationMatrixFromZyxEulerAngles(eulerAngles);
  Eigen::Vector3d eulerAnglesCalculated = getEulerAnglesFromRotationMatrix(rotationMatrixEuler);
  const auto norm = (eulerAnglesCalculated - eulerAngles).norm();
  ASSERT_NEAR(norm, 0.0, 1e-6);
};

TEST(QuaterionEulerTransformsTest, QuaterionAndEulerMappings)
{
  Eigen::Vector3d eulerAngles = Eigen::Vector3d::Random();
  const auto quaterion = getQuaternionFromEulerAnglesZyx(eulerAngles);
  const auto quterion_euler_mapping = getQuaternionFromEulerAnglesZyxGradient(eulerAngles);
  const auto fun_quaterion_derivative = test_func_derivative(quaterion);
  const auto fun_euler_derivative = test_func_derivative(eulerAngles);
  const auto norm = (fun_quaterion_derivative * quterion_euler_mapping - fun_euler_derivative).norm();
  ASSERT_NEAR(norm, 0.0, 1e-6);
};

TEST(QuaterionEulerTransformsTest, QuaterionAndEulerRotationMatrixGradient)
{
  Eigen::Vector3d eulerAngles = Eigen::Vector3d::Random();
  const auto quaterion = getQuaternionFromEulerAnglesZyx(eulerAngles);

  const auto quterion_euler_mapping = getQuaternionFromEulerAnglesZyxGradient(eulerAngles);
  
  const auto dRdq = quaterion_euler_transforms::getRotationMatrixQuaterionGradient(quaterion);
  const auto dRde = ocs2::getRotationMatrixZyxGradient(eulerAngles);

  const auto dRdz_euler = dRde[0];
  const auto dRdy_euler = dRde[1];
  const auto dRdx_euler = dRde[2];

  Eigen::Vector4d dqdz = quterion_euler_mapping.col(0);
  Eigen::Vector4d dqdy = quterion_euler_mapping.col(1);
  Eigen::Vector4d dqdx = quterion_euler_mapping.col(2);

  const auto dRdz_quaterion = dRdq[0] * dqdz[0] + dRdq[1] * dqdz[1] + dRdq[2] * dqdz[2] + dRdq[3] * dqdz[3];
  const auto dRdy_quaterion = dRdq[0] * dqdy[0] + dRdq[1] * dqdy[1] + dRdq[2] * dqdy[2] + dRdq[3] * dqdy[3];
  const auto dRdx_quaterion = dRdq[0] * dqdx[0] + dRdq[1] * dqdx[1] + dRdq[2] * dqdx[2] + dRdq[3] * dqdx[3];
  
  const auto norm_z = (dRdz_quaterion - dRdz_euler).norm();
  const auto norm_y = (dRdy_quaterion - dRdy_euler).norm();
  const auto norm_x = (dRdx_quaterion - dRdx_euler).norm();

  ASSERT_NEAR(norm_z, 0.0, 1e-6);
  ASSERT_NEAR(norm_y, 0.0, 1e-6);
  ASSERT_NEAR(norm_x, 0.0, 1e-6);

  Eigen::Vector3d V = Eigen::Vector3d::Random();
  const auto dVdq = getRotatedVectorQuaterionGraient(quaterion, V);

  Eigen::Matrix<double, 3, 4> dVdq_2;
  dVdq_2 << dRdq[0] * V, dRdq[1] * V, dRdq[2] * V, dRdq[3] * V;

  const auto norm_v = (dVdq  - dVdq_2).norm();
  ASSERT_NEAR(norm_v, 0.0, 1e-6);
};

TEST(QuaterionEulerTransformsTest, LocalAngularVelocityMappings)
{
  Eigen::Vector3d eulerAngles = Eigen::Vector3d::Random();
  const auto quaterion = getQuaternionFromEulerAnglesZyx(eulerAngles);

  const auto E_euler = getMappingFromLocalAngularVelocitytoEulerAnglesDerivative(eulerAngles);
  const auto E_quaterion = getMappingFromLocalAngularVelocitytoEulerAnglesDerivative(quaterion);

  const auto norm_E = (E_euler - E_quaterion).norm();
  ASSERT_NEAR(norm_E, 0.0, 1e-6);

  Eigen::Vector3d localAngularVelocity = Eigen::Vector3d::Random();
  const auto eulerAnglesDerivative = ocs2::getEulerAnglesZyxDerivativesFromLocalAngularVelocity(eulerAngles, localAngularVelocity);

  const auto norm_derivative = (eulerAnglesDerivative - E_euler * localAngularVelocity).norm();
  ASSERT_NEAR(norm_derivative, 0.0, 1e-6);
};

TEST(QuaterionEulerTransformsTest, LocalAngularVelocityMappingGradients)
{
  Eigen::Vector3d eulerAngles = Eigen::Vector3d::Random();
  const auto quaterion = getQuaternionFromEulerAnglesZyx(eulerAngles);
  
  const auto quterion_euler_mapping = getQuaternionFromEulerAnglesZyxGradient(eulerAngles);

  const auto dEde = getMappingFromLocalAngularVelocitytoEulerAnglesDerivativeZyxGradient(eulerAngles);
  const auto dEdq = getMappingFromLocalAngularVelocitytoEulerAnglesDerivativeQuaterionGradient(quaterion);

  const auto dEdz_euler = dEde[0];
  const auto dEdy_euler = dEde[1];
  const auto dEdx_euler = dEde[2];

  Eigen::Vector4d dqdz = quterion_euler_mapping.col(0);
  Eigen::Vector4d dqdy = quterion_euler_mapping.col(1);
  Eigen::Vector4d dqdx = quterion_euler_mapping.col(2);

  const auto dEdz_quaterion = dEdq[0] * dqdz[0] + dEdq[1] * dqdz[1] + dEdq[2] * dqdz[2] + dEdq[3] * dqdz[3];
  const auto dEdy_quaterion = dEdq[0] * dqdy[0] + dEdq[1] * dqdy[1] + dEdq[2] * dqdy[2] + dEdq[3] * dqdy[3];
  const auto dEdx_quaterion = dEdq[0] * dqdx[0] + dEdq[1] * dqdx[1] + dEdq[2] * dqdx[2] + dEdq[3] * dqdx[3];
  
  const auto norm_z = (dEdz_quaterion - dEdz_euler).norm();
  const auto norm_y = (dEdy_quaterion - dEdy_euler).norm();
  const auto norm_x = (dEdx_quaterion - dEdx_euler).norm();

  ASSERT_NEAR(norm_z, 0.0, 1e-6);
  ASSERT_NEAR(norm_y, 0.0, 1e-6);
  ASSERT_NEAR(norm_x, 0.0, 1e-6);
};

