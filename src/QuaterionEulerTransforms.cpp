#include "floating_base_model/QuaterionEulerTransforms.hpp"

namespace quaterion_euler_transforms
{
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  template <typename SCALAR_T>
  Eigen::Quaternion<SCALAR_T> getQuaternionFromEulerAnglesZyx(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesZyx) 
  {
    // clang-format off
    return Eigen::AngleAxis<SCALAR_T>(eulerAnglesZyx(0), Eigen::Matrix<SCALAR_T, 3, 1>::UnitZ()) *
           Eigen::AngleAxis<SCALAR_T>(eulerAnglesZyx(1), Eigen::Matrix<SCALAR_T, 3, 1>::UnitY()) *
           Eigen::AngleAxis<SCALAR_T>(eulerAnglesZyx(2), Eigen::Matrix<SCALAR_T, 3, 1>::UnitX());
    // clang-format on
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  template <typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, 3, 1> getEulerAnglesFromRotationMatrix(const Eigen::Matrix<SCALAR_T, 3, 3>& rotationMatrix)
  {
    Eigen::Matrix<SCALAR_T, 3, 1> eulerAngles;
    if(rotationMatrix(0, 0) == 0 && rotationMatrix(1, 0) == 0)
    {
      eulerAngles(0) = 0;
      eulerAngles(1) = M_PI_2;
      eulerAngles(2) = atan2(rotationMatrix(0, 1), rotationMatrix(1, 1));
    }
    else
    {
      SCALAR_T temp = sqrt(rotationMatrix(0, 0) * rotationMatrix(0, 0) + rotationMatrix(1, 0) * rotationMatrix(1, 0));
      eulerAngles(0) = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));
      eulerAngles(1) = atan2(-rotationMatrix(2, 0), temp);
      eulerAngles(2) = atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));
    }
    return eulerAngles;
  }
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  template <typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, 4, 3> getQuaternionFromEulerAnglesZyxGradient(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesZyx)
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

    Eigen::Matrix<SCALAR_T, 4, 3> quaterionDerivativeMatrix;

                                  // dz euler               // dy euler               //dx euler        
    quaterionDerivativeMatrix << -half * (sxcysz + cxsycz), -half * (sxsycz + cxcysz),  half * (cxcycz + sxsysz), // dx
                                 -half * (cxsysz - sxcycz),  half * (cxcycz - sxsysz), -half * (sxsycz - cxcysz), // dy
                                  half * (cxcycz + sxsysz), -half * (cxsysz + sxcycz), -half * (sxcysz + cxsycz), // dz
                                 -half * (cxcysz - sxsycz), -half * (cxsycz - sxcysz), -half * (sxcycz - cxsysz); // dw
    
    return quaterionDerivativeMatrix;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  template <typename SCALAR_T>
  std::array<Eigen::Matrix<SCALAR_T, 3, 3>, 4> getRotationMatrixQuaterionGradient(const Eigen::Quaternion<SCALAR_T>& quaterion)
  {
    Eigen::Matrix<SCALAR_T, 3, 3> dRdx, dRdy, dRdz, dRdw;

    const SCALAR_T x = 2 * quaterion.x();
    const SCALAR_T y = 2 * quaterion.y();
    const SCALAR_T z = 2 * quaterion.z();
    const SCALAR_T w = 2 * quaterion.w();

    dRdx << x,  y, z,
            y, -x, -w,
            z,  w, -x;

    dRdy << -y,  x,  w,
             x,  y,  z,
            -w,  z, -y;    

    dRdz << -z, -w, x,
             w, -z, y,
             x,  y, z;

    dRdw << w, -z,  y,
            z,  w, -x,
            -y, x,  w;

    return {dRdx, dRdy, dRdz, dRdw};     
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  template<typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, 3, 3> getMappingFromLocalAngularVelocitytoEulerAnglesDerivative(const Eigen::Matrix<SCALAR_T, 3, 1> &eulerAngles) 
  {
    const SCALAR_T sinx = sin(eulerAngles(2));
    const SCALAR_T cosx = cos(eulerAngles(2));
    const SCALAR_T siny = sin(eulerAngles(1));
    const SCALAR_T cosy = cos(eulerAngles(1));
    const SCALAR_T tgy = tan(eulerAngles(1));
    const SCALAR_T inv_cosy = 1/cos(eulerAngles(1));
    Eigen::Matrix<SCALAR_T, 3, 3> transformationMatrix;
    // clang-format off
    transformationMatrix << SCALAR_T(0.0), sinx * inv_cosy, cosx * inv_cosy,
                            SCALAR_T(0.0),          cosx,         -sinx,
                            SCALAR_T(1.0),    sinx * tgy,    cosx * tgy;
    // clang-format on
    return transformationMatrix;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  template<typename SCALAR_T>
  std::array<Eigen::Matrix<SCALAR_T, 3, 3>, 3> getMappingFromLocalAngularVelocitytoEulerAnglesDerivativeZyxGradient(const Eigen::Matrix<SCALAR_T, 3, 1> &eulerAngles) 
  {
    const SCALAR_T sinx = sin(eulerAngles(2));
    const SCALAR_T cosx = cos(eulerAngles(2));
    const SCALAR_T siny = sin(eulerAngles(1));
    const SCALAR_T cosy = cos(eulerAngles(1));
    const SCALAR_T tgy = tan(eulerAngles(1));
    const SCALAR_T inv_cosy = 1/cos(eulerAngles(1));
    const SCALAR_T inv_cosy2 = inv_cosy * inv_cosy;
    Eigen::Matrix<SCALAR_T, 3, 3> dEdz, dEdy, dEdx;

    dEdz = Eigen::Matrix<SCALAR_T, 3, 3>::Zero();

    dEdy << SCALAR_T(0), sinx * siny * inv_cosy2, cosx * siny * inv_cosy2,
            SCALAR_T(0), SCALAR_T(0), SCALAR_T(0),
            SCALAR_T(0), sinx * inv_cosy2, cosx * inv_cosy2;

    dEdx << SCALAR_T(0), cosx * inv_cosy, -sinx * inv_cosy,
            SCALAR_T(0), -sinx, -cosx,
            SCALAR_T(0), cosx * tgy, -sinx * tgy;  
    
    return {dEdz, dEdy, dEdx};
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  template<typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, 3, 3> getMappingFromLocalAngularVelocitytoEulerAnglesDerivative(const Eigen::Quaternion<SCALAR_T>& quaterion) 
  {
    const SCALAR_T x = quaterion.x();
    const SCALAR_T y = quaterion.y();
    const SCALAR_T z = quaterion.z();
    const SCALAR_T w = quaterion.w();

    const SCALAR_T inv_cosy = SCALAR_T(1.0) / sqrt(SCALAR_T(1.0) - SCALAR_T(4.0) * (w * y - x * z) * (w * y - x * z));

    const SCALAR_T sinx = SCALAR_T(2.0) * (w * x + y * z) * inv_cosy ;
    const SCALAR_T cosx = (SCALAR_T(1.0) - SCALAR_T(2.0) * (x * x + y * y)) * inv_cosy;

    const SCALAR_T tgy = SCALAR_T(2.0) * (w * y - x * z) * inv_cosy;
    
    Eigen::Matrix<SCALAR_T, 3, 3> transformationMatrix;
    // clang-format off
    transformationMatrix << SCALAR_T(0.0), sinx * inv_cosy, cosx * inv_cosy,
                            SCALAR_T(0.0),          cosx,         -sinx,
                            SCALAR_T(1.0),    sinx * tgy,    cosx * tgy;
    // clang-format on
    return transformationMatrix;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  template<typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, 3, 4> getRotatedVectorQuaterionGraient(const Eigen::Quaternion<SCALAR_T>& quaterion, const Eigen::Matrix<SCALAR_T, 3, 1> & vector)
  {
    std::array<Eigen::Matrix<SCALAR_T, 3, 3>, 4> dRdq  = getRotationMatrixQuaterionGradient(quaterion);

    Eigen::Matrix<SCALAR_T, 3, 4> dvectordq;

    
    // [dRdx * v | dRdy * v | dRdz * v | dRdw * v |] 3x4
    dvectordq.col(0) = dRdq[0] * vector;
    dvectordq.col(1) = dRdq[1] * vector;
    dvectordq.col(2) = dRdq[2] * vector;
    dvectordq.col(3) = dRdq[3] * vector;

    return dvectordq;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  template<typename SCALAR_T>
  std::array<Eigen::Matrix<SCALAR_T, 3, 3>, 4> getMappingFromLocalAngularVelocitytoEulerAnglesDerivativeQuaterionGradient(const Eigen::Quaternion<SCALAR_T>& quaterion) 
  {
    const SCALAR_T qx = quaterion.x();
    const SCALAR_T qy = quaterion.y();
    const SCALAR_T qz = quaterion.z();
    const SCALAR_T qw = quaterion.w();

    // Helper values
    const SCALAR_T a = sqrt(SCALAR_T(1.0) - SCALAR_T(4.0) * (qw * qy - qx * qz) * (qw * qy - qx * qz));
    const SCALAR_T a2 = a * a;
    const SCALAR_T inv_a = SCALAR_T(1.0) / a;
    const SCALAR_T inv_a2 = inv_a * inv_a;
    const SCALAR_T inv_a3 = inv_a2 * inv_a;
    const SCALAR_T inv_a4 = inv_a3 * inv_a;
    const SCALAR_T b = (SCALAR_T(1.0) - SCALAR_T(2.0) * (qx * qx + qy * qy));
    const SCALAR_T c = 2 * (qw * qx + qy * qz);
    const SCALAR_T d = 2 * (qw * qy - qx * qz);

    const SCALAR_T cd = c * d;
    const SCALAR_T bd = b * d;


    const SCALAR_T sinx = c * inv_a;
    const SCALAR_T cosx = b * inv_a;

    const SCALAR_T cosy = a;
    const SCALAR_T inv_cosy2 = inv_a2;
    const SCALAR_T tgy = d * inv_a;


    const SCALAR_T dsinx_dqx = SCALAR_T(2.0) * (qw * a2 - qz * cd) * inv_a3;
    const SCALAR_T dsinx_dqy = SCALAR_T(2.0) * (qz * a2 + qw * cd) * inv_a3;
    const SCALAR_T dsinx_dqz = SCALAR_T(2.0) * (qy * a2 - qx * cd) * inv_a3;
    const SCALAR_T dsinx_dqw = SCALAR_T(2.0) * (qx * a2 + qy * cd) * inv_a3;

    const SCALAR_T dcosx_dqx = -SCALAR_T(2.0) * (2 * qx * a2 + qz * bd) * inv_a3;
    const SCALAR_T dcosx_dqy = -SCALAR_T(2.0) * (2 * qy * a2 - qw * bd) * inv_a3;
    const SCALAR_T dcosx_dqz = -SCALAR_T(2.0) * qx * bd * inv_a3;
    const SCALAR_T dcosx_dqw =  SCALAR_T(2.0) * qy * bd * inv_a3;

    const SCALAR_T dcosy_dqx =  SCALAR_T(2.0) * tgy * qz;
    const SCALAR_T dcosy_dqy = -SCALAR_T(2.0) * tgy * qw;
    const SCALAR_T dcosy_dqz =  SCALAR_T(2.0) * tgy * qx;
    const SCALAR_T dcosy_dqw = -SCALAR_T(2.0) * tgy * qy;

    const SCALAR_T dtgy_dqx = -SCALAR_T(2.0) * qz * inv_a3;
    const SCALAR_T dtgy_dqy =  SCALAR_T(2.0) * qw * inv_a3;
    const SCALAR_T dtgy_dqz = -SCALAR_T(2.0) * qx * inv_a3;
    const SCALAR_T dtgy_dqw =  SCALAR_T(2.0) * qy * inv_a3;

    Eigen::Matrix<SCALAR_T, 3, 3> dEdx, dEdy, dEdz, dEdw;

    dEdx << SCALAR_T(0), (dsinx_dqx * cosy - sinx * dcosy_dqx) * inv_cosy2, (dcosx_dqx * cosy - cosx * dcosy_dqx) * inv_cosy2,
            SCALAR_T(0), dcosx_dqx, - dsinx_dqx,
            SCALAR_T(0), dsinx_dqx * tgy + sinx * dtgy_dqx, dcosx_dqx * tgy + cosx * dtgy_dqx;
    
    dEdy << SCALAR_T(0), (dsinx_dqy * cosy - sinx * dcosy_dqy) * inv_cosy2, (dcosx_dqy * cosy - cosx * dcosy_dqy) * inv_cosy2,
            SCALAR_T(0), dcosx_dqy, - dsinx_dqy,
            SCALAR_T(0), dsinx_dqy * tgy + sinx * dtgy_dqy, dcosx_dqy * tgy + cosx * dtgy_dqy;

    dEdz << SCALAR_T(0), (dsinx_dqz * cosy - sinx * dcosy_dqz) * inv_cosy2, (dcosx_dqz * cosy - cosx * dcosy_dqz) * inv_cosy2,
            SCALAR_T(0), dcosx_dqz, - dsinx_dqz,
            SCALAR_T(0), dsinx_dqz * tgy + sinx * dtgy_dqz, dcosx_dqz * tgy + cosx * dtgy_dqz;

    dEdw << SCALAR_T(0), (dsinx_dqw * cosy - sinx * dcosy_dqw) * inv_cosy2, (dcosx_dqw * cosy - cosx * dcosy_dqw) * inv_cosy2,
            SCALAR_T(0), dcosx_dqw, - dsinx_dqw,
            SCALAR_T(0), dsinx_dqw * tgy + sinx * dtgy_dqw, dcosx_dqw * tgy + cosx * dtgy_dqw;

    return {dEdx, dEdy, dEdz, dEdw};
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // Explicit template instantiation

  template Eigen::Quaternion<ocs2::scalar_t> getQuaternionFromEulerAnglesZyx(const Eigen::Matrix<ocs2::scalar_t, 3, 1>& eulerAnglesZyx);
  template Eigen::Quaternion<ocs2::ad_scalar_t> getQuaternionFromEulerAnglesZyx(const Eigen::Matrix<ocs2::ad_scalar_t, 3, 1>& eulerAnglesZyx);

  template Eigen::Matrix<ocs2::scalar_t, 3, 1> getEulerAnglesFromRotationMatrix(const Eigen::Matrix<ocs2::scalar_t, 3, 3>& rotationMatrix);
  template Eigen::Matrix<ocs2::ad_scalar_t, 3, 1> getEulerAnglesFromRotationMatrix(const Eigen::Matrix<ocs2::ad_scalar_t, 3, 3>& rotationMatrix);

  template Eigen::Matrix<ocs2::scalar_t, 4, 3> getQuaternionFromEulerAnglesZyxGradient(const Eigen::Matrix<ocs2::scalar_t, 3, 1>& eulerAnglesZyx);
  template Eigen::Matrix<ocs2::ad_scalar_t, 4, 3> getQuaternionFromEulerAnglesZyxGradient(const Eigen::Matrix<ocs2::ad_scalar_t, 3, 1>& eulerAnglesZyx);

  template std::array<Eigen::Matrix<ocs2::scalar_t, 3, 3>, 4> getRotationMatrixQuaterionGradient(const Eigen::Quaternion<ocs2::scalar_t>& quaterion);
  template std::array<Eigen::Matrix<ocs2::ad_scalar_t, 3, 3>, 4> getRotationMatrixQuaterionGradient(const Eigen::Quaternion<ocs2::ad_scalar_t>& quaterion);

  template Eigen::Matrix<ocs2::scalar_t, 3, 3> getMappingFromLocalAngularVelocitytoEulerAnglesDerivative(const Eigen::Matrix<ocs2::scalar_t, 3, 1> &eulerAngles);
  template Eigen::Matrix<ocs2::ad_scalar_t, 3, 3> getMappingFromLocalAngularVelocitytoEulerAnglesDerivative(const Eigen::Matrix<ocs2::ad_scalar_t, 3, 1> &eulerAngles);

  template std::array<Eigen::Matrix<ocs2::scalar_t, 3, 3>, 3> getMappingFromLocalAngularVelocitytoEulerAnglesDerivativeZyxGradient(const Eigen::Matrix<ocs2::scalar_t, 3, 1> &eulerAngles);
  template std::array<Eigen::Matrix<ocs2::ad_scalar_t, 3, 3>, 3> getMappingFromLocalAngularVelocitytoEulerAnglesDerivativeZyxGradient(const Eigen::Matrix<ocs2::ad_scalar_t, 3, 1> &eulerAngles); 

  template Eigen::Matrix<ocs2::scalar_t, 3, 3> getMappingFromLocalAngularVelocitytoEulerAnglesDerivative(const Eigen::Quaternion<ocs2::scalar_t>& quaterion);
  template Eigen::Matrix<ocs2::ad_scalar_t, 3, 3> getMappingFromLocalAngularVelocitytoEulerAnglesDerivative(const Eigen::Quaternion<ocs2::ad_scalar_t>& quaterion);  

  template Eigen::Matrix<ocs2::scalar_t, 3, 4> getRotatedVectorQuaterionGraient(const Eigen::Quaternion<ocs2::scalar_t>& quaterion, const Eigen::Matrix<ocs2::scalar_t, 3, 1> & vector);
  template Eigen::Matrix<ocs2::ad_scalar_t, 3, 4> getRotatedVectorQuaterionGraient(const Eigen::Quaternion<ocs2::ad_scalar_t>& quaterion, const Eigen::Matrix<ocs2::ad_scalar_t, 3, 1> & vector);

  template std::array<Eigen::Matrix<ocs2::scalar_t, 3, 3>, 4> getMappingFromLocalAngularVelocitytoEulerAnglesDerivativeQuaterionGradient(const Eigen::Quaternion<ocs2::scalar_t>& quaterion); 
  template std::array<Eigen::Matrix<ocs2::ad_scalar_t, 3, 3>, 4> getMappingFromLocalAngularVelocitytoEulerAnglesDerivativeQuaterionGradient(const Eigen::Quaternion<ocs2::ad_scalar_t>& quaterion);
}