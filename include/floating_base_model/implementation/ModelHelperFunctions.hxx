
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/SkewSymmetricMatrix.h>

namespace floating_base_model
{
  namespace model_helper_functions
  {

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename SCALAR_T>
    pinocchio::container::aligned_vector<pinocchio::ForceTpl<SCALAR_T, 0>> 
    computeFloatingBaseGeneralizedTorques(
      ocs2::PinocchioInterfaceTpl<SCALAR_T>& interface,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info,
      const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& input)
    {
      using Force = pinocchio::ForceTpl<SCALAR_T, 0>;
      pinocchio::container::aligned_vector<Force> fext;
      
    };

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename SCALAR_T>
    Eigen::Matrix<SCALAR_T, 6, 1> computeFloatingBaseGeneralizedTorques(
      ocs2::PinocchioInterfaceTpl<SCALAR_T>& interface,
      const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q,
      const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& v,
      const pinocchio::container::aligned_vector<pinocchio::ForceTpl<SCALAR_T, 0>>& fext)
    {
      const auto& model = interface.getModel();
      auto& data = interface.getData();
      Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> a = Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>::Zero(model.nv);
      return pinocchio::rnea(model, data, q, v, a, fext).template block<6, 1>(0, 0);
    };

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename SCALAR_T>
    Eigen::Matrix<SCALAR_T, 6, 6> computeFloatingBaseLockedInertia(
      ocs2::PinocchioInterfaceTpl<SCALAR_T>& interface,
      const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q)
    {
      const auto& model = interface.getModel();
      auto& data = interface.getData();

      pinocchio::forwardKinematics(model, data, q);
      using Inertia = pinocchio::InertiaTpl<SCALAR_T, 0>;
      using SE3 = pinocchio::SE3Tpl<SCALAR_T, 0>;

      Inertia inertia;
      std::vector<SE3> bMi(model.njoints);
      bMi[1] = SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
      inertia = model.inertias[1];
      for(int i = 2; i < model.njoints; ++i)
      {
        int parent = model.parents[i];
        bMi[i] = bMi[parent] * data.liMi[i];
        inertia += bMi[i].act(model.inertias[i]);
      }
      return inertia.matrix();
    };
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename SCALAR_T>
    Eigen::Matrix<SCALAR_T, 6, 6> computeFloatingBaseLockedInertiaInverse(const Eigen::Matrix<SCALAR_T, 6, 6>& Mb)
    {
      const SCALAR_T mass_inv = SCALAR_T(1.0) / Mb(0, 0);
      const Eigen::Matrix<SCALAR_T, 3, 3> cx = mass_inv * Mb.template block<3, 3>(3, 0);
      const Eigen::Matrix<SCALAR_T, 3, 3> Ic_inv = (Mb.template block<3, 3>(3, 3)  + Mb.template block<3, 3>(3, 0) * cx).inverse();
      const Eigen::Matrix<SCALAR_T, 3, 3> Ic_inv_cx = Ic_inv * cx;
      Eigen::Matrix<SCALAR_T, 6, 6> Mb_inv;
      Mb_inv << mass_inv * Eigen::Matrix<SCALAR_T, 3, 3>::Identity() - cx * Ic_inv_cx, cx * Ic_inv,
      - Ic_inv_cx, Ic_inv;
      
      return Mb_inv;
    };

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename SCALAR_T>
    Eigen::Matrix<SCALAR_T, 6, 1> computeBaseBodyAcceleration(const Eigen::Matrix<SCALAR_T, 6, 6>& Mb, const Eigen::Matrix<SCALAR_T, 6, 1>& tau)
    {
      const SCALAR_T mass_inv = SCALAR_T(1.0) / Mb(0, 0);
      const Eigen::Matrix<SCALAR_T, 3, 3> cx = mass_inv * Mb.template block<3, 3>(3, 0);
      const Eigen::Matrix<SCALAR_T, 3, 3> Ic_inv = (Mb.template block<3, 3>(3, 3)  + Mb.template block<3, 3>(3, 0) * cx).inverse();
      
      Eigen::Matrix<SCALAR_T, 6, 1> baseAcceleration;
      baseAcceleration.template block<3,1>(3, 0).noalias() = Ic_inv * (tau.template block<3, 1>(3, 0) - cx * tau.template block<3, 1>(0, 0));
      baseAcceleration.template block<3,1>(0, 0).noalias() = mass_inv * tau.template block<3, 1>(0, 0) + cx * baseAcceleration.template block<3,1>(3, 0);

      return baseAcceleration;
    };
  
  };
}; // namespace floating_base_model
