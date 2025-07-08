#include <floating_base_model/ModelHelperFunctions.hpp>

namespace floating_base_model
{
  namespace model_helper_functions
  {

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename SCALAR_T>
    void computeForceVectors(
      const ocs2::PinocchioInterfaceTpl<SCALAR_T>& interface,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info,
      const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& input,
      pinocchio::container::aligned_vector<pinocchio::ForceTpl<SCALAR_T, 0>>& fext)
    {
      const auto& model = interface.getModel();
      auto& data = interface.getData();

      assert(fext.size() == model.njoints);

      using Force = pinocchio::ForceTpl<SCALAR_T, 0>;
      Force force;
      force.linear() = Eigen::Vector<SCALAR_T, 3>::Zero();
      force.angular() = Eigen::Vector<SCALAR_T, 3>::Zero();

      for (size_t i = 0; i < info.numThreeDofContacts; i++) {
        const auto forceWorldFrame = access_helper_functions::getContactForces(input, i, info);
        size_t parentJointIndex = info.endEffectorJointIndices[i];
        size_t contactFrameIndex = info.endEffectorFrameIndices[i];
        const auto& contactFrame = model.frames[contactFrameIndex];
        const auto& jointFramePlacement = contactFrame.placement.translation();
        force.linear() = data.oMi[parentJointIndex].rotation().transpose() * forceWorldFrame;
        force.angular() = jointFramePlacement.cross(force.linear());
        fext[parentJointIndex] = force;
      }  
    
      for (size_t i = info.numThreeDofContacts; i < info.numThreeDofContacts + info.numSixDofContacts; i++) {
        const auto wrenchWorldFrame = access_helper_functions::getContactWrenches(input, i, info);
        size_t parentJointIndex = info.endEffectorJointIndices[i];
        size_t contactFrameIndex = info.endEffectorFrameIndices[i];
        const auto& contactFrame = model.frames[contactFrameIndex];
        const auto& jointFramePlacement = contactFrame.placement.translation();
        Eigen::Matrix3<SCALAR_T> worldToJointRotation = data.oMi[parentJointIndex].rotation().transpose();
        force.linear() = worldToJointRotation * wrenchWorldFrame.template block<3,1>(0, 0);
        force.angular() = worldToJointRotation * wrenchWorldFrame.template block<3,1>(3, 0) + jointFramePlacement.cross(force.linear());
        fext[parentJointIndex] = force;
      }  

    }

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
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename SCALAR_T>
    Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> computeActuatedJointGeneralizedTorques(
      ocs2::PinocchioInterfaceTpl<SCALAR_T>& interface,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info,
      const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q,
      const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& v,
      const pinocchio::container::aligned_vector<pinocchio::ForceTpl<SCALAR_T, 0>>& fext)
    {
      const auto& model = interface.getModel();
      auto& data = interface.getData();
      Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> a = Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>::Zero(model.nv);
      return pinocchio::rnea(model, data, q, v, a, fext).block(6, 0, info.actuatedDofNum, 1);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename SCALAR_T>
    Eigen::Matrix<SCALAR_T, 6, 6> computeFloatingBaseLockedInertia(
      ocs2::PinocchioInterfaceTpl<SCALAR_T>& interface)
    {
      const auto& model = interface.getModel();
      auto& data = interface.getData();
      
      using Inertia = pinocchio::InertiaTpl<SCALAR_T, 0>;
      using SE3 = pinocchio::SE3Tpl<SCALAR_T, 0>;

      Inertia inertia;
      std::vector<SE3> bMi(model.njoints);
      bMi[1] = SE3(Eigen::Matrix<SCALAR_T, 3, 3>::Identity(), Eigen::Vector<SCALAR_T, 3>::Zero());
      inertia = model.inertias[1];
      for(pinocchio::JointIndex i = 2; i < (pinocchio::JointIndex)model.njoints; ++i)
      {
        pinocchio::JointIndex parent = model.parents[i];
        bMi[i] = bMi[parent] * data.liMi[i];
        inertia += bMi[i].act(model.inertias[i]);
      }
      return inertia.matrix();
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename SCALAR_T>
    Eigen::Matrix<SCALAR_T, 6, 6> computeFloatingBaseLockedInertiaInverse(
      const Eigen::Matrix<SCALAR_T, 6, 6>& Mb)
    {
      const SCALAR_T mass_inv = SCALAR_T(1.0) / Mb(0, 0);
      const Eigen::Matrix<SCALAR_T, 3, 3> cx = mass_inv * Mb.template block<3, 3>(3, 0);
      const Eigen::Matrix<SCALAR_T, 3, 3> Ic_inv = (Mb.template block<3, 3>(3, 3)  + Mb.template block<3, 3>(3, 0) * cx).inverse();
      const Eigen::Matrix<SCALAR_T, 3, 3> Ic_inv_cx = Ic_inv * cx;
      Eigen::Matrix<SCALAR_T, 6, 6> Mb_inv;
      Mb_inv << mass_inv * Eigen::Matrix<SCALAR_T, 3, 3>::Identity() - cx * Ic_inv_cx, cx * Ic_inv,
      - Ic_inv_cx, Ic_inv;
      
      return Mb_inv;
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename SCALAR_T>
    Eigen::Matrix<SCALAR_T, 6, 1> computeBaseBodyAcceleration(
      const Eigen::Matrix<SCALAR_T, 6, 6>& Mb,
      const Eigen::Matrix<SCALAR_T, 6, 1>& tau)
    {
      const SCALAR_T mass_inv = SCALAR_T(1.0) / Mb(0, 0);
      const Eigen::Matrix<SCALAR_T, 3, 3> cx = mass_inv * Mb.template block<3, 3>(3, 0);
      const Eigen::Matrix<SCALAR_T, 3, 3> Ic_inv = (Mb.template block<3, 3>(3, 3)  + Mb.template block<3, 3>(3, 0) * cx).inverse();
      
      Eigen::Matrix<SCALAR_T, 6, 1> baseAcceleration;
      baseAcceleration.template block<3,1>(3, 0).noalias() = Ic_inv * (tau.template block<3, 1>(3, 0) - cx * tau.template block<3, 1>(0, 0));
      baseAcceleration.template block<3,1>(0, 0).noalias() = mass_inv * tau.template block<3, 1>(0, 0) + cx * baseAcceleration.template block<3,1>(3, 0);

      return baseAcceleration;
    }
    
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    // Explicit template instantiation

    template void computeForceVectors(
      const ocs2::PinocchioInterfaceTpl<ocs2::scalar_t>& interface,
      const FloatingBaseModelInfoTpl<ocs2::scalar_t>& info,
      const Eigen::Matrix<ocs2::scalar_t, Eigen::Dynamic, 1>& input,
      pinocchio::container::aligned_vector<pinocchio::ForceTpl<ocs2::scalar_t, 0>>& fext);
    
    template void computeForceVectors(
      const ocs2::PinocchioInterfaceTpl<ocs2::ad_scalar_t>& interface,
      const FloatingBaseModelInfoTpl<ocs2::ad_scalar_t>& info,
      const Eigen::Matrix<ocs2::ad_scalar_t, Eigen::Dynamic, 1>& input,
      pinocchio::container::aligned_vector<pinocchio::ForceTpl<ocs2::ad_scalar_t, 0>>& fext);

    template Eigen::Matrix<ocs2::scalar_t, 6, 1> computeFloatingBaseGeneralizedTorques(
      ocs2::PinocchioInterfaceTpl<ocs2::scalar_t>& interface,
      const Eigen::Matrix<ocs2::scalar_t, Eigen::Dynamic, 1>& q,
      const Eigen::Matrix<ocs2::scalar_t, Eigen::Dynamic, 1>& v,
      const pinocchio::container::aligned_vector<pinocchio::ForceTpl<ocs2::scalar_t, 0>>& fext);

    template Eigen::Matrix<ocs2::ad_scalar_t, 6, 1> computeFloatingBaseGeneralizedTorques(
      ocs2::PinocchioInterfaceTpl<ocs2::ad_scalar_t>& interface,
      const Eigen::Matrix<ocs2::ad_scalar_t, Eigen::Dynamic, 1>& q,
      const Eigen::Matrix<ocs2::ad_scalar_t, Eigen::Dynamic, 1>& v,
      const pinocchio::container::aligned_vector<pinocchio::ForceTpl<ocs2::ad_scalar_t, 0>>& fext);
    
    template Eigen::Matrix<ocs2::scalar_t, Eigen::Dynamic, 1> computeActuatedJointGeneralizedTorques(
      ocs2::PinocchioInterfaceTpl<ocs2::scalar_t>& interface,
      const FloatingBaseModelInfoTpl<ocs2::scalar_t>& info,
      const Eigen::Matrix<ocs2::scalar_t, Eigen::Dynamic, 1>& q,
      const Eigen::Matrix<ocs2::scalar_t, Eigen::Dynamic, 1>& v,
      const pinocchio::container::aligned_vector<pinocchio::ForceTpl<ocs2::scalar_t, 0>>& fext);
    
    template Eigen::Matrix<ocs2::ad_scalar_t, Eigen::Dynamic, 1> computeActuatedJointGeneralizedTorques(
      ocs2::PinocchioInterfaceTpl<ocs2::ad_scalar_t>& interface,
      const FloatingBaseModelInfoTpl<ocs2::ad_scalar_t>& info,
      const Eigen::Matrix<ocs2::ad_scalar_t, Eigen::Dynamic, 1>& q,
      const Eigen::Matrix<ocs2::ad_scalar_t, Eigen::Dynamic, 1>& v,
      const pinocchio::container::aligned_vector<pinocchio::ForceTpl<ocs2::ad_scalar_t, 0>>& fext);
    
    template Eigen::Matrix<ocs2::scalar_t, 6, 6> computeFloatingBaseLockedInertia(
      ocs2::PinocchioInterfaceTpl<ocs2::scalar_t>& interface);
    
    template Eigen::Matrix<ocs2::ad_scalar_t, 6, 6> computeFloatingBaseLockedInertia(
      ocs2::PinocchioInterfaceTpl<ocs2::ad_scalar_t>& interface);

    template Eigen::Matrix<ocs2::scalar_t, 6, 6> computeFloatingBaseLockedInertiaInverse(
      const Eigen::Matrix<ocs2::scalar_t, 6, 6>& Mb);

    template Eigen::Matrix<ocs2::ad_scalar_t, 6, 6> computeFloatingBaseLockedInertiaInverse(
      const Eigen::Matrix<ocs2::ad_scalar_t, 6, 6>& Mb);

    template Eigen::Matrix<ocs2::scalar_t, 6, 1> computeBaseBodyAcceleration(
      const Eigen::Matrix<ocs2::scalar_t, 6, 6>& Mb,
      const Eigen::Matrix<ocs2::scalar_t, 6, 1>& tau);

    template Eigen::Matrix<ocs2::ad_scalar_t, 6, 1> computeBaseBodyAcceleration(
      const Eigen::Matrix<ocs2::ad_scalar_t, 6, 6>& Mb,
      const Eigen::Matrix<ocs2::ad_scalar_t, 6, 1>& tau);
  };
}; // namespace floating_base_model
