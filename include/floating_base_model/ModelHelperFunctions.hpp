#ifndef __FLOATING_BASE_MODEL_HELPER_FUNCTIONS__
#define __FLOATING_BASE_MODEL_HELPER_FUNCTIONS__


#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>
#include <floating_base_model/AccessHelperFunctions.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/SkewSymmetricMatrix.h>

#include <pinocchio/spatial/inertia.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>

namespace floating_base_model 
{
  namespace model_helper_functions
  {

    /**
     * Compute vector of pinocchio forces (used in rigid body algorithms)
     *
     * @param [in] interface: pinocchio interface
     * @param [in] info: floating base model info
     * @param [in] input: system input vector
     * @return fext: vector of pinocchio forces (ForceTpl<SCALAR_T, 0>)
     * 
     * @remark: This function require (before using call):
     * pinocchio::forwardKinematics(model, data, q)
     * pinocchio::updateFramePlacements(model, data)
     * 
     */
    template <typename SCALAR_T>
    void computeForceVector(
      const ocs2::PinocchioInterfaceTpl<SCALAR_T>& interface,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info,
      const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& input,
      pinocchio::container::aligned_vector<pinocchio::ForceTpl<SCALAR_T, 0>>& fext);

    /**
     * Compute floating base generalized torques due to:
     * - Gravity: G(q)
     * - Coriolis Effects: C(q, v) * v
     * - External Forces: -J^t * f_ext
     *
     * @param [in] interface: pinocchio interface
     * @param [in] q: pinocchio joint configuration
     * @param [in] v: pinocchio joint velocity
     * @param [in] fext: external forces expressed in the local frame of the joints (dim model.njoints)
     * @return tau: generalized torques of floating base (6x1 wrench)
     * 
     * @remark: This function also internally calls:
     * pinocchio::rnea(model, data, q, v, 0, fext)
     */
    template <typename SCALAR_T>
    Eigen::Matrix<SCALAR_T, 6, 1> computeFloatingBaseGeneralizedTorques(
      ocs2::PinocchioInterfaceTpl<SCALAR_T>& interface,
      const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q,
      const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& v,
      const pinocchio::container::aligned_vector<pinocchio::ForceTpl<SCALAR_T, 0>>& fext);
    
      /**
     * Compute actuated joint torques due to:
     * - Gravity: G(q)
     * - Coriolis Effects: C(q, v) * v
     * - External Forces: -J^t * f_ext
     *
     * @param [in] interface: pinocchio interface
     * @param [in] info: floating base model info
     * @param [in] q: pinocchio joint configuration
     * @param [in] v: pinocchio joint velocity
     * @param [in] fext: external forces expressed in the local frame of the joints (dim model.njoints)
     * @return tau: generalized torques of actuated joints
     * 
     * @remark: This function also internally calls:
     * pinocchio::rnea(model, data, q, v, 0, fext)
     */
    template <typename SCALAR_T>
    Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> computeActuatedJointGeneralizedTorques(
      ocs2::PinocchioInterfaceTpl<SCALAR_T>& interface,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info,
      const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q,
      const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& v,
      const pinocchio::container::aligned_vector<pinocchio::ForceTpl<SCALAR_T, 0>>& fext);
    

    /**
     * Compute locked 6D rigid body inertia of the multi-body system Mb.
     * Mb = [m * I_{3,3},           - m * [r_com],
     *       m * [r_com],  Ic - m * [r_com] * [r_com]]
     * 
     * @param [in] interface: pinocchio interface
     * @param [in] q: pinocchio joint configuration
     * @return Mb(q): 6x6 left-block of M(q)
     * 
     * @remark: This function require:
     * pinocchio::forwardKinematics(model, data, q)
     * pinocchio::updateFramePlacements(model, data)
     */
    template <typename SCALAR_T>
    Eigen::Matrix<SCALAR_T, 6, 6> computeFloatingBaseLockedInertia(
      ocs2::PinocchioInterfaceTpl<SCALAR_T>& interface);

    /**
     * Compute the inverse of locked 6D rigid body inertia of the multi-body system Mb.
     * Mb = [m * I_{3,3},           - m * [r_com],
     *       m * [r_com],  Ic - m * [r_com] * [r_com]]
     * 
     *  Mb_inv = [ 1/m I_{3,3} - [r_com] * Ic_inv * cx,    -Ic_inv * [r_com],,
     *             [r_com] * Ic_inv,                         Ic_inv]
     *
     * @param [in] Mb(q): locked 6D rigid body inertia of the multi-body system
     * @return Mb_inv(q): inverse of the 6x6 left-block of M(q)
     */
    template <typename SCALAR_T>
    Eigen::Matrix<SCALAR_T, 6, 6> computeFloatingBaseLockedInertiaInverse(
      const Eigen::Matrix<SCALAR_T, 6, 6>& Mb);

    /**
     * Get the inverse of locked 6D rigid body inertia of the multi-body system Mb.
     * Mb = [m * I_{3,3},           - m * [r_com],
     *       m * [r_com],  Ic - m * [r_com] * [r_com]]
     * 
     *  Mb_inv = [ 1/m I_{3,3} - [r_com] * Ic_inv * cx,    -Ic_inv * [r_com],,
     *             [r_com] * Ic_inv,                         Ic_inv]
     *
     * @param [in] Mb(q): locked 6D rigid body inertia of the multi-body system
     * @param [in] tau(q, dq, fext): top rows of inverse dynamics of the multi-body system without acceleration
     * @return aB: spatial acceleration of base frame with respect to base frame
     */
    template <typename SCALAR_T>
    Eigen::Matrix<SCALAR_T, 6, 1> computeBaseBodyAcceleration(
      const Eigen::Matrix<SCALAR_T, 6, 6>& Mb, 
      const Eigen::Matrix<SCALAR_T, 6, 1>& tau);
  


    /* Explicit template instantiation for scalar_t and ad_scalar_t */
    extern template void computeForceVector(
      const ocs2::PinocchioInterfaceTpl<ocs2::scalar_t>& interface,
      const FloatingBaseModelInfoTpl<ocs2::scalar_t>& info,
      const Eigen::Matrix<ocs2::scalar_t, Eigen::Dynamic, 1>& input,
      pinocchio::container::aligned_vector<pinocchio::ForceTpl<ocs2::scalar_t, 0>>& fext);
    
    extern template void computeForceVector(
      const ocs2::PinocchioInterfaceTpl<ocs2::ad_scalar_t>& interface,
      const FloatingBaseModelInfoTpl<ocs2::ad_scalar_t>& info,
      const Eigen::Matrix<ocs2::ad_scalar_t, Eigen::Dynamic, 1>& input,
      pinocchio::container::aligned_vector<pinocchio::ForceTpl<ocs2::ad_scalar_t, 0>>& fext);

    extern template Eigen::Matrix<ocs2::scalar_t, 6, 1> computeFloatingBaseGeneralizedTorques(
      ocs2::PinocchioInterfaceTpl<ocs2::scalar_t>& interface,
      const Eigen::Matrix<ocs2::scalar_t, Eigen::Dynamic, 1>& q,
      const Eigen::Matrix<ocs2::scalar_t, Eigen::Dynamic, 1>& v,
      const pinocchio::container::aligned_vector<pinocchio::ForceTpl<ocs2::scalar_t, 0>>& fext);

    extern template Eigen::Matrix<ocs2::ad_scalar_t, 6, 1> computeFloatingBaseGeneralizedTorques(
      ocs2::PinocchioInterfaceTpl<ocs2::ad_scalar_t>& interface,
      const Eigen::Matrix<ocs2::ad_scalar_t, Eigen::Dynamic, 1>& q,
      const Eigen::Matrix<ocs2::ad_scalar_t, Eigen::Dynamic, 1>& v,
      const pinocchio::container::aligned_vector<pinocchio::ForceTpl<ocs2::ad_scalar_t, 0>>& fext);
    
    extern template Eigen::Matrix<ocs2::scalar_t, Eigen::Dynamic, 1> computeActuatedJointGeneralizedTorques(
      ocs2::PinocchioInterfaceTpl<ocs2::scalar_t>& interface,
      const FloatingBaseModelInfoTpl<ocs2::scalar_t>& info,
      const Eigen::Matrix<ocs2::scalar_t, Eigen::Dynamic, 1>& q,
      const Eigen::Matrix<ocs2::scalar_t, Eigen::Dynamic, 1>& v,
      const pinocchio::container::aligned_vector<pinocchio::ForceTpl<ocs2::scalar_t, 0>>& fext);
    
    extern template Eigen::Matrix<ocs2::ad_scalar_t, Eigen::Dynamic, 1> computeActuatedJointGeneralizedTorques(
      ocs2::PinocchioInterfaceTpl<ocs2::ad_scalar_t>& interface,
      const FloatingBaseModelInfoTpl<ocs2::ad_scalar_t>& info,
      const Eigen::Matrix<ocs2::ad_scalar_t, Eigen::Dynamic, 1>& q,
      const Eigen::Matrix<ocs2::ad_scalar_t, Eigen::Dynamic, 1>& v,
      const pinocchio::container::aligned_vector<pinocchio::ForceTpl<ocs2::ad_scalar_t, 0>>& fext);
    
    extern template Eigen::Matrix<ocs2::scalar_t, 6, 6> computeFloatingBaseLockedInertia(
      ocs2::PinocchioInterfaceTpl<ocs2::scalar_t>& interface);
    
    extern template Eigen::Matrix<ocs2::ad_scalar_t, 6, 6> computeFloatingBaseLockedInertia(
      ocs2::PinocchioInterfaceTpl<ocs2::ad_scalar_t>& interface);

    extern template Eigen::Matrix<ocs2::scalar_t, 6, 6> computeFloatingBaseLockedInertiaInverse(
      const Eigen::Matrix<ocs2::scalar_t, 6, 6>& Mb);

    extern template Eigen::Matrix<ocs2::ad_scalar_t, 6, 6> computeFloatingBaseLockedInertiaInverse(
      const Eigen::Matrix<ocs2::ad_scalar_t, 6, 6>& Mb);

    extern template Eigen::Matrix<ocs2::scalar_t, 6, 1> computeBaseBodyAcceleration(
      const Eigen::Matrix<ocs2::scalar_t, 6, 6>& Mb,
      const Eigen::Matrix<ocs2::scalar_t, 6, 1>& tau);

    extern template Eigen::Matrix<ocs2::ad_scalar_t, 6, 1> computeBaseBodyAcceleration(
      const Eigen::Matrix<ocs2::ad_scalar_t, 6, 6>& Mb,
      const Eigen::Matrix<ocs2::ad_scalar_t, 6, 1>& tau);

  }; // namespace model_helper_functions
};  // namespace floating_base_model

#endif
