#ifndef __FLOATING_BASE_HELPER_FUNCTIONS__
#define __FLOATING_BASE_HELPER_FUNCTIONS__


#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

namespace floating_base_model 
{
  namespace model_helper_functions
  {
    /**
     * Get the inverse of locked 6D rigid body inertia of the multi-body system Mb.
     * Mb = [m * I_{3,3},           - m * [r_com],
     *       m * [r_com],  Ic - m * [r_com] * [r_com]]
     * 
     *  Mb_inv = [ 1/m I_{3,3} - [r_com] * Ic_inv * cx,    -Ic_inv * [r_com],,
     *             [r_com] * Ic_inv,                         Ic_inv]
     *
     * @param [in] Mb(q): locked 6D rigid body inertia of the multi-body system
     * @return Mb_inv(q): inverse of the 6x6 left-block of Mb(q)
     */
    template <typename SCALAR_T>
    Eigen::Matrix<SCALAR_T, 6, 6> computeFloatingBaseLockedInertiaInverse(const Eigen::Matrix<SCALAR_T, 6, 6>& Mb);

  // /**
  //  * Updates the centroidal momentum matrix in data.Ag and the CoM position in data.com[0] for the FullCentroidalDynamics
  //  * model 
  //  * @param [in] interface: pinocchio robot interface containing model + data
  //  * @param [in] params: floating base model information
  //  * @param [in] q: pinocchio joint positions (generalized coordinates)
  //  *
  //  * @remark: This function also internally calls:
  //  *       pinocchio::forwardKinematics(model, data, q)
  //  *       pinocchio::computeJointJacobians(model, data, q) (only for the FullCentroidalDynamics case)
  //  *       pinocchio::updateFramePlacements(model, data)
  //  */
  // template <typename SCALAR_T>
  // void updateCentroidalDynamics(PinocchioInterfaceTpl<SCALAR_T>& interface, const FloatingBaseModelInfo& params,
  //                               const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q);

  // /**
  //  * Updates the centroidal momentum derivatives (such as in data.dHdq) for the FullCentroidalDynamics model
  //  *
  //  * @param [in] interface: pinocchio robot interface containing model + data
  //  * @param [in] params: floating base model informationn
  //  * @param [in] q: pinocchio joint positions (generalized coordinates)
  //  * @param [in] v: pinocchio joint velocities (derivatives of generalized coordinates)
  //  *
  //  * @remark: This function also internally calls:
  //  *       pinocchio::forwardKinematics(model, data, q)
  //  *       pinocchio::computeJointJacobians(model, data, q)
  //  *       pinocchio::updateFramePlacements(model, data)
  //  */
  // template <typename SCALAR_T>
  // void updateCentroidalDynamicsDerivatives(PinocchioInterfaceTpl<SCALAR_T>& interface, const FloatingBaseModelInfo& params,
  //                                          const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q,
  //                                          const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& v);

  // /**
  //  * Computes derivatives of the mapping (ZYX-Euler angles derivatives --> Global angular velocities)
  //  * with respect to the base orientation (ZYX-Euler angles)
  //  *
  //  * @param [in] eulerAngles: ZYX-Euler angles extracted from qPinocchio
  //  * @return A tensor representing the derivative of the mapping w.r.t the ZYX-Euler angles
  //  */
  // template <typename SCALAR_T>
  // std::array<Eigen::Matrix<SCALAR_T, 3, 3>, 3> getMappingZyxGradient(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles);

  // /**
  //  * Computes derivatives of the rotation matrix (base frame --> world frame) with respect to
  //  * the base orientation (in ZYX-Euler angles)
  //  *
  //  * @param [in] eulerAngles: ZYX-Euler angles extracted from qPinocchio
  //  * @return A tensor representing the derivative of the rotation matrix w.r.t the ZYX-Euler angles
  //  */
  // template <typename SCALAR_T>
  // std::array<Eigen::Matrix<SCALAR_T, 3, 3>, 3> getRotationMatrixZyxGradient(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles);

  // /**
  //  * Computes derivatives of centroidal momentum with respect to the base orientation (in ZYX-Euler angles)
  //  *
  //  * @param [in] interface: pinocchio robot interface containing model + data
  //  * @param [in] params: floating base model information
  //  * @param [in] q: pinocchio joint positions (generalized coordinates)
  //  * @param [in] v: pinocchio joint velocities (derivatives of generalized coordinates)
  //  * @return Derivative of centroidal momentum w.r.t the ZYX-Euler Angles
  //  */
  // template <typename SCALAR_T>
  // Eigen::Matrix<SCALAR_T, 6, 3> getCentroidalMomentumZyxGradient(const PinocchioInterfaceTpl<SCALAR_T>& interface,
  //                                                                const FloatingBaseModelInfo& params,
  //                                                                const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q,
  //                                                                const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& v);

  // /**
  //  * Returns the centroidal momentum matrix from the pinocchioInterface data
  //  * @param [in] interface: pinocchio robot interface containing model + data
  //  * @return centroidal momentum matrix from data.Ag
  //  *
  //  * @note requires pinocchioInterface to be updated with:
  //  *       ocs2::updateCentroidalDynamics(interface, info, q)
  //  */
  // template <typename SCALAR_T>
  // const Eigen::Matrix<SCALAR_T, 6, Eigen::Dynamic>& getCentroidalMomentumMatrix(const PinocchioInterfaceTpl<SCALAR_T>& interface);

  // /**
  //  * Computes the CoM to contact point position in world frame
  //  *
  //  * @param [in] interface: pinocchio robot interface containing model + data
  //  * @param [in] params: floating base model information
  //  * @param [in] contactIndex: index of the contact point
  //  * @return: position of the contact point w.r.t CoM expressed in world frame
  //  *
  //  * @note requires pinocchioInterface to be updated with:
  //  *       ocs2::updateCentroidalDynamics(interface, info, q)
  //  */
  // template <typename SCALAR_T>
  // Eigen::Matrix<SCALAR_T, 3, 1> getPositionComToContactPointInWorldFrame(const PinocchioInterfaceTpl<SCALAR_T>& interface,
  //                                                                        const FloatingBaseModelInfo& params, size_t contactIndex);

  // /**
  //  * Computes the CoM to contact point translational Jacobian in world frame
  //  *
  //  * @param [in] interface: pinocchio robot interface containing model + data
  //  * @param [in] params: floating base model information
  //  * @param [in] contactIndex: index of the contact point
  //  * @return: CoM to contact point translational Jacobian expressed in world frame
  //  *
  //  * @note requires pinocchioInterface to be updated with:
  //  *       ocs2::updateCentroidalDynamics(interface, info, q) (should be called first)
  //  *       pinocchio::computeJointJacobians(model, data, q)
  //  *       pinocchio::updateFramePlacements(model, data)
  //  */
  // // TODO: Need to copy data here because getFrameJacobian() modifies data. Will be fixed in pinocchio version 3.
  // template <typename SCALAR_T>
  // Eigen::Matrix<SCALAR_T, 3, Eigen::Dynamic> getTranslationalJacobianComToContactPointInWorldFrame(
  //     const PinocchioInterfaceTpl<SCALAR_T>& interface, const FloatingBaseModelInfo& params, size_t contactIndex);

  // /**
  //  * Computes the derivative of the normalized centroidal momentum (linear + angular) expressed in the centroidal frame
  //  *
  //  * @param [in] interface: pinocchio robot interface containing model + data
  //  * @param [in] params: floating base model information
  //  * @param [in] input: system input vector
  //  * @return: time derivative of normalized centroidal momentum
  //  *
  //  * @note requires pinocchioInterface to be updated with:
  //  *       ocs2::updateCentroidalDynamics(interface, info, q)
  //  */
  // template <typename SCALAR_T>
  // Eigen::Matrix<SCALAR_T, 6, 1> getNormalizedCentroidalMomentumRate(const PinocchioInterfaceTpl<SCALAR_T>& interface,
  //                                                                   const FloatingBaseModelInfo* params,
  //                                                                   const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& input);
  }; // namespace model_helper_functions
};  // namespace floating_base_model

#include "implementation/ModelHelperFunctions.hxx"

#endif
