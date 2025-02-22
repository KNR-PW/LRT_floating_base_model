#ifndef __FLOATING_BASE_HELPER_FUNCTIONS__
#define __FLOATING_BASE_HELPER_FUNCTIONS__


#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <pinocchio/spatial/inertia.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace floating_base_model 
{
  namespace model_helper_functions
  {

    /**
     * Compute locked 6D rigid body inertia of the multi-body system Mb.
     * Mb = [m * I_{3,3},           - m * [r_com],
     *       m * [r_com],  Ic - m * [r_com] * [r_com]]
     * 
     *
     * @param [in] model: pinocchio model of multibody system
     * @param [in] data:  pinocchio data of multibody system
     * @return Mb(q): 6x6 left-block of M(q)
     */
    template <typename SCALAR_T>
    Eigen::Matrix<SCALAR_T, 6, 6> computeFloatingBaseLockedInertia(
      ocs2::PinocchioInterfaceTpl<SCALAR_T>& interface,
      const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q);

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
    Eigen::Matrix<SCALAR_T, 6, 6> computeFloatingBaseLockedInertiaInverse(const Eigen::Matrix<SCALAR_T, 6, 6>& Mb);

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
    Eigen::Matrix<SCALAR_T, 6, 1> computeBaseBodyAcceleration(const Eigen::Matrix<SCALAR_T, 6, 6>& Mb, const Eigen::Matrix<SCALAR_T, 6, 1>& tau);
  
  }; // namespace model_helper_functions
};  // namespace floating_base_model

#include "implementation/ModelHelperFunctions.hxx"

#endif
