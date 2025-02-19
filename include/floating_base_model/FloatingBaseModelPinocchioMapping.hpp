#ifndef __FLOATING_BASE_MODEL_PINOCCHIO_MAPPING__
#define __FLOATING_BASE_MODEL_PINOCCHIO_MAPPING__

#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

#include "floating_base_model/FloatingBaseModelInfo.hpp"
#include "floating_base_model/AccessHelperFunctions.hpp"
#include "floating_base_model/QuaterionEulerTransforms.hpp"
namespace floating_base_model 
{

  template <typename SCALAR_T>
  class FloatingBaseModelPinocchioMappingTpl;

  using FloatingBaseModelPinocchioMapping = FloatingBaseModelPinocchioMappingTpl<ocs2::scalar_t>;
  using FloatingBaseModelPinocchioMappingCppAd = FloatingBaseModelPinocchioMappingTpl<ocs2::ad_scalar_t>;

  /**
   * Floating Base Model Dynamics:
   *
   * State: x = [ base_linear_velocity, base_angular_velocity, base_position, base_orientation_zyx, joint_positions ]'
   * @remark: The base classical linear and angular velocities are expressed in base frame of reference,
   * where position and orientation are expressed with respect to the world inertial frame
   *
   * Input: u = [ contact_forces, contact_wrenches, joint_velocities ]'
   * @remark: Contact forces and wrenches are expressed with respect to the inertial frame.
   *
   *
   * Pinocchio Joint Positions: qPinocchio = [ base_position, base_orientation_quaterion, joint_positions ]'
   * @remark: Base position is expressed with respect to the world inertial frame
   *
   * Pinocchio Joint Velocities: vPinocchio = [ base_linear_velocity, base_angular_velocity, joint_velocities ]'
   * @remark: Base linear and angular velocities are spatial velocities are expressed with respect to the base frame
   */

  template <typename SCALAR_T>
  class FloatingBaseModelPinocchioMappingTpl final : public ocs2::PinocchioStateInputMapping<SCALAR_T> 
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      using vector_t = Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>;
      using matrix_t = Eigen::Matrix<SCALAR_T, Eigen::Dynamic, Eigen::Dynamic>;

      /**
       * Constructor
       * @param [in] floatingBaseModelInfo : floating base model information.
       */
      explicit FloatingBaseModelPinocchioMappingTpl(FloatingBaseModelInfoTpl<SCALAR_T> floatingBaseModelInfo);

      ~FloatingBaseModelPinocchioMappingTpl() override = default;
      FloatingBaseModelPinocchioMappingTpl* clone() const override;

      /** Sets the pinocchio interface for caching
       * @param [in] pinocchioInterface: pinocchio interface on which computations are expected. It will keep a pointer for the getters.
       * @note The pinocchio interface must be set before calling the getters.
       */
      void setPinocchioInterface(const ocs2::PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface) override;

      /**
       * Computes the vector of generalized coordinates (qPinocchio) used by pinocchio functions from the robot state variables
       *
       * @param [in] state: system state vector
       * @return pinocchio joint positions, which are also the robot's position and quaterion orientation
       * with respect to world inertial frame
       */
      vector_t getPinocchioJointPosition(const vector_t& state) const override;

      /**
       * Computes the vector of generalized velocities (vPinocchio) used by pinocchio functions from the robot state and input variables
       * @param [in] state: system state vector
       * @param [in] input: system input vector
       * @return pinocchio joint velocities
       *
       * @note Pincchio joint velocities ARE NOT derivatives of joint positions 
       * in regard to floating base joint, they are tangent space
       */
      vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override;

      /**
       * Maps pinocchio jacobians dfdq, dfdv to OCS2 jacobians dfdx, dfdu.
       * @param [in] state: system state vector
       * @param [in] Jq: jacobian with respect to pinocchio joint positions
       * @param [in] Jv: jacobian with respect to pinocchio joint velocities
       * @return a pair {dfdx, dfdu} containing the jacobians with respect to the system state and input
       */
      std::pair<matrix_t, matrix_t> getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override;

      /**
       * Returns a structure containing robot-specific information needed for the floating base dynamics computations.
       */
      const FloatingBaseModelInfoTpl<SCALAR_T>& getCentroidalModelInfo() const { return floatingBaseModelInfo_; }

    private:
      FloatingBaseModelPinocchioMappingTpl(const FloatingBaseModelPinocchioMappingTpl& rhs);

      const ocs2::PinocchioInterfaceTpl<SCALAR_T>* pinocchioInterfacePtr_;
      const FloatingBaseModelInfoTpl<SCALAR_T> floatingBaseModelInfo_;
  };

  /* Explicit template instantiation for scalar_t and ad_scalar_t */
  extern template class FloatingBaseModelPinocchioMappingTpl<ocs2::scalar_t>;
  extern template class FloatingBaseModelPinocchioMappingTpl<ocs2::ad_scalar_t>;

}  // namespace floating_base_model

#endif