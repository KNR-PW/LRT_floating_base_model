#ifndef __FLOATING_BASE_ACCESS_HELPER_FUNCTIONS__
#define __FLOATING_BASE_ACCESS_HELPER_FUNCTIONS__

#include <Eigen/Core>
#include <floating_base_model/FloatingBaseModelInfo.hpp>


namespace floating_base_model
{
  namespace access_helper_functions
  { 
    /**
    * Provides read/write access to the contact forces.
    * @param [in] input: system input vector
    * @param [in] contactIndex: index of contact frame
    * @param [in] info: info of FloatingBase model
    * @return block with contact force
    */
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 3, 1> getContactForces(Eigen::MatrixBase<Derived>& input,
      size_t contactIndex, 
      const FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read access to the contact forces.
    * @param [in] input: system input vector
    * @param [in] contactIndex: index of contact frame
    * @param [in] info: info of FloatingBase model
    * @return block with contact force
    */
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 3, 1> getContactForces(const Eigen::MatrixBase<Derived>& input, 
      size_t contactIndex,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read/write access to the contact torques.
    * @param [in] input: system input vector
    * @param [in] contactIndex: index of contact frame
    * @param [in] info: info of FloatingBase model
    * @return block with contact torque
    */
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 3, 1> getContactTorques(Eigen::MatrixBase<Derived>& input,
      size_t contactIndex,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read access to the contact torques.
    * @param [in] input: system input vector
    * @param [in] contactIndex: index of contact frame
    * @param [in] info: info of FloatingBase model
    * @return block with contact torque
    */
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 3, 1> getContactTorques(const Eigen::MatrixBase<Derived>& input,
      size_t contactIndex,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info);
    
    /**
    * Provides read/write access to the contact wrenches [force, torque].
    * @param [in] input: system input vector
    * @param [in] contactIndex: index of contact frame
    * @param [in] info: info of FloatingBase model
    * @return block with contact wrenches [force, torque]
    */
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<const Derived, 6, 1> getContactWrenches(Eigen::MatrixBase<Derived>& input,
      size_t contactIndex,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info);
    
    /**
    * Provides read access to the contact wrenches [force, torque].
    * @param [in] input: system input vector
    * @param [in] contactIndex: index of contact frame
    * @param [in] info: info of FloatingBase model
    * @return block with contact wrenches [force, torque]
    */
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 6, 1> getContactWrenches(const Eigen::MatrixBase<Derived>& input,
      size_t contactIndex,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read/write access to the base pose: [position, orientation zyx].
    * @param [in] state: system state vector
    * @param [in] info: info of FloatingBase model
    * @return block with base pose: [position, orientation zyx]
    * @note Orientation is defined in Euler ZYX angles
    */
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 6, 1> getBasePose(Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read access to the base pose: [position, orientation zyx].
    * @param [in] state: system state vector
    * @param [in] info: info of FloatingBase model
    * @return block with base pose: [position, orientation zyx]
    * @note Orientation is defined in Euler ZYX angles
    */
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 6, 1> getBasePose(const Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read/write access to the base position.
    * @param [in] state: system state vector
    * @param [in] info: info of FloatingBase model
    * @return block with base position
    */
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 3, 1> getBasePosition(Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read access to the base position.
    * @param [in] state: system state vector
    * @param [in] info: info of FloatingBase model
    * @return block with base position
    */
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 3, 1> getBasePosition(const Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read/write access to the base orientation zyx.
    * @param [in] state: system state vector
    * @param [in] info: info of FloatingBase model
    * @return block with base orientation zyx
    * @note Orientation is defined in Euler ZYX angles
    */
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 3, 1> getBaseOrientationZyx(Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read access to the base orientation zyx.
    * @param [in] state: system state vector
    * @param [in] info: info of FloatingBase model
    * @return block with base orientation zyx
    * @note Orientation is defined in Euler ZYX angles
    */
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 3, 1> getBaseOrientationZyx(const Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info);

    
    /**
    * Provides read/write access to the base 
    * velocity [linear velocity, angular velocity].
    * @param [in] state: system state vector
    * @param [in] info: info of FloatingBase model
    * @return block with base velocity [linear velocity, angular velocity]
    * @note Linear and angular velocities are described in base frame
    */
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 6, 1> getBaseVelocity(Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read access to the base 
    * velocity [linear velocity, angular velocity].
    * @param [in] state: system state vector
    * @param [in] info: info of FloatingBase model
    * @return block with base velocity [linear velocity, angular velocity]
    * @note Linear and angular velocities are described in base frame
    */
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 6, 1> getBaseVelocity(const Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info);
    
    /**
    * Provides read/write access to the base linear velocity.
    * @param [in] state: system state vector
    * @param [in] info: info of FloatingBase model
    * @note Linear velocitiy is described in base frame
    */
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 3, 1> getBaseLinearVelocity(Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read access to the base linear velocity.
    * @param [in] state: system state vector
    * @param [in] info: info of FloatingBase model
    * @note Linear velocitiy is described in base frame
    */
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 3, 1> getBaseLinearVelocity(const Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info);
    
    /**
    * Provides read/write access to the base angular velocity.
    * @param [in] state: system state vector
    * @param [in] info: info of FloatingBase model
    * @note Angular velocitiy is described in base frame
    */
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 3, 1> getBaseAngularVelocity(Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read access to the base angular velocity.
    * @param [in] state: system state vector
    * @param [in] info: info of FloatingBase model
    * @note Angular velocitiy is described in base frame
    */
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 3, 1> getBaseAngularVelocity(const Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read/write access to the FloatingBase model generalized coordinates.
    * @param [in] state: system state vector
    * @param [in] info: info of FloatingBase model
    * @return block with FloatingBase model generalized coordinates
    * @note This is not joint configuration vector (q)
    */
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, -1, 1> getGeneralizedCoordinates(Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read access to the FloatingBase model generalized coordinates.
    * @param [in] state: system state vector
    * @param [in] info: info of FloatingBase model
    * @return block with FloatingBase model generalized coordinates
    * @note This is not joint configuration vector (q)
    */
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, -1, 1> getGeneralizedCoordinates(const Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read/write access to the actuated joint angles.
    * @param [in] state: system state vector
    * @param [in] info: info of FloatingBase model
    * @return block with actuated joint angles
    */
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, -1, 1> getJointAngles(Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read access to the actuated joint angles.
    * @param [in] state: system state vector
    * @param [in] info: info of FloatingBase model
    * @return block with actuated joint angles
    */
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, -1, 1> getJointAngles(const Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info);
    
    /**
    * Provides read/write access to the actuated joint velocities.
    * @param [in] input: system input vector
    * @param [in] info: info of FloatingBase model
    * @return block with actuated joint velocities
    */
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, -1, 1> getJointVelocities(Eigen::MatrixBase<Derived>& input,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read access to the actuated joint velocities.
    * @param [in] input: system input vector
    * @param [in] info: info of FloatingBase model
    * @return block with actuated joint velocities
    */
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, -1, 1> getJointVelocities(const Eigen::MatrixBase<Derived>& input,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info);
      
  }; // namespace acess_helper_functions
}; // namespace floating_base_model

#include "implementation/AccessHelperFunctions.hxx"

#endif