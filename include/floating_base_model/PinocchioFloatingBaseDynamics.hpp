#ifndef __FLOATING_BASE_MODEL_DYNAMICS_HPP__

#define __FLOATING_BASE_MODEL_DYNAMICS_HPP__

#include <ostream>
#include <string>
#include <type_traits>

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <floating_base_model/FactoryFunctions.hpp>
#include <floating_base_model/FloatingBaseModelInfo.hpp>
#include <floating_base_model/AccessHelperFunctions.hpp>
#include <floating_base_model/ModelHelperFunctions.hpp>
#include <floating_base_model/FloatingBaseModelPinocchioMapping.hpp>

using namespace ocs2;

namespace floating_base_model
{

  /**
   * Floating Base Dynamics:
   *
   * State: x = [ base_linear_velocity, base_angular_velocity, base_position, base_orientation_zyx, joint_positions ]'
   * @remark: The base classical linear and angular velocities are expressed in base frame of reference,
   * where position and orientation are expressed with respect to the world inertial frame
   *
   * Input: u = [ contact_forces, contact_wrenches, joint_velocities ]'
   * @remark: Contact forces and wrenches are expressed with respect to the inertial frame.
  */
class PinocchioFloatingBaseDynamics final 
{
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 
    using Vector3 = Eigen::Matrix<scalar_t, 3, 1>;
    using Matrix3x = Eigen::Matrix<scalar_t, 3, Eigen::Dynamic>;
    using Matrix6x = Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>;
    using Matrix3 = Eigen::Matrix<scalar_t, 3, 3>;
    using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;
 
    /**
     * Constructor
     * @param [in] FloatingBaseModelInfo : The floating base model information.
     */
    explicit PinocchioFloatingBaseDynamics(FloatingBaseModelInfo floatingBaseModelInfo);
  
    /** Copy Constructor */
    PinocchioFloatingBaseDynamics(const PinocchioFloatingBaseDynamics& rhs);
  
    /** Set the pinocchio interface for caching.
     * @param [in] pinocchioInterface: pinocchio interface on which computations are expected. It will keep a pointer for the getters.
     * @note The pinocchio interface must be set before calling the getters.
     */
    void setPinocchioInterface(PinocchioInterface& pinocchioInterface);
    
    /**
     * Computes system flow map x_dot = f(x, u)
     *
     * @param time: time
     * @param state: system state vector
     * @param input: system input vector
     * @return system flow map x_dot = f(x, u)
     *
     */
    ocs2::vector_t getValue(ocs2::scalar_t time,
       const ocs2::vector_t& state, const ocs2::vector_t& input);
      
    /**
     * Computes first order approximation of the system flow map x_dot = f(x, u)
     *
     * @param time: time
     * @param state: system state vector
     * @param input: system input vector
     * @return linear approximation of system flow map x_dot = f(x, u)
     *
     */
    ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time,
       const ocs2::vector_t& state, const ocs2::vector_t& input);
   
  private:
   
    PinocchioInterface* pinocchioInterfacePtr_;
    FloatingBaseModelPinocchioMapping mapping_;
    
    pinocchio::container::aligned_vector<pinocchio::ForceTpl<ocs2::scalar_t, 0>> fext_;
 
 };

};  // namespace floating_base_model

#endif