#ifndef _CENTROIDAL_MODEL_DYNAMICS_HPP_

#define _CENTROIDAL_MODEL_DYNAMICS_HPP_

#include <pinocchio/fwd.hpp>

#include <ostream>
#include <string>
#include <type_traits>


#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>

#include <floating_base_model/FloatingBaseModelParams.hpp>
#include <floating_base_model/FloatingBaseGetters.hpp>

using namespace ocs2;

namespace floating_base_model
{

class FloatingBaseModelDynamics final
{

  public:
  /**
   * Constructor
   * @param params: Parameters of centroidal model
   * @param interface: Interface for centroidal model
   */
  FloatingBaseModelDynamics(const FloatingBaseModelParams& params, PinocchioInterface& interface);


  /**
   * Computes system flow map x_dot = f(x, u)
   *
   * @param time: time
   * @param state: system state vector
   * @param input: system input vector
   * @return system flow map x_dot = f(x, u)
   *
   * @note requires pinocchioInterface to be updated with:
   *       ocs2::updateCentroidalDynamics(interface, info, q)
   */
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input);

  /**
   * Computes first order approximation of the system flow map x_dot = f(x, u)
   *
   * @param time: time
   * @param state: system state vector
   * @param input: system input vector
   * @return linear approximation of system flow map x_dot = f(x, u)
   *
   * @note requires pinocchioInterface to be updated with:
   *       ocs2::updateCentroidalDynamicsDerivatives(interface, info, q, v)
   */
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input);




  private:

  /**
   * Compute momentum rate of centroidal model 
   * @param input: system input vector
   * @return time derivative of centroidal momentum
   */
  Eigen::Matrix<scalar_t, 6, 1> getCentroidalMomentumRate(const Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>& input);
                                                                  



  const FloatingBaseModelParams& params_;
  PinocchioInterface& interface_;
  const Eigen::Matrix<scalar_t, 3, 1> gravityVector_{scalar_t(0), scalar_t(0), scalar_t(-9.81)};



};


/**
 * Create a FloatingBaseModelDynamics PinocchioInterface from a URDF.
 * @param [in] urdfFilePath: The absolute path to the URDF file for the robot.
 */
PinocchioInterface makeFloatingBaseInterface(const std::string& urdfFilePath);
PinocchioInterface makeFloatingBaseInterface2(const std::string& urdfFilePath, const std::string baseFrameName);


};  // namespace floating_base_model

#endif