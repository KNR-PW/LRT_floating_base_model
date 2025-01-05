#ifndef _CENTROIDAL_MODEL_DYNAMICS_HPP_

#define _CENTROIDAL_MODEL_DYNAMICS_HPP_

#include <pinocchio/fwd.hpp>

#include <ostream>
#include <string>
#include <type_traits>


#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>

#include <centroidal_model/CentroidalModelParams.hpp>

using namespace ocs2;

namespace legged_mpc
{

class CentroidalModelDynamics final
{

  public:

  CentroidalModelDynamics(const CentroidalModelParams& params, PinocchioInterface& interface);

  private:

  /**
  * Provides read/write access to the contact forces.
  */
  template <typename Derived>
  Eigen::Block<Derived, 3, 1> getContactForces(Eigen::MatrixBase<Derived>& input, size_t contactIndex);

  /**
   * Provides read access to the contact forces.
   */
  template <typename Derived>
  const Eigen::Block<const Derived, 3, 1> getContactForces(const Eigen::MatrixBase<Derived>& input, size_t contactIndex);
                                                           

  /**
   * Provides read/write access to the contact torques.
   */
  template <typename Derived>
  Eigen::Block<Derived, 3, 1> getContactTorques(Eigen::MatrixBase<Derived>& input, size_t contactIndex);
                                                

  /**
   * Provides read access to the contact torques.
   */
  template <typename Derived>
  const Eigen::Block<const Derived, 3, 1> getContactTorques(const Eigen::MatrixBase<Derived>& input, size_t contactIndex);
                                                            

  /**
   * Provides read/write access to the joint velocities.
   */
  template <typename Derived>
  Eigen::Block<Derived, -1, 1> getJointVelocities(Eigen::MatrixBase<Derived>& input);

  /**
   * Provides read access to the joint velocities.
   */
  template <typename Derived>
  const Eigen::Block<const Derived, -1, 1> getJointVelocities(const Eigen::MatrixBase<Derived>& input);
                                                              
  /**
   * Provides read/write access to the base pose.
   */
  template <typename Derived>
  Eigen::Block<Derived, 6, 1> getBasePose(Eigen::MatrixBase<Derived>& state);

  /**
   * Provides read access to the base pose.
   */
  template <typename Derived>
  const Eigen::Block<const Derived, 6, 1> getBasePose(const Eigen::MatrixBase<Derived>& state);

  /**
   * Provides read/write access to the base velocity.
   */
  template <typename Derived>
  Eigen::Block<Derived, 6, 1> getBaseVelocity(Eigen::MatrixBase<Derived>& state);

  /**
   * Provides read access to the base velocity.
   */
  template <typename Derived>
  const Eigen::Block<const Derived, 6, 1> getBaseVelocity(const Eigen::MatrixBase<Derived>& state);

  /**
   * Provides read/write access to the joint angles.
   */
  template <typename Derived>
  Eigen::Block<Derived, -1, 1> getJointAngles(Eigen::MatrixBase<Derived>& state);

  /**
   * Provides read access to the joint angles.
   */
  template <typename Derived>
  const Eigen::Block<const Derived, -1, 1> getJointAngles(const Eigen::MatrixBase<Derived>& state);
                                                          

  /**
   * Provides read/write access to the generalized coordinates.
   */
  template <typename Derived>
  Eigen::Block<Derived, -1, 1> getGeneralizedCoordinates(Eigen::MatrixBase<Derived>& state);

  /**
   * Provides read access to the generalized coordinates.
   */
  template <typename Derived>
  const Eigen::Block<const Derived, -1, 1> getGeneralizedCoordinates(const Eigen::MatrixBase<Derived>& state);
                                                                     


  const CentroidalModelParams& params_;
  PinocchioInterface& interface_;



};


/**
 * Create a CentroidalModelDynamics PinocchioInterface from a URDF.
 * @param [in] urdfFilePath: The absolute path to the URDF file for the robot.
 */
PinocchioInterface makeCentroidalInterface(const std::string& urdfFilePath);


};

#include "implementation/CentroidalGetters.hpp"

#endif