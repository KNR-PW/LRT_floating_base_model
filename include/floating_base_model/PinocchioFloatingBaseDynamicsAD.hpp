// Copyright (c) 2025, Koło Naukowe Robotyków
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

/*
 * Authors: Bartłomiej Krajewski (https://github.com/BartlomiejK2)
 */

#ifndef __FLOATING_BASE_MODEL_DYNAMICS_AD_HPP__
#define __FLOATING_BASE_MODEL_DYNAMICS_AD_HPP__

#include <memory>
#include <string>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <floating_base_model/FactoryFunctions.hpp>
#include <floating_base_model/FloatingBaseModelInfo.hpp>
#include <floating_base_model/AccessHelperFunctions.hpp>
#include <floating_base_model/ModelHelperFunctions.hpp>
#include <floating_base_model/FloatingBaseModelPinocchioMapping.hpp>

namespace floating_base_model
{

   /**
   * Floating Base Dynamics:
   *
   * State: x = [ base_linear_velocity, base_angular_velocity, base_position, base_orientation_zyx, joint_positions ]'
   * @remark: The base classical linear and angular velocities are expressed in base frame of reference,
   * where position and orientation are expressed with respect to the world inertial frame
   *
   * Input disturbanceurbance: disturbance = [ wrench acting on floating base (6x1) ]
   * @remark: Contact forces and wrenches are expressed with respect to the inertial frame.
   * @remark: Wrench acting on floating base is applied to center of base frame, 
   * expressed in base frame
  */
  class PinocchioFloatingBaseDynamicsAD final 
  {
    public:

    /**
     * Constructor
     * @param [in] pinocchioInterface : The pinocchio interface.
     * @param [in] FloatingBaseModelInfo : The floating base model information.
     * @param [in] modelName : Name of the generate model library
     * @param [in] modelFolder : Folder to save the model library files to
     * @param [in] recompileLibraries : If true, the model library will be newly compiled. If false, an existing library will be loaded if
     *                                  available.
     * @param [in] verbose : print information.
     */

    PinocchioFloatingBaseDynamicsAD(const ocs2::PinocchioInterface& pinocchioInterface,
      const FloatingBaseModelInfo& info,
      const std::string& modelName,
      const std::string& modelFolder = "/tmp/ocs2",
      bool recompileLibraries = true,
      bool verbose = false);

    /** Copy Constructor */
    PinocchioFloatingBaseDynamicsAD(const PinocchioFloatingBaseDynamicsAD& rhs);

    /**
     * Computes system flow map x_dot = f(x, u)
     *
     * @param time: time
     * @param state: system state vector
     * @param input: system input vector
     * @return system flow map x_dot = f(x, u)
     */
    ocs2::vector_t getValue(ocs2::scalar_t time,
      const ocs2::vector_t& state,
      const ocs2::vector_t& input,
      const Eigen::Matrix<ocs2::scalar_t, 6, 1>& disturbance) const;

    /**
     * Computes first order approximation of the system flow map x_dot = f(x, u)
     *
     * @param time: time
     * @param state: system state vector
     * @param input: system input vector
     * @return linear approximation of system flow map x_dot = f(x, u)
     */
    ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time,
      const ocs2::vector_t& state, const ocs2::vector_t& input,
      const Eigen::Matrix<ocs2::scalar_t, 6, 1>& disturbance) const;

   private:

   ocs2::ad_vector_t getValueCppAd(ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
      const FloatingBaseModelPinocchioMappingCppAd& mapping,
      const ocs2::ad_vector_t& state,
      const ocs2::ad_vector_t& input,
      const Eigen::Matrix<ocs2::ad_scalar_t, 6, 1>& disturbance);

    std::unique_ptr<ocs2::CppAdInterface> systemFlowMapCppAdInterfacePtr_;
  };

};  // namespace floating_base_model

#endif
