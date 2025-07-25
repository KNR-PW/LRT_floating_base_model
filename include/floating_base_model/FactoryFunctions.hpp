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

#ifndef __FLOATING_BASE_FACTORY_FUNCTIONS__
#define __FLOATING_BASE_FACTORY_FUNCTIONS__

#include <string>
#include <vector>
#include <stdexcept>

#include <urdf_parser/urdf_parser.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

namespace floating_base_model {

  /**
   * Create a FloatingBaseModel PinocchioInterface from a URDF.
   * @param [in] urdfFilePath: The absolute path to the URDF file for the robot.
   * @param [in] baseLinkName: Name of base link (main body of legged robot)
   * @note All links and joints before base link (parents) will be removed,
   *  base link will become root of the model (after default pinocchio world frame)
   */
  ocs2::PinocchioInterface createPinocchioInterface(const std::string& urdfFilePath,
    const std::string& baseLinkName);

  /**
   * Create a scalar-typed FloatingBaseModelInfo.
   * @param [in] interface: Pinocchio interface
   * @param [in] type: Type of template model (SRBD or FRBD)
   * @param [in] nominalJointAngles: nominal joint angles used in the SRBD model.
   * @param [in] threeDofContactNames: Names of end-effectors with 3 DoF contacts (force)
   * @param [in] sixDofContactNames: Names of end-effectors with 6 DoF contacts (force + torque)
   * @return FloatingBaseModelInfo
   */
  FloatingBaseModelInfo createFloatingBaseModelInfo(const ocs2::PinocchioInterface& interface,
    const std::vector<std::string>& threeDofContactNames,
    const std::vector<std::string>& sixDofContactNames);

}; // namespace floating_base_model

#endif