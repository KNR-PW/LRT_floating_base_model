#ifndef __FLOATING_BASE_FACTORY_FUNCTIONS__
#define __FLOATING_BASE_FACTORY_FUNCTIONS__

#include <string>
#include <vector>

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
  * Create a scalar-typed CentroidalModelInfo.
  * @param [in] interface: Pinocchio interface
  * @param [in] type: Type of template model (SRBD or FRBD)
  * @param [in] nominalJointAngles: nominal joint angles used in the SRBD model.
  * @param [in] threeDofContactNames: Names of end-effectors with 3 DoF contacts (force)
  * @param [in] sixDofContactNames: Names of end-effectors with 6 DoF contacts (force + torque)
  * @return CentroidalModelInfo
  */
  FloatingBaseModelInfo createFloatingBaseModelInfo(const ocs2::PinocchioInterface& interface,
    const std::vector<std::string>& threeDofContactNames,
    const std::vector<std::string>& sixDofContactNames);

}; // namespace floating_base_model

#endif