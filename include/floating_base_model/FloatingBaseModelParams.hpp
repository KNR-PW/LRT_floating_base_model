
#ifndef _CENTROIDAL_MODEL_PARAMS_HPP_

#define _CENTROIDAL_MODEL_PARAMS_HPP_

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>

#include <ostream>
#include <string>
#include <type_traits>
#include <urdf_parser/urdf_parser.h>

#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include "ocs2_pinocchio_interface/urdf.h"

using namespace ocs2;

namespace floating_base_model
{

  struct FloatingBaseModelParams
  {
    FloatingBaseModelParams(const PinocchioInterface& interface, 
                        const std::vector<std::string>& threeDofContactNames,
                        const std::vector<std::string>& sixDofContactNames);
    void print();
  
    size_t numThreeDofContacts;                   // 3DOF contacts, force only
    size_t numSixDofContacts;                     // 6DOF contacts, force and torque
    std::vector<size_t> endEffectorFrameIndices;  // indices of end-effector frames [3DOF contacts, 6DOF contacts]
    size_t generalizedCoordinatesNum;             // number of generalized coordinates in the pinocchio model
    size_t actuatedDofNum;                        // number of actuated degrees of freedom
    size_t stateDim;                              // number of states needed to define the system flow map
    size_t inputDim;                              // number of inputs needed to define the system flow map
    scalar_t robotMass;                           // total robot mass
  };

};  // namespace floating_base_model

#endif