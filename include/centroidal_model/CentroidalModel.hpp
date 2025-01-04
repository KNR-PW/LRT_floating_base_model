#ifndef _CENTROIDAL_MODEL_HPP_

#define _CENTROIDAL_MODEL_HPP_

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <ostream>
#include <string>
#include <type_traits>
#include <urdf_parser/urdf_parser.h>

#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
// #include <ocs2_core/automatic_differentiation/Types.h>

using namespace ocs2;

struct CentroidalModelInfo
{
  CentroidalModelInfo(const PinocchioInterface& _interface, 
                      const std::vector<std::string>& _threeDofContactNames,
                      const std::vector<std::string>& _sixDofContactNames);

  size_t numThreeDofContacts;                   // 3DOF contacts, force only
  size_t numSixDofContacts;                     // 6DOF contacts, force and torque
  std::vector<size_t> endEffectorFrameIndices;  // indices of end-effector frames [3DOF contacts, 6DOF contacts]
  size_t generalizedCoordinatesNum;             // number of generalized coordinates in the pinocchio model
  size_t actuatedDofNum;                        // number of actuated degrees of freedom
  size_t stateDim;                              // number of states needed to define the system flow map
  size_t inputDim;                              // number of inputs needed to define the system flow map
  scalar_t robotMass;                           // total robot mass
  

};


class CentroidalModel
{

  public:

  CentroidalModel(const CentroidalModelInfo& _info, const PinocchioInterface& _interface);


  private:

  CentroidalModelInfo& info_;
  PinocchioInterface& interface_;



};

#endif