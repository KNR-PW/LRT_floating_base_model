#include <centroidal_model/CentroidalModelParams.hpp>


using namespace ocs2;

legged_mpc::CentroidalModelParams::CentroidalModelParams(const PinocchioInterface& interface, 
                    const std::vector<std::string>& threeDofContactNames,
                    const std::vector<std::string>& sixDofContactNames)
{
  const auto& model = interface.getModel();
  auto data = interface.getData();

  numThreeDofContacts = threeDofContactNames.size();
  numSixDofContacts = sixDofContactNames.size();
  generalizedCoordinatesNum = model.nq;
  actuatedDofNum = generalizedCoordinatesNum - model.joints[1].nq();
  stateDim = generalizedCoordinatesNum + model.joints[1].nq();
  inputDim = actuatedDofNum + 3 * numThreeDofContacts + 6 * numSixDofContacts;
  robotMass = pinocchio::computeTotalMass(model);

  for (const auto& name : threeDofContactNames) {
    endEffectorFrameIndices.push_back(model.getBodyId(name));
  }

  for (const auto& name : sixDofContactNames) {
    endEffectorFrameIndices.push_back(model.getBodyId(name));
  }
}

void legged_mpc::CentroidalModelParams::CentroidalModelParams::print()
{
  std::cout << "|--Definitions of centroidal model--|" << std::endl;
  std::cout << "State vector definition: " << std::endl;
  std::cout << "x = [base_linear_velocity, base_angular_velocity, base_position, base_orientation_zyx,  joint_positions]" << std::endl;
  std::cout << "|-----------------------------------|" << std::endl;
  std::cout << "Input vector definition: " << std::endl;
  std::cout << "u = [contact_forces, contact_wrenches, joint_velocities]" << std::endl;
  std::cout << "|-----------------------------------|" << std::endl << std::endl;
  std::cout << "|--Parameters of centroidal model--|" << std::endl;
  std::cout << "Number of 3 DOF contacts: " << numThreeDofContacts << std::endl;
  std::cout << "Number of 6 DOF contacts: " << numSixDofContacts << std::endl;
  std::cout << "Number of generalized coorditanes: " << generalizedCoordinatesNum << std::endl;
  std::cout << "State dimension: " << stateDim << std::endl;
  std::cout << "Input dimension: " << inputDim << std::endl;
  std::cout << "Robot total mass: " << robotMass << std::endl;
  std::cout << "|----------------------------------|" << std::endl;
}