#include <floating_base_model/FloatingBaseModelInfo.hpp>

namespace floating_base_model
{ 

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::string toString(const FloatingBaseModelInfo& info)
  {
    std::string message;
    message += "|--Definitions of centroidal model--|\n";
    message += "State vector definition:\n";
    message += "x = [base_linear_velocity, base_orientation_zyx_velocity, base_position, base_orientation_zyx,  joint_positions]\n";
    message += "|-----------------------------------|\n";
    message += "Input vector definition: \n";
    message += "u = [contact_forces, contact_wrenches, joint_velocities]\n";
    message += "|-----------------------------------|\n";
    message += "|--Parameters of centroidal model--|\n";
    message += std::string("Number of 3 DOF contacts: ") + std::to_string(info.numThreeDofContacts) + std::string("\n");
    message += std::string("Number of 6 DOF contacts: ") + std::to_string(info.numSixDofContacts) + std::string("\n");
    message += std::string("Number of generalized coorditanes: ") + std::to_string(info.generalizedCoordinatesNum) + std::string("\n");
    message += std::string("State dimension: ")  + std::to_string(info.stateDim) + std::string("\n");
    message += std::string("Input dimension: ")  + std::to_string(info.inputDim) + std::string("\n");
    message += std::string("Robot total mass: ") + std::to_string(info.robotMass) + std::string("\n");
    message += "|----------------------------------|";

    return message;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::ostream& operator<<(std::ostream& os, const FloatingBaseModelInfo& info)
  {
    os << toString(info);
    return os;
  }
  
  // explicit template instantiation
  template struct FloatingBaseModelInfoTpl<ocs2::scalar_t>;
  template struct FloatingBaseModelInfoTpl<ocs2::ad_scalar_t>;

} // namespace floating_base_model