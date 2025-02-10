#include <floating_base_model/FloatingBaseModelDynamics.hpp>
#include <urdf_model/model.h>

using namespace floating_base_model;

FloatingBaseModelDynamics::FloatingBaseModelDynamics(const FloatingBaseModelParams& params, PinocchioInterface& interface):
params_{params}, interface_{interface}{ }



PinocchioInterface floating_base_model::makeFloatingBaseInterface(const std::string& urdfFilePath)
{
  pinocchio::JointModelFreeFlyer freeFlyerJoint;
  return getPinocchioInterfaceFromUrdfFile(urdfFilePath, freeFlyerJoint);
}

PinocchioInterface floating_base_model::makeFloatingBaseInterface2(const std::string& urdfFilePath, const std::string baseFrameName)
{
  using joint_pair_t = std::pair<const std::string, std::shared_ptr<::urdf::Joint>>;
  using link_pair_t = std::pair<const std::string, std::shared_ptr<::urdf::Link>>;

  ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDFFile(urdfFilePath);
  if (urdfTree == nullptr) {
    throw std::invalid_argument("The file " + urdfFilePath + " does not contain a valid URDF model!");
  }

  // remove extraneous joints from urdf
  std::vector<std::string> jointsToRemoveNames;
  std::vector<std::string> linksToRemoveNames; 

  ::urdf::ModelInterfaceSharedPtr newModel = std::make_shared<::urdf::ModelInterface>(*urdfTree);
  auto baseLink = newModel->getLink(baseFrameName);

  while(baseLink->getParent() != nullptr)
  {
    linksToRemoveNames.push_back(baseLink->getParent()->name);
    baseLink = baseLink->getParent();
  }

  for (joint_pair_t& jointPair : newModel->joints_) 
  {
    std::string parent_name = jointPair.second->parent_link_name;
    if (std::find(linksToRemoveNames.begin(), linksToRemoveNames.end(), parent_name) != linksToRemoveNames.end()) 
    {
      jointsToRemoveNames.push_back(jointPair.second->name);
    }
  }

  for(auto& jointToRemoveName : jointsToRemoveNames)
  {
    newModel->joints_.erase(jointToRemoveName);
  }

  for(auto& linkToRemoveName : linksToRemoveNames)
  {
    newModel->links_.erase(linkToRemoveName);
  }
  
  std::map<std::string, std::string> parent_link_tree;
  try
  {
    newModel->initTree(parent_link_tree);
  }
  catch(::urdf::ParseError &e)
  {
    std::cout << "Failed to build tree: " << e.what() << std::endl;
    newModel.reset();
  }

  try
  {
    newModel->initRoot(parent_link_tree);
  }
  catch(::urdf::ParseError &e)
  {
    std::cout << "Failed to build tree: " << e.what() << std::endl;
    newModel.reset();
  }
  
  pinocchio::JointModelFreeFlyer freeFlyerJoint;
  return getPinocchioInterfaceFromUrdfFile(urdfFilePath, freeFlyerJoint);
}



vector_t FloatingBaseModelDynamics::getValue(scalar_t time, const vector_t& state, const vector_t& input)
{
  assert(params_.stateDim == state.rows());
  assert(params_.inputDim == input.rows());
  vector_t f(params_.stateDim);
  Eigen::Matrix<scalar_t, 6, 1> centroidalMomentumRate = getCentroidalMomentumRate(input);
  const auto baseVelocity = floating_base_model::getBaseVelocity(params_, state);
  const auto jointVelocities = floating_base_model::getJointVelocities(params_, input);
  f << baseVelocity, jointVelocities; // Dodaj przyspieszenia BASE
  return f;
}

Eigen::Matrix<scalar_t, 6, 1> FloatingBaseModelDynamics::getCentroidalMomentumRate(const Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>& input)
{
  Eigen::Matrix<scalar_t, 6, 1> centroidalMomentumRate;
  centroidalMomentumRate << params_.robotMass * gravityVector_, Eigen::Matrix<scalar_t, 3, 1>::Zero();

  // for (size_t i = 0; i < info.numThreeDofContacts; i++) {
  //   const auto contactForceInWorldFrame = floating_base_model::getContactForces(params_, input, i);
  //   const auto positionComToContactPointInWorldFrame = getPositionComToContactPointInWorldFrame(interface, info, i);
  //   centroidalMomentumRate.template head<3>() += contactForceInWorldFrame;
  //   centroidalMomentumRate.template tail<3>().noalias() += positionComToContactPointInWorldFrame.cross(contactForceInWorldFrame);
  // }  // end of i loop

  // for (size_t i = info.numThreeDofContacts; i < info.numThreeDofContacts + info.numSixDofContacts; i++) {
  //   const auto contactForceInWorldFrame = floating_base_model::getContactForces(input, i, info);
  //   const auto contactTorqueInWorldFrame = floating_base_model::getContactTorques(input, i, info);
  //   const auto positionComToContactPointInWorldFrame = getPositionComToContactPointInWorldFrame(interface, info, i);
  //   centroidalMomentumRate.template head<3>() += contactForceInWorldFrame;
  //   centroidalMomentumRate.template tail<3>().noalias() +=
  //       positionComToContactPointInWorldFrame.cross(contactForceInWorldFrame) + contactTorqueInWorldFrame;
  // }  // end of i loop

  // // normalize by the total mass
  // centroidalMomentumRate /= info.robotMass;

  return centroidalMomentumRate;
}