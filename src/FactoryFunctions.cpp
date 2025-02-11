#include "floating_base_model/FactoryFunctions.hpp"


namespace floating_base_model
{

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ocs2::PinocchioInterface createPinocchioInterface(const std::string& urdfFilePath, const std::string& baseLinkName)
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
    auto baseLink = newModel->getLink(baseLinkName);

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
        std::cout << jointPair.second->name << std::endl;
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

    for(auto& link : newModel->links_)
    {
      link.second->child_joints.clear();
      link.second->child_links.clear();
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
    return ocs2::getPinocchioInterfaceFromUrdfModel(newModel, freeFlyerJoint);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  FloatingBaseModelInfo createFloatingBaseModelInfo(const ocs2::PinocchioInterface& interface,
    const std::vector<std::string>& threeDofContactNames,
    const std::vector<std::string>& sixDofContactNames)
  {

    FloatingBaseModelInfo info;
    const auto& model = interface.getModel();
    auto data = interface.getData();

    info.numThreeDofContacts = threeDofContactNames.size();
    info.numSixDofContacts = sixDofContactNames.size();
    info.generalizedCoordinatesNum = model.nq;
    info.actuatedDofNum = info.generalizedCoordinatesNum - model.joints[1].nq();
    info.stateDim = info.generalizedCoordinatesNum + model.joints[1].nq();
    info.inputDim = info.actuatedDofNum + 3 * info.numThreeDofContacts + 6 * info.numSixDofContacts;
    info.robotMass = pinocchio::computeTotalMass(model);

    for (const auto& name : threeDofContactNames) {
      info.endEffectorFrameIndices.push_back(model.getBodyId(name));
    }

    for (const auto& name : sixDofContactNames) {
      info.endEffectorFrameIndices.push_back(model.getBodyId(name));
    }
  }

} // namespace floating_base_model