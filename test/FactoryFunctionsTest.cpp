#include <gtest/gtest.h>

#include <floating_base_model/FactoryFunctions.hpp>
#include "include/definitions.h"
#include <algorithm>


using namespace floating_base_model;

TEST(FactoryFunctions, PinocchioInterface)
{
  ocs2::PinocchioInterface interfaceWithBaseLink = createPinocchioInterface(meldogWithBaseLinkUrdfFile, baseLink);
  ocs2::PinocchioInterface interfaceWithoutBaseLink = createPinocchioInterface(meldogWithoutBaseLinkUrdfFile, baseLink);

  const auto& modelWithBaseLink = interfaceWithBaseLink.getModel();
  const auto& modelWithoutBaseLink = interfaceWithoutBaseLink.getModel();
  ASSERT_TRUE(modelWithBaseLink == modelWithoutBaseLink);
}

TEST(FactoryFunctions, FloatingBaseModelInfo)
{
  ocs2::PinocchioInterface interface = createPinocchioInterface(meldogWithBaseLinkUrdfFile, baseLink);
  auto info = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);

 
  ASSERT_TRUE(info.numThreeDofContacts == Meldog::NUM_THREE_DOF_CONTACTS);                
  ASSERT_TRUE(info.numSixDofContacts == Meldog::NUM_SIX_DOF_CONTACTS);               
  ASSERT_TRUE(info.generalizedCoordinatesNum == Meldog::GENERALIZED_COORDINATES_NUM);             
  ASSERT_TRUE(info.actuatedDofNum == Meldog::ACTUATED_DOF_NUM);                
  ASSERT_TRUE(info.stateDim == Meldog::STATE_DIM);
  ASSERT_TRUE(info.inputDim == Meldog::INPUT_DIM);
  
  const auto& model = interface.getModel();

  std::vector<size_t> meldogContactJointIndices;

  for(const auto& contactJointName: meldogEndEffectorJointNames)
  {
    meldogContactJointIndices.push_back(model.getJointId(contactJointName));
  }

  std::sort(meldogContactJointIndices.begin(), meldogContactJointIndices.end());
  std::sort(info.endEffectorJointIndices.begin(), info.endEffectorJointIndices.end());
  
  ASSERT_TRUE(info.endEffectorJointIndices == meldogContactJointIndices);
}
