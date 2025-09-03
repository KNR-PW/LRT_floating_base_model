#include <gtest/gtest.h>

#include <floating_base_model/AccessHelperFunctions.hpp>
#include <floating_base_model/FactoryFunctions.hpp>
#include "include/definitions.h"

using namespace floating_base_model;
using namespace access_helper_functions;



TEST(AccessHelperFunctions, StateConst)
{
  ocs2::PinocchioInterface interface = createPinocchioInterface(meldogWithBaseLinkUrdfFile, baseLink);
  auto info = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);
  const auto state = getAccessTestRobotState();

  // READ ACCESS

  const Eigen::Vector<double, 6> baseVelocity = getBaseVelocity(state, info);
  const Eigen::Vector<double, 3> baseLinearVelocity = getBaseLinearVelocity(state, info);
  const Eigen::Vector<double, 3> baseAngularVelocity = getBaseAngularVelocity(state, info);

  ASSERT_TRUE(baseLinearVelocity== Eigen::Vector3d(0.0, 1.0, 2.0));
  ASSERT_TRUE(baseAngularVelocity == Eigen::Vector3d(3.0, 4.0, 5.0));
  const Eigen::Vector<double, 6> baseVelocityTrue = Eigen::Vector<double, 6>(0.0, 1.0, 2.0, 3.0, 4.0, 5.0);
  ASSERT_TRUE(baseVelocity == baseVelocityTrue);
  
  const Eigen::Vector<double, 6> basePose = getBasePose(state, info);
  const Eigen::Vector<double, 3> basePosition = getBasePosition(state, info);
  const Eigen::Vector<double, 3> baseOrientation = getBaseOrientationZyx(state, info);

  ASSERT_TRUE(basePosition == Eigen::Vector3d(6.0, 7.0, 8.0));
  ASSERT_TRUE(baseOrientation == Eigen::Vector3d(9.0, 10.0, 11.0));
  const Eigen::Vector<double, 6> basePoseTrue = Eigen::Vector<double, 6>(6.0, 7.0, 8.0, 9.0, 10.0, 11.0);
  ASSERT_TRUE(basePose == basePoseTrue);


  const Eigen::Vector<double, 12> jointAngles = getJointAngles(state, info);
  const Eigen::Vector<double, 12> jointAnglesTrue = state.block<12, 1>(12, 0);
  ASSERT_TRUE(jointAngles == jointAnglesTrue);

  const Eigen::Vector<double, 18> generalizedCoordinates = getGeneralizedCoordinates(state, info);
  Eigen::Vector<double, 18> generalizedCoordinatesTrue;
  generalizedCoordinatesTrue << Eigen::Vector<double, 6>(6.0, 7.0, 8.0, 9.0, 10.0, 11.0), jointAnglesTrue;
  ASSERT_TRUE(generalizedCoordinates == generalizedCoordinatesTrue);
}

TEST(AccessHelperFunctions, StateNonConst)
{
  ocs2::PinocchioInterface interface = createPinocchioInterface(meldogWithBaseLinkUrdfFile, baseLink);
  auto info = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);
  const auto state = getAccessTestRobotState();

  // READ/WRITE ACCESS

  Eigen::Vector<double, 6> baseVelocity = getBaseVelocity(state, info);
  Eigen::Vector<double, 3> baseLinearVelocity = getBaseLinearVelocity(state, info);
  Eigen::Vector<double, 3> baseAngularVelocity = getBaseAngularVelocity(state, info);

  ASSERT_TRUE(baseLinearVelocity== Eigen::Vector3d(0.0, 1.0, 2.0));
  ASSERT_TRUE(baseAngularVelocity == Eigen::Vector3d(3.0, 4.0, 5.0));
  Eigen::Vector<double, 6> baseVelocityTrue = Eigen::Vector<double, 6>(0.0, 1.0, 2.0, 3.0, 4.0, 5.0);
  ASSERT_TRUE(baseVelocity == baseVelocityTrue);
  
  Eigen::Vector<double, 6> basePose = getBasePose(state, info);
  Eigen::Vector<double, 3> basePosition = getBasePosition(state, info);
  Eigen::Vector<double, 3> baseOrientation = getBaseOrientationZyx(state, info);

  ASSERT_TRUE(basePosition == Eigen::Vector3d(6.0, 7.0, 8.0));
  ASSERT_TRUE(baseOrientation == Eigen::Vector3d(9.0, 10.0, 11.0));
  Eigen::Vector<double, 6> basePoseTrue = Eigen::Vector<double, 6>(6.0, 7.0, 8.0, 9.0, 10.0, 11.0);
  ASSERT_TRUE(basePose == basePoseTrue);


  Eigen::Vector<double, 12> jointAngles = getJointAngles(state, info);
  Eigen::Vector<double, 12> jointAnglesTrue = state.block<12, 1>(12, 0);
  ASSERT_TRUE(jointAngles == jointAnglesTrue);

  Eigen::Vector<double, 18> generalizedCoordinates = getGeneralizedCoordinates(state, info);
  Eigen::Vector<double, 18> generalizedCoordinatesTrue;
  generalizedCoordinatesTrue << Eigen::Vector<double, 6>(6.0, 7.0, 8.0, 9.0, 10.0, 11.0), jointAnglesTrue;
  ASSERT_TRUE(generalizedCoordinates == generalizedCoordinatesTrue);
}

TEST(AccessHelperFunctions, InputConst)
{
  ocs2::PinocchioInterface interface = createPinocchioInterface(meldogWithBaseLinkUrdfFile, baseLink);
  auto info = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);
  const auto input = getAccessTestRobotInput();

  // READ ACCESS

  const Eigen::Vector3d forceRFFLink = getContactForces(input, 0, info);
  const Eigen::Vector3d forceRRFLink = getContactForces(input, 1, info);
  const Eigen::Vector3d forceLFFLink = getContactForces(input, 2, info);
  const Eigen::Vector3d forceLRFLink = getContactForces(input, 3, info);

  ASSERT_TRUE(forceRFFLink == Eigen::Vector3d(0.0, 1.0, 2.0));
  ASSERT_TRUE(forceRRFLink == Eigen::Vector3d(3.0, 4.0, 5.0));
  ASSERT_TRUE(forceLFFLink == Eigen::Vector3d(6.0, 7.0, 8.0));
  ASSERT_TRUE(forceLRFLink == Eigen::Vector3d(12.0, 13.0, 14.0));


  EXPECT_EXIT(getContactTorques(input, 0, info), ::testing::KilledBySignal(SIGABRT), ".*");
  EXPECT_EXIT(getContactTorques(input, 1, info), ::testing::KilledBySignal(SIGABRT), ".*");

  const Eigen::Vector3d torqueLFFLink = getContactTorques(input, 2, info);
  const Eigen::Vector3d torqueLRFLink = getContactTorques(input, 3, info);
  ASSERT_TRUE(torqueLFFLink == Eigen::Vector3d(9.0, 10.0, 11.0));
  ASSERT_TRUE(torqueLRFLink == Eigen::Vector3d(15.0, 16.0, 17.0));

  EXPECT_EXIT(getContactWrenches(input, 0, info), ::testing::KilledBySignal(SIGABRT), ".*");
  EXPECT_EXIT(getContactWrenches(input, 1, info), ::testing::KilledBySignal(SIGABRT), ".*");

  const Eigen::Vector<double, 6> wrenchLFFLink = getContactWrenches(input, 2, info);
  const Eigen::Vector<double, 6> wrenchLRFLink = getContactWrenches(input, 3, info);

  const Eigen::Vector<double, 6> wrenchLFFLinkTrue = Eigen::Vector<double, 6>(6.0, 7.0, 8.0, 9.0, 10.0, 11.0);
  const Eigen::Vector<double, 6> wrenchLRFLinkTrue = Eigen::Vector<double, 6>(12.0, 13.0, 14.0, 15.0, 16.0, 17.0);
  ASSERT_TRUE(wrenchLFFLink == wrenchLFFLinkTrue);
  ASSERT_TRUE(wrenchLRFLink == wrenchLRFLinkTrue);

  const Eigen::Vector<double, 12> jointVelocities = getJointVelocities(input, info);
  const Eigen::Vector<double, 12> jointVelocitiesTrue = input.block<12, 1>(18, 0);
  ASSERT_TRUE(jointVelocities == jointVelocitiesTrue);
}

TEST(AccessHelperFunctions, InputNonConst)
{
  ocs2::PinocchioInterface interface = createPinocchioInterface(meldogWithBaseLinkUrdfFile, baseLink);
  auto info = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);
  const auto input = getAccessTestRobotInput();

  // READ/WRITE ACCESS
  
  Eigen::Vector3d forceRFFLink = getContactForces(input, 0, info);
  Eigen::Vector3d forceRRFLink = getContactForces(input, 1, info);
  Eigen::Vector3d forceLFFLink = getContactForces(input, 2, info);
  Eigen::Vector3d forceLRFLink = getContactForces(input, 3, info);

  ASSERT_TRUE(forceRFFLink == Eigen::Vector3d(0.0, 1.0, 2.0));
  ASSERT_TRUE(forceRRFLink == Eigen::Vector3d(3.0, 4.0, 5.0));
  ASSERT_TRUE(forceLFFLink == Eigen::Vector3d(6.0, 7.0, 8.0));
  ASSERT_TRUE(forceLRFLink == Eigen::Vector3d(12.0, 13.0, 14.0));


  EXPECT_EXIT(getContactTorques(input, 0, info), ::testing::KilledBySignal(SIGABRT), ".*");
  EXPECT_EXIT(getContactTorques(input, 1, info), ::testing::KilledBySignal(SIGABRT), ".*");

  Eigen::Vector3d torqueLFFLink = getContactTorques(input, 2, info);
  Eigen::Vector3d torqueLRFLink = getContactTorques(input, 3, info);
  ASSERT_TRUE(torqueLFFLink == Eigen::Vector3d(9.0, 10.0, 11.0));
  ASSERT_TRUE(torqueLRFLink == Eigen::Vector3d(15.0, 16.0, 17.0));

  EXPECT_EXIT(getContactWrenches(input, 0, info), ::testing::KilledBySignal(SIGABRT), ".*");
  EXPECT_EXIT(getContactWrenches(input, 1, info), ::testing::KilledBySignal(SIGABRT), ".*");

  Eigen::Vector<double, 6> wrenchLFFLink = getContactWrenches(input, 2, info);
  Eigen::Vector<double, 6> wrenchLRFLink = getContactWrenches(input, 3, info);

  Eigen::Vector<double, 6> wrenchLFFLinkTrue = Eigen::Vector<double, 6>(6.0, 7.0, 8.0, 9.0, 10.0, 11.0);
  Eigen::Vector<double, 6> wrenchLRFLinkTrue = Eigen::Vector<double, 6>(12.0, 13.0, 14.0, 15.0, 16.0, 17.0);
  ASSERT_TRUE(wrenchLFFLink == wrenchLFFLinkTrue);
  ASSERT_TRUE(wrenchLRFLink == wrenchLRFLinkTrue);

  Eigen::Vector<double, 12> jointVelocities = getJointVelocities(input, info);
  Eigen::Vector<double, 12> jointVelocitiesTrue = input.block<12, 1>(18, 0);
  ASSERT_TRUE(jointVelocities == jointVelocitiesTrue);
}