#include <gtest/gtest.h>

#include <floating_base_model/QuaterionEulerTransforms.hpp>
#include <floating_base_model/ModelHelperFunctions.hpp>
#include <floating_base_model/FactoryFunctions.hpp>
#include <floating_base_model/FloatingBaseModelPinocchioMapping.hpp>

#include <pinocchio/algorithm/crba.hpp>

#include "include/definitions.h"

using namespace floating_base_model;
using namespace model_helper_functions;
using namespace access_helper_functions;
using namespace quaterion_euler_transforms;

static constexpr ocs2::scalar_t tolerance = 1e-6;
static constexpr size_t numTests = 100;

TEST(FloatingBaseModelPinocchioMapping, Getters)
{
  ocs2::PinocchioInterface interface = createPinocchioInterface(meldogWithBaseLinkUrdfFile, baseLink);
  auto info = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);
  const auto& model = interface.getModel();
  auto& data = interface.getData();

  FloatingBaseModelPinocchioMapping mapping(info);
  mapping.setPinocchioInterface(interface);

  const auto state = getAccessTestRobotState();
  const auto input = getAccessTestRobotInput();

  const auto qPinocchio = mapping.getPinocchioJointPosition(state);

  Eigen::VectorXd qPinocchioTrue(model.nq);

  qPinocchioTrue.block<3, 1>(0, 0) = state.block<3,1>(6,0);
  Eigen::Vector3d eulerAngles = state.block<3, 1>(9, 0);
  qPinocchioTrue.block<4, 1>(3, 0) = getQuaternionFromEulerAnglesZyx(eulerAngles).coeffs();
  qPinocchioTrue.block<12, 1>(7, 0) = state.block<12,1>(12,0);

  EXPECT_TRUE(qPinocchio .isApprox(qPinocchioTrue, tolerance));


  const auto vPinocchio = mapping.getPinocchioJointVelocity(state, input);

  Eigen::VectorXd vPinocchioTrue(model.nv);
  vPinocchioTrue.block<6, 1>(0, 0) = state.block<6, 1>(0, 0);
  vPinocchioTrue.block<12, 1>(6, 0) = input.block<12, 1>(18, 0);

  EXPECT_TRUE(vPinocchio .isApprox(vPinocchioTrue, tolerance));

};