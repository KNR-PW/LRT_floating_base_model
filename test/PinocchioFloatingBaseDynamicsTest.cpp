#include <gtest/gtest.h>

#include <floating_base_model/ModelHelperFunctions.hpp>
#include <floating_base_model/FactoryFunctions.hpp>
#include <floating_base_model/PinocchioFloatingBaseDynamics.hpp>
#include <floating_base_model/PinocchioFloatingBaseDynamicsAD.hpp>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include "include/definitions.h"

#include <chrono>

using namespace floating_base_model;
using namespace model_helper_functions;
using namespace access_helper_functions;

static constexpr ocs2::scalar_t tolerance = 1e-6;
static constexpr size_t numTests = 20;


ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(meldogWithBaseLinkUrdfFile, baseLink);
auto info = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);

// Interface model and data (references)
const pinocchio::Model& model = interface.getModel();
pinocchio::Data& data = interface.getData();

// New model and data (copies)
const pinocchio::Model modelTrue = interface.getModel();
pinocchio::Data dataTrue = interface.getData();

PinocchioFloatingBaseDynamics dynamics(info);

std::string modelName = "floating_base_model";
std::string modelFolder = "tmp/ocs2";

PinocchioFloatingBaseDynamicsAD dynamicsAD(interface, info, modelName, modelFolder, true, true);

TEST(PinocchioFloatingBaseDynamicsTest, getValue)
{
  dynamics.setPinocchioInterface(interface);
  FloatingBaseModelPinocchioMapping mapping(info);
  mapping.setPinocchioInterface(interface);
  for(size_t i = 0; i < numTests; ++i)
  {
    ocs2::vector_t state = ocs2::vector_t::Random(Meldog::STATE_DIM);
    ocs2::vector_t input = ocs2::vector_t::Random(Meldog::INPUT_DIM);
    Eigen::Matrix<ocs2::scalar_t, 6, 1> disturbance = Eigen::Matrix<ocs2::scalar_t, 6, 1>::Random();

    const auto q = mapping.getPinocchioJointPosition(state);
    const auto v = mapping.getPinocchioJointVelocity(state, input);

    pinocchio::forwardKinematics(model, data, q);
    pinocchio::forwardKinematics(modelTrue, dataTrue, q);

    using Force = pinocchio::ForceTpl<ocs2::scalar_t, 0>;
    Force force;
    force.linear() = Eigen::Vector<ocs2::scalar_t, 3>::Zero();
    force.angular() = Eigen::Vector<ocs2::scalar_t, 3>::Zero();
    pinocchio::container::aligned_vector<Force> fext(model.njoints, force);

    model_helper_functions::computeSpatialForces(interface, info, input, fext);

    Eigen::Matrix<ocs2::scalar_t, 6, 6> baseInertiaMatrix = pinocchio::crba(modelTrue, dataTrue, q).block<6, 6>(0, 0);
    baseInertiaMatrix.triangularView<Eigen::StrictlyLower>() = baseInertiaMatrix.transpose().triangularView<Eigen::StrictlyLower>();
    const Eigen::Matrix<ocs2::scalar_t, 6, 1> baseCoriollisVector = pinocchio::computeCoriolisMatrix(modelTrue, dataTrue, q, v).block(0, 0, 6, model.nv) * v;
    const Eigen::Matrix<ocs2::scalar_t, 6, 1> baseGravityExternalForcesVector = pinocchio::computeStaticTorque(modelTrue, dataTrue, q, fext).block<6, 1>(0, 0);
    const Eigen::Matrix<ocs2::scalar_t, 6, 1> baseTorque = baseCoriollisVector + baseGravityExternalForcesVector + disturbance;
    
    Eigen::Matrix<ocs2::scalar_t, 6, 1> baseAccelerationTrue = baseInertiaMatrix.ldlt().solve(baseTorque);
    const Eigen::Matrix<ocs2::scalar_t, 3, 1> baseLinearVelocity = v.block<3, 1>(0, 0);
    const Eigen::Matrix<ocs2::scalar_t, 3, 1> baseAngularVelocity = v.block<3, 1>(3, 0);
    baseAccelerationTrue.block<3, 1>(0, 0) += baseAngularVelocity.cross(baseLinearVelocity);

    const auto value = dynamics.getValue(0, state, input, disturbance);
    const Eigen::Matrix<ocs2::scalar_t, 6, 1> baseAcceleration = value.block<6, 1>(0, 0);

    const auto valueAD = dynamicsAD.getValue(0, state, input, disturbance);

    EXPECT_TRUE(baseAccelerationTrue.isApprox(baseAcceleration, tolerance));
    EXPECT_TRUE(value.isApprox(valueAD, tolerance));
  }

};

TEST(PinocchioFloatingBaseDynamicsTest, speed)
{
  dynamics.setPinocchioInterface(interface);

  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

  for(int i = 0; i < 100; ++i)
  {
    ocs2::vector_t state = ocs2::vector_t::Random(Meldog::STATE_DIM);
    ocs2::vector_t input = ocs2::vector_t::Random(Meldog::INPUT_DIM);
    Eigen::Matrix<ocs2::scalar_t, 6, 1> disturbance = Eigen::Matrix<ocs2::scalar_t, 6, 1>::Random();

    const auto value = dynamics.getValue(0, state, input, disturbance);
  }

  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

  size_t analitycalTime = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

  std::cout << "Analytical getValue time = " << analitycalTime << "[µs]" << std::endl;
  

  begin = std::chrono::steady_clock::now();

  for(int i = 0; i < 100; ++i)
  {
    ocs2::vector_t state = ocs2::vector_t::Random(Meldog::STATE_DIM);
    ocs2::vector_t input = ocs2::vector_t::Random(Meldog::INPUT_DIM);
    Eigen::Matrix<ocs2::scalar_t, 6, 1> disturbance = Eigen::Matrix<ocs2::scalar_t, 6, 1>::Random();

    const auto valueAD = dynamicsAD.getValue(0, state, input, disturbance);
  }

  end = std::chrono::steady_clock::now();

  size_t AdTime = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
  std::cout << "Automatic Differentiation getValue time = " << AdTime << "[µs]" << std::endl;

  EXPECT_TRUE(AdTime < analitycalTime);
};

TEST(PinocchioFloatingBaseDynamicsTest, getLinearApproximation)
{
  ocs2::vector_t state = ocs2::vector_t::Random(Meldog::STATE_DIM);
  ocs2::vector_t input = ocs2::vector_t::Random(Meldog::INPUT_DIM);
  Eigen::Matrix<ocs2::scalar_t, 6, 1> disturbance = Eigen::Matrix<ocs2::scalar_t, 6, 1>::Random();

  const auto valueAD = dynamicsAD.getValue(0, state, input, disturbance);
  const auto linearApproxAD = dynamicsAD.getLinearApproximation(0, state, input, disturbance);
  
  EXPECT_TRUE(linearApproxAD.f.rows() == Meldog::STATE_DIM);
  EXPECT_TRUE(linearApproxAD.dfdx.rows() == Meldog::STATE_DIM);
  EXPECT_TRUE(linearApproxAD.dfdx.cols() == Meldog::STATE_DIM);
  EXPECT_TRUE(linearApproxAD.dfdu.rows() == Meldog::STATE_DIM);
  EXPECT_TRUE(linearApproxAD.dfdu.cols() == Meldog::INPUT_DIM);

  Eigen::Matrix<ocs2::scalar_t, 6, 1> disturbance2 = Eigen::Matrix<ocs2::scalar_t, 6, 1>::Random();
  const auto valueAD2 = dynamicsAD.getValue(0, state, input, disturbance2);

  EXPECT_TRUE(valueAD != valueAD2);

};
