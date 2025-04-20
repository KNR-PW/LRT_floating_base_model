#include <gtest/gtest.h>

#include <floating_base_model/ModelHelperFunctions.hpp>
#include <floating_base_model/FactoryFunctions.hpp>
#include <floating_base_model/PinocchioFloatingBaseDynamics.hpp>
#include <floating_base_model/PinocchioFloatingBaseDynamicsAD.hpp>

#include <pinocchio/algorithm/crba.hpp>

#include "include/definitions.h"

#include <chrono>

using namespace floating_base_model;
using namespace model_helper_functions;
using namespace access_helper_functions;

static constexpr ocs2::scalar_t tolerance = 1e-6;
static constexpr size_t numTests = 20;


ocs2::PinocchioInterface interface = createPinocchioInterface(meldogWithBaseLinkUrdfFile, baseLink);
auto info = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);
const auto& model = interface.getModel();
auto& data = interface.getData();

PinocchioFloatingBaseDynamics dynamics(info);

std::string modelName = "floating_base_model";
std::string modelFolder = "tmp/ocs2";

PinocchioFloatingBaseDynamicsAD dynamicsAD(interface, info, modelName, modelFolder, true, true);

TEST(PinocchioFloatingBaseDynamicsTest, getValue)
{
  dynamics.setPinocchioInterface(interface);
  for(size_t i = 0; i < numTests; ++i)
  {
    ocs2::vector_t state = ocs2::vector_t::Random(Meldog::STATE_DIM);
    ocs2::vector_t input = ocs2::vector_t::Random(Meldog::INPUT_DIM);
    Eigen::Matrix<ocs2::scalar_t, 6, 1> disturbance = Eigen::Matrix<ocs2::scalar_t, 6, 1>::Random();

    const auto value = dynamics.getValue(0, state, input, disturbance);
    const auto valueAD = dynamicsAD.getValue(0, state, input, disturbance);

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
