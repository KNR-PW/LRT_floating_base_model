#include <gtest/gtest.h>

#include <floating_base_model/ModelHelperFunctions.hpp>
#include <floating_base_model/FactoryFunctions.hpp>
#include <floating_base_model/PinocchioFloatingBaseDynamics.hpp>
#include <floating_base_model/PinocchioFloatingBaseDynamicsAD.hpp>

#include <pinocchio/algorithm/crba.hpp>

#include "include/definitions.h"

using namespace floating_base_model;
using namespace model_helper_functions;
using namespace access_helper_functions;

static constexpr ocs2::scalar_t tolerance = 1e-6;
static constexpr size_t numTests = 100;

TEST(PinocchioFloatingBaseDynamicsTest, getValue)
{
  ocs2::PinocchioInterface interface = createPinocchioInterface(meldogWithBaseLinkUrdfFile, baseLink);
  auto info = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);
  const auto& model = interface.getModel();
  auto& data = interface.getData();

  PinocchioFloatingBaseDynamics dynamics(info);
  dynamics.setPinocchioInterface(interface);

  std::string modelName = "floating_base_model";
  std::string modelFolder = "tmp/ocs2";

  PinocchioFloatingBaseDynamicsAD dynamicsAD(interface, info, modelName, modelFolder, true, true);

  for(size_t i = 0; i < numTests; ++i)
  {
    ocs2::vector_t state = ocs2::vector_t::Random(Meldog::STATE_DIM);
    ocs2::vector_t input = ocs2::vector_t::Random(Meldog::INPUT_DIM);

    const auto value = dynamics.getValue(0, state, input);
    const auto valueAD = dynamicsAD.getValue(0, state, input);

    EXPECT_TRUE(value .isApprox(valueAD, tolerance));
  }

};
