#include <gtest/gtest.h>

#include <floating_base_model/QuaterionEulerTransforms.hpp>
#include <floating_base_model/ModelHelperFunctions.hpp>
#include <floating_base_model/FactoryFunctions.hpp>
#include <floating_base_model/FloatingBaseModelPinocchioMapping.hpp>


#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/frames-derivatives.hpp>

#include "include/definitions.h"

using namespace floating_base_model;
using namespace model_helper_functions;
using namespace access_helper_functions;
using namespace quaterion_euler_transforms;

static constexpr ocs2::scalar_t tolerance = 1e-6;
static constexpr size_t numTests = 100;

class PinocchioPartialVelocityDerivativesAD final 
  {
    public:

    /**
     * Constructor
     * @param [in] pinocchioInterface : The pinocchio interface.
     * @param [in] FloatingBaseModelInfo : The floating base model information.
     * @param [in] modelName : Name of the generate model library
     * @param [in] modelFolder : Folder to save the model library files to
     * @param [in] recompileLibraries : If true, the model library will be newly compiled. If false, an existing library will be loaded if
     *                                  available.
     * @param [in] verbose : print information.
     */

    PinocchioPartialVelocityDerivativesAD(const ocs2::PinocchioInterface& pinocchioInterface,
      const FloatingBaseModelInfo& info,
      const std::string& modelName,
      const std::string frameName,
      const std::string& modelFolder = "/tmp/ocs2",
      bool recompileLibraries = true,
      bool verbose = false)
    {
      auto systemFlowMapFunc = [&](const ocs2::ad_vector_t& x, ocs2::ad_vector_t& y) {
        // initialize CppAD interface
        auto pinocchioInterfaceCppAd = pinocchioInterface.toCppAd();
    
        // mapping
        FloatingBaseModelPinocchioMappingCppAd mappingCppAd(info.toCppAd());
        mappingCppAd.setPinocchioInterface(pinocchioInterfaceCppAd);
    
        ocs2::ad_vector_t state = x.head(info.stateDim);
        ocs2::ad_vector_t input = x.tail(info.inputDim);
        y = getValueCppAd(pinocchioInterfaceCppAd, mappingCppAd, frameName, state, input);
      };
    
      systemFlowMapCppAdInterfacePtr_.reset(
          new ocs2::CppAdInterface(systemFlowMapFunc, info.stateDim + info.inputDim, modelName + "_systemFlowMap", modelFolder));
    
      if (recompileLibraries) {
        systemFlowMapCppAdInterfacePtr_->createModels(ocs2::CppAdInterface::ApproximationOrder::First, verbose);
      } else {
        systemFlowMapCppAdInterfacePtr_->loadModelsIfAvailable(ocs2::CppAdInterface::ApproximationOrder::First, verbose);
      }
    };

    /**
     * Computes frame velocity
     *
     * @param time: time
     * @param state: system state vector
     * @param input: system input vector
     * @return frame velocity v_f
     */
    ocs2::vector_t getValue(ocs2::scalar_t time,
      const ocs2::vector_t& state,
      const ocs2::vector_t& input) const
    {
      const ocs2::vector_t stateInput = (ocs2::vector_t(state.rows() + input.rows()) << state, input).finished();
      return systemFlowMapCppAdInterfacePtr_->getFunctionValue(stateInput);
    };

    /**
     * Computes first order approximation of frame velocity
     *
     * @param time: time
     * @param state: system state vector
     * @param input: system input vector
     * @return linear approximation of frame velocity v_f
     */
    ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time,
      const ocs2::vector_t& state,
      const ocs2::vector_t& input) const
    {
      const ocs2::vector_t stateInput = (ocs2::vector_t(state.rows() + input.rows()) << state, input).finished();
      ocs2::VectorFunctionLinearApproximation approx;
      approx.f = systemFlowMapCppAdInterfacePtr_->getFunctionValue(stateInput);
      const ocs2::matrix_t dynamicsJacobian = systemFlowMapCppAdInterfacePtr_->getJacobian(stateInput);
      approx.dfdx = dynamicsJacobian.leftCols(state.rows());
      approx.dfdu = dynamicsJacobian.rightCols(input.rows());
      return approx;
    };

   private:

   ocs2::ad_vector_t getValueCppAd(ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
      const FloatingBaseModelPinocchioMappingCppAd& mapping,
      const std::string frameName,
      const ocs2::ad_vector_t& state,
      const ocs2::ad_vector_t& input)
    {
      const auto& info = mapping.getFloatingBaseModelInfo();
      assert(info.stateDim == state.rows());
    
      const auto& model = pinocchioInterfaceCppAd.getModel();
      auto& data = pinocchioInterfaceCppAd.getData();

      const auto q = mapping.getPinocchioJointPosition(state);
      const auto v = mapping.getPinocchioJointVelocity(state, input);

      pinocchio::forwardKinematics(model, data, q, v);

      size_t frameIndex = model.getFrameId(frameName);

      ocs2::ad_vector_t frameVelocity = pinocchio::getFrameVelocity(model, data, frameIndex, pinocchio::LOCAL).toVector();
      
      return frameVelocity;
    };

    std::unique_ptr<ocs2::CppAdInterface> systemFlowMapCppAdInterfacePtr_;
};

TEST(FloatingBaseModelPinocchioMapping, Getters)
{
  ocs2::PinocchioInterface interface = createPinocchioInterface(meldogWithBaseLinkUrdfFile, baseLink);
  auto info = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);
  const auto& model = interface.getModel();
  auto& data = interface.getData();

  FloatingBaseModelPinocchioMapping mapping(info);
  mapping.setPinocchioInterface(interface);

  for(size_t i = 0; i < numTests; ++i)
  {
    ocs2::vector_t state = ocs2::vector_t::Random(Meldog::STATE_DIM);
    ocs2::vector_t input = ocs2::vector_t::Random(Meldog::INPUT_DIM);

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
  }

};


TEST(FloatingBaseModelPinocchioMapping, ocs2Jacobian)
{
  ocs2::PinocchioInterface interface = createPinocchioInterface(meldogWithBaseLinkUrdfFile, baseLink);
  auto info = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);
  const auto& model = interface.getModel();
  auto& data = interface.getData();

  FloatingBaseModelPinocchioMapping mapping(info);
  mapping.setPinocchioInterface(interface);

  std::string modelName = "floating_base_model";
  std::string modelFolder = "tmp/ocs2";
  std::string frameName = "RFF_link";

  size_t frameIndex = model.getFrameId(frameName);
  PinocchioPartialVelocityDerivativesAD frameVelocityGenAD(interface, info, modelName, frameName, modelFolder, true, true);

  ocs2::vector_t a(model.nv);
  a.setZero();
  
  for(size_t i = 0; i < numTests; ++i)
  {
    ocs2::vector_t state = ocs2::vector_t::Random(Meldog::STATE_DIM);
    ocs2::vector_t input = ocs2::vector_t::Random(Meldog::INPUT_DIM);
    
    const auto qPinocchio = mapping.getPinocchioJointPosition(state);
    
    const auto q = mapping.getPinocchioJointPosition(state);
    const auto v = mapping.getPinocchioJointVelocity(state, input);

    
    Eigen::MatrixXd v_partial_dq(6, model.nv);
    v_partial_dq.setZero();

    Eigen::MatrixXd v_partial_dv(6, model.nv);
    v_partial_dv.setZero();
    
    pinocchio::forwardKinematics(model, data, q, v);
    pinocchio::computeForwardKinematicsDerivatives(model, data, q, v, a);
    
    pinocchio::getFrameVelocityDerivatives(model, data, frameIndex, pinocchio::LOCAL, v_partial_dq, v_partial_dv);
    
    ocs2::vector_t frameVelocity = pinocchio::getFrameVelocity(model, data, frameIndex, pinocchio::LOCAL).toVector();
    ocs2::vector_t frameVelocityAD = frameVelocityGenAD.getValue(0, state, input);
    
    EXPECT_TRUE(frameVelocity.isApprox(frameVelocityAD, tolerance));

    const auto dv = mapping.getOcs2Jacobian(state, v_partial_dq, v_partial_dv);
    
    const auto dvAD = frameVelocityGenAD.getLinearApproximation(0, state, input);
    
    EXPECT_TRUE(dv.first.isApprox(dvAD.dfdx, tolerance));
    EXPECT_TRUE(dv.second.isApprox(dvAD.dfdu, tolerance));
  }

};