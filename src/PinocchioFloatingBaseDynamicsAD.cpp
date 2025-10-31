#include <floating_base_model/PinocchioFloatingBaseDynamicsAD.hpp>


namespace floating_base_model
{

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PinocchioFloatingBaseDynamicsAD::PinocchioFloatingBaseDynamicsAD(
    const ocs2::PinocchioInterface& pinocchioInterface,
    const FloatingBaseModelInfo& info,
    const std::string& modelName,
    const std::string& modelFolder,
    bool recompileLibraries,
    bool verbose) 
  {

    // initialize CppAD interface
    auto pinocchioInterfaceCppAd = pinocchioInterface.toCppAd();
  
    // mapping
    FloatingBaseModelPinocchioMappingCppAd mappingCppAd(info.toCppAd());
    mappingCppAd.setPinocchioInterface(pinocchioInterfaceCppAd);
    
    auto systemFlowMapFunc = [&](const ocs2::ad_vector_t& x, Eigen::Matrix<ocs2::ad_scalar_t, 6, 1> params, ocs2::ad_vector_t& y) 
    {
      ocs2::ad_vector_t state = x.head(info.stateDim);
      ocs2::ad_vector_t input = x.tail(info.inputDim);
      Eigen::Matrix<ocs2::ad_scalar_t, 6, 1> disturbance = params;
      y = getValueCppAd(pinocchioInterfaceCppAd, mappingCppAd, state, input, disturbance);
    };
  
    systemFlowMapCppAdInterfacePtr_.reset(
        new ocs2::CppAdInterface(systemFlowMapFunc, info.stateDim + info.inputDim, 6, modelName + "_systemFlowMap", modelFolder));
  
    if (recompileLibraries) {
      systemFlowMapCppAdInterfacePtr_->createModels(ocs2::CppAdInterface::ApproximationOrder::First, verbose);
    } else {
      systemFlowMapCppAdInterfacePtr_->loadModelsIfAvailable(ocs2::CppAdInterface::ApproximationOrder::First, verbose);
    }
  }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PinocchioFloatingBaseDynamicsAD::PinocchioFloatingBaseDynamicsAD(const PinocchioFloatingBaseDynamicsAD& rhs)
    : systemFlowMapCppAdInterfacePtr_(new ocs2::CppAdInterface(*rhs.systemFlowMapCppAdInterfacePtr_)) {}
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ocs2::ad_vector_t PinocchioFloatingBaseDynamicsAD::getValueCppAd(
    ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
    const FloatingBaseModelPinocchioMappingCppAd& mappingCppAd,
    const ocs2::ad_vector_t& state,
    const ocs2::ad_vector_t& input,
    const Eigen::Matrix<ocs2::ad_scalar_t, 6, 1>& disturbance) 
  {
    const auto& info = mappingCppAd.getFloatingBaseModelInfo();
    assert(info.stateDim == state.rows());
    
    const auto& model = pinocchioInterfaceCppAd.getModel();
    auto& data = pinocchioInterfaceCppAd.getData();

    const auto baseLinearVelocity = access_helper_functions::getBaseLinearVelocity(state, info);
    const Eigen::Vector<ocs2::ad_scalar_t, 3> baseAngularVelocity = access_helper_functions::getBaseAngularVelocity(state, info);
    const Eigen::Vector<ocs2::ad_scalar_t, 3> eulerAngles = access_helper_functions::getBaseOrientationZyx(state, info);
    const Eigen::Matrix<ocs2::ad_scalar_t, 3, 3> baseRotationMatrix = ocs2::getRotationMatrixFromZyxEulerAngles(eulerAngles);

    const Eigen::Vector<ocs2::ad_scalar_t, 3> eulerAnglesDerivative = ocs2::getEulerAnglesZyxDerivativesFromLocalAngularVelocity(eulerAngles, baseAngularVelocity);
    const Eigen::Vector<ocs2::ad_scalar_t, 3> basePositionDerivative = baseRotationMatrix * baseLinearVelocity;
    const auto actuatedJointPositionDerivative = access_helper_functions::getJointVelocities(input, info);
    
    const auto q = mappingCppAd.getPinocchioJointPosition(state);
    const auto v = mappingCppAd.getPinocchioJointVelocity(state, input);

    pinocchio::forwardKinematics(model, data, q);

    const auto Mb = model_helper_functions::computeFloatingBaseLockedInertia(pinocchioInterfaceCppAd);

    using Force = pinocchio::ForceTpl<ocs2::ad_scalar_t, 0>;
    Force force;
    force.linear() = Eigen::Vector<ocs2::ad_scalar_t, 3>::Zero();
    force.angular() = Eigen::Vector<ocs2::ad_scalar_t, 3>::Zero();
    pinocchio::container::aligned_vector<Force> fext(model.njoints, force);

    model_helper_functions::computeSpatialForces(pinocchioInterfaceCppAd, info, input, fext);
    const Eigen::Matrix<ocs2::ad_scalar_t, 6, 1> tau = model_helper_functions::computeFloatingBaseGeneralizedTorques(pinocchioInterfaceCppAd, q, v, fext) + disturbance;
    
    auto bodyVelocityDerivative = model_helper_functions::computeBaseBodyAcceleration(Mb, tau);

    bodyVelocityDerivative.block<3,1>(0,0) += baseAngularVelocity.cross(baseLinearVelocity);

    ocs2::ad_vector_t stateDerivative(info.stateDim);
    stateDerivative << bodyVelocityDerivative, basePositionDerivative, eulerAnglesDerivative, actuatedJointPositionDerivative;

    return stateDerivative;
  }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ocs2::vector_t PinocchioFloatingBaseDynamicsAD::getValue(ocs2::scalar_t time,
    const ocs2::vector_t& state,
    const ocs2::vector_t& input,
    const Eigen::Matrix<ocs2::scalar_t, 6, 1>& disturbance) const 
  {
    const ocs2::vector_t stateInput = (ocs2::vector_t(state.rows() + input.rows()) << state, input).finished();
    return systemFlowMapCppAdInterfacePtr_->getFunctionValue(stateInput, disturbance);
  }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ocs2::VectorFunctionLinearApproximation PinocchioFloatingBaseDynamicsAD::getLinearApproximation(
    ocs2::scalar_t time,
    const ocs2::vector_t& state,
    const ocs2::vector_t& input,
    const Eigen::Matrix<ocs2::scalar_t, 6, 1>& disturbance) const 
  {
    const ocs2::vector_t stateInput = (ocs2::vector_t(state.rows() + input.rows()) << state, input).finished();
    ocs2::VectorFunctionLinearApproximation approx;
    approx.f = systemFlowMapCppAdInterfacePtr_->getFunctionValue(stateInput, disturbance);
    const ocs2::matrix_t dynamicsJacobian = systemFlowMapCppAdInterfacePtr_->getJacobian(stateInput, disturbance);
    approx.dfdx = dynamicsJacobian.leftCols(state.rows());
    approx.dfdu = dynamicsJacobian.middleCols(state.rows(), input.rows());
    return approx;
  }
  
} // namespace floating_base_model
  