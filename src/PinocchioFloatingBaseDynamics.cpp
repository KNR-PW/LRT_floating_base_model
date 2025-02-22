#include <floating_base_model/PinocchioFloatingBaseDynamics.hpp>

namespace floating_base_model 
{

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PinocchioFloatingBaseDynamics::PinocchioFloatingBaseDynamics(FloatingBaseModelInfo floatingBaseModelInfo)
  : mapping_(floatingBaseModelInfo), pinocchioInterfacePtr_(nullptr){}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PinocchioFloatingBaseDynamics::PinocchioFloatingBaseDynamics(const PinocchioFloatingBaseDynamics& rhs)
  : mapping_(rhs.mapping_.getFloatingBaseModelInfo()), 
    pinocchioInterfacePtr_(rhs.pinocchioInterfacePtr_)
  {
    mapping_.setPinocchioInterface(*rhs.pinocchioInterfacePtr_);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void PinocchioFloatingBaseDynamics::setPinocchioInterface(PinocchioInterface& pinocchioInterface) 
  {
    pinocchioInterfacePtr_ = &pinocchioInterface;
    mapping_.setPinocchioInterface(pinocchioInterface);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ocs2::vector_t PinocchioFloatingBaseDynamics::getValue(ocs2::scalar_t time,
    const ocs2::vector_t& state, const ocs2::vector_t& input)
  {
    const FloatingBaseModelInfo& info = mapping_.getFloatingBaseModelInfo();
    const auto& model = pinocchioInterfacePtr_->getModel();
    auto& data = pinocchioInterfacePtr_->getData();
  
    const auto baseLinearVelocity = access_helper_functions::getBaseLinearVelocity(state, info);
    const Eigen::Vector<ocs2::scalar_t, 3> baseAngularVelocity = access_helper_functions::getBaseAngularVelocity(state, info);
    const Eigen::Vector<ocs2::scalar_t, 3> eulerAngles = access_helper_functions::getBaseOrientationZyx(state, info);

    const Eigen::Matrix<ocs2::scalar_t, 3, 3> baseRotationMatrix = ocs2::getRotationMatrixFromZyxEulerAngles(eulerAngles);

    const Eigen::Vector<ocs2::scalar_t, 3> eulerAnglesDerivative = ocs2::getEulerAnglesZyxDerivativesFromLocalAngularVelocity(eulerAngles, baseAngularVelocity);
    const Eigen::Vector<ocs2::scalar_t, 3> basePositionDerivative = baseRotationMatrix * baseLinearVelocity;
    const ocs2::vector_t actuatedJointPositionDerivative = access_helper_functions::getJointVelocities(input, info);

    const auto q = mapping_.getPinocchioJointPosition(state);
    const auto v = mapping_.getPinocchioJointVelocity(state, input);

    const Eigen::Vector<ocs2::scalar_t, Eigen::Dynamic> a = Eigen::Vector<ocs2::scalar_t, Eigen::Dynamic>::Zero(model.nv);


    // TODO DODAJ SILY ZEWNETRZNE TUTAJ 
    //const auto tau = pinocchio::rnea(model, data, q, v, a);
    const auto Mb = model_helper_functions::computeFloatingBaseLockedInertia(*pinocchioInterfacePtr_, q);
    //const auto bodyBaseAcceleration = model_helper_functions::computeBaseBodyAcceleration();


    return ocs2::vector_t();

  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ocs2::VectorFunctionLinearApproximation PinocchioFloatingBaseDynamics::getLinearApproximation(ocs2::scalar_t time,
    const ocs2::vector_t& state, const ocs2::vector_t& input)
  {
    return ocs2::VectorFunctionLinearApproximation(); // TODO
  }

} // namespace floating_base_model

// vector_t FloatingBaseModelDynamics::getValue(scalar_t time, const vector_t& state, const vector_t& input)
// {
//   assert(params_.stateDim == state.rows());
//   assert(params_.inputDim == input.rows());
//   vector_t f(params_.stateDim);
//   Eigen::Matrix<scalar_t, 6, 1> centroidalMomentumRate = getCentroidalMomentumRate(input);
//   const auto baseVelocity = floating_base_model::getBaseVelocity(params_, state);
//   const auto jointVelocities = floating_base_model::getJointVelocities(params_, input);
//   f << baseVelocity, jointVelocities; // Dodaj przyspieszenia BASE
//   return f;
// }
