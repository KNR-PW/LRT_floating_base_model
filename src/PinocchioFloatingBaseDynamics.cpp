#include <floating_base_model/PinocchioFloatingBaseDynamics.hpp>

namespace floating_base_model 
{

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PinocchioFloatingBaseDynamics::PinocchioFloatingBaseDynamics(FloatingBaseModelInfo floatingBaseModelInfo)
  : mapping_(floatingBaseModelInfo), pinocchioInterfacePtr_(nullptr), fext_(){}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PinocchioFloatingBaseDynamics::PinocchioFloatingBaseDynamics(const PinocchioFloatingBaseDynamics& rhs)
  : mapping_(rhs.mapping_.getFloatingBaseModelInfo()), 
    pinocchioInterfacePtr_(rhs.pinocchioInterfacePtr_)
  {
    mapping_.setPinocchioInterface(*rhs.pinocchioInterfacePtr_);
    fext_ = rhs.fext_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void PinocchioFloatingBaseDynamics::setPinocchioInterface(PinocchioInterface& pinocchioInterface) 
  {
    pinocchioInterfacePtr_ = &pinocchioInterface;
    mapping_.setPinocchioInterface(pinocchioInterface);

    auto& interface = *pinocchioInterfacePtr_;
    const auto& model = interface.getModel();

    using Force = pinocchio::ForceTpl<ocs2::scalar_t, 0>;
    Force force;
    force.linear() = Eigen::Vector<ocs2::scalar_t, 3>::Zero();
    force.angular() = Eigen::Vector<ocs2::scalar_t, 3>::Zero();
    pinocchio::container::aligned_vector<Force> fext(model.njoints, force);
    fext_ = fext;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ocs2::vector_t PinocchioFloatingBaseDynamics::getValue(ocs2::scalar_t time,
    const ocs2::vector_t& state, const ocs2::vector_t& input)
  {
    auto& interface = *pinocchioInterfacePtr_;
    const FloatingBaseModelInfo& info = mapping_.getFloatingBaseModelInfo();
    const auto& model = interface.getModel();
    auto& data = interface.getData();

    const auto baseLinearVelocity = access_helper_functions::getBaseLinearVelocity(state, info);
    const Eigen::Vector<ocs2::scalar_t, 3> baseAngularVelocity = access_helper_functions::getBaseAngularVelocity(state, info);
    const Eigen::Vector<ocs2::scalar_t, 3> eulerAngles = access_helper_functions::getBaseOrientationZyx(state, info);
    const Eigen::Matrix<ocs2::scalar_t, 3, 3> baseRotationMatrix = ocs2::getRotationMatrixFromZyxEulerAngles(eulerAngles);

    const Eigen::Vector<ocs2::scalar_t, 3> eulerAnglesDerivative = ocs2::getEulerAnglesZyxDerivativesFromLocalAngularVelocity(eulerAngles, baseAngularVelocity);
    const Eigen::Vector<ocs2::scalar_t, 3> basePositionDerivative = baseRotationMatrix * baseLinearVelocity;
    const auto actuatedJointPositionDerivative = access_helper_functions::getJointVelocities(input, info);

    const auto q = mapping_.getPinocchioJointPosition(state);
    const auto v = mapping_.getPinocchioJointVelocity(state, input);
    pinocchio::forwardKinematics(model, data, q);

    const auto Mb = model_helper_functions::computeFloatingBaseLockedInertia(interface, q);
    model_helper_functions::computeForceVector(interface, info, input, fext_);
    const auto tau  = model_helper_functions::computeFloatingBaseGeneralizedTorques(interface, q, v, fext_);
    
    auto bodyVelocityDerivative = model_helper_functions::computeBaseBodyAcceleration(Mb, tau);

    bodyVelocityDerivative.block<3,1>(0,0) += baseAngularVelocity.cross(baseLinearVelocity);
    
    ocs2::vector_t dynamics(info.stateDim);
    dynamics << bodyVelocityDerivative, basePositionDerivative, eulerAnglesDerivative, actuatedJointPositionDerivative;
    
    return dynamics;
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
