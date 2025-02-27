#include <floating_base_model/PinocchioFloatingBaseDynamics.hpp>

namespace floating_base_model 
{

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PinocchioFloatingBaseDynamics::PinocchioFloatingBaseDynamics(FloatingBaseModelInfo floatingBaseModelInfo)
  : mapping_(floatingBaseModelInfo), pinocchioInterfacePtr_(nullptr)
  {
    using Force = pinocchio::ForceTpl<SCALAR_T, 0>;
    Force force;
    force.linear() = Eigen::Vector<SCALAR_T, 3>::Zero();
    force.angular() = Eigen::Vector<SCALAR_T, 3>::Zero();
    fext_(model.njoints, force);
  }

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

    const auto Mb   = model_helper_functions::computeFloatingBaseLockedInertia(interface, q);
    model_helper_functions::computeForceVector(interface, info, input, fext_);
    const auto tau  = model_helper_functions::computeFloatingBaseGeneralizedTorques(interface, q, v, fext_);
    
    auto bodyVelocityDerivative = model_helper_functions::computeBaseBodyAcceleration(Mb, tau);

    bodyVelocityDerivative.block<3,1>(0,0) += baseAngularVelocity.cross(baseLinearVelocity);
    ocs2::vector_t dynamics(6 + model.nv);
    dynamics << bodyVelocityDerivative, basePositionDerivative, eulerAnglesDerivative, actuatedJointPositionDerivative;
    
    return dynamics;
  }

  ocs2::vector_t PinocchioFloatingBaseDynamics::getValue2(ocs2::scalar_t time,
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
    pinocchio::updateFramePlacements(model, data);
    pinocchio::computeJointJacobians(model, data);
    const Eigen::Vector<ocs2::scalar_t, Eigen::Dynamic> a = Eigen::Vector<ocs2::scalar_t, Eigen::Dynamic>::Zero(model.nv);
    const auto Mb   = model_helper_functions::computeFloatingBaseLockedInertia(interface, q);
    const auto nonlin = pinocchio::nonLinearEffects(model, data, q, v).block<6,1>(0,0);
    Eigen::Vector<ocs2::scalar_t, 6> tau = Eigen::Vector<ocs2::scalar_t, 6>::Zero();
    tau = nonlin;

    for (size_t i = 0; i < info.numThreeDofContacts; i++) {
      const auto forceWorldFrame = access_helper_functions::getContactForces(input, i, info);
      size_t contactFrameIndex = info.endEffectorFrameIndices[i];
      pinocchio::Data::Matrix6x J(6, model.nv);
      J.setZero();
      pinocchio::computeFrameJacobian(model, data, q, contactFrameIndex, pinocchio::LOCAL_WORLD_ALIGNED, J);
      tau += -J.transpose().block<6,3>(0,0) * forceWorldFrame;
    }  
    std::cout << "4" << std::endl;
    for (size_t i = info.numThreeDofContacts; i < info.numThreeDofContacts + info.numSixDofContacts; i++) {
      const auto wrenchWorldFrame = access_helper_functions::getContactWrenches(input, i, info);
      size_t contactFrameIndex = info.endEffectorFrameIndices[i];
      pinocchio::Data::Matrix6x J(6, model.nv);
      J.setZero();
      pinocchio::computeFrameJacobian(model, data, q, contactFrameIndex, pinocchio::LOCAL_WORLD_ALIGNED, J);
      tau += -J.transpose().block<6,6>(0,0) * wrenchWorldFrame;
    }  
  
    const auto fext = model_helper_functions::computeForceVector(interface, info, input);
 
    auto bodyVelocityDerivative = model_helper_functions::computeBaseBodyAcceleration(Mb, tau);

   
    bodyVelocityDerivative.block<3,1>(0,0) += baseAngularVelocity.cross(baseLinearVelocity);
    
    ocs2::vector_t dynamics(6 + model.nv);
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
