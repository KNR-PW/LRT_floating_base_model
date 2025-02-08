#include <floating_base_model/FloatingBaseModelDynamics.hpp>

using namespace floating_base_model;

FloatingBaseModelDynamics::FloatingBaseModelDynamics(const FloatingBaseModelParams& params, PinocchioInterface& interface):
params_{params}, interface_{interface}{ }



PinocchioInterface floating_base_model::makeFloatingBaseInterface(const std::string& urdfFilePath)
{
  pinocchio::JointModelFreeFlyer freeFlyerJoint;
  return getPinocchioInterfaceFromUrdfFile(urdfFilePath, freeFlyerJoint);
  // return centroidal_model::createPinocchioInterface(urdfFilePath);
}


vector_t FloatingBaseModelDynamics::getValue(scalar_t time, const vector_t& state, const vector_t& input)
{
  assert(params_.stateDim == state.rows());
  assert(params_.inputDim == input.rows());
  vector_t f(params_.stateDim);
  Eigen::Matrix<scalar_t, 6, 1> centroidalMomentumRate = getCentroidalMomentumRate(input);
  const auto baseVelocity = floating_base_model::getBaseVelocity(params_, state);
  const auto jointVelocities = floating_base_model::getJointVelocities(params_, input);
  f << baseVelocity, jointVelocities; // Dodaj przyspieszenia BASE
  return f;
}

Eigen::Matrix<scalar_t, 6, 1> FloatingBaseModelDynamics::getCentroidalMomentumRate(const Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>& input)
{
  Eigen::Matrix<scalar_t, 6, 1> centroidalMomentumRate;
  centroidalMomentumRate << params_.robotMass * gravityVector_, Eigen::Matrix<scalar_t, 3, 1>::Zero();

  // for (size_t i = 0; i < info.numThreeDofContacts; i++) {
  //   const auto contactForceInWorldFrame = floating_base_model::getContactForces(params_, input, i);
  //   const auto positionComToContactPointInWorldFrame = getPositionComToContactPointInWorldFrame(interface, info, i);
  //   centroidalMomentumRate.template head<3>() += contactForceInWorldFrame;
  //   centroidalMomentumRate.template tail<3>().noalias() += positionComToContactPointInWorldFrame.cross(contactForceInWorldFrame);
  // }  // end of i loop

  // for (size_t i = info.numThreeDofContacts; i < info.numThreeDofContacts + info.numSixDofContacts; i++) {
  //   const auto contactForceInWorldFrame = floating_base_model::getContactForces(input, i, info);
  //   const auto contactTorqueInWorldFrame = floating_base_model::getContactTorques(input, i, info);
  //   const auto positionComToContactPointInWorldFrame = getPositionComToContactPointInWorldFrame(interface, info, i);
  //   centroidalMomentumRate.template head<3>() += contactForceInWorldFrame;
  //   centroidalMomentumRate.template tail<3>().noalias() +=
  //       positionComToContactPointInWorldFrame.cross(contactForceInWorldFrame) + contactTorqueInWorldFrame;
  // }  // end of i loop

  // // normalize by the total mass
  // centroidalMomentumRate /= info.robotMass;

  return centroidalMomentumRate;
}