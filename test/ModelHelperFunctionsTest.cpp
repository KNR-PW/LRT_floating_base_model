#include <gtest/gtest.h>

#include <floating_base_model/ModelHelperFunctions.hpp>
#include <floating_base_model/FactoryFunctions.hpp>

#include <pinocchio/algorithm/crba.hpp>

#include "include/definitions.h"

using namespace ocs2;
using namespace floating_base_model;
using namespace model_helper_functions;
using namespace access_helper_functions;

static constexpr ocs2::scalar_t tolerance = 1e-6;
static constexpr size_t numTests = 100;

TEST(ModelHelperFunctions, FloatingBaseLockedInertia)
{
  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(meldogWithBaseLinkUrdfFile, baseLink);
  auto info = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);
  
  // Interface model and data (references)
  const pinocchio::Model& model = interface.getModel();
  pinocchio::Data& data = interface.getData();

  // New model and data (copies)
  const pinocchio::Model modelTrue = interface.getModel();
  pinocchio::Data dataTrue = interface.getData();

  for(size_t i = 0; i < numTests; ++i)
  {
    ocs2::vector_t q = ocs2::vector_t::Random(model.nq);
    Eigen::Quaternion quaterion(q[6], q[3], q[4], q[5]);
    quaterion.normalize();
    q.block<4,1>(3, 0) = quaterion.coeffs();

    pinocchio::forwardKinematics(model, data, q);

    pinocchio::crba(modelTrue, dataTrue, q, pinocchio::Convention::LOCAL);
    dataTrue.M.triangularView<Eigen::StrictlyLower>() = dataTrue.M.transpose().triangularView<Eigen::StrictlyLower>();

    Eigen::Matrix<scalar_t, 6, 6> MbTrue = dataTrue.M.block<6,6>(0, 0);

    Eigen::Matrix<scalar_t, 6, 6> Mb = computeFloatingBaseLockedInertia(interface);

    Eigen::Matrix<scalar_t, 6, 6> MbInvTrue = MbTrue.inverse();

    Eigen::Matrix<scalar_t, 6, 6> MbInv = computeFloatingBaseLockedInertiaInverse(Mb);

    EXPECT_TRUE(Mb.isApprox(MbTrue, tolerance));
    EXPECT_TRUE(MbInv.isApprox(MbInvTrue, tolerance));
  }
}

TEST(ModelHelperFunctions, BaseBodyAcceleration)
{
  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(meldogWithBaseLinkUrdfFile, baseLink);
  auto info = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);
  
  // Interface model and data (references)
  const pinocchio::Model& model = interface.getModel();
  pinocchio::Data& data = interface.getData();

  // New model and data (copies)
  const pinocchio::Model modelTrue = interface.getModel();
  pinocchio::Data dataTrue = interface.getData();

  for(size_t i = 0; i < numTests; ++i)
  {
    ocs2::vector_t q = ocs2::vector_t::Random(model.nq);
    Eigen::Quaternion quaterion(q[6], q[3], q[4], q[5]);
    quaterion.normalize();
    q.block<4,1>(3, 0) = quaterion.coeffs();

    pinocchio::forwardKinematics(model, data, q);

    pinocchio::crba(modelTrue, dataTrue, q, pinocchio::Convention::LOCAL);
    dataTrue.M.triangularView<Eigen::StrictlyLower>() = dataTrue.M.transpose().triangularView<Eigen::StrictlyLower>();

    Eigen::Matrix<scalar_t, 6, 6> MbTrue = dataTrue.M.block<6,6>(0, 0);

    Eigen::Matrix<scalar_t, 6, 6> Mb = computeFloatingBaseLockedInertia(interface);

    Eigen::Vector<scalar_t, 6> tau = Eigen::Vector<scalar_t, 6>::Random();

    ocs2::vector_t ddq = computeBaseBodyAcceleration(Mb, tau);

    ocs2::vector_t ddqTrue = MbTrue.inverse() * tau;

    EXPECT_TRUE(ddq.isApprox(ddqTrue, tolerance));
  }
}

TEST(ModelHelperFunctions, GeneralizedTorques)
{
  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(meldogWithBaseLinkUrdfFile, baseLink);
  auto info = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);
  
  // Interface model and data (references)
  const pinocchio::Model& model = interface.getModel();
  pinocchio::Data& data = interface.getData();

  // New model and data (copies)
  const pinocchio::Model modelTrue = interface.getModel();
  pinocchio::Data dataTrue = interface.getData();

  for(size_t i = 0; i < numTests; ++i)
  {
    ocs2::vector_t q = ocs2::vector_t::Random(model.nq);
    Eigen::Quaternion quaterion(q[6], q[3], q[4], q[5]);
    quaterion.normalize();
    q.block<4,1>(3, 0) = quaterion.coeffs();

    pinocchio::forwardKinematics(model, data, q);
    pinocchio::forwardKinematics(modelTrue, dataTrue, q);

    pinocchio::Force zero_force(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    pinocchio::container::aligned_vector<pinocchio::Force> fext(model.njoints, zero_force);

    const vector_t input = vector_t::Random(info.inputDim);

    computeSpatialForces(interface, info, input, fext);

    const auto tauStatic = pinocchio::computeStaticTorque(model, data, q, fext) - pinocchio::computeGeneralizedGravity(model, data, q);

    pinocchio::updateFramePlacements(modelTrue, dataTrue);
    pinocchio::computeJointJacobians(modelTrue, dataTrue);

    Eigen::Vector<scalar_t, Eigen::Dynamic> tauStaticTrue = Eigen::Vector<scalar_t, Eigen::Dynamic>::Zero(model.nv);

    for (size_t i = 0; i < info.numThreeDofContacts; i++) 
    {
      const auto forceWorldFrame = getContactForces(input, i, info);
      size_t contactFrameIndex = info.endEffectorFrameIndices[i];
      pinocchio::Data::Matrix6x J(6, model.nv);
      J.setZero();
      pinocchio::computeFrameJacobian(modelTrue, dataTrue, q, contactFrameIndex, pinocchio::LOCAL_WORLD_ALIGNED, J);
      tauStaticTrue += -J.transpose().block(0, 0, model.nv, 3) * forceWorldFrame;
    }

    for (size_t i = info.numThreeDofContacts; i < info.numThreeDofContacts + info.numSixDofContacts; i++) 
    {
      const auto wrenchWorldFrame = getContactWrenches(input, i, info);
      size_t contactFrameIndex = info.endEffectorFrameIndices[i];
      pinocchio::Data::Matrix6x J(6, model.nv);
      J.setZero();
      pinocchio::computeFrameJacobian(modelTrue, dataTrue, q, contactFrameIndex, pinocchio::LOCAL_WORLD_ALIGNED, J);
      tauStaticTrue += -J.transpose() * wrenchWorldFrame;
    }

    EXPECT_TRUE(tauStatic.isApprox(tauStaticTrue, tolerance));

    ocs2::vector_t v = ocs2::vector_t::Random(model.nv);

    const auto tauDynamicBase = computeFloatingBaseGeneralizedTorques(interface, q, v, fext);
    const auto tauDynamicBaseTrue = pinocchio::nonLinearEffects(modelTrue, dataTrue, q, v).block<6, 1>(0, 0) + tauStaticTrue.block<6, 1>(0, 0);

    EXPECT_TRUE(tauDynamicBase.isApprox(tauDynamicBaseTrue, tolerance));

    const auto tauDynamicJoints = computeActuatedJointGeneralizedTorques(interface, info, q, v, fext);
    const auto tauDynamicJointsTrue = pinocchio::nonLinearEffects(modelTrue, dataTrue, q, v).block(6, 0, info.actuatedDofNum, 1) + tauStaticTrue.block(6, 0, info.actuatedDofNum, 1);

    EXPECT_TRUE(tauDynamicJoints.isApprox(tauDynamicJointsTrue, tolerance));


  }
}