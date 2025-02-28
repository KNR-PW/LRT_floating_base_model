#include <ocs2_pinocchio_interface/urdf.h>
#include <urdf_parser/urdf_parser.h>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/container/aligned-vector.hpp>

#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/center-of-mass-derivatives.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>


#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <floating_base_model/ModelHelperFunctions.hpp>

#include <floating_base_model/FactoryFunctions.hpp>
#include <floating_base_model/QuaterionEulerTransforms.hpp>
#include <floating_base_model/AccessHelperFunctions.hpp>
#include <floating_base_model/PinocchioFloatingBaseDynamics.hpp>


#include <chrono>

int main()
{
  std::string urdfPath = "/home/bartek/meldog.urdf";
  std::string urdfPath2 = "/home/bartek/meldog_2.urdf";
  ocs2::PinocchioInterface interface = floating_base_model::createPinocchioInterface(urdfPath2, "trunk_link");
  std::vector<std::string> threeDofContactNames;
  std::vector<std::string> sixDofContactNames;
  threeDofContactNames.push_back("RFF_link");
  threeDofContactNames.push_back("RRF_link");
  threeDofContactNames.push_back("LFF_link");
  threeDofContactNames.push_back("LRF_link");
  floating_base_model::FloatingBaseModelInfo info = floating_base_model::createFloatingBaseModelInfo(interface, threeDofContactNames, sixDofContactNames);
  std::cout << interface << std::endl;
  std::cout << info << std::endl;
  
  for(int i = 0; i < info.endEffectorFrameIndices.size(); ++i)
  {
    std::cout << "Contact Frame: " << threeDofContactNames[i] << ", " << info.endEffectorFrameIndices[i] << std::endl;
    std::cout << "Contact Joint: " << interface.getModel().names[info.endEffectorJointIndices[i]] << ", " << info.endEffectorJointIndices[i] << std::endl;
  }

  //floating_base_model::FloatingBaseModelInfoCppAd info_ad = info.toCppAd();

  const auto& model = interface.getModel();
  auto& data = interface.getData();
  Eigen::Vector<ocs2::scalar_t, Eigen::Dynamic> q = Eigen::Vector<ocs2::scalar_t, Eigen::Dynamic>::Random(model.nq);
  Eigen::Quaternion quaterion(q[6], q[3], q[4], q[5]);
  quaterion.normalize();
  q[3] = quaterion.x();
  q[4] = quaterion.y();
  q[5] = quaterion.z();
  q[6] = quaterion.w();
  Eigen::Vector<ocs2::scalar_t, Eigen::Dynamic> dq = Eigen::Vector<ocs2::scalar_t, Eigen::Dynamic>::Random(model.nv);
  Eigen::Vector<ocs2::scalar_t, Eigen::Dynamic> tau = Eigen::Vector<ocs2::scalar_t, Eigen::Dynamic>::Random(model.nv);
  Eigen::Vector<ocs2::scalar_t, Eigen::Dynamic> ddq = Eigen::Vector<ocs2::scalar_t, Eigen::Dynamic>::Zero(model.nv);


  
  ocs2::vector_t input(24);
  ocs2::vector_t state(24);
  
  input = ocs2::vector_t::Random(24);
  state = ocs2::vector_t::Random(24);
  
  Eigen::Vector<ocs2::scalar_t, 3> eulerAngles = Eigen::Vector<ocs2::scalar_t, 3>::Random();
  ocs2::makeEulerAnglesUnique(eulerAngles);
  state.block<3,1>(9, 0) = eulerAngles;
  
  floating_base_model::PinocchioFloatingBaseDynamics dynamics(info);
  dynamics.setPinocchioInterface(interface);
  
  auto input_old = input;
  const auto value = dynamics.getValue(0, state, input);
  const auto value_1 = dynamics.getValue2(0, state, input);
  std::cout << "TEST: " << std::endl;
  std::cout << value - value_1 << std::endl;
  std::cout << "AAAA" << std::endl;

  std::cout << pinocchio::rnea(model, data, q, dq, ddq) - pinocchio::nonLinearEffects(model, data, q, dq) << std::endl;
  pinocchio::Force zero_force(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  pinocchio::container::aligned_vector<pinocchio::Force> fext(model.njoints, zero_force);
  std::cout << "AAAA" << std::endl;
  floating_base_model::model_helper_functions::computeForceVector(interface, info, input, fext);
  std::cout << "AAAA" << std::endl;
  floating_base_model::FloatingBaseModelPinocchioMapping mapping(info);
  std::cout << "AAAA" << std::endl;
  mapping.setPinocchioInterface(interface);
  q = mapping.getPinocchioJointPosition(state);
  dq = mapping.getPinocchioJointVelocity(state, input);
  
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);
  std::cout << "AAAA" << std::endl;
  Eigen::Vector<ocs2::scalar_t, 6> tau_1 = Eigen::Vector<ocs2::scalar_t, 6>::Zero();
  //tau_1 = pinocchio::nonLinearEffects(model, data, q, dq).block<6,1>(0,0);
  for (size_t i = 0; i < info.numThreeDofContacts; i++) {
    const auto forceWorldFrame = floating_base_model::access_helper_functions::getContactForces(input, i, info);
    size_t contactFrameIndex = info.endEffectorFrameIndices[i];
    pinocchio::Data::Matrix6x J(6, model.nv);
    J.setZero();
    pinocchio::computeFrameJacobian(model, data, q, contactFrameIndex, pinocchio::LOCAL_WORLD_ALIGNED, J);
    
    std::cout << "Jacobian: " << contactFrameIndex << ": " << "\n" << J.transpose().block<6,3>(0,0) << std::endl;
    tau_1 += -J.transpose().block<6,3>(0,0) * forceWorldFrame;
  }  
  std::cout << "4" << std::endl;
  // for (size_t i = info.numThreeDofContacts; i < info.numThreeDofContacts + info.numSixDofContacts; i++) {
  //   const Eigen::Block<Eigen::VectorXd, 6, 1> wrenchWorldFrame = floating_base_model::access_helper_functions::getContactWrenches(input, i, info);
  //   size_t contactFrameIndex = info.endEffectorFrameIndices[i];
  //   const auto jacobian_T = pinocchio::getFrameJacobian(model, data, contactFrameIndex, pinocchio::LOCAL_WORLD_ALIGNED).transpose();
  //   tau_1 += -jacobian_T.block<6,6>(0,0) * wrenchWorldFrame;
  // }  
  std::cout << "AAAA" << std::endl;
  std::cout << "TEST TAU:" << std::endl;
  std::cout << tau_1 << std::endl;
  std::cout << pinocchio::computeStaticTorque(model, data, q, fext).block<6,1>(0,0) - pinocchio::computeGeneralizedGravity(model, data, q).block<6,1>(0,0)   << std::endl;
  std::cout << "AAAA" << std::endl;
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::crba(model, data, q, pinocchio::Convention::LOCAL);

  Eigen::Matrix<double, 6, 6> Mb_1 = data.M.block<6,6>(0, 0);

  q = Eigen::Vector<ocs2::scalar_t, Eigen::Dynamic>::Random(model.nq);
  Eigen::Quaternion quaterion2(q[6], q[3], q[4], q[5]);
  quaterion2.normalize();
  q[3] = quaterion2.x();
  q[4] = quaterion2.y();
  q[5] = quaterion2.z();
  q[6] = quaterion2.w();

  pinocchio::forwardKinematics(model, data, q);
  auto start = std::chrono::high_resolution_clock::now();
  for(int i = 0; i < 1000; ++i)
  {
    pinocchio::crba(model, data, q, pinocchio::Convention::LOCAL);
  }
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  std::cout << "Algo1: " << duration.count() << std::endl;
  // Eigen::Matrix<double, 6, 6> Mb_2 = data.M.block<6,6>(0, 0);

  // std::cout << "MB1:" << std::endl;
  // std::cout << Mb_1 << std::endl;

  // std::cout << "MB2:" << std::endl;
  // std::cout << Mb_2 << std::endl;

  // std::cout << "MB1 - MB2:" << std::endl;
  // std::cout << Mb_1 - Mb_2 << std::endl;

  // pinocchio::centerOfMass(model, data, true);
  // Eigen::Vector3d r_com = data.com[1];
  // // DZIALA!!!!!!
  Eigen::Matrix<double, 6, 6> Mb_moje;
  start = std::chrono::high_resolution_clock::now();
  for(int j = 0; j < 1000; ++j)
  {
    Mb_moje = floating_base_model::model_helper_functions::computeFloatingBaseLockedInertia(interface, q);
  }
  stop = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  std::cout << "Algo2: " << duration.count() << std::endl;

  // std::cout << "MB_moje - MB2:" << std::endl;
  // std::cout << (Mb_moje - Mb_2).norm() << std::endl;
  // std::cout << Mb_moje - Mb_2 << std::endl;

  // FAIL :(
  // Eigen::Matrix3d inertia_B;
  // inertia_B = model.inertias[1].inertia();
  // std::vector<pinocchio::SE3> bMi(model.njoints);
  // bMi[1] = pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  // for(int i = 2; i < model.njoints; ++i)
  // {
  //   int parent = model.parents[i];
  //   bMi[i] = bMi[parent] * data.liMi[i];
  //   inertia_B += bMi[i].rotation() * model.inertias[i].inertia().matrix() * bMi[i].rotation().transpose() + model.inertias[i].mass() * ocs2::skewSymmetricMatrix(bMi[i].translation()) * ocs2::skewSymmetricMatrix(bMi[i].translation()).transpose();
  // }


  // std::cout << "TEST: " << std::endl;
  // std::cout << (Mb_2.block<3,3>(3,0) - info.robotMass * ocs2::skewSymmetricMatrix(r_com)).norm() << std::endl;
  // std::cout << (Mb_2.block<3,3>(0,3) + info.robotMass * ocs2::skewSymmetricMatrix(r_com)).norm() << std::endl;
  // std::cout << Mb_2.block<3,3>(3,3) - inertia_B << std::endl;
  // auto Mb_inv = floating_base_model::model_helper_functions::computeFloatingBaseLockedInertiaInverse(Mb_2);
  // std::cout << "TEST ODWROTNOSC MB: " << std::endl;
  // std::cout << Mb_inv * Mb_2 << std::endl;
  // std::cout << Mb_2 << std::endl;
  // std::cout << Mb_inv << std::endl;

  // std::cout << Mb_inv.block<3,3>(3,0) * Mb_2.block<3,3>(0,3) + Mb_inv.block<3,3>(3,3) * Mb_2.block<3,3>(3,3) << std::endl;
  // pinocchio::Force force_1;
  // pinocchio::Force force_2;
  // pinocchio::Force force_3;
  // pinocchio::Force force_4;
  // // force_1.linear(Eigen::Vector<ocs2::scalar_t, 3>(1.0, 1.0, 1.0));

  pinocchio::Force force;
  force.linear()  = Eigen::Vector3d::Random();
  force.angular() = Eigen::Vector3d::Random();
  std::string feetLinkName = "LFF_link";
  size_t feetId = model.getFrameId(feetLinkName);
  size_t feetJointId = model.frames[feetId].parentJoint;

  pinocchio::Force new_force;
  new_force.linear() = data.oMi[feetJointId].rotation().transpose() * force.linear();
  new_force.angular() = data.oMi[feetJointId].rotation().transpose() * force.angular();
  new_force.angular() += model.frames[feetId].placement.translation().cross(new_force.linear());

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  // // std::cout << data.oMf[feetId] << std::endl;
  // // std::cout << data.oMi[feetJointId] << std::endl;

  pinocchio::SE3 iMf = data.oMi[feetJointId].actInv(data.oMf[feetId]);
  // // std::cout << iMf << std::endl;

  std::cout << "FORCE TEST:" << std::endl;
  std::cout << iMf.act(force) << std::endl;
  std::cout << new_force << std::endl;

  // // pinocchio::Force force_world = force_1;
  // // force_world.angular(force_1.angular() + data.oMf[feetId].translation().cross(force_1.linear()));
  // // std::cout << force_1 << std::endl;
  // // std::cout << force_world << std::endl;
  

  // // pinocchio::computeCentroidalMap(model, data, q);
  // // // std::cout << data.Ag.block(0, 0, 6, 6) << std::endl;
  // // // std::cout << data.Ycrb[1] << std::endl;

  // // pinocchio::container::aligned_vector<pinocchio::Force> forces;
  // // std::cout << model.njoints << std::endl;
  // // std::cout << model.nbodies << std::endl;
  // // std::cout << model.names[0] << std::endl;
  // // std::cout << model.names[1] << std::endl;
  // // std::cout << model.names[2] << std::endl;

  // // std::cout << data.f.size() << std::endl;

  // // for(size_t i = 0; i < model.njoints; ++i)
  // // {
  // //   if(i == feetJointId-1)
  // //   {
  // //     forces.push_back(force_1);
  // //   }
  // //   else
  // //   {
  // //     forces.push_back(pinocchio::Force());
  // //   }
  // // }
  // // std::cout << data.f[feetJointId] << std::endl;
  // // pinocchio::aba(model, data, q, dq, tau, forces, pinocchio::Convention::WORLD);
  // // auto ddq_1 = data.ddq;
  // // pinocchio::aba(model, data, q, dq, tau, forces);

  // // std::cout << (ddq_1 - data.ddq).norm() << std::endl;
  // // std::cout << data.f[0] << std::endl;
  // // std::cout << data.f[feetJointId] << std::endl;

  // // pinocchio::nonLinearEffects(model, data, q, dq);
  // // pinocchio::computeCoriolisMatrix(model, data, q, dq);
  // // pinocchio::computeGeneralizedGravity(model, data, q);

  // // std::cout << quaterion << std::endl;


  // // auto quaterion_2 = ocs2::matrixToQuaternion(data.oMi[1].rotation());
  // // std::cout << quaterion_2 << std::endl;
  // // Eigen::Vector<ocs2::scalar_t, Eigen::Dynamic> error = data.nle - data.g - data.C*dq;
  // // std::cout << "Error: " << error.norm() << std::endl;



  // Eigen::Vector3d test_vector(3, 5, 7);
  // Eigen::Vector3d euler(-2, -1, 1);
  // quaterion = ocs2::getQuaternionFromEulerAnglesZyx(euler);
  // auto dRdq = quaterion_euler_transforms::getRotationMatrixQuaterionGradient(quaterion);
  // auto dRde = ocs2::getRotationMatrixZyxGradient(euler);
  // auto quaterion_euler_map = quaterion_euler_transforms::getQuaternionFromEulerAnglesZyxGradient(euler);

  // auto dRdz_euler = dRde[0];

  // Eigen::Vector4d dqdz = quaterion_euler_map.leftCols<1>();
  // auto dRdz_quaterion = dRdq[0] * dqdz[0] + dRdq[1] * dqdz[1] + dRdq[2] * dqdz[2] + dRdq[3] * dqdz[3];
  // std::cout << "NORM: " << (dRdz_euler - dRdz_quaterion).norm() << std::endl;
  // std::cout << quaterion_euler_transforms::test_func_derivative(euler) << std::endl;
  // std::cout << quaterion_euler_transforms::test_func_derivative(quaterion) * quaterion_euler_map << std::endl;
  
  // auto dvectordq = quaterion_euler_transforms::getRotatedVectorQuaterionGraient(quaterion, test_vector);

  // Eigen::Matrix3d dvectorde;
  
  // dvectorde.col(0) = dRde[0] * test_vector;
  // dvectorde.col(1) = dRde[1] * test_vector;
  // dvectorde.col(2) = dRde[2] * test_vector;

  // std::cout << (dvectorde - dvectordq * quaterion_euler_map).norm() << std::endl;


  // auto dEde = quaterion_euler_transforms::getMappingFromLocalAngularVelocitytoEulerAnglesDerivativeZyxGradient(euler);
  // auto dEdq = quaterion_euler_transforms::getMappingFromLocalAngularVelocitytoEulerAnglesDerivativeQuaterionGradient(quaterion);
  
  // Eigen::Matrix3d dvBde;

  // dvBde.col(0) = dEde[0] * test_vector;
  // dvBde.col(1) = dEde[1] * test_vector;
  // dvBde.col(2) = dEde[2] * test_vector;

  // Eigen::Matrix<double, 3, 4> dvBdq;

  // dvBdq.col(0) = dEdq[0] * test_vector;
  // dvBdq.col(1) = dEdq[1] * test_vector;
  // dvBdq.col(2) = dEdq[2] * test_vector;
  // dvBdq.col(3) = dEdq[3] * test_vector;

  // dqdz = quaterion_euler_map.col(0);
  // Eigen::Vector4d dqdy = quaterion_euler_map.col(1);
  // Eigen::Vector4d dqdx = quaterion_euler_map.col(2);

  // Eigen::Matrix3d dEdz_quaterion = dEdq[0] * dqdz[0] + dEdq[1] * dqdz[1] + dEdq[2] * dqdz[2] + dEdq[3] * dqdz[3];
  // Eigen::Matrix3d dEdy_quaterion = dEdq[0] * dqdy[0] + dEdq[1] * dqdy[1] + dEdq[2] * dqdy[2] + dEdq[3] * dqdy[3];
  // Eigen::Matrix3d dEdx_quaterion = dEdq[0] * dqdx[0] + dEdq[1] * dqdx[1] + dEdq[2] * dqdx[2] + dEdq[3] * dqdx[3];

  // std::cout << (dvBde - dvBdq * quaterion_euler_map).norm() << std::endl;
  // std::cout << "Z: " << std::endl;
  // std::cout << (dEde[0] - dEdz_quaterion) << std::endl;

  // std::cout << "Y: " << std::endl;
  // std::cout << (dEde[1] - dEdy_quaterion) << std::endl;

  // std::cout << "X: " << std::endl;
  // std::cout << (dEde[2] - dEdx_quaterion) << std::endl;

  // auto model1 = interface.getModel();
  // auto model2 = interface2.getModel();

  // if(model1 == model2)
  // {
  //   std::cout << "SA TAKIE SAME" << std::endl;
  // }
  // else
  // {
  //   std::cout << model1.nframes << std::endl;
  //   std::cout << model2.nframes << std::endl;
  //   std::cout << model1.njoints << std::endl;
  //   std::cout << model2.njoints << std::endl;
  //   std::cout << model1.nbodies << std::endl;
  //   std::cout << model2.nbodies << std::endl;
  // }

  // Eigen::MatrixXd v_partial_dq(6, model.nv);
  // Eigen::MatrixXd v_partial_dv(6, model.nv);
  // pinocchio::computeForwardKinematicsDerivatives(model, data, q, dq, ddq);
  // pinocchio::getJointVelocityDerivatives(model, data, 1, pinocchio::LOCAL, v_partial_dq, v_partial_dv);
  // std::cout << "v_partial_dq: " << std::endl;
  // std::cout << v_partial_dq.leftCols(6) << std::endl;
  // std::cout << "v_partial_dv: " << std::endl;
  // std::cout << v_partial_dv.leftCols(6) << std::endl;

  // int frame_id = model.getFrameId("trunk_link");
  // Eigen::MatrixXd frame_v_partial_dq(6, model.nv);
  // frame_v_partial_dq.setZero();
  // Eigen::MatrixXd frame_v_partial_dv(6, model.nv);
  // frame_v_partial_dv.setZero();
  // pinocchio::computeForwardKinematicsDerivatives(model, data, q, dq, ddq);
  // pinocchio::getFrameVelocityDerivatives(model, data, frame_id, pinocchio::LOCAL, frame_v_partial_dq, frame_v_partial_dv);
  // std::cout << "frame_v_partial_dq: " << std::endl;
  // std::cout << frame_v_partial_dq << std::endl;
  // std::cout << "frame_v_partial_dv: " << std::endl;
  // std::cout << frame_v_partial_dv.leftCols(6)<< std::endl;
  
  // pinocchio::forwardKinematics(model, data, q);
  // auto frame_vel = pinocchio::getFrameVelocity(model, data, frame_id, pinocchio::WORLD);
  // std::cout << "TESTY:\n";
  // std::cout << data.oMi[1] << std::endl;
  // std::cout << data.oMf[frame_id] << std::endl;
  // std::cout << data.v[1] << std::endl;
  // std::cout << frame_vel << std::endl;

  // std::cout << model.frames[frame_id].placement << std::endl;

  // Eigen::MatrixXd frame_v_partial_dq_2(6, model.nv);
  // pinocchio::motionSet::se3Action(data.oMi[1],frame_v_partial_dq, frame_v_partial_dq_2);
  // std::cout << "frame_v_partial_dq local: " << std::endl;
  // std::cout << frame_v_partial_dq_2 << std::endl;

  // std::cout << model.parents[1] << std::endl;



  // Eigen::Vector3d wB = Eigen::Vector3d::Random();
  // std::cout << "wB:\n" << wB << std::endl;

  // Eigen::Vector3d eulerAngles = Eigen::Vector3d::Random();
  // std::cout << "euler:\n" << eulerAngles << std::endl;

  // auto dEd_euler = quaterion_euler_transforms::getMappingFromLocalAngularVelocitytoEulerAnglesDerivativeZyxGradient(eulerAngles);
  // auto E = quaterion_euler_transforms::getMappingFromLocalAngularVelocitytoEulerAnglesDerivative(eulerAngles);
  // Eigen::Matrix3d E_inv = E.inverse();

  // Eigen::Vector3d deuler = E * wB;
  // auto wB_skew = ocs2::skewSymmetricMatrix(wB);
  

  // Eigen::Matrix3d test_1 = - E * wB_skew * E_inv;
  // Eigen::Matrix3d test_1_2;
  // test_1_2.col(0) = dEd_euler[0] * deuler;
  // test_1_2.col(1) = dEd_euler[1] * deuler;
  // test_1_2.col(2) = dEd_euler[2] * deuler;
  // Eigen::Matrix3d test_3 = test_1_2 * E_inv + test_1; 
  // Eigen::Matrix3d test_2 = Eigen::Matrix3d::Zero();
  // test_2.col(0) = dEd_euler[0] * wB;
  // test_2.col(1) = dEd_euler[1] * wB;
  // test_2.col(2) = dEd_euler[2] * wB;

  // std::cout << "PLEASE1: " << std::endl;
  // std::cout << test_1 << std::endl;
  // std::cout << "POPRAWNE: " << std::endl;
  // std::cout << test_2 << std::endl;

  // std::cout << "PLEASE2: " << std::endl;
  // std::cout << test_1_2 << std::endl;




  // pinocchio::crba(model, data, q, pinocchio::Convention::LOCAL);
  // Eigen::Matrix<double, 6, 6> Mb_p = data.M.block<6,6>(0, 0);

  // auto Mb_inv = floating_base_model::model_helper_functions::computeFloatingBaseLockedInertiaInverse(Mb_p);

  // Eigen::Vector<double, 6> tau_base = tau.block<6, 1>(0, 0);

  // auto my_acceleration = floating_base_model::model_helper_functions::computeBaseBodyAcceleration(Mb_p, tau_base);

  // std::cout << Mb_p.inverse() * tau_base - my_acceleration << std::endl;
  // std::cout << Mb_p.inverse() * tau_base - Mb_inv * tau_base << std::endl;

  // //auto Mb_moje = floating_base_model::model_helper_functions::computeFloatingBaseLockedInertia(interface, q);

  // std::cout << Mb_p - Mb_moje << std::endl;

  return 0;
}

