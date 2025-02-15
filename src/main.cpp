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

#include <ocs2_robotic_tools/common/RotationTransforms.h>


#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <floating_base_model/ModelHelperFunctions.hpp>

#include <floating_base_model/FactoryFunctions.hpp>
#include <floating_base_model/QuaterionEulerTransforms.hpp>
//#include <floating_base_model/FloatingBaseModelDynamics.hpp>


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
  Eigen::Vector<ocs2::scalar_t, Eigen::Dynamic> q = Eigen::Vector<ocs2::scalar_t, Eigen::Dynamic>::Ones(model.nq);
  Eigen::Quaternion quaterion(q[6], q[3], q[4], q[5]);
  quaterion.normalize();
  q[3] = quaterion.x();
  q[4] = quaterion.y();
  q[5] = quaterion.z();
  q[6] = quaterion.w();
  Eigen::Vector<ocs2::scalar_t, Eigen::Dynamic> dq = Eigen::Vector<ocs2::scalar_t, Eigen::Dynamic>::Ones(model.nv);
  Eigen::Vector<ocs2::scalar_t, Eigen::Dynamic> tau = Eigen::Vector<ocs2::scalar_t, Eigen::Dynamic>::Zero(model.nv);

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
  Eigen::Matrix<double, 6, 6> Mb_2 = data.M.block<6,6>(0, 0);

  std::cout << "MB1:" << std::endl;
  std::cout << Mb_1 << std::endl;

  std::cout << "MB2:" << std::endl;
  std::cout << Mb_2 << std::endl;

  std::cout << "MB1 - MB2:" << std::endl;
  std::cout << Mb_1 - Mb_2 << std::endl;

  pinocchio::centerOfMass(model, data, true);
  Eigen::Vector3d r_com = data.com[1];
  // DZIALA!!!!!!
  start = std::chrono::high_resolution_clock::now();
  for(int j = 0; j < 1000; ++j)
  {
    pinocchio::Inertia inertia;
    Eigen::Matrix<double, 6, 6> Mb_moje;
    std::vector<pinocchio::SE3> bMi(model.njoints);
    bMi[1] = pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
    inertia = model.inertias[1];
    for(int i = 2; i < model.njoints; ++i)
    {
      int parent = model.parents[i];
      bMi[i] = bMi[parent] * data.liMi[i];
      inertia += bMi[i].act(model.inertias[i]);
    }
    Mb_moje = inertia.matrix();
  }
  stop = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  std::cout << "Algo2: " << duration.count() << std::endl;

  // std::cout << "MB_moje - MB2:" << std::endl;
  // std::cout << (Mb_moje - Mb_2).norm() << std::endl;
  // std::cout << Mb_moje - Mb_2 << std::endl;

  // FAIL :(
  Eigen::Matrix3d inertia_B;
  inertia_B = model.inertias[1].inertia();
  std::vector<pinocchio::SE3> bMi(model.njoints);
  bMi[1] = pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  for(int i = 2; i < model.njoints; ++i)
  {
    int parent = model.parents[i];
    bMi[i] = bMi[parent] * data.liMi[i];
    inertia_B += bMi[i].rotation() * model.inertias[i].inertia().matrix() * bMi[i].rotation().transpose() + model.inertias[i].mass() * ocs2::skewSymmetricMatrix(bMi[i].translation()) * ocs2::skewSymmetricMatrix(bMi[i].translation()).transpose();
  }


  std::cout << "TEST: " << std::endl;
  std::cout << (Mb_2.block<3,3>(3,0) - info.robotMass * ocs2::skewSymmetricMatrix(r_com)).norm() << std::endl;
  std::cout << (Mb_2.block<3,3>(0,3) + info.robotMass * ocs2::skewSymmetricMatrix(r_com)).norm() << std::endl;
  std::cout << Mb_2.block<3,3>(3,3) - inertia_B << std::endl;

  auto Mb_inv = floating_base_model::computeFloatingBaseLockedInertiaInverse(Mb_2);
  std::cout << "TEST ODWROTNOSC MB: " << std::endl;
  std::cout << Mb_inv * Mb_2 << std::endl;
  std::cout << Mb_2 << std::endl;
  std::cout << Mb_inv << std::endl;

  std::cout << Mb_inv.block<3,3>(3,0) * Mb_2.block<3,3>(0,3) + Mb_inv.block<3,3>(3,3) * Mb_2.block<3,3>(3,3) << std::endl;
  // pinocchio::Force force_1;
  // pinocchio::Force force_2;
  // pinocchio::Force force_3;
  // pinocchio::Force force_4;
  // // force_1.linear(Eigen::Vector<ocs2::scalar_t, 3>(1.0, 1.0, 1.0));

  // // std::string feetLinkName = "LFF_link";
  // // size_t feetId = model.getFrameId(feetLinkName);
  // // size_t feetJointId = model.frames[feetId].parentJoint;

  // // pinocchio::forwardKinematics(model, data, q);
  // // pinocchio::updateFramePlacements(model, data);
  // // std::cout << data.oMf[feetId] << std::endl;
  // // std::cout << data.oMi[feetJointId] << std::endl;

  // // pinocchio::SE3 iMf = data.oMi[feetJointId].actInv(data.oMf[feetId]);
  // // std::cout << iMf << std::endl;

  // // std::cout << force_1 << std::endl;
  // // std::cout << iMf.act(force_1) << std::endl;

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


  return 0;
}

