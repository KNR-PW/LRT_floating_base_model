#include <floating_base_model/FloatingBaseModelDynamics.hpp>
#include <ocs2_pinocchio_interface/urdf.h>
#include <urdf_parser/urdf_parser.h>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/container/aligned-vector.hpp>

#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <floating_base_model/QuaterionEulerTransforms.hpp>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
int main()
{
  std::string urdfPath = "/home/bartek/meldog.urdf";
  std::string urdfPath2 = "/home/bartek/meldog_2.urdf";
  ocs2::PinocchioInterface interface = floating_base_model::makeFloatingBaseInterface(urdfPath);
  ocs2::PinocchioInterface interface2 = floating_base_model::makeFloatingBaseInterface2(urdfPath2, "trunk_link");
  std::vector<std::string> threeDofContactNames;
  std::vector<std::string> sixDofContactNames;
  floating_base_model::FloatingBaseModelParams params(interface, threeDofContactNames, sixDofContactNames);
  params.print();

  const auto& model = interface.getModel();
  auto& data = interface.getData();
  Eigen::Vector<scalar_t, Eigen::Dynamic> q = Eigen::Vector<scalar_t, Eigen::Dynamic>::Ones(model.nq);
  Eigen::Quaternion quaterion(q[6], q[3], q[4], q[5]);
  quaterion.normalize();
  q[3] = quaterion.x();
  q[4] = quaterion.y();
  q[5] = quaterion.z();
  q[6] = quaterion.w();
  Eigen::Vector<scalar_t, Eigen::Dynamic> dq = Eigen::Vector<scalar_t, Eigen::Dynamic>::Ones(model.nv);
  Eigen::Vector<scalar_t, Eigen::Dynamic> tau = Eigen::Vector<scalar_t, Eigen::Dynamic>::Zero(model.nv);

  pinocchio::Force force_1;
  pinocchio::Force force_2;
  pinocchio::Force force_3;
  pinocchio::Force force_4;
  // force_1.linear(Eigen::Vector<scalar_t, 3>(1.0, 1.0, 1.0));

  // std::string feetLinkName = "LFF_link";
  // size_t feetId = model.getFrameId(feetLinkName);
  // size_t feetJointId = model.frames[feetId].parentJoint;

  // pinocchio::forwardKinematics(model, data, q);
  // pinocchio::updateFramePlacements(model, data);
  // std::cout << data.oMf[feetId] << std::endl;
  // std::cout << data.oMi[feetJointId] << std::endl;

  // pinocchio::SE3 iMf = data.oMi[feetJointId].actInv(data.oMf[feetId]);
  // std::cout << iMf << std::endl;

  // std::cout << force_1 << std::endl;
  // std::cout << iMf.act(force_1) << std::endl;

  // pinocchio::Force force_world = force_1;
  // force_world.angular(force_1.angular() + data.oMf[feetId].translation().cross(force_1.linear()));
  // std::cout << force_1 << std::endl;
  // std::cout << force_world << std::endl;
  

  // pinocchio::computeCentroidalMap(model, data, q);
  // // std::cout << data.Ag.block(0, 0, 6, 6) << std::endl;
  // // std::cout << data.Ycrb[1] << std::endl;

  // pinocchio::container::aligned_vector<pinocchio::Force> forces;
  // std::cout << model.njoints << std::endl;
  // std::cout << model.nbodies << std::endl;
  // std::cout << model.names[0] << std::endl;
  // std::cout << model.names[1] << std::endl;
  // std::cout << model.names[2] << std::endl;

  // std::cout << data.f.size() << std::endl;

  // for(size_t i = 0; i < model.njoints; ++i)
  // {
  //   if(i == feetJointId-1)
  //   {
  //     forces.push_back(force_1);
  //   }
  //   else
  //   {
  //     forces.push_back(pinocchio::Force());
  //   }
  // }
  // std::cout << data.f[feetJointId] << std::endl;
  // pinocchio::aba(model, data, q, dq, tau, forces, pinocchio::Convention::WORLD);
  // auto ddq_1 = data.ddq;
  // pinocchio::aba(model, data, q, dq, tau, forces);

  // std::cout << (ddq_1 - data.ddq).norm() << std::endl;
  // std::cout << data.f[0] << std::endl;
  // std::cout << data.f[feetJointId] << std::endl;

  // pinocchio::nonLinearEffects(model, data, q, dq);
  // pinocchio::computeCoriolisMatrix(model, data, q, dq);
  // pinocchio::computeGeneralizedGravity(model, data, q);

  // std::cout << quaterion << std::endl;


  // auto quaterion_2 = ocs2::matrixToQuaternion(data.oMi[1].rotation());
  // std::cout << quaterion_2 << std::endl;
  // Eigen::Vector<scalar_t, Eigen::Dynamic> error = data.nle - data.g - data.C*dq;
  // std::cout << "Error: " << error.norm() << std::endl;



  Eigen::Vector3d test_vector(3, 5, 7);
  Eigen::Vector3d euler(-2, -1, 1);
  quaterion = ocs2::getQuaternionFromEulerAnglesZyx(euler);
  auto dRdq = quaterion_euler_transforms::getRotationMatrixQuaterionGradient(quaterion);
  auto dRde = ocs2::getRotationMatrixZyxGradient(euler);
  auto quaterion_euler_map = quaterion_euler_transforms::getQuaternionFromEulerAnglesZyxGradient(euler);

  auto dRdz_euler = dRde[0];

  Eigen::Vector4d dqdz = quaterion_euler_map.leftCols<1>();
  auto dRdz_quaterion = dRdq[0] * dqdz[0] + dRdq[1] * dqdz[1] + dRdq[2] * dqdz[2] + dRdq[3] * dqdz[3];
  std::cout << "NORM: " << (dRdz_euler - dRdz_quaterion).norm() << std::endl;
  std::cout << quaterion_euler_transforms::test_func_derivative(euler) << std::endl;
  std::cout << quaterion_euler_transforms::test_func_derivative(quaterion) * quaterion_euler_map << std::endl;
  
  auto dvectordq = quaterion_euler_transforms::getRotatedVectorQuaterionGraient(quaterion, test_vector);

  Eigen::Matrix3d dvectorde;
  
  dvectorde.col(0) = dRde[0] * test_vector;
  dvectorde.col(1) = dRde[1] * test_vector;
  dvectorde.col(2) = dRde[2] * test_vector;

  std::cout << (dvectorde - dvectordq * quaterion_euler_map).norm() << std::endl;


  auto dEde = quaterion_euler_transforms::getMappingFromLocalAngularVelocitytoEulerAnglesDerivativeZyxGradient(euler);
  auto dEdq = quaterion_euler_transforms::getMappingFromLocalAngularVelocitytoEulerAnglesDerivativeQuaterionGradient(quaterion);
  
  Eigen::Matrix3d dvBde;

  dvBde.col(0) = dEde[0] * test_vector;
  dvBde.col(1) = dEde[1] * test_vector;
  dvBde.col(2) = dEde[2] * test_vector;

  Eigen::Matrix<double, 3, 4> dvBdq;

  dvBdq.col(0) = dEdq[0] * test_vector;
  dvBdq.col(1) = dEdq[1] * test_vector;
  dvBdq.col(2) = dEdq[2] * test_vector;
  dvBdq.col(3) = dEdq[3] * test_vector;

  dqdz = quaterion_euler_map.col(0);
  Eigen::Vector4d dqdy = quaterion_euler_map.col(1);
  Eigen::Vector4d dqdx = quaterion_euler_map.col(2);

  Eigen::Matrix3d dEdz_quaterion = dEdq[0] * dqdz[0] + dEdq[1] * dqdz[1] + dEdq[2] * dqdz[2] + dEdq[3] * dqdz[3];
  Eigen::Matrix3d dEdy_quaterion = dEdq[0] * dqdy[0] + dEdq[1] * dqdy[1] + dEdq[2] * dqdy[2] + dEdq[3] * dqdy[3];
  Eigen::Matrix3d dEdx_quaterion = dEdq[0] * dqdx[0] + dEdq[1] * dqdx[1] + dEdq[2] * dqdx[2] + dEdq[3] * dqdx[3];

  std::cout << (dvBde - dvBdq * quaterion_euler_map).norm() << std::endl;
  std::cout << "Z: " << std::endl;
  std::cout << (dEde[0] - dEdz_quaterion) << std::endl;

  std::cout << "Y: " << std::endl;
  std::cout << (dEde[1] - dEdy_quaterion) << std::endl;

  std::cout << "X: " << std::endl;
  std::cout << (dEde[2] - dEdx_quaterion) << std::endl;

  auto model1 = interface.getModel();
  auto model2 = interface2.getModel();

  if(model1 == model2)
  {
    std::cout << "SA TAKIE SAME" << std::endl;
  }


  return 0;
}

