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

#include <floating_base_model/QuaterionTransforms.hpp>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
int main()
{
  std::string urdfPath = "/home/bartek/meldog.urdf";
  ocs2::PinocchioInterface interface = floating_base_model::makeFloatingBaseInterface(urdfPath);
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

  pinocchio::nonLinearEffects(model, data, q, dq);
  pinocchio::computeCoriolisMatrix(model, data, q, dq);
  pinocchio::computeGeneralizedGravity(model, data, q);

  std::cout << quaterion << std::endl;


  auto quaterion_2 = ocs2::matrixToQuaternion(data.oMi[1].rotation());
  std::cout << quaterion_2 << std::endl;
  Eigen::Vector<scalar_t, Eigen::Dynamic> error = data.nle - data.g - data.C*dq;
  std::cout << "Error: " << error.norm() << std::endl;



  Eigen::Vector3d test_vector(1, 2, 3);
  Eigen::Vector3d euler(1, -1, M_PI/6);
  auto quaterion_map = quaterion_transforms::getVectorQuaterionDerivative(test_vector, quaterion);
  auto quaterion_euler_map = quaterion_transforms::getQuaternionFromEulerAnglesZyxDerivative(euler);

  Eigen::Matrix3d vec_1;
  std::cout << quaterion_map << std::endl;
  std::cout << quaterion_euler_map << std::endl;
  vec_1 = quaterion_map * quaterion_euler_map;

  auto rotation_derivatives = ocs2::getRotationMatrixZyxGradient(euler);

  Eigen::Matrix3d vec_2 = Eigen::Matrix3d::Zero();

  vec_2.leftCols<1>() = rotation_derivatives[0] * test_vector;
  vec_2.middleCols<1>(1) = rotation_derivatives[1] * test_vector;
  vec_2.rightCols<1>() = rotation_derivatives[2] * test_vector;
  std::cout << vec_1 << std::endl;
  std::cout << vec_2 << std::endl;
  std::cout << "WYNIK"<< std::endl;
  std::cout << vec_1 - vec_2 << std::endl;


  // auto q_1 = quaterion_transforms::getQuaternionFromEulerAnglesZyx(euler);
  // auto q_2 = quaterion_transforms::getQuaternionFromEulerAnglesZyx2(euler);
  // std::cout << q_1 << std::endl;
  // std::cout << q_2 << std::endl;

  auto r_quat = quaterion_transforms::getVectorQuaterionDerivative2(quaterion);

  Eigen::Matrix<double, 3, 3> matrix_1;

  Eigen::Vector4d temp = quaterion_euler_map * test_vector;
  matrix_1.noalias() = r_quat[0] * temp[0] + r_quat[1] * temp[1] + r_quat[2] * temp[2] + r_quat[3] * temp[3];
  

  //vec_1 = matrix_1 * quaterion_euler_map;
  std::cout << "WYNIK"<< std::endl;
  std::cout << matrix_1 - vec_2 << std::endl;

  // std::cout << (matrix_1) - quaterion_map << std::endl;


  auto euler_local = ocs2::getMappingFromEulerAnglesZyxDerivativeToLocalAngularVelocity(euler);
  auto quaterion_3 = quaterion_transforms::getQuaternionFromEulerAnglesZyx(euler);
  auto local_euler = quaterion_transforms::getMappingFromLocalAngularVelocitytoEulerAnglesDerivative(quaterion_3);
  std::cout << "REAL SIN: " << sin(euler(2)) << std::endl;
  std::cout << "REAL COS: " << cos(euler(2)) << std::endl;
  std::cout << euler_local * local_euler << std::endl;

  std::cout << quaterion_transforms::test_func_derivative(euler) << std::endl;
  std::cout << quaterion_transforms::test_func_derivative(quaterion) * quaterion_euler_map << std::endl;
  return 0;
}

