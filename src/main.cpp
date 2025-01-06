#include <floating_base_model/FloatingBaseModelDynamics.hpp>
#include <ocs2_pinocchio_interface/urdf.h>
#include <urdf_parser/urdf_parser.h>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/aba.hpp>
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
  Eigen::Vector<scalar_t, Eigen::Dynamic> q = Eigen::Vector<scalar_t, Eigen::Dynamic>::Zero(model.nq);

  pinocchio::computeCentroidalMap(model, data, q);
  std::cout << data.Ag.block(0, 0, 6, 6) << std::endl;
  std::cout << data.Ycrb[1] << std::endl;
  return 0;
}

