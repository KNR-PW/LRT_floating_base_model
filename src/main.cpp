#include <centroidal_model/CentroidalModelDynamics.hpp>
#include <ocs2_pinocchio_interface/urdf.h>
#include <urdf_parser/urdf_parser.h>

int main()
{
  std::string urdfPath = "/home/bartek/meldog.urdf";
  ocs2::PinocchioInterface interface = legged_mpc::makeCentroidalInterface(urdfPath);
  std::vector<std::string> threeDofContactNames;
  std::vector<std::string> sixDofContactNames;
  legged_mpc::CentroidalModelParams params(interface, threeDofContactNames, sixDofContactNames);
  params.print();
  return 0;
}

