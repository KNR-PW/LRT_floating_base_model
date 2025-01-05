#include "centroidal_model/CentroidalModelDynamics.hpp"

legged_mpc::CentroidalModelDynamics::CentroidalModelDynamics(const CentroidalModelParams& params, PinocchioInterface& interface):
params_{params}, interface_{interface}{}



PinocchioInterface legged_mpc::makeCentroidalInterface(const std::string& urdfFilePath)
{
  return centroidal_model::createPinocchioInterface(urdfFilePath);
}
