#include "floating_base_model/FloatingBaseModelPinocchioMapping.hpp"


namespace floating_base_model
{ 

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  template <typename SCALAR_T>
  FloatingBaseModelPinocchioMappingTpl<SCALAR_T>::FloatingBaseModelPinocchioMappingTpl(FloatingBaseModelInfoTpl<SCALAR_T> floatingBaseModelInfo)
    : pinocchioInterfacePtr_(nullptr), floatingBaseModelInfo_(std::move(floatingBaseModelInfo)) {}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  template <typename SCALAR_T>
  FloatingBaseModelPinocchioMappingTpl<SCALAR_T>* FloatingBaseModelPinocchioMappingTpl<SCALAR_T>::clone() const
  {
    return new FloatingBaseModelPinocchioMappingTpl<SCALAR_T>(*this);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  template <typename SCALAR_T>
  void FloatingBaseModelPinocchioMappingTpl<SCALAR_T>::setPinocchioInterface(const ocs2::PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface)
  {
    pinocchioInterfacePtr_ = &pinocchioInterface;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  template <typename SCALAR_T>
  auto FloatingBaseModelPinocchioMappingTpl<SCALAR_T>::getPinocchioJointPosition(const vector_t& state) const -> vector_t
  {
    const auto& info = floatingBaseModelInfo_;
    assert(info.stateDim == state.rows());

    const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();

    const Eigen::Block<SCALAR_T, 3, 1> basePosition = access_helper_functions::getBasePosition(state, info);
    const Eigen::Block<SCALAR_T, 3, 1> baseEulerAngles = access_helper_functions::getBaseOrientationZyx(state, info);
    const Eigen::Quaternion<SCALAR_T>  baseQuaterion = quaterion_euler_transforms::getQuaternionFromEulerAnglesZyx(baseEulerAngles);
    const Eigen::Block<SCALAR_T, Eigen::Dynamic, 1> actuatedJointPostition = access_helper_functions::getJointAngles(state, info);
    
    vector_t pinocchioJointPosition(model.nq);
    pinocchioJointPosition << basePosition, baseQuaterion.coeffs(), actuatedJointPostition;

    return pinocchioJointPosition;
  }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  template <typename SCALAR_T>
  auto FloatingBaseModelPinocchioMappingTpl<SCALAR_T>::getPinocchioJointVelocity(const vector_t& state,
     const vector_t& input) const -> vector_t
  {
    const auto& info = floatingBaseModelInfo_;
    assert(info.stateDim == state.rows());
    assert(info.inputDim == input.rows());

    const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
    const Eigen::Block<SCALAR_T, 6, 1> baseVelocity = access_helper_functions::getBaseVelocity(state, info);
    const Eigen::Block<SCALAR_T, Eigen::Dynamic, 1> actuatedJointVelocities = access_helper_functions::getJointVelocities(input, info);
    
    vector_t pinocchioJointVelocities(model.nq);
    pinocchioJointVelocities << baseVelocity, actuatedJointVelocities;

    return pinocchioJointVelocities;
  }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  template <typename SCALAR_T>
  auto FloatingBaseModelPinocchioMappingTpl<SCALAR_T>::getOcs2Jacobian(const vector_t& state,
     const matrix_t& Jq,
     const matrix_t& Jv) const -> std::pair<matrix_t, matrix_t>
  {
    const auto& info = floatingBaseModelInfo_;
    assert(info.stateDim == state.rows());
    const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
    matrix_t t1(model.nq, model.nq);
    matrix_t t2(model.nq, model.nq);

    return {t1, t2};
  }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  template <typename SCALAR_T>
  FloatingBaseModelPinocchioMappingTpl<SCALAR_T>::FloatingBaseModelPinocchioMappingTpl(const FloatingBaseModelPinocchioMappingTpl& rhs)
  : pinocchioInterfacePtr_(nullptr), floatingBaseModelInfo_(rhs.floatingBaseModelInfo_) {}

} // namespace floating_base_model