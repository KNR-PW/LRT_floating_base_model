namespace legged_mpc
{
  template <typename Derived>
  Eigen::Block<Derived, 3, 1> CentroidalModelDynamics::getContactForces(Eigen::MatrixBase<Derived>& input, size_t contactIndex)
  {
    assert(input.rows() == params_.inputDim);
    assert(input.cols() == 1);
    assert(contactIndex < params_.numThreeDofContacts + params_.numSixDofContacts);
    const size_t contactForceIndex = 3 * contactIndex;
    const size_t contactWrenchIndex = 3 * params_.numThreeDofContacts + 6 * (contactIndex - params_.numThreeDofContacts);
    const size_t startRow = (contactIndex < params_.numThreeDofContacts) ? contactForceIndex : contactWrenchIndex;
    return Eigen::Block<Derived, 3, 1>(input.derived(), startRow, 0);
  };
  
  template <typename Derived>
  const Eigen::Block<const Derived, 3, 1> CentroidalModelDynamics::getContactForces(const Eigen::MatrixBase<Derived>& input, size_t contactIndex)
  {
    assert(input.rows() == params_.inputDim);
    assert(input.cols() == 1);
    assert(contactIndex < params_.numThreeDofContacts + params_.numSixDofContacts);
    const size_t contactForceIndex = 3 * contactIndex;
    const size_t contactWrenchIndex = 3 * params_.numThreeDofContacts + 6 * (contactIndex - params_.numThreeDofContacts);
    const size_t startRow = (contactIndex < params_.numThreeDofContacts) ? contactForceIndex : contactWrenchIndex;
    return Eigen::Block<Derived, 3, 1>(input.derived(), startRow, 0);
  };
                                                           
  
  template <typename Derived>
  Eigen::Block<Derived, 3, 1> CentroidalModelDynamics::getContactTorques(Eigen::MatrixBase<Derived>& input, size_t contactIndex)
  {
    assert(input.rows() == params_.inputDim);
    assert(input.cols() == 1);
    assert(contactIndex < params_.numThreeDofContacts + params_.numSixDofContacts);
    assert(contactIndex >= params_.numThreeDofContacts);
    const size_t startRow = 3 * params_.numThreeDofContacts + 6 * (contactIndex - params_.numThreeDofContacts) + 3;
    return Eigen::Block<Derived, 3, 1>(input.derived(), startRow, 0);
  };
                                                
  
  template <typename Derived>
  const Eigen::Block<const Derived, 3, 1> CentroidalModelDynamics::getContactTorques(const Eigen::MatrixBase<Derived>& input, size_t contactIndex)
  {
    assert(input.rows() == params_.inputDim);
    assert(input.cols() == 1);
    assert(contactIndex < params_.numThreeDofContacts + params_.numSixDofContacts);
    assert(contactIndex >= params_.numThreeDofContacts);
    const size_t startRow = 3 * params_.numThreeDofContacts + 6 * (contactIndex - params_.numThreeDofContacts) + 3;
    return Eigen::Block<const Derived, 3, 1>(input.derived(), startRow, 0);
  };
                                                            
  
  template <typename Derived>
  Eigen::Block<Derived, -1, 1> CentroidalModelDynamics::getJointVelocities(Eigen::MatrixBase<Derived>& input)
  {
    assert(input.rows() == params_.inputDim);
    assert(input.cols() == 1);
    const size_t startRow = 3 * params_.numThreeDofContacts + 6 * params_.numSixDofContacts;
    return Eigen::Block<Derived, -1, 1>(input.derived(), startRow, 0, params_.actuatedDofNum, 1);
  };
  
  template <typename Derived>
  const Eigen::Block<const Derived, -1, 1> CentroidalModelDynamics::getJointVelocities(const Eigen::MatrixBase<Derived>& input)
  {
    assert(input.rows() == params_.inputDim);
    assert(input.cols() == 1);
    const size_t startRow = 3 * params_.numThreeDofContacts + 6 * params_.numSixDofContacts;
    return Eigen::Block<const Derived, -1, 1>(input.derived(), startRow, 0, params_.actuatedDofNum, 1);
  };                                                           
  
  template <typename Derived>
  Eigen::Block<Derived, 6, 1> CentroidalModelDynamics::getBasePose(Eigen::MatrixBase<Derived>& state)
  {
    assert(state.rows() == params_.stateDim);
    assert(state.cols() == 1);
    return Eigen::Block<Derived, 6, 1>(state.derived(), 6, 0);
  };
  
  template <typename Derived>
  const Eigen::Block<const Derived, 6, 1> CentroidalModelDynamics::getBasePose(const Eigen::MatrixBase<Derived>& state)
  {
    assert(state.rows() == params_.stateDim);
    assert(state.cols() == 1);
    return Eigen::Block<const Derived, 6, 1>(state.derived(), 6, 0);
  };
  
  template <typename Derived>
  Eigen::Block<Derived, 6, 1> CentroidalModelDynamics::getBaseVelocity(Eigen::MatrixBase<Derived>& state)
  {
    assert(state.rows() == params_.stateDim);
    assert(state.cols() == 1);
    return Eigen::Block<Derived, 6, 1>(state.derived(), 0, 0);
  };
  
  template <typename Derived>
  const Eigen::Block<const Derived, 6, 1> CentroidalModelDynamics::getBaseVelocity(const Eigen::MatrixBase<Derived>& state)
  {
    assert(state.rows() == params_.stateDim);
    assert(state.cols() == 1);
    return Eigen::Block<const Derived, 6, 1>(state.derived(), 0, 0);
  }
  
  template <typename Derived>
  Eigen::Block<Derived, -1, 1> CentroidalModelDynamics::getJointAngles(Eigen::MatrixBase<Derived>& state)
  {
    assert(state.rows() == params_.stateDim);
    assert(state.cols() == 1);
    return Eigen::Block<Derived, -1, 1>(state.derived(), 12, 0, params_.actuatedDofNum, 1);
  };
  
  template <typename Derived>
  const Eigen::Block<const Derived, -1, 1> CentroidalModelDynamics::getJointAngles(const Eigen::MatrixBase<Derived>& state)
  {
    assert(state.rows() == params_.stateDim);
    assert(state.cols() == 1);
    return Eigen::Block<const Derived, -1, 1>(state.derived(), 12, 0, params_.actuatedDofNum, 1);
  };
  
  template <typename Derived>
  Eigen::Block<Derived, -1, 1> CentroidalModelDynamics::getGeneralizedCoordinates(Eigen::MatrixBase<Derived>& state)
  {
    assert(state.rows() == params_.stateDim);
    assert(state.cols() == 1);
    return Eigen::Block<Derived, -1, 1>(state.derived(), 6, 0, params_.generalizedCoordinatesNum, 1);
  };
  
  template <typename Derived>
  const Eigen::Block<const Derived, -1, 1> CentroidalModelDynamics::getGeneralizedCoordinates(const Eigen::MatrixBase<Derived>& state)
  {
    assert(state.rows() == params_.stateDim);
    assert(state.cols() == 1);
    return Eigen::Block<Derived, -1, 1>(state.derived(), 6, 0, params_.generalizedCoordinatesNum, 1);
  };

};