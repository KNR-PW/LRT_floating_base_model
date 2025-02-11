namespace floating_base_model
{
  template <typename Derived>
  Eigen::Block<Derived, 3, 1> getContactForces(const FloatingBaseModelInfo params, Eigen::MatrixBase<Derived>& input, size_t contactIndex)
  {
    assert(input.rows() == params.inputDim);
    assert(input.cols() == 1);
    assert(contactIndex < params.numThreeDofContacts + params.numSixDofContacts);
    const size_t contactForceIndex = 3 * contactIndex;
    const size_t contactWrenchIndex = 3 * params.numThreeDofContacts + 6 * (contactIndex - params.numThreeDofContacts);
    const size_t startRow = (contactIndex < params.numThreeDofContacts) ? contactForceIndex : contactWrenchIndex;
    return Eigen::Block<Derived, 3, 1>(input.derived(), startRow, 0);
  };
  
  template <typename Derived>
  const Eigen::Block<const Derived, 3, 1> getContactForces(const FloatingBaseModelInfo params, const Eigen::MatrixBase<Derived>& input, size_t contactIndex)
  {
    assert(input.rows() == params.inputDim);
    assert(input.cols() == 1);
    assert(contactIndex < params.numThreeDofContacts + params.numSixDofContacts);
    const size_t contactForceIndex = 3 * contactIndex;
    const size_t contactWrenchIndex = 3 * params.numThreeDofContacts + 6 * (contactIndex - params.numThreeDofContacts);
    const size_t startRow = (contactIndex < params.numThreeDofContacts) ? contactForceIndex : contactWrenchIndex;
    return Eigen::Block<Derived, 3, 1>(input.derived(), startRow, 0);
  };
                                                           
  
  template <typename Derived>
  Eigen::Block<Derived, 3, 1> getContactTorques(const FloatingBaseModelInfo params, Eigen::MatrixBase<Derived>& input, size_t contactIndex)
  {
    assert(input.rows() == params.inputDim);
    assert(input.cols() == 1);
    assert(contactIndex < params.numThreeDofContacts + params.numSixDofContacts);
    assert(contactIndex >= params.numThreeDofContacts);
    const size_t startRow = 3 * params.numThreeDofContacts + 6 * (contactIndex - params.numThreeDofContacts) + 3;
    return Eigen::Block<Derived, 3, 1>(input.derived(), startRow, 0);
  };
                                                
  
  template <typename Derived>
  const Eigen::Block<const Derived, 3, 1> getContactTorques(const FloatingBaseModelInfo params, const Eigen::MatrixBase<Derived>& input, size_t contactIndex)
  {
    assert(input.rows() == params.inputDim);
    assert(input.cols() == 1);
    assert(contactIndex < params.numThreeDofContacts + params.numSixDofContacts);
    assert(contactIndex >= params.numThreeDofContacts);
    const size_t startRow = 3 * params.numThreeDofContacts + 6 * (contactIndex - params.numThreeDofContacts) + 3;
    return Eigen::Block<const Derived, 3, 1>(input.derived(), startRow, 0);
  };
                                                            
  
  template <typename Derived>
  Eigen::Block<Derived, -1, 1> getJointVelocities(const FloatingBaseModelInfo params, Eigen::MatrixBase<Derived>& input)
  {
    assert(input.rows() == params.inputDim);
    assert(input.cols() == 1);
    const size_t startRow = 3 * params.numThreeDofContacts + 6 * params.numSixDofContacts;
    return Eigen::Block<Derived, -1, 1>(input.derived(), startRow, 0, params.actuatedDofNum, 1);
  };
  
  template <typename Derived>
  const Eigen::Block<const Derived, -1, 1> getJointVelocities(const FloatingBaseModelInfo params, const Eigen::MatrixBase<Derived>& input)
  {
    assert(input.rows() == params.inputDim);
    assert(input.cols() == 1);
    const size_t startRow = 3 * params.numThreeDofContacts + 6 * params.numSixDofContacts;
    return Eigen::Block<const Derived, -1, 1>(input.derived(), startRow, 0, params.actuatedDofNum, 1);
  };                                                           
  
  template <typename Derived>
  Eigen::Block<Derived, 6, 1> getBasePose(const FloatingBaseModelInfo params, Eigen::MatrixBase<Derived>& state)
  {
    assert(state.rows() == params.stateDim);
    assert(state.cols() == 1);
    return Eigen::Block<Derived, 6, 1>(state.derived(), 6, 0);
  };
  
  template <typename Derived>
  const Eigen::Block<const Derived, 6, 1> getBasePose(const FloatingBaseModelInfo params, const Eigen::MatrixBase<Derived>& state)
  {
    assert(state.rows() == params.stateDim);
    assert(state.cols() == 1);
    return Eigen::Block<const Derived, 6, 1>(state.derived(), 6, 0);
  };
  
  template <typename Derived>
  Eigen::Block<Derived, 6, 1> getBaseVelocity(const FloatingBaseModelInfo params, Eigen::MatrixBase<Derived>& state)
  {
    assert(state.rows() == params.stateDim);
    assert(state.cols() == 1);
    return Eigen::Block<Derived, 6, 1>(state.derived(), 0, 0);
  };
  
  template <typename Derived>
  const Eigen::Block<const Derived, 6, 1> getBaseVelocity(const FloatingBaseModelInfo params, const Eigen::MatrixBase<Derived>& state)
  {
    assert(state.rows() == params.stateDim);
    assert(state.cols() == 1);
    return Eigen::Block<const Derived, 6, 1>(state.derived(), 0, 0);
  }
  
  template <typename Derived>
  Eigen::Block<Derived, -1, 1> getJointAngles(const FloatingBaseModelInfo params, Eigen::MatrixBase<Derived>& state)
  {
    assert(state.rows() == params.stateDim);
    assert(state.cols() == 1);
    return Eigen::Block<Derived, -1, 1>(state.derived(), 12, 0, params.actuatedDofNum, 1);
  };
  
  template <typename Derived>
  const Eigen::Block<const Derived, -1, 1> getJointAngles(const FloatingBaseModelInfo params, const Eigen::MatrixBase<Derived>& state)
  {
    assert(state.rows() == params.stateDim);
    assert(state.cols() == 1);
    return Eigen::Block<const Derived, -1, 1>(state.derived(), 12, 0, params.actuatedDofNum, 1);
  };
  
  template <typename Derived>
  Eigen::Block<Derived, -1, 1> getGeneralizedCoordinates(const FloatingBaseModelInfo params, Eigen::MatrixBase<Derived>& state)
  {
    assert(state.rows() == params.stateDim);
    assert(state.cols() == 1);
    return Eigen::Block<Derived, -1, 1>(state.derived(), 6, 0, params.generalizedCoordinatesNum, 1);
  };
  
  template <typename Derived>
  const Eigen::Block<const Derived, -1, 1> getGeneralizedCoordinates(const FloatingBaseModelInfo params, const Eigen::MatrixBase<Derived>& state)
  {
    assert(state.rows() == params.stateDim);
    assert(state.cols() == 1);
    return Eigen::Block<Derived, -1, 1>(state.derived(), 6, 0, params.generalizedCoordinatesNum, 1);
  };

};