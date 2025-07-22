// Copyright (c) 2025, Koło Naukowe Robotyków
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

/*
 * Authors: Bartłomiej Krajewski (https://github.com/BartlomiejK2)
 */

namespace floating_base_model
{
  namespace access_helper_functions
    { 
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 3, 1> getContactForces(Eigen::MatrixBase<Derived>& input,
      size_t contactIndex, 
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(input.rows() == info.inputDim);
      assert(input.cols() == 1);
      assert(contactIndex < info.numThreeDofContacts + info.numSixDofContacts);
      const size_t contactForceIndex = 3 * contactIndex;
      const size_t contactWrenchIndex = 3 * info.numThreeDofContacts + 6 * (contactIndex - info.numThreeDofContacts);
      const size_t startRow = (contactIndex < info.numThreeDofContacts) ? contactForceIndex : contactWrenchIndex;
      return Eigen::Block<Derived, 3, 1>(input.derived(), startRow, 0);
    };
    
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 3, 1> getContactForces(const Eigen::MatrixBase<Derived>& input, 
      size_t contactIndex,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(input.rows() == info.inputDim);
      assert(input.cols() == 1);
      assert(contactIndex < info.numThreeDofContacts + info.numSixDofContacts);
      const size_t contactForceIndex = 3 * contactIndex;
      const size_t contactWrenchIndex = 3 * info.numThreeDofContacts + 6 * (contactIndex - info.numThreeDofContacts);
      const size_t startRow = (contactIndex < info.numThreeDofContacts) ? contactForceIndex : contactWrenchIndex;
      return Eigen::Block<const Derived, 3, 1>(input.derived(), startRow, 0);
    };
                                                             
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 3, 1> getContactTorques(Eigen::MatrixBase<Derived>& input,
      size_t contactIndex,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(input.rows() == info.inputDim);
      assert(input.cols() == 1);
      assert(contactIndex < info.numThreeDofContacts + info.numSixDofContacts);
      assert(contactIndex >= info.numThreeDofContacts);
      const size_t startRow = 3 * info.numThreeDofContacts + 6 * (contactIndex - info.numThreeDofContacts) + 3;
      return Eigen::Block<Derived, 3, 1>(input.derived(), startRow, 0);
    };
                                                  
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 3, 1> getContactTorques(const Eigen::MatrixBase<Derived>& input,
      size_t contactIndex,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(input.rows() == info.inputDim);
      assert(input.cols() == 1);
      assert(contactIndex < info.numThreeDofContacts + info.numSixDofContacts);
      assert(contactIndex >= info.numThreeDofContacts);
      const size_t startRow = 3 * info.numThreeDofContacts + 6 * (contactIndex - info.numThreeDofContacts) + 3;
      return Eigen::Block<const Derived, 3, 1>(input.derived(), startRow, 0);
    };
    
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 6, 1> getContactWrenches(Eigen::MatrixBase<Derived>& input,
      size_t contactIndex,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(input.rows() == info.inputDim);
      assert(input.cols() == 1);
      assert(contactIndex < info.numThreeDofContacts + info.numSixDofContacts);
      assert(contactIndex >= info.numThreeDofContacts);
      const size_t startRow = 3 * info.numThreeDofContacts + 6 * (contactIndex - info.numThreeDofContacts);
      return Eigen::Block<Derived, 6, 1>(input.derived(), startRow, 0);
    };
  
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 6, 1> getContactWrenches(const Eigen::MatrixBase<Derived>& input,
      size_t contactIndex,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(input.rows() == info.inputDim);
      assert(input.cols() == 1);
      assert(contactIndex < info.numThreeDofContacts + info.numSixDofContacts);
      assert(contactIndex >= info.numThreeDofContacts);
      const size_t startRow = 3 * info.numThreeDofContacts + 6 * (contactIndex - info.numThreeDofContacts);
      return Eigen::Block<const Derived, 6, 1>(input.derived(), startRow, 0);
    };                                                           
    
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 6, 1> getBasePose(Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(state.rows() == info.stateDim);
      assert(state.cols() == 1);
      return Eigen::Block<Derived, 6, 1>(state.derived(), 6, 0);
    };
    
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 6, 1> getBasePose(const Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(state.rows() == info.stateDim);
      assert(state.cols() == 1);
      return Eigen::Block<const Derived, 6, 1>(state.derived(), 6, 0);
    };
  
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 3, 1> getBasePosition(Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(state.rows() == info.stateDim);
      assert(state.cols() == 1);
      return Eigen::Block<Derived, 3, 1>(state.derived(), 6, 0);
    };
  
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 3, 1> getBasePosition(const Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(state.rows() == info.stateDim);
      assert(state.cols() == 1);
      return Eigen::Block<const Derived, 3, 1>(state.derived(), 6, 0);
    };
  
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 3, 1> getBaseOrientationZyx(Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(state.rows() == info.stateDim);
      assert(state.cols() == 1);
      return Eigen::Block<Derived, 3, 1>(state.derived(), 9, 0);
    };
  
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 3, 1> getBaseOrientationZyx(const Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(state.rows() == info.stateDim);
      assert(state.cols() == 1);
      return Eigen::Block<const Derived, 3, 1>(state.derived(), 9, 0);
    };
    
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 6, 1> getBaseVelocity(Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(state.rows() == info.stateDim);
      assert(state.cols() == 1);
      return Eigen::Block<Derived, 6, 1>(state.derived(), 0, 0);
    };
    
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 6, 1> getBaseVelocity(const Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(state.rows() == info.stateDim);
      assert(state.cols() == 1);
      return Eigen::Block<const Derived, 6, 1>(state.derived(), 0, 0);
    };
  
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 3, 1> getBaseLinearVelocity(Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(state.rows() == info.stateDim);
      assert(state.cols() == 1);
      return Eigen::Block<Derived, 3, 1>(state.derived(), 0, 0);
    };
  
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 3, 1> getBaseLinearVelocity(const Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(state.rows() == info.stateDim);
      assert(state.cols() == 1);
      return Eigen::Block<const Derived, 3, 1>(state.derived(), 0, 0);
    };
  
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 3, 1>  getBaseAngularVelocity(Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(state.rows() == info.stateDim);
      assert(state.cols() == 1);
      return Eigen::Block<Derived, 3, 1>(state.derived(), 3, 0);
    };
  
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 3, 1>  getBaseAngularVelocity(const Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(state.rows() == info.stateDim);
      assert(state.cols() == 1);
      return Eigen::Block<const Derived, 3, 1>(state.derived(), 3, 0);
    };
  
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, -1, 1> getGeneralizedCoordinates(Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(state.rows() == info.stateDim);
      assert(state.cols() == 1);
      return Eigen::Block<Derived, -1, 1>(state.derived(), 6, 0, info.generalizedCoordinatesNum, 1);
    };
    
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, -1, 1> getGeneralizedCoordinates(const Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(state.rows() == info.stateDim);
      assert(state.cols() == 1);
      return Eigen::Block<const Derived, -1, 1>(state.derived(), 6, 0, info.generalizedCoordinatesNum, 1);
    };
    
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, -1, 1> getJointAngles(Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(state.rows() == info.stateDim);
      assert(state.cols() == 1);
      return Eigen::Block<Derived, -1, 1>(state.derived(), 12, 0, info.actuatedDofNum, 1);
    };
    
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, -1, 1> getJointAngles(const Eigen::MatrixBase<Derived>& state,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(state.rows() == info.stateDim);
      assert(state.cols() == 1);
      return Eigen::Block<const Derived, -1, 1>(state.derived(), 12, 0, info.actuatedDofNum, 1);
    };
  
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, -1, 1> getJointVelocities(Eigen::MatrixBase<Derived>& input,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(input.rows() == info.inputDim);
      assert(input.cols() == 1);
      const size_t startRow = 3 * info.numThreeDofContacts + 6 * info.numSixDofContacts;
      return Eigen::Block<Derived, -1, 1>(input.derived(), startRow, 0, info.actuatedDofNum, 1);
    };
    
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, -1, 1> getJointVelocities(const Eigen::MatrixBase<Derived>& input,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(input.rows() == info.inputDim);
      assert(input.cols() == 1);
      const size_t startRow = 3 * info.numThreeDofContacts + 6 * info.numSixDofContacts;
      return Eigen::Block<const Derived, -1, 1>(input.derived(), startRow, 0, info.actuatedDofNum, 1);
    };
    
  }; // namespace access_helper_functions
}; // namespace floating_base_model