
#ifndef _CENTROIDAL_MODEL_INFO_HPP_

#define _CENTROIDAL_MODEL_INFO_HPP_

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>

#include <ostream>
#include <string>
#include <type_traits>
#include <urdf_parser/urdf_parser.h>

#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_pinocchio_interface/urdf.h>


namespace floating_base_model
{
  template <typename SCALAR_T>
  class FloatingBaseModelInfoTpl;

  using FloatingBaseModelInfo = FloatingBaseModelInfoTpl<ocs2::scalar_t>;
  using FloatingBaseModelInfoCppAd = FloatingBaseModelInfoTpl<ocs2::ad_scalar_t>;

  template <typename SCALAR_T>
  struct FloatingBaseModelInfoTpl
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using scalar_t = SCALAR_T;

    template <typename T>  // Template for conditional compilation using SFINAE
    using EnableIfScalar_t = typename std::enable_if<std::is_same<T, scalar_t>::value, bool>::type;
  
    size_t numThreeDofContacts;                   // 3DOF contacts, force only
    size_t numSixDofContacts;                     // 6DOF contacts, force and torque
    std::vector<size_t> endEffectorFrameIndices;  // indices of end-effector frames [3DOF contacts, 6DOF contacts]
    std::vector<size_t> endEffectorJointIndices;  // indices of end-effector parent joints [3DOF contacts, 6DOF contacts]
    size_t generalizedCoordinatesNum;             // number of generalized coordinates in the model (not pinocchio!)
    size_t actuatedDofNum;                        // number of actuated degrees of freedom
    size_t stateDim;                              // number of states needed to define the system flow map
    size_t inputDim;                              // number of inputs needed to define the system flow map
    scalar_t robotMass;                           // total robot mass

    /** Casts FloatingBaseModelInfo to FloatingBaseModelInfoCppAD. */
    template <typename T = SCALAR_T, EnableIfScalar_t<T> = true>
    FloatingBaseModelInfoCppAd toCppAd() const;
  };

  std::string toString(const FloatingBaseModelInfo& info);
  std::ostream& operator<<(std::ostream& os,const FloatingBaseModelInfo& info);

  /* Explicit template instantiation for scalar_t and ad_scalar_t */
  extern template struct FloatingBaseModelInfoTpl<ocs2::scalar_t>;
  extern template struct FloatingBaseModelInfoTpl<ocs2::ad_scalar_t>;
}; // namespace floating_base_model

#endif