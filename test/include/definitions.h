#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <floating_base_model/path_management/package_path.h>

namespace floating_base_model 
{

  enum Meldog : size_t {
    STATE_DIM = 6 + 6 + 12,  // base body velocity, generalized coordinates
    INPUT_DIM = 4 * 3 + 12,  // end effector forces, actuated joint velocities
    ACTUATED_DOF_NUM = 12,
    GENERALIZED_COORDINATES_NUM = 6 + 12,
    NUM_THREE_DOF_CONTACTS = 4,
    NUM_SIX_DOF_CONTACTS = 0,
  };

  // Contact frames
  static const std::vector<std::string> meldog3DofContactNames = {"RFF_link", "RRF_link", "LFF_link", "LRF_link"};
  static const std::vector<std::string> meldog6DofContactNames = {};

  // Two urdfs, one with additional link before base link (will be removed by factory function)
  static const std::string meldogWithBaseLinkUrdfFile = package_path::getPath() + "/test/test_models/meldog_base_link.urdf";
  static const std::string meldogWithoutBaseLinkUrdfFile = package_path::getPath() + "/test/test_models/meldog_no_base_link.urdf";
  
  // Base link name
  static const std::string baseLink = "trunk_link";

  // End effector parent joint frame names
  static const std::vector<std::string> meldogEndEffectorJointNames = {"LFK_joint", "LRK_joint", "RFK_joint", "RRK_joint"};
  
}  // namespace floating_base_model
