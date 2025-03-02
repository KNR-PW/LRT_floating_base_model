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
    INPUT_DIM = 2 * 3 + 2 * 6 + 12,  // end effector forces, actuated joint velocities
    ACTUATED_DOF_NUM = 12,
    GENERALIZED_COORDINATES_NUM = 6 + 12, 
    NUM_THREE_DOF_CONTACTS = 2, // FOR TEST
    NUM_SIX_DOF_CONTACTS = 2,   // FOR TEST
  };

  // Contact frames
  static const std::vector<std::string> meldog3DofContactNames = {"RFF_link", "RRF_link"};
  static const std::vector<std::string> meldog6DofContactNames = {"LFF_link", "LRF_link"};

  // Two urdfs, one with additional link before base link (will be removed by factory function)
  static const std::string meldogWithBaseLinkUrdfFile = package_path::getPath() + "/test/test_models/meldog_base_link.urdf";
  static const std::string meldogWithoutBaseLinkUrdfFile = package_path::getPath() + "/test/test_models/meldog_no_base_link.urdf";
  
  // Base link name
  static const std::string baseLink = "trunk_link";

  // End effector parent joint frame names
  static const std::vector<std::string> meldogEndEffectorJointNames = {"LFK_joint", "LRK_joint", "RFK_joint", "RRK_joint"};

  inline ocs2::vector_t getAccessTestRobotState() 
  {
    ocs2::vector_t x = ocs2::vector_t::Zero(Meldog::STATE_DIM);
    
    // Base Velocitiy: [linear, angular]
    x(0) = 0.0;  // vB_x
    x(1) = 1.0;  // vB_y
    x(2) = 2.0;  // vB_z
    x(3) = 3.0;  // wB_x
    x(4) = 4.0;  // wB_y
    x(5) = 5.0;  // wB_z
  
    // Base Pose: [position, orientation]
    x(6)  = 6.0;   // p_base_x
    x(7)  = 7.0;   // p_base_y
    x(8)  = 8.0;   // p_base_z
    x(9)  = 9.0;   // theta_base_z
    x(10) = 10.0;  // theta_base_y
    x(11) = 11.0;  // theta_base_x
  
    // Leg Joint Positions: 
    x(12) = 12.0;   
    x(13) = 13.0;
    x(14) = 14.0;
    x(15) = 15.0;
    x(16) = 16.0;
    x(17) = 17.0;
    x(18) = 18.0;
    x(19) = 19.0;
    x(20) = 20.0;    
    x(21) = 21.0;
    x(22) = 22.0;
    x(23) = 23.0;
   
    return x;
  };

  inline ocs2::vector_t getAccessTestRobotInput() 
  {
    ocs2::vector_t x = ocs2::vector_t::Zero(Meldog::INPUT_DIM);
  
    // Contact Forces:
    x(0) = 0.0;   // F_x RFF_link
    x(1) = 1.0;   // F_y RFF_link
    x(2) = 2.0;   // F_z RFF_link

    x(3) = 3.0;   // F_x RRF_link
    x(4) = 4.0;   // F_y RRF_link
    x(5) = 5.0;   // F_z RRF_link

    x(6)  = 6.0;   // F_x LFF_link
    x(7)  = 7.0;   // F_y LFF_link
    x(8)  = 8.0;   // F_z LFF_link
    x(9)  = 9.0;   // T_x LFF_link
    x(10) = 10.0;  // T_y LFF_link
    x(11) = 11.0;  // T_z LFF_link


    x(12) = 12.0;  // F_x LRF_link
    x(13) = 13.0;  // F_y LRF_link
    x(14) = 14.0;  // F_z LRF_link
    x(15) = 15.0;  // T_x LRF_link
    x(16) = 16.0;  // T_y LRF_link
    x(17) = 17.0;  // T_z LRF_link
  
    // Leg Joint Velocities: 
    x(18) = 18.0;   
    x(19) = 19.0;
    x(20) = 20.0;    
    x(21) = 21.0;
    x(22) = 22.0;
    x(23) = 23.0;
    x(24) = 24.0;
    x(25) = 25.0;
    x(26) = 26.0;
    x(27) = 27.0;
    x(28) = 28.0;
    x(29) = 29.0;
   
    return x;
  };

}  // namespace floating_base_model
