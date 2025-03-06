cmake_minimum_required(VERSION 3.16 )
project(floating_base_model LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(CMAKE_CXX_STANDARD 17)

##################
## Dependencies ##
##################
find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_DEPENDS
  ocs2_pinocchio_interface
  ocs2_robotic_tools 
  ocs2_centroidal_model
)

###########
## Build ##
###########

set(FLAGS
        ${OCS2_CXX_FLAGS}
        ${pinocchio_CFLAGS_OTHER}
        -Wno-ignored-attributes
        -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
        -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
        -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

add_library(${PROJECT_NAME}
  src/FloatingBaseModelInfo.cpp
  src/FactoryFunctions.cpp
  src/QuaterionEulerTransforms.cpp
  src/ModelHelperFunctions.cpp
  src/FloatingBaseModelPinocchioMapping.cpp
  src/PinocchioFloatingBaseDynamics.cpp
  src/PinocchioFloatingBaseDynamicsAD.cpp
  # src/main.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_DEPENDS})
target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

# Resolve for the package path at compile time.
configure_file (
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/path_management/package_path.h.in"
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/path_management/package_path.h" @ONLY
)


#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS src include test
            CT_HEADER_DIRS include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)


#############
## Testing ##
#############
# find_package(ament_lint_auto REQUIRED)
# ament_lint_auto_find_test_dependencies()

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(${PROJECT_NAME}_test
            test/QuaterionEulerTransformsTest.cpp
            test/FactoryFunctionsTest.cpp
            test/AccessHelperFunctionsTest.cpp
            test/ModelHelperFunctionsTest.cpp
            test/FloatingBaseModelPinocchioMappingTest.cpp
            test/PinocchioFloatingBaseDynamicsTest.cpp
    )
    ament_target_dependencies(${PROJECT_NAME}_test ${THIS_PACKAGE_DEPENDS})
    target_include_directories(${PROJECT_NAME}_test PRIVATE test/include)
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
    target_compile_options(${PROJECT_NAME}_test PUBLIC ${FLAGS})

endif ()

ament_package(CONFIG_EXTRAS "cmake/pinocchio_config.cmake")