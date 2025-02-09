cmake_minimum_required(VERSION 3.16 )
project(floating_base_model LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(CMAKE_CXX_STANDARD 17)
# find dependencies
find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_DEPENDS
  ocs2_pinocchio_interface 
  ocs2_centroidal_model
)


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

add_executable(${PROJECT_NAME}
  src/FloatingBaseModelDynamics.cpp
  src/FloatingBaseModelParams.cpp
  src/main.cpp
  src/QuaterionEulerTransforms.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_DEPENDS})

target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()




# cmake_minimum_required(VERSION 3.16)
# project(floating_base_model LANGUAGES CXX)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# set(CMAKE_CXX_STANDARD 17)
# find_package(ament_cmake REQUIRED)

# set(THIS_PACKAGE_DEPENDS
#   pinocchio
#   ocs2_pinocchio_interface 
#   eigen3_cmake_module
#   Eigen3
# )

# # find dependencies
# foreach(Dependency IN ITEMS ${THIS_PACKAGE_DEPENDS})
#   find_package(${Dependency} REQUIRED)
# endforeach()

# include_directories(include)

# add_executable(${PROJECT_NAME} 
#   src/CentroidalModel.cpp
#   src/MultiEndEffectorKinematics.cpp
# )

# # find_package(cmake_clang_tools QUIET)
# # if (cmake_clang_tools_FOUND)
# #     message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
# #     add_clang_tooling(
# #             TARGETS ${PROJECT_NAME}
# #             SOURCE_DIRS src include test
# #             CT_HEADER_DIRS include
# #             CF_WERROR
# #     )
# # endif (cmake_clang_tools_FOUND)

# ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_DEPENDS})

# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# install(TARGETS
#   ${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

# install(
#   DIRECTORY include/
#   DESTINATION include/${PROJECT_NAME}
# )

# ament_package()
