cmake_minimum_required(VERSION 3.8)
project(robot_simulation_utilities)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Install cpp "include" directory
include_directories(include)

set(grasp_capture_lib
  robot_simulation_utilities
)

## Define a library target called robot_simulation_pkg
add_library(${grasp_capture_lib} SHARED
  src/object_simulation.cpp
)

## Add Target dependencies
ament_target_dependencies(${grasp_capture_lib}
  rclcpp
  tf2_geometry_msgs
  tf2_ros
  geometry_msgs
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# This line to exports the library
# ament_export_targets(${grasp_capture_lib} HAS_LIBRARY_TARGET)

## Install Library include/library directory to the install/include/library ##
 install(DIRECTORY
  include/${PROJECT_NAME}
  DESTINATION include
)

install(
  TARGETS ${grasp_capture_lib}
  EXPORT ${grasp_capture_lib}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

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

ament_package()
