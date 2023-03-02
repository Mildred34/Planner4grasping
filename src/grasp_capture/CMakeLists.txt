cmake_minimum_required(VERSION 3.8)
project(grasp_capture)

# Default to C11
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)

install(DIRECTORY
  launch config rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

## Install Cpp node ##
# Install cpp "include" directory
include_directories(include)

## Create Cpp executable
set(NODE_0 moveit_node)
set(EXECUTABLE_0 ex_follow_target)

# Premier arg = nom de l'exe
add_executable(${EXECUTABLE_0} src/${NODE_0}.cpp)
ament_target_dependencies(${EXECUTABLE_0} rclcpp
                                 geometry_msgs
                                 moveit_ros_planning_interface
                                 moveit_visual_tools)

# Install Cpp executables
install(TARGETS
${EXECUTABLE_0}
 DESTINATION lib/${PROJECT_NAME})


##  Install Python modules ##
# ament_python_install_package(${PROJECT_NAME})

# # Install Python executables
# install(PROGRAMS
#   scripts/testyaml.py
#   scripts/get_expert_pose.py
#   DESTINATION lib/${PROJECT_NAME}
# )

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
