cmake_minimum_required(VERSION 3.8)
project(ur5_robot_gripper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_executable(robot_control src/robot_control.cpp)
target_include_directories(robot_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robot_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  robot_control
  "rclcpp"
  "moveit_ros_planning_interface"
)

install(TARGETS robot_control
  DESTINATION lib/${PROJECT_NAME})

add_executable(gripper_control src/gripper_control.cpp)
target_include_directories(gripper_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(gripper_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  gripper_control
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(PROGRAMS
  scripts/pub_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS gripper_control
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
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
