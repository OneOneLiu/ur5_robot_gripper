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

# Generate custom ROS messages
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StringArray.msg"
  "srv/PoseTransform.srv"
  "srv/MoveToPose.srv"
  "srv/MoveToPosition.srv"
  "srv/PrintPose.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

# Build a robot_control c++ library
add_library(robot_control STATIC src/robot_control.cpp)
target_include_directories(robot_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robot_control PUBLIC cxx_std_17)  # Require C++17
ament_target_dependencies(
  robot_control
  "rclcpp"
  "moveit_ros_planning_interface"
)

# Build a robot_control service c++ executable
add_executable(robot_control_node src/robot_control_node.cpp)
target_include_directories(robot_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robot_control_node PUBLIC cxx_std_17)  # Require C++17
ament_target_dependencies(
  robot_control_node
  "rclcpp"
  "moveit_ros_planning_interface"
  "geometry_msgs"
)
target_link_libraries(robot_control_node robot_control)
rosidl_target_interfaces(robot_control_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
# Install the library and executable
install(TARGETS robot_control_node
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
install(TARGETS gripper_control
  DESTINATION lib/${PROJECT_NAME})

include_directories(${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

# Install folders
install(PROGRAMS
  scripts/pub_tf.py
  scripts/pose_transform_server.py
  scripts/robot_control_client.py
  scripts/grasp_tube.py
  scripts/fake_sim_time.py
  DESTINATION lib/${PROJECT_NAME}
)
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
install(DIRECTORY
  include/
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
