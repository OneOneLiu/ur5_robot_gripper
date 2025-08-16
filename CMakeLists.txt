cmake_minimum_required(VERSION 3.8)
project(ur5_robot_gripper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_DEPENDS
  ament_cmake
  Boost
  Eigen3
  moveit_visual_tools
  rviz_visual_tools
  rclcpp
  moveit_ros_planning_interface
  Python3
  rosidl_default_generators
  action_msgs
  nlohmann_json
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Generate custom ROS messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StringArray.msg"
  "msg/BoundingBox2D.msg"
  "msg/BoundingBoxes2D.msg"
  "srv/PoseTransform.srv"
  "srv/MoveToPose.srv"
  "srv/MoveToPosition.srv"
  "srv/PrintPose.srv"
  "srv/SetConstraints.srv"
  "srv/MoveToJointPosition.srv"
  "srv/SetCameraPose.srv"
  "action/MoveToPositionAction.action"
  "action/MoveToPoseAction.action"
  "action/MoveGripperAction.action"
  "action/ExecuteGrasp.action"
  "action/MoveToJointPositionAction.action"
  DEPENDENCIES builtin_interfaces geometry_msgs action_msgs trajectory_msgs
)

# Macro to reduce target重复代码
macro(add_my_library name src)
  add_library(${name} STATIC ${src})
  target_include_directories(${name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)
  target_compile_features(${name} PUBLIC cxx_std_17)
  ament_target_dependencies(${name}
    rclcpp
    moveit_ros_planning_interface
    rosidl_typesupport_cpp
    Boost
    Eigen3
    moveit_visual_tools
    rviz_visual_tools
  )
  target_link_libraries(${name} ${PROJECT_NAME}__rosidl_typesupport_cpp)
endmacro()

macro(add_my_executable name src link_libs)
  add_executable(${name} ${src})
  target_include_directories(${name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)
  target_compile_features(${name} PUBLIC cxx_std_17)
  ament_target_dependencies(${name}
    rclcpp
    moveit_ros_planning_interface
    geometry_msgs
    rclcpp_action
  )
  target_link_libraries(${name} ${link_libs} ${PROJECT_NAME}__rosidl_typesupport_cpp)
endmacro()

# Libraries
add_my_library(robot_control src/robot_control.cpp)
add_my_library(robot_control_real src/robot_control_real.cpp)
add_my_library(gripper_control src/gripper_control.cpp)

# Executables
add_my_executable(robot_control_node src/robot_control_node.cpp robot_control)
add_my_executable(robot_function_test src/robot_function_test.cpp robot_control)
add_my_executable(robot_control_node_real src/robot_control_node_real.cpp robot_control_real)
add_my_executable(robot_function_test_real src/robot_function_test_real.cpp robot_control_real)
add_my_executable(gripper_control_node src/gripper_control_node.cpp gripper_control)

# Install all executables
install(TARGETS
  robot_control_node
  robot_function_test
  robot_control_node_real
  robot_function_test_real
  gripper_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install scripts and folders
install(PROGRAMS
  scripts/pub_tf.py
  scripts/pose_transform_server.py
  scripts/robot_control_client.py
  scripts/fake_sim_time.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY include/ DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
