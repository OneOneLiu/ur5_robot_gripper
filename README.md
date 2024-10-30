# Readme.md

> 由于 moveit2 在humble上的 Python 支持还不太好，这里写一个基于C++的package。
>

## Trouble shooting
### Cmake
1. 使用`rosidl_target_interfaces(gripper_control ${PROJECT_NAME} "rosidl_typesupport_cpp")`会出现弃用警告，应当使用`target_link_libraries(gripper_control ${PROJECT_NAME}__rosidl_typesupport_cpp)`