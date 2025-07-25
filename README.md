# ur5_robot_gripper

## 项目简介

本仓库旨在为基于 MoveIt 的机器人控制提供一套更易用的 C++ 封装。通过本项目定义的 ROS2 service 和 action，用户可以方便地与机器人进行通信，实现如获取当前状态、移动到指定位置或姿态、设置运动约束等常用操作。  
虽然仓库名称以 UR 机械臂为例，但本项目实际上适用于所有类型的机器人，只需将 MoveIt 控制的 group 名称修改为目标机器人对应的 group 即可。

## 主要功能

- **服务与动作接口**：通过 ROS2 的 service 和 action，封装了常用的机器人操作，便于上层系统或脚本调用。
- **支持仿真与实机**：同时兼容仿真环境（如 Isaac Sim）和真实机器人硬件，用户可根据实际需求选择不同的 node。
- **良好的适配性**：只需更改 group 配置，即可适配不同类型的机械臂和末端执行器。
- **调试与展示支持**：提供多种 launch 文件和 RViz 配置，方便开发调试和运动可视化。

## 目录结构说明

- `src/`：核心 C++ 源码，包括仿真与实机两套控制实现，以及夹爪控制等。
- `include/`：头文件。
- `msg/`、`srv/`、`action/`：自定义消息、服务和动作接口定义。
- `launch/`：主要启动文件，推荐使用 `ur_robotiq_control.launch.py`，该文件负责启动与机器人和夹爪的通信。使用不同真实机器人时，建议首先修改此文件，确保通信配置正确。
- `scripts/`：常用 Python 脚本，如 tf 发布、姿态变换服务、客户端测试等。
- `config/`：控制器参数、更新频率等配置文件。
- `urdf/`、`meshes/`：存放机器人和夹爪的描述文件及 mesh 模型（目前内容较多，后续会整理）。
- `display.launch.py`、`description.launch.py` 等：主要用于调试和展示。

## 适用范围

- **UR 机械臂**：可直接使用。
- **其他机械臂**：只需更改 MoveIt 的 group 配置即可适配。

---

## 版本说明

### 当前版本状态
- **robot_control.cpp/hpp** 和 **robot_control node**: 这是之前针对Isaac Sim仿真环境开发的版本
- **robot_control_real.cpp/hpp** 和 **robot_control_real node**: 这是为真实机器人硬件开发的新版本

### 版本差异说明
原始的 `robot_control.cpp`、`robot_control.hpp` 以及 `robot_control` node 是为了与Isaac Sim通信而设计的，其中包含了许多针对仿真环境的权衡设计：

1. **规划和执行分离**: 为了与Isaac Sim的通信机制兼容，规划生成后不会自动执行，需要手动调用执行函数
3. **通信协议**: 使用了特定的service通信协议来与Isaac Sim交互

### 兼容性考虑
为了保持与现有Isaac Sim程序的兼容性，同时支持真实机器人硬件，我们采用了以下策略：

1. **保留原版本**: 保持原有的 `robot_control` 相关文件不变，确保Isaac Sim环境下的程序继续正常工作
2. **新增real版本**: 创建了带 `_real` 后缀的新文件，专门用于真实机器人硬件
3. **功能改进**: 新版本中规划和执行已经集成，会自动执行规划好的轨迹
4. **未来计划**: 调试完成后，计划将两个版本的功能合并，提供统一的接口

### 使用建议
- **Isaac Sim环境**: 使用原有的 `robot_control` 相关文件
- **真实硬件环境**: 使用 `robot_control_real` 相关文件

## Trouble shooting
### Cmake
1. 使用`rosidl_target_interfaces(gripper_control ${PROJECT_NAME} "rosidl_typesupport_cpp")`会出现弃用警告，应当使用`target_link_libraries(gripper_control ${PROJECT_NAME}__rosidl_typesupport_cpp)`