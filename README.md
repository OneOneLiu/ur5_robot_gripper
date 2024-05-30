# UR5 机器人与Robotiq 2F-85 夹爪模型

> 本软件包提供
> - 机器人与夹爪的联合模型文件
> - 使用python编写的moveit运动控制接口. 具体的运动规划使用请查看[src/ur5_gripper_moveit](../ur5_gripper_moveit)
> - Luanch
>   - 一个简单的可视化launch文件
>   - 启动抓取环境的lanuch文件, 依赖[src/ur5_gripper_moveit](../ur5_gripper_moveit)以及[src/robotiq_gripper](../robotiq_gripper)

## ToDo
- [ ] Use slider to control joints (like robotiq_gripper)