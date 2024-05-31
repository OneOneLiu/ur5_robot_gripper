#!/bin/bash
# entrypoint.sh

# 输出正在开始同步仓库的信息
echo "Starting synchronization of Git repositories..."
echo "-----------------------------------------------"

# Update repositories
cd /catkin_ws/src/robotiq_gripper && git pull
cd /catkin_ws/src/ur5_robot && git pull
cd /catkin_ws/src/ur5_gripper_moveit && git pull

# 输出完成同步的信息
echo "-----------------------------------------------"
echo "Synchronization of all repositories completed."

# 移动到工作空间文件夹
cd /catkin_ws

# 执行容器的主命令
exec "$@"
