#!/bin/bash
# entrypoint.sh"

# 移动到工作空间文件夹
cd /catkin_ws && \ catkin_make

source /catkin_ws/devel/setup.bash

# 执行容器的主命令
exec "$@"
