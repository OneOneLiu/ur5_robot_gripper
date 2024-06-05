FROM osrf/ros:noetic-desktop-full
# Set the environment variable to non-interactive, this is to avoid the tzdata configuration during the installation of the packages
ENV DEBIAN_FRONTEND=noninteractive

# System layer installation
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y wget && \
    apt-get install -y iputils-ping && \
    apt-get -y install build-essential && \
    apt install -y cmake && \
    apt install -y tree && \
    apt install -y gdb && \
    apt-get install -y git && \
    apt-get install -y python3-pip && \
    apt-get install -y xauth x11-apps # This is for GUI testing

# Dependencies layer # Ur_driver
RUN mkdir -p /catkin_ws/src && \
    cd /catkin_ws/src && \
    git clone -b noetic-devel https://github.com/ros-industrial/universal_robot.git && \
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git && \
    apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y ros-${ROS_DISTRO}-moveit-planners-chomp && \
    apt-get install -y ros-${ROS_DISTRO}-pilz-industrial-motion-planner && \
    apt-get install -y ros-${ROS_DISTRO}-moveit-ros-control-interface && \
    apt-get install -y ros-${ROS_DISTRO}-moveit-commander

# Robotiq gripper (gripper model, gazebo ros control plugin and usb communication interfaces)
RUN mkdir -p /catkin_ws/src && \
    cd /catkin_ws/src && \
    git clone https://github.com/filesmuggler/robotiq.git && \
    cp -r robotiq/robotiq_description/  .&& \
    rm -rf robotiq && \
    git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins && \
    git clone https://github.com/TAMS-Group/robotiq.git && \
    apt-get install -y ros-noetic-soem && \
    apt-get install ros-noetic-socketcan-interface && \
    apt install ros-noetic-rqt-controller-manager && \
    apt-get install -y python3-tk && \
    apt-get install -y ros-noetic-ros-control ros-noetic-ros-controllers

# Clone my custimized repositories
RUN cd /catkin_ws/src && \
    git clone https://github.com/OneOneLiu/robotiq_gripper.git && \
    git clone https://github.com/OneOneLiu/ur5_robot.git && \
    git clone https://github.com/OneOneLiu/ur5_gripper_moveit.git

# Sync the cloned repositories every time the container is started
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# Fix issues when using remote server from ssh **Start**
## Fix network setup
ENV ROS_HOSTNAME=localhost
ENV ROS_MASTER_URI=http://localhost:11311
# For correct configure QT library on the remote server
RUN strip --remove-section=.note.ABI-tag /usr/lib/x86_64-linux-gnu/libQt5Core.so.5
# Fix issues when using remote server from ssh End** 

# Use rosdep to install all dependencies according to the package.xml files in all src folders
# And compile the workspace
RUN cd /catkin_ws && \
    rosdep install --from-paths src --ignore-src -r -y && \
    /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'

# Automatically source
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc


RUN echo "ALL Done"