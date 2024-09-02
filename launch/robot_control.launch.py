from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取 `ur5_gripper_moveit_config` 包的路径
    ur5_gripper_moveit_config_dir = get_package_share_directory('ur5_gripper_moveit_config')

    # 包含 demo.launch.py 的路径
    demo_launch_path = os.path.join(ur5_gripper_moveit_config_dir, 'launch', 'demo.launch.py')

    # 启动 demo.launch.py
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(demo_launch_path)
    )

    # 启动 `ur5_robot_gripper` 包中的 `pub_tf.py` 节点
    pub_tf_node = Node(
        package='ur5_robot_gripper',
        executable='pub_tf.py',
        name='pub_tf_node',
        output='screen'
    )

    # 返回 LaunchDescription，其中包含要启动的两个节点
    return LaunchDescription([
        demo_launch,
        pub_tf_node
    ])