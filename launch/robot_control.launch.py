from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    # 获取 `ur5_gripper_moveit_config` 包的路径
    ur5_gripper_moveit_config_dir = get_package_share_directory('ur5_gripper_moveit_config')

    # 包含 demo.launch.py 的路径
    demo_launch_path = os.path.join(ur5_gripper_moveit_config_dir, 'launch', 'demo.launch.py')

    # 启动 demo.launch.py
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(demo_launch_path)
        # launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 启动 `ur5_robot_gripper` 包中的 `pub_tf.py` 节点
    pub_tf_node = Node(
        package='ur5_robot_gripper',
        executable='pub_tf.py',
        name='pub_tf_node',
        output='screen'
    )
    
    # 启动 `ur5_robot_gripper` 包中的 `pose_transform_server.py` 节点
    pose_transform_server_node = Node(
        package='ur5_robot_gripper',
        executable='pose_transform_server.py',
        name='pose_transform_server_node',
        output='screen'
    )
    
    moveit_config = MoveItConfigsBuilder("ur5_gripper").to_moveit_configs()
    
    # 启动 `ur5_robot_gripper` 包中的 `robot_control_node` 节点
    robot_control_node = Node(
        package='ur5_robot_gripper',
        executable='robot_control_node',
        output='screen',
        parameters=[
            moveit_config.robot_description_kinematics,  # Load kinematics.yaml
        ],
    )
    
    # 启动 `ur5_robot_gripper` 包中的 `gripper_control_node` 节点
    gripper_control_node = Node(
        package='ur5_robot_gripper',
        executable='gripper_control_node',
        output='screen',
        parameters=[
            moveit_config.robot_description_kinematics,  # Load kinematics.yaml
        ],
    )

    # 返回 LaunchDescription，其中包含要启动的两个节点
    return LaunchDescription([
        demo_launch,
        pub_tf_node,
        pose_transform_server_node,
        robot_control_node,
        gripper_control_node
    ])