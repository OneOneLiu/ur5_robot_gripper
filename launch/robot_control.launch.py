from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Set MoveIt configuration
    moveit_config = MoveItConfigsBuilder("ur5_gripper").to_moveit_configs()

    # Define robot_moveit node
    ur_gripper_moveit_node = Node(
        package="ur5_robot_gripper",
        executable="robot_control",
        output="screen",
        parameters=[
            moveit_config.robot_description,  # Load URDF
            moveit_config.robot_description_semantic,  # Load SRDF
            moveit_config.robot_description_kinematics,  # Load kinematics.yaml
        ],
    )

    return LaunchDescription([ur_gripper_moveit_node])