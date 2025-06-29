from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('urdf_package', # 包名，比如 ur5_robot_gripper
                                        description='The package where the robot description is located'))
    ld.add_action(DeclareLaunchArgument('urdf_package_path', # 包内urdf文件的路径，比如 urdf/ur5/raw_ur5.urdf
                                        description='The path to the robot description relative to the package root'))

    package_dir = FindPackageShare(LaunchConfiguration('urdf_package'))
    urdf_path = PathJoinSubstitution([package_dir, LaunchConfiguration('urdf_package_path')])

    robot_description_content = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                      }])

    ld.add_action(robot_state_publisher_node)
    return ld