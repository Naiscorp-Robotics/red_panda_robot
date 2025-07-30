import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Package configuration
    package_description = "redpanda_base"
    package_directory = get_package_share_directory(package_description)

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Whether to use sim time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # URDF Configuration
    urdf_file = 'redpanda_base.urdf.xacro'
    robot_desc_path = os.path.join(package_directory, "urdf", urdf_file)

    if not os.path.exists(robot_desc_path):
        raise FileNotFoundError(f"URDF/XACRO file not found: {robot_desc_path}")

    robot_description_content = Command(['xacro clear', robot_desc_path])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output="screen",
        emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher_node,
    ])
