#!/usr/bin/env python3
"""
Display Launch File for RedPanda Robot
This file launches the robot_state_publisher and Gazebo simulation
"""

import os
from ament_index_python.packages import get_package_share_directory #this function is used to retreive the full path to share/ directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """
    Generate launch description for RedPanda robot display
    """
    # ==========================================
    # Package and file paths
    # ==========================================
    package_name = "redpanda_base"
    package_directory = get_package_share_directory(package_name)
    
    # URDF file path
    urdf_file = "redpanda_base.urdf.xacro"
    robot_desc_path = os.path.join(package_directory, "urdf", urdf_file)
    
    # World file path
    world_file = "empty.sdf"
    world_file_path = os.path.join(package_directory, "worlds", world_file)
    
    # ==========================================
    # Launch Arguments
    # ==========================================
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Whether to use simulation time'
    )
    
    # Spawn Position Arguments
    declare_spawn_x = DeclareLaunchArgument(
        "x", 
        default_value="-2.0",
        description="Model Spawn X Axis Value"
    )
    
    declare_spawn_y = DeclareLaunchArgument(
        "y", 
        default_value="0.0",
        description="Model Spawn Y Axis Value"
    )
    
    declare_spawn_z = DeclareLaunchArgument(
        "z", 
        default_value="0.5",
        description="Model Spawn Z Axis Value"
    )
    
    # Gazebo Headless Mode
    declare_headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Whether to run Gazebo in headless mode"
    )
    
    # ==========================================
    # Launch Configurations
    # ==========================================
    use_sim_time = LaunchConfiguration('use_sim_time')
    run_headless = LaunchConfiguration("headless")
    
    # ==========================================
    # URDF Processing
    # ==========================================
    # Check if URDF file exists
    if not os.path.exists(robot_desc_path):
        raise FileNotFoundError(f"URDF/XACRO file not found: {robot_desc_path}")
    
    print(f"Loading URDF/XACRO from: {robot_desc_path}")
    
    # Process URDF with xacro
    robot_description_content = Command(['xacro ', robot_desc_path])
    robot_description = ParameterValue(robot_description_content, value_type=str)
    
    # Check if world file exists
    if not os.path.exists(world_file_path):
        raise FileNotFoundError(f"World file not found: {world_file_path}")
    print(f"Loading world file from: {world_file_path}")
    
    # ==========================================
    # Environment variables for Gazebo
    # ==========================================
    gz_verbosity = "4"
    gz_models_path = ":".join([
        package_directory,
        os.path.join(package_directory, "models"),
        os.path.join(package_directory, "worlds"),
        os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    ])
    gz_env = {
        'GZ_SIM_SYSTEM_PLUGIN_PATH':
            ':'.join([os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
                    os.environ.get('LD_LIBRARY_PATH', default='')]),
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':
            ':'.join([os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                    os.environ.get('LD_LIBRARY_PATH', default='')]),
        'IGN_GAZEBO_RESOURCE_PATH': gz_models_path,
        'IGN_GAZEBO_RENDER_ENGINE': 'ogre2',
        'IGN_GAZEBO_RENDER_ENGINE_PATH': os.environ.get('IGN_GAZEBO_RENDER_ENGINE_PATH', '')
    }
    
    # ==========================================
    # Nodes
    # ==========================================
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # Joint State Publisher Node (for visualization)
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # Robot Spawn Node
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="my_robot_spawn",
        arguments=[
            "-name", "my_robot",
            "-allow_renaming", "true",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
        output="screen",
    )
    
    # ==========================================
    # Gazebo Processes
    # ==========================================
    # Gazebo Launch Configuration (Headless Mode)
    gazebo_headless = ExecuteProcess(
        condition=IfCondition(run_headless),
        cmd=["ign", "gazebo", "-r", "-v", gz_verbosity, "-s", "--headless-rendering", "--network-role", "primary", world_file_path],
        output="screen",
        additional_env=gz_env,
        shell=False,
    )
    
    # Gazebo Launch Configuration (Not Headless Mode)
    gazebo = ExecuteProcess(
        condition=UnlessCondition(run_headless),
        cmd=["ign", "gazebo", "-r", "-v", gz_verbosity, "--network-role", "primary", world_file_path],
        output="screen",
        additional_env=gz_env,
        shell=False,
    )
    
    # ==========================================
    # Launch Description
    # ==========================================
    ld = LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        declare_headless_arg,
        
        # Nodes
        robot_state_publisher_node,
        joint_state_publisher_node,
        gz_spawn_entity,
        
        # Gazebo processes
        gazebo_headless,
        gazebo,
    ])
    
    return ld