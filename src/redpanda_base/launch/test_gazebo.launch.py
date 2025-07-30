#!/usr/bin/env python3
"""
Test Gazebo Launch File
Simple test to check if Gazebo can load worlds
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    """
    Generate launch description for testing Gazebo
    """
    package_directory = get_package_share_directory("redpanda_base")
    
    # Use empty world
    world_file = "bookstore.sdf"
    world_file_path = os.path.join(package_directory, "worlds", world_file)
    
    # Environment variables
    gz_env = {
        'IGN_GAZEBO_RESOURCE_PATH': f"{package_directory}/worlds:{package_directory}/models:/usr/share/gz/gz-sim-6/worlds:/usr/share/gz/gz-sim-6/models",
        'IGN_GAZEBO_RENDER_ENGINE': 'ogre2',
        'GZ_SIM_SYSTEM_PLUGIN_PATH': os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ''),
        'IGN_GAZEBO_RENDER_ENGINE_PATH': os.environ.get('IGN_GAZEBO_RENDER_ENGINE_PATH', ''),
    }
    
    # Simple Gazebo launch
    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-v", "4", "--network-role", "primary", world_file_path],
        output="screen",
        additional_env=gz_env,
        shell=False,
    )
    
    ld = LaunchDescription([
        gazebo,
    ])
    
    return ld 