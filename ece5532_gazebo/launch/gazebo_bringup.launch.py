"""
BSD 2-Clause License

Copyright (c) 2025, Micho Radovnikovich

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    verbose_mode_str = LaunchConfiguration('verbose').perform(context)
    verbose_mode = (verbose_mode_str.lower() == 'true')
    start_paused_str = LaunchConfiguration('start_paused').perform(context)
    start_paused = (start_paused_str.lower() == 'true')
    headless_str = LaunchConfiguration('headless').perform(context)
    headless = (headless_str.lower() == 'true')

    gz_arg_str = LaunchConfiguration('world_sdf_file').perform(context)

    if verbose_mode:
        gz_arg_str += ' --verbose'

    if not start_paused:
        gz_arg_str += ' -r'

    if headless:
        gz_arg_str += ' -s'

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': gz_arg_str
        }.items()
    )

    bridge_config_file = LaunchConfiguration('gz_bridge_file').perform(context)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_file,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    sdf_file = LaunchConfiguration('robot_sdf_file').perform(context)
    with open(sdf_file, 'r') as f:
        robot_desc = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    return [gz_sim, bridge, robot_state_publisher]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_sdf_file', default_value='', description='Full path to robot SDF file'),
        DeclareLaunchArgument('world_sdf_file', default_value='', description='Full path to world SDF file'),
        DeclareLaunchArgument('gz_bridge_file', default_value='', description='Full path to ROS/GZ bridge configuration YAML file'),
        DeclareLaunchArgument('verbose', default_value='false', description='Configure Gazebo to put verbose output on terminal'),
        DeclareLaunchArgument('start_paused', default_value='false', description='Start the simulation in paused state'),
        DeclareLaunchArgument('headless', default_value='false', description='Only run the simulation server without the graphical frontend'),
        OpaqueFunction(function=launch_setup)
    ])