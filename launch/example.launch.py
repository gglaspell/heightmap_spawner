# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Open RViz.'
    ))

    # Setup project paths
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_heightmap_spawner = get_package_share_directory('heightmap_spawner')

    # Load the SDF file from "description" package
    world_file = PathJoinSubstitution([pkg_heightmap_spawner, 'example', 'empty.sdf'])
    rviz_file = PathJoinSubstitution([pkg_heightmap_spawner, 'example', 'map.rviz'])
    map_file = PathJoinSubstitution([get_package_share_directory('heightmap_spawner'), 'example', 'berlin.yaml'])

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
        }],
    )

    lifecycle_nodes = ['map_server']

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        arguments=['lifecycle_manager', '--ros-args', '--log-level', 'info'],
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': lifecycle_nodes
        }],
    )

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': world_file}.items(),
    )

    heightmap_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_heightmap_spawner, 'launch', 'spawner.launch.py'])),
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', rviz_file],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    nodes = [
        gz_sim,
        map_server,
        lifecycle_manager,
        heightmap_spawner,
        rviz,
    ]

    return LaunchDescription(launch_arguments + nodes)
