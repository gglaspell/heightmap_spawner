import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument(
        'save_path',
        default_value='/tmp',
        description='Path to temporarily save the heightmap image.'
    ))
    launch_arguments.append(DeclareLaunchArgument(
        'height',
        default_value='0.5',
        description='Height of the heightmap.'
    ))
    launch_arguments.append(DeclareLaunchArgument(
        'use_median_filtering',
        default_value='false',
        description='Use median filtering.'
    ))
    launch_arguments.append(DeclareLaunchArgument(
        'use_color_inverse',
        default_value='true',
        description='Use color inverse.'
    ))
    launch_arguments.append(DeclareLaunchArgument(
        'low_thresh',
        default_value='0',
        description='Bottom threshold.'
    ))
    launch_arguments.append(DeclareLaunchArgument(
        'high_thresh',
        default_value='100',
        description='Top threshold.'
    ))
    
    heightmap_spawner = Node(
        package='heightmap_spawner',
        executable='heightmap_spawner',
        name='heightmap_spawner',
        output='screen',
        parameters=[{
            'save_path': LaunchConfiguration('save_path'),
            'height': LaunchConfiguration('height'),
            'use_median_filtering': LaunchConfiguration('use_median_filtering'),
            'use_color_inverse': LaunchConfiguration('use_color_inverse'),
            'low_thresh': LaunchConfiguration('low_thresh'),
            'high_thresh': LaunchConfiguration('high_thresh')
        }]
    )

    nodes = [heightmap_spawner]

    return LaunchDescription(launch_arguments + nodes)
