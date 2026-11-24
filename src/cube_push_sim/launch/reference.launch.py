from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('cube_push_sim'))
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to plan-based reference map YAML'),
        Node(package='nav2_map_server', executable='map_server', name='map_server', namespace='reference',
             parameters=[{'yaml_filename': LaunchConfiguration('map'), 'frame_id': 'arena_reference', 'topic_name': 'map'}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager', namespace='reference',
             parameters=[{'autostart': True, 'node_names': ['map_server']}]),
        Node(package='rviz2', executable='rviz2', name='reference_rviz',
             arguments=['-d', str(share / 'config/reference.rviz')]),
    ])
