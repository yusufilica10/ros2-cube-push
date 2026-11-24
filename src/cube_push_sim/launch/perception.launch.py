from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('viewer', default_value='false'),
        Node(package='cube_push_sim', executable='cube_detector',
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='rqt_image_view', executable='rqt_image_view',
             arguments=['/perception/debug_image'], condition=IfCondition(LaunchConfiguration('viewer'))),
    ])
