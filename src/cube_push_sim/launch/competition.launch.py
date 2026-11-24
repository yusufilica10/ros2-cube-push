"""All project components; mission remains armed until /mission/start is called."""
from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    root=Path(get_package_share_directory('cube_push_sim'))
    def include(name,args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root/'launch'/name)),launch_arguments=args.items())
    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', os.environ.get('FASTDDS_BUILTIN_TRANSPORTS','UDPv4')),
        SetEnvironmentVariable('RMW_FASTRTPS_PUBLICATION_MODE','ASYNCHRONOUS'),
        SetEnvironmentVariable('OPENBLAS_NUM_THREADS','1'),
        SetEnvironmentVariable('OMP_NUM_THREADS','1'),
        DeclareLaunchArgument('domain_id',default_value='47'),
        SetEnvironmentVariable('ROS_DOMAIN_ID',LaunchConfiguration('domain_id')),
        DeclareLaunchArgument('headless',default_value='false'),
        DeclareLaunchArgument('rviz',default_value='true'),
        include('sim.launch.py',{'headless':LaunchConfiguration('headless')}),
        include('navigation.launch.py',{'rviz':LaunchConfiguration('rviz')}),
        include('perception.launch.py',{}),
        Node(package='cube_push_sim',executable='mission',parameters=[{'use_sim_time':True}],output='screen'),
    ])
