from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('cube_push_sim'))
    gz_share = Path(get_package_share_directory('ros_gz_sim'))
    headless = LaunchConfiguration('headless')
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gz_share / 'launch/gz_sim.launch.py')),
            launch_arguments={'gz_args': [PythonExpression(["'-s --headless-rendering' if '", headless, "' == 'true' else ''"]), ' -r ', str(share / 'worlds/competition.sdf')]}.items()),
        Node(package='cube_push_sim', executable='clock_throttle', output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'use_sim_time': True, 'robot_description': (share / 'urdf/robot.urdf').read_text()}]),
        Node(package='ros_gz_sim', executable='create', arguments=[
            '-world', 'competition', '-name', 'cube_pusher', '-file', str(share / 'urdf/robot.urdf'),
            '-x', '-2.4', '-y', '-1.4', '-z', '0.01'], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': str(share / 'config/bridge.yaml')}], output='screen'),
    ])
