"""Saved-map navigation. Stop SLAM/teleop; reset simulation before using initial pose."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ParameterFile, ComposableNode
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    share = Path(get_package_share_directory('cube_push_sim'))
    params = ParameterFile(str(share / 'config/nav2.yaml'), allow_substs=True)
    nodes = []
    specifications = [
        ('nav2_map_server', 'map_server', 'map_server', [], {'yaml_filename': LaunchConfiguration('map')}),
        ('nav2_amcl', 'amcl', 'amcl', [], {}),
        ('nav2_controller', 'controller_server', 'controller_server', [('cmd_vel', 'cmd_vel_nav')], {}),
        ('nav2_planner', 'planner_server', 'planner_server', [], {}),
        ('nav2_smoother', 'smoother_server', 'smoother_server', [], {}),
        ('nav2_behaviors', 'behavior_server', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')], {}),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', [], {}),
        ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother', [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel_smoothed')], {}),
        ('nav2_collision_monitor', 'collision_monitor', 'collision_monitor', [], {}),
    ]
    plugins={
        'map_server':'nav2_map_server::MapServer','amcl':'nav2_amcl::AmclNode',
        'controller_server':'nav2_controller::ControllerServer','planner_server':'nav2_planner::PlannerServer',
        'smoother_server':'nav2_smoother::SmootherServer','behavior_server':'behavior_server::BehaviorServer',
        'bt_navigator':'nav2_bt_navigator::BtNavigator','velocity_smoother':'nav2_velocity_smoother::VelocitySmoother',
        'collision_monitor':'nav2_collision_monitor::CollisionMonitor'}
    components=[]
    for package, executable, name, remaps, extra in specifications:
        components.append(ComposableNode(package=package,plugin=plugins[name],name=name,
                          parameters=[params,extra],remappings=remaps,
                          extra_arguments=[{'use_intra_process_comms':False}]))
        nodes.append(Node(package=package, executable=executable, name=name,
                          parameters=[params, extra], remappings=remaps, output='screen',
                          condition=UnlessCondition(LaunchConfiguration('composition'))))
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=str(share / 'maps/arena_check_20261122_153109.yaml')),
        DeclareLaunchArgument('composition',default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        *nodes,
        ComposableNodeContainer(name='navigation_container',namespace='',package='rclcpp_components',
            executable='component_container_isolated',composable_node_descriptions=components,output='screen',
            condition=IfCondition(LaunchConfiguration('composition'))),
        Node(package='cube_push_sim', executable='velocity_mux', parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
             parameters=[{'use_sim_time': True, 'autostart': False, 'bond_timeout': 60.0,
                          'node_names': [s[2] for s in specifications]}], output='screen'),
        Node(package='cube_push_sim', executable='navigation_ready', parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(share / 'config/navigation.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
