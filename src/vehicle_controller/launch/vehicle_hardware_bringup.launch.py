"""Real-hardware bringup: ODrive HWI -> HSH -> vehicle controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('vehicle_controller')
    robot_description_path = os.path.join(share, 'urdf', 'vehicle_hardware.urdf')
    # Default is unchanged (bench profile). Lane following passes its own
    # profile with require_function_command: true.
    controller_config_arg = DeclareLaunchArgument(
        'controller_config', default_value=os.path.join(share, 'config', 'vehicle_controller.yaml'))
    controller_config = LaunchConfiguration('controller_config')
    with open(robot_description_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        parameters=[{'robot_description': robot_description}, controller_config],
        output='screen',
    )
    hsh = Node(
        package='hardware_system_handler',
        executable='hsh_node',
        name='hsh_node',
        parameters=[{'bypass_position_homing': True}],
        output='screen',
    )
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-type',
                   'joint_state_broadcaster/JointStateBroadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )
    vehicle_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['vehicle_controller', '--controller-type',
                   'vehicle_controller/VehicleController',
                   '--param-file', controller_config,
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # HWI creates the shared-memory segments during initialization. Starting
    # HSH after it prevents a startup race; spawners wait for manager services.
    start_dependents = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[
                TimerAction(period=1.0, actions=[joint_state_broadcaster, vehicle_controller]),
                TimerAction(period=2.0, actions=[hsh]),
            ],
        )
    )
    return LaunchDescription([controller_config_arg, controller_manager, start_dependents])
