"""Live preview from a camera device or an existing ROS image topic.

  ros2 launch lane_following camera_preview.launch.py source:=ros topic:=/camera/image_raw
  ros2 launch lane_following camera_preview.launch.py source:=camera device:=0

Still PREVIEW only: publishes /lane_following/debug/motion_cmd, never the
vehicle command or function topics.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('lane_following')
    default_cfg = os.path.join(share, 'config', 'lane_following.yaml')
    args = [
        DeclareLaunchArgument('config', default_value=default_cfg),
        DeclareLaunchArgument('source', default_value='ros', description='ros | camera'),
        DeclareLaunchArgument('topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('device', default_value='0'),
        DeclareLaunchArgument('check_stamp_age', default_value='false'),
        DeclareLaunchArgument('image_reliability', default_value='best_effort'),
        DeclareLaunchArgument('viewer', default_value='false'),
        DeclareLaunchArgument('viewer_mode', default_value='--web', description='--web | --opencv'),
        DeclareLaunchArgument('viewer_port', default_value='8080'),
    ]
    node = Node(
        package='lane_following', executable='lane_following_node', name='lane_following_node',
        output='screen',
        parameters=[LaunchConfiguration('config'), {
            'input.mode': ParameterValue(LaunchConfiguration('source'), value_type=str),
            'input.topic': ParameterValue(LaunchConfiguration('topic'), value_type=str),
            'input.path': ParameterValue(LaunchConfiguration('device'), value_type=str),
            'input.image_reliability': ParameterValue(LaunchConfiguration('image_reliability'), value_type=str),
            'input.check_stamp_age': ParameterValue(LaunchConfiguration('check_stamp_age'), value_type=bool),
        }])
    viewer = Node(
        package='lane_following', executable='lane_viewer', name='lane_viewer', output='screen',
        arguments=[LaunchConfiguration('viewer_mode'), '--port', LaunchConfiguration('viewer_port')],
        condition=IfCondition(LaunchConfiguration('viewer')))
    return LaunchDescription(args + [node, viewer])
