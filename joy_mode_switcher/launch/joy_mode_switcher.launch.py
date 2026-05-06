"""
Launch joy_node + joy_mode_switcher_node together.

Usage:
  ros2 launch joy_mode_switcher joy_mode_switcher.launch.py

With custom joy device:
  ros2 launch joy_mode_switcher joy_mode_switcher.launch.py joy_dev:=/dev/input/js1

With custom config:
  ros2 launch joy_mode_switcher joy_mode_switcher.launch.py params_file:=/path/to/custom.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── joy_node arguments ──
    joy_dev = LaunchConfiguration('joy_dev')
    joy_deadzone = LaunchConfiguration('joy_deadzone')
    joy_autorepeat_rate = LaunchConfiguration('joy_autorepeat_rate')
    joy_topic = LaunchConfiguration('joy_topic')

    # ── joy_mode_switcher arguments ──
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device path'),
        DeclareLaunchArgument(
            'joy_deadzone',
            default_value='0.05',
            description='Joystick deadzone (joy_node)'),
        DeclareLaunchArgument(
            'joy_autorepeat_rate',
            default_value='0.0',
            description='Autorepeat rate for joy buttons (0.0 = disabled)'),
        DeclareLaunchArgument(
            'joy_topic',
            default_value='/joy',
            description='Joy topic name'),

        DeclareLaunchArgument(
            'params_file',
            default_value=[
                FindPackageShare('joy_mode_switcher'),
                '/config/joy_mode_switcher.yaml',
            ],
            description='Path to joy_mode_switcher YAML config'),

        # ── joy_node ──
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': joy_dev,
                'deadzone': joy_deadzone,
                'autorepeat_rate': joy_autorepeat_rate,
            }],
            remappings=[
                ('/joy', joy_topic),
            ],
        ),

        # ── joy_mode_switcher_node ──
        Node(
            package='joy_mode_switcher',
            executable='joy_mode_switcher_node',
            name='joy_mode_switcher',
            output='screen',
            parameters=[params_file],
        ),
    ])
