from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output_topic', default_value='/input/velocity_cmd',
                              description='Final velocity command topic sent to velocity_controller'),
        DeclareLaunchArgument('aruco_topic', default_value='/aruco/velocity_cmd',
                              description='ArUco landing controller velocity input'),
        DeclareLaunchArgument('gui_topic', default_value='/gui/velocity_cmd',
                              description='GUI velocity input'),
        DeclareLaunchArgument('joy_topic', default_value='/joy/velocity_cmd',
                              description='Joystick velocity input'),
        DeclareLaunchArgument('source_cmd_topic', default_value='/control_mux/source_cmd',
                              description='Topic to switch active control source'),
        DeclareLaunchArgument('active_source_topic', default_value='/control_mux/active_source',
                              description='Topic publishing the current active source'),
        DeclareLaunchArgument('control_enable_topic', default_value='/offboard/control_enable',
                              description='Offboard control enable gate'),
        DeclareLaunchArgument('takeoff_override_topic', default_value='/offboard/takeoff_override',
                              description='Takeoff override gate'),
        DeclareLaunchArgument('flight_state_topic', default_value='/offboard/flight_state',
                              description='Flight state from offboard_manager'),
        DeclareLaunchArgument('default_source', default_value='hold',
                              description='Default control source on startup'),
        DeclareLaunchArgument('input_timeout_sec', default_value='0.5',
                              description='Seconds before a candidate input is considered stale'),
        DeclareLaunchArgument('publish_rate_hz', default_value='30.0',
                              description='Output publish rate'),

        Node(
            package='control_mux',
            executable='control_mux_node',
            name='control_mux',
            output='screen',
            parameters=[{
                'output_topic': LaunchConfiguration('output_topic'),
                'aruco_topic': LaunchConfiguration('aruco_topic'),
                'gui_topic': LaunchConfiguration('gui_topic'),
                'joy_topic': LaunchConfiguration('joy_topic'),
                'source_cmd_topic': LaunchConfiguration('source_cmd_topic'),
                'active_source_topic': LaunchConfiguration('active_source_topic'),
                'control_enable_topic': LaunchConfiguration('control_enable_topic'),
                'takeoff_override_topic': LaunchConfiguration('takeoff_override_topic'),
                'flight_state_topic': LaunchConfiguration('flight_state_topic'),
                'default_source': LaunchConfiguration('default_source'),
                'input_timeout_sec': LaunchConfiguration('input_timeout_sec'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
            }],
        ),
    ])
