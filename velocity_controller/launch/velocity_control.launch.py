from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    offboard_control_enable_topic = LaunchConfiguration('offboard_control_enable_topic')
    velocity_cmd_topic = LaunchConfiguration('velocity_cmd_topic')
    trajectory_setpoint_topic = LaunchConfiguration('trajectory_setpoint_topic')

    control_hz = LaunchConfiguration('control_hz')
    cmd_timeout_sec = LaunchConfiguration('cmd_timeout_sec')
    max_xy_speed_mps = LaunchConfiguration('max_xy_speed_mps')
    max_z_speed_mps = LaunchConfiguration('max_z_speed_mps')
    max_yawspeed_radps = LaunchConfiguration('max_yawspeed_radps')

    return LaunchDescription([
        DeclareLaunchArgument(
            'offboard_control_enable_topic',
            default_value='/offboard/control_enable',
            description='Gate signal published by offboard_manager'),
        DeclareLaunchArgument(
            'velocity_cmd_topic',
            default_value='/input/velocity_cmd',
            description='Upstream ENU velocity command topic'),
        DeclareLaunchArgument(
            'trajectory_setpoint_topic',
            default_value='/fmu/in/trajectory_setpoint',
            description='Trajectory setpoint output to PX4'),
        DeclareLaunchArgument('control_hz', default_value='50.0'),
        DeclareLaunchArgument('cmd_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('max_xy_speed_mps', default_value='5.0'),
        DeclareLaunchArgument('max_z_speed_mps', default_value='2.0'),
        DeclareLaunchArgument('max_yawspeed_radps', default_value='1.5'),

        Node(
            package='offboard_manager',
            executable='offboard_manager_node',
            name='offboard_manager',
            output='screen',
            parameters=[{
                'topics.offboard_control_enable': offboard_control_enable_topic,
            }],
        ),

        Node(
            package='velocity_controller',
            executable='velocity_controller_node',
            name='velocity_controller',
            output='screen',
            parameters=[{
                'control_hz': control_hz,
                'cmd_timeout_sec': cmd_timeout_sec,
                'max_xy_speed_mps': max_xy_speed_mps,
                'max_z_speed_mps': max_z_speed_mps,
                'max_yawspeed_radps': max_yawspeed_radps,
                'topics.input_velocity_cmd': velocity_cmd_topic,
                'topics.offboard_control_enable': offboard_control_enable_topic,
                'topics.fmu_in_trajectory_setpoint': trajectory_setpoint_topic,
            }],
        ),
    ])
