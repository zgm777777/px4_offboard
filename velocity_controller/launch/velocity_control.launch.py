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

    # control_mux parameters
    mux_default_source = LaunchConfiguration('mux_default_source')
    mux_input_timeout_sec = LaunchConfiguration('mux_input_timeout_sec')
    mux_publish_rate_hz = LaunchConfiguration('mux_publish_rate_hz')

    # offboard_manager takeoff parameters
    takeoff_timeout_sec = LaunchConfiguration('takeoff_timeout_sec')
    takeoff_reached_tolerance_m = LaunchConfiguration('takeoff_reached_tolerance_m')
    allow_ground_control_while_landed = LaunchConfiguration('allow_ground_control_while_landed')

    return LaunchDescription([
        # ── velocity_controller args ──
        DeclareLaunchArgument(
            'offboard_control_enable_topic',
            default_value='/offboard/control_enable',
            description='Gate signal published by offboard_manager'),
        DeclareLaunchArgument(
            'velocity_cmd_topic',
            default_value='/input/velocity_cmd',
            description='Upstream ENU velocity command topic (published by control_mux)'),
        DeclareLaunchArgument(
            'trajectory_setpoint_topic',
            default_value='/fmu/in/trajectory_setpoint',
            description='Trajectory setpoint output to PX4'),
        DeclareLaunchArgument('control_hz', default_value='50.0'),
        DeclareLaunchArgument('cmd_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('max_xy_speed_mps', default_value='5.0'),
        DeclareLaunchArgument('max_z_speed_mps', default_value='2.0'),
        DeclareLaunchArgument('max_yawspeed_radps', default_value='1.5'),

        # ── control_mux args ──
        DeclareLaunchArgument(
            'mux_default_source',
            default_value='hold',
            description='Default control source: hold / gui / joy / aruco'),
        DeclareLaunchArgument(
            'mux_input_timeout_sec',
            default_value='0.5',
            description='Seconds before a candidate input is considered stale'),
        DeclareLaunchArgument(
            'mux_publish_rate_hz',
            default_value='30.0',
            description='control_mux output publish rate'),

        # ── offboard_manager takeoff args ──
        DeclareLaunchArgument(
            'takeoff_timeout_sec',
            default_value='25.0',
            description='Max seconds for takeoff override before timeout'),
        DeclareLaunchArgument(
            'takeoff_reached_tolerance_m',
            default_value='0.5',
            description='Position tolerance to consider takeoff target reached'),
        DeclareLaunchArgument(
            'allow_ground_control_while_landed',
            default_value='false',
            description='Allow velocity control while landed (true for sim testing, false for safe ops)'),

        # ── Nodes ──
        Node(
            package='offboard_manager',
            executable='offboard_manager_node',
            name='offboard_manager',
            output='screen',
            parameters=[{
                'topics.offboard_control_enable': offboard_control_enable_topic,
                'takeoff_timeout_sec': takeoff_timeout_sec,
                'takeoff_reached_tolerance_m': takeoff_reached_tolerance_m,
                'allow_ground_control_while_landed': allow_ground_control_while_landed,
            }],
        ),

        Node(
            package='control_mux',
            executable='control_mux_node',
            name='control_mux',
            output='screen',
            parameters=[{
                'output_topic': velocity_cmd_topic,
                'aruco_topic': '/aruco/velocity_cmd',
                'gui_topic': '/gui/velocity_cmd',
                'joy_topic': '/joy/velocity_cmd',
                'source_cmd_topic': '/control_mux/source_cmd',
                'active_source_topic': '/control_mux/active_source',
                'control_enable_topic': offboard_control_enable_topic,
                'takeoff_override_topic': '/offboard/takeoff_override',
                'flight_state_topic': '/offboard/flight_state',
                'default_source': mux_default_source,
                'input_timeout_sec': mux_input_timeout_sec,
                'publish_rate_hz': mux_publish_rate_hz,
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
