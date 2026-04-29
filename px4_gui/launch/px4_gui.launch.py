from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_gui',
            executable='px4_gui_node',
            name='px4_gui',
            output='screen',
            parameters=[{
                'max_vx': 2.0,
                'max_vy': 2.0,
                'max_vz': 1.0,
                'max_yaw': 1.2,
            }],
        ),
    ])
