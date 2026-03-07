from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    lidar_bridge = Node(
        package='unity_lidar_bridge',
        executable='lidar_bridge_node',
        name='lidar_bridge_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'host': '127.0.0.1',
            'port': 9090,
            'frame_id': 'lidar_frame',
        }],
    )

    return LaunchDescription([
        lidar_bridge,
    ])
