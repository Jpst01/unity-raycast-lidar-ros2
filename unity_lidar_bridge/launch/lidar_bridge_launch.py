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

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_lidar',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'lidar_frame',
        ],
        output='screen',
    )

    return LaunchDescription([
        static_tf,
        lidar_bridge,
    ])
