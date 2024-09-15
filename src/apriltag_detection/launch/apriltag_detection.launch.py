from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_detector',
            output='screen',
            parameters=[{
                'family': '16h5',
                'size': 0.173,
                'image_transport': 'raw'
            }],
            remappings=[('/image_rect', '/camera/image_raw')],
        ),
    ])
