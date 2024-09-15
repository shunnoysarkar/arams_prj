import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    apriltag_ros_share_dir = get_package_share_directory('apriltag_ros')
    
    tags_16h5_yaml_file = os.path.join(apriltag_ros_share_dir, 'cfg', 'tags_16h5.yaml')
    
    #Create the launch description
    launch_description = LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            parameters=[{
                'family': '16h5',
                'size': 0.173,
                'image_transport': 'raw'
            }],
            remappings=[('/image_rect', '/camera/image_raw'),
                        ('camera_info', '/camera/camera_info')],
        ),
        Node(
            package='apriltag_navigation',
            executable='apriltag_navigation_node',
            output='screen',
        )
    ])
    
    return launch_description
