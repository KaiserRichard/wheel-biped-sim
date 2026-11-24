from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(
        package='wbr_obstacle_node', executable='obstacle_node', name='wbr_obstacle_node',
        output='screen', parameters=[{
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'info_topic': '/camera/camera/color/camera_info',
        }],
    )])
