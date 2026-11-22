from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(package='wbr_planner_node', executable='planner_node',
                                   name='wbr_planner_node', output='screen')])
