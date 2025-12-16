from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab4',
            executable='lidar_avoidance',
            name='lidar_avoidance_node'
        ),
        Node(
            package='lab4',
            executable='color_tracker',
            name='color_tracker_node'
        ),
        Node(
            package='lab4',
            executable='fusion_node',
            name='fusion_node'
        ),
    ])
