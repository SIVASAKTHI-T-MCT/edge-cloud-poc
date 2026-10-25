from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='handoff_edge', executable='arm_node', name='arm_r1',
             parameters=[{'robot_id': 'r1', 'base_x': -0.6}]),
        Node(package='handoff_edge', executable='arm_node', name='arm_r2',
             parameters=[{'robot_id': 'r2', 'base_x': 0.6}]),
        Node(package='handoff_edge', executable='zone_monitor', name='zone_monitor'),
    ])
