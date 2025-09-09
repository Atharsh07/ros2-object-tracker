from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='ros2_object_tracker',
             executable='object_tracker.py',
             output='screen'),
    ])
