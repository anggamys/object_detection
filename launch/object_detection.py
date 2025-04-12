from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_detection',
            executable='camera_node',
            output='screen',
            parameters=[{'source': '0'}],
        ),
        Node(
            package='object_detection',
            executable='detection_node',
            output='screen',
        ),
    ])