from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # drone 1: moves 1.0 m per second
        Node(
            package='slam_demo',
            executable='my_publisher',
            namespace='drone1',
            parameters=[{'step_size': 1.0}],
        ),
        Node(
            package='slam_demo',
            executable='my_subscriber',
            namespace='drone1',
        ),

        # drone 2: same code, different namespace and a different setting
        Node(
            package='slam_demo',
            executable='my_publisher',
            namespace='drone2',
            parameters=[{'step_size': 3.0}],
        ),
        Node(
            package='slam_demo',
            executable='my_subscriber',
            namespace='drone2',
        ),
    ])