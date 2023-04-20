from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simulation',
            executable='receive_camera',
        ),
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            parameters=[
                '/camera',
            ],
        )
    ])