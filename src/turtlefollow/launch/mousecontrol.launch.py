from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription([
        Node(
            package='turtlefollow',
            executable='mousecontrol',
            name='mousecontroller'
        )
    ])
    return ld
