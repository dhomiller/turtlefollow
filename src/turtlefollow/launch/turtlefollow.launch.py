from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlefollow',
            executable='follow',
            name='turtlecontroller'
        ),
        Node(
            package='turtlefollow',
            executable='modeswitch',
            name='switcher'
        )
    ])
    ld.add_action(
        ExecuteProcess(
            cmd=[
                [
                    FindExecutable(name="ros2"),
                    " service call ",
                    "/spawn ",
                    "turtlesim/srv/Spawn ",
                    "\"{x: 2.0, y: 4.0, theta: 2, name: 'turtle2'}\"",
                ]
            ],
            shell=True
        )
    )
    return ld
