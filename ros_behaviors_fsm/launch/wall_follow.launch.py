from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_behaviors_fsm',
            namespace='ros_behaviors_fsm',
            executable='wall_follow',
            name='neato_fsm'
        ),
        Node(
            package='ros_behaviors_fsm',
            namespace='ros_behaviors_fsm',
            executable='simple_teleop',
            name='simple_teleop'
        )
    ])