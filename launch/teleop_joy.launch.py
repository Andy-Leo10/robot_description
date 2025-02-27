# https://github.com/ros2/teleop_twist_joy/tree/humble

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    snes_gamepad_node = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'joy_dev': '0'}]
        )
    
    teleop_node = Node(
            package='robot_description',
            executable='teleop_joy',
            name='teleop_node',
            output='screen'
        )
    
    return LaunchDescription([
        snes_gamepad_node,
        teleop_node
    ])