from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control_actuator',
            executable='control_actuator_by_keyboard_executable',
            output='screen'),
    ])