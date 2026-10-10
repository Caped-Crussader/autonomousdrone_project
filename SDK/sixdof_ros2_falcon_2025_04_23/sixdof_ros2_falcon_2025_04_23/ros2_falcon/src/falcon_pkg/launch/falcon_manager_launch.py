import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='falcon_pkg',
            executable='falcon_manager_node',
            name='falcon_manager_node',
            output='screen'
        )
    ])