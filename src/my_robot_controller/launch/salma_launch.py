from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_controller',  
            executable='salma_node',  
            name='salma_node',  
            output='screen'  
        )
    ])
