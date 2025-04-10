from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_controller',  
            executable='hazem_node',  
            name='hazem_node',  
            output='screen'  
        )
    ])
