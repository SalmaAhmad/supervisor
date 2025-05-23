import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Path to the YAML file containing the parameters
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    param_dir = os.path.join(get_package_share_directory('supervisor'), 'config', 'supervisor_params.yaml')

    supervisor_node = launch_ros.actions.Node(
        package='supervisor',
        executable='supervisor_node',
        parameters=[param_dir],
        output='screen'
    )

    mission_launcher_node = launch_ros.actions.Node(
        package='supervisor',
        executable='missionLauncher',
        output='screen'
    )
    
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Full path to the parameter file to load'),
        supervisor_node,
        mission_launcher_node
        

    ])

if __name__ == '__main__':
    generate_launch_description()
