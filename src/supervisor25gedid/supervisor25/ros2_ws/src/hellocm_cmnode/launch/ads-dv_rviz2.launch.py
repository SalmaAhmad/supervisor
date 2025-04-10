import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

  # Declare arguments to be used from command line
  config_file_arg = DeclareLaunchArgument(
    'rviz2_config_file',
    default_value=os.path.join(
                    get_package_share_directory('hellocm_cmnode'),
                    get_package_share_directory('path_csv'),
                    get_package_share_directory('ipg_bridge_ros2'),
                    'launch/ads-dv_rviz2.rviz'),
    description='Rviz2 configuration file')

  # Launch the rviz2 node
  rviz2_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', LaunchConfiguration('rviz2_config_file')],
  )

  path_node = Node(
    package='path_csv',
    executable='path_csv',
    name='path_bridge'
  )

  ackermann_node = Node(
    package='ipg_bridge_ros2',
    executable='IPG_CONTROL_BRIDGE',
    name='control_bridge'
  )

  return LaunchDescription([
    config_file_arg,
    rviz2_node,
  ])