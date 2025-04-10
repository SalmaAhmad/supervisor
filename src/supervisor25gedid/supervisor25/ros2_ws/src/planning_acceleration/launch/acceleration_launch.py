# pylint: disable=all
# mypy: ignore-errors
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    base_path = os.path.realpath(
        get_package_share_directory("planning_acceleration")
    )  # also tried without realpath
    # rviz_path = base_path + "/bicycle.rviz"
    return LaunchDescription(
        [
            Node(
                package="planning_acceleration",
                namespace="",
                executable="myscript",
                name="main",
                # parameters=[base_path+'/config/params.yaml']
            ),
            # Node(
            #     package="rviz2",
            #     namespace="",
            #     executable="rviz2",
            #     name="rviz2",
            #     output="screen",
            #     arguments=["-d" + str(rviz_path)],
            # ),
            # # Node(
            #     package='kinematic_bicycle',
            #     namespace='',
            #     executable='path_gen',
            #     name='main'
            # )
        ]
    )
