
import os
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    # set environment variable
    attachable_joint_plugin_path = get_package_share_directory('attachable_joint_plugin')
    attachable_joint_plugin_env = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=os.path.join(attachable_joint_plugin_path, '../', '../', '../', '../', 'build', 'attachable_joint_plugin'),
    )

    # get the path to the gz_sensors package
    pkg_gz_sensors = get_package_share_directory('attachable_plugin_demos')

    # set the world path
    gz_world_file_path = os.path.join(pkg_gz_sensors, 'world', 'attachable_joint_demo.world')

    # start gazebo
    gazebo_share_path = FindPackageShare("ros_gz_sim")
    gazebo_launch = IncludeLaunchDescription(
        launch_description_source=PathJoinSubstitution(
            [gazebo_share_path, "launch", "gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": [
                "-r ",
                PathJoinSubstitution(["worlds", gz_world_file_path]),
            ]
        }.items(),
    )
    return LaunchDescription(
        [
            attachable_joint_plugin_env,
            gazebo_launch,
        ]
    )