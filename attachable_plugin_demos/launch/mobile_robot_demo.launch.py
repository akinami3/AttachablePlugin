
import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    # set environment variable
    attachable_joint_plugin_path = get_package_share_directory('attachable_joint_plugin')
    attachable_joint_plugin_env = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=os.path.join(attachable_joint_plugin_path, '../', '../', '../', '../', 'build', 'attachable_joint_plugin'),
    )
    
    # get the path to the gz_sensors package
    attachable_demos_pkg = get_package_share_directory('attachable_plugin_demos')

    # set the world path
    gz_world_file_path = os.path.join(attachable_demos_pkg, 'world', 'mobile_robot_demo.world')

    # start gazebo
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', gz_world_file_path], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 '}.items()
    )
    # load altimeter model sdf
    model_file_xacro = PathJoinSubstitution([
        get_package_share_directory("attachable_plugin_demos"),
        "urdf",
        "simple_mobile_robot.urdf.xacro"
    ])
    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            model_file_xacro
        ]),
        value_type=str
    )

    # define altimeter model name
    mobile_robot_name = "mobile_robot_model"

    # altimeter model spawn node
    gazebo_model_spawner_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="gazebo_robot_model_spawner",
        output="screen",
        arguments=[
            "-name",
            mobile_robot_name,
            "-topic",
            "/robot_description",
            "-x", "0.25",
            "-y", "0.25",
            "-z", "0.2",
            "-R", "0.0",
            "-P", "0.0",
            "-Y", "0.0",
        ],
    )

    # altimeter model state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description},
        ]
    )

    bridge_params = os.path.join(
        attachable_demos_pkg,
        'config',
        'mobile_robot_bridge.yaml'
    )


    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    return LaunchDescription(
        [
            attachable_joint_plugin_env,
            gzserver_cmd,
            gzclient_cmd,
            gazebo_model_spawner_node,
            robot_state_publisher_node,
            start_gazebo_ros_bridge_cmd
        ]
    )