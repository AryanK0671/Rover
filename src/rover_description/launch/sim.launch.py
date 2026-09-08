from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import xacro


def generate_launch_description():

    pkg_share = get_package_share_directory("rover_description")

    xacro_file = os.path.join(
        pkg_share,
        "urdf",
        "rover_description.urdf.xacro"
    )

    robot_description = xacro.process_file(xacro_file).toxml()

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True
            }
        ]
    )

    world_file = os.path.join(pkg_share, "worlds", "empty.sdf")
    # Start Ignition Gazebo
    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", world_file],
        output="screen"
        )

    # Spawn robot
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "rover",
            "-x", "0",
            "-y", "0",
            "-z", "0.2",
            "-R","-1.5708",
            # "P","1.57"
            # "Y","1.57"
        ],
        output="screen"
    )



    bridge_yaml = os.path.join(
        pkg_share,
        'config',
        'bridge.yaml'

    )
    # 1. Spawner for the Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
)

    # 2. Spawner for the Rover Controller
    rover_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rover_base_controller"],
)

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',

        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_yaml}'
        ],

        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        joint_state_broadcaster_spawner,
        rover_controller_spawner,
        bridge
    ])
