from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import xacro


def generate_launch_description():

    pkg_path = get_package_share_directory("rover_description")

    xacro_file = os.path.join(
        pkg_path,
        "urdf",
        "rover_description.urdf.xacro"
    )

    robot_description = xacro.process_file(xacro_file).toxml()

    rviz_config = os.path.join(
        pkg_path,
        "config",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": False
            }
        ],
        output="screen",
    )

    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
        output="screen",
    )

    return LaunchDescription([
        joint_state_publisher,
        robot_state_publisher,
        rviz,
    ])
