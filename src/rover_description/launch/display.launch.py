from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import xacro


def generate_launch_description():

    pkg_path = get_package_share_directory("rover_description")

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Flag to enable joint_state_publisher_gui",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Flag to launch RViz",
    )
    obstacle_arg = DeclareLaunchArgument(
        "obstacle",
        default_value="true",
        description="Flag to spawn obstacle marker and pointcloud in RViz",
    )

    gui = LaunchConfiguration("gui")
    rviz_enabled = LaunchConfiguration("rviz")
    obstacle_enabled = LaunchConfiguration("obstacle")

    xacro_file = os.path.join(
        pkg_path,
        "urdf",
        "rover_description.urdf.xacro"
    )

    robot_description = xacro.process_file(xacro_file).toxml()

    rviz_config = os.path.join(
        pkg_path,
        "rviz",
        "robot_config.rviz"
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

    joint_state_publisher_gui = Node(
        condition=IfCondition(gui),
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    joint_state_publisher = Node(
        condition=UnlessCondition(gui),
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
    )

    rviz = Node(
        condition=IfCondition(rviz_enabled),
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
        output="screen",
    )

    obstacle_node = Node(
        condition=IfCondition(obstacle_enabled),
        package="rover_description",
        executable="spawn_obstacle.py",
        name="obstacle_spawner",
        arguments=["--test-cloud"],
        output="screen",
    )

    return LaunchDescription([
        gui_arg,
        rviz_arg,
        obstacle_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        joint_state_publisher,
        obstacle_node,
        rviz,
    ])
