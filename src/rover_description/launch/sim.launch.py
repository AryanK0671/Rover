import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rover_description")

    # Launch arguments
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Gazebo in headless mode (server only, no GUI)",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Launch RViz for visualization",
    )
    cmd_vel_arg = DeclareLaunchArgument(
        "cmd_vel",
        default_value="true",
        description="Launch /cmd_vel Twist-to-wheel velocity bridge node",
    )
    points_arg = DeclareLaunchArgument(
        "points",
        default_value="true",
        description="Launch scan_to_points PointCloud2 converter node",
    )
    obstacle_arg = DeclareLaunchArgument(
        "obstacle",
        default_value="true",
        description="Launch obstacle marker publisher for RViz visualization",
    )

    headless = LaunchConfiguration("headless")
    rviz = LaunchConfiguration("rviz")
    cmd_vel = LaunchConfiguration("cmd_vel")
    points = LaunchConfiguration("points")
    obstacle = LaunchConfiguration("obstacle")

    # Set Gazebo resource paths dynamically so meshes are found without manual ~/.bashrc edits
    parent_share = os.path.abspath(os.path.join(pkg_share, ".."))
    current_ign_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    current_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")

    ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=f"{parent_share}:{pkg_share}:{current_ign_path}".rstrip(":"),
    )
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=f"{parent_share}:{pkg_share}:{current_gz_path}".rstrip(":"),
    )

    # Robot Description from Xacro
    xacro_file = os.path.join(pkg_share, "urdf", "rover_description.urdf.xacro")
    robot_description = xacro.process_file(xacro_file).toxml()

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
    )

    # World file
    world_file = os.path.join(pkg_share, "worlds", "empty.sdf")

    # Gazebo with GUI
    gazebo_gui = ExecuteProcess(
        condition=UnlessCondition(headless),
        cmd=["ign", "gazebo", "-r", world_file],
        output="screen",
    )

    # Gazebo Headless (Server only)
    gazebo_headless = ExecuteProcess(
        condition=IfCondition(headless),
        cmd=["ign", "gazebo", "-s", "-r", world_file],
        output="screen",
    )

    # Spawn robot:
    # Root frame is base_link (REP-103: +X forward towards obstacles, +Z up, +Y left)
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "rover",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.20",
            "-R", "0.0",
            "-P", "0.0",
            "-Y", "0.0",
        ],
        output="screen",
    )

    # Bridge between Ignition Gazebo and ROS 2
    bridge_yaml = os.path.join(pkg_share, "config", "bridge.yaml")
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_yaml}",
        ],
        output="screen",
    )

    # Controller spawners (sequenced after robot is spawned to prevent race conditions)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    rover_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rover_base_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Event handlers to sequence controller startup after spawn
    spawn_to_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    jsb_to_rover_ctrl = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rover_controller_spawner],
        )
    )

    # Skid-steer /cmd_vel Twist bridge node
    cmd_vel_node = Node(
        condition=IfCondition(cmd_vel),
        package="rover_description",
        executable="cmd_vel_to_rover.py",
        name="cmd_vel_to_rover",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # LaserScan to PointCloud2 converter node
    scan_to_points_node = Node(
        condition=IfCondition(points),
        package="rover_description",
        executable="scan_to_points.py",
        name="scan_to_points",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # Obstacle visualizer node (publishes /obstacle_marker for RViz)
    obstacle_node = Node(
        condition=IfCondition(obstacle),
        package="rover_description",
        executable="spawn_obstacle.py",
        name="obstacle_spawner",
        arguments=["--marker-only"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # RViz (optional)
    rviz_config = os.path.join(pkg_share, "rviz", "robot_config.rviz")
    rviz_node = Node(
        condition=IfCondition(rviz),
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([
        headless_arg,
        rviz_arg,
        cmd_vel_arg,
        points_arg,
        obstacle_arg,
        ign_resource_path,
        gz_resource_path,
        gazebo_gui,
        gazebo_headless,
        robot_state_publisher,
        spawn_robot,
        bridge,
        spawn_to_jsb,
        jsb_to_rover_ctrl,
        cmd_vel_node,
        scan_to_points_node,
        obstacle_node,
        rviz_node,
    ])
