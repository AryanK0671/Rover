import math
import os
import xml.etree.ElementTree as ET
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray

import sys

scripts_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
if scripts_dir not in sys.path:
    sys.path.insert(0, scripts_dir)

from spawn_obstacle import (  # noqa: E402
    generate_sdf,
    ObstacleSpawnerNode,
    parse_args,
    COLOR_MAP,
)
from scan_to_points import ScanToPoints  # noqa: E402


@pytest.fixture(scope="module")
def rclpy_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_empty_sdf_contains_obstacles():
    """Verify empty.sdf has box_obstacle and cylinder_obstacle intersecting LiDAR plane."""
    pkg_share = get_package_share_directory("rover_description")
    sdf_file = os.path.join(pkg_share, "worlds", "empty.sdf")
    assert os.path.exists(sdf_file), f"World file missing: {sdf_file}"

    tree = ET.parse(sdf_file)
    root = tree.getroot()

    models = {m.attrib.get("name"): m for m in root.findall(".//model")}
    assert "box_obstacle" in models, "box_obstacle must be present in empty.sdf"
    assert "cylinder_obstacle" in models, "cylinder_obstacle must be present in empty.sdf"

    # Verify box_obstacle pose and height intersects LiDAR scan plane (~0.785m)
    box = models["box_obstacle"]
    pose_str = box.find("pose").text.strip()
    x, y, z, r, p, yaw = [float(v) for v in pose_str.split()]
    box_size = [float(v) for v in box.find(".//geometry/box/size").text.strip().split()]

    assert x > 0.5, "Obstacle must be in front of the rover (+X)"
    box_z_min = z - box_size[2] / 2.0
    box_z_max = z + box_size[2] / 2.0
    assert box_z_min <= 0.785 <= box_z_max, (
        f"box_obstacle [{box_z_min}, {box_z_max}] does not intersect LiDAR plane z=0.785"
    )

    # Verify cylinder_obstacle intersects LiDAR plane
    cyl = models["cylinder_obstacle"]
    cyl_pose = [float(v) for v in cyl.find("pose").text.strip().split()]
    cyl_len = float(cyl.find(".//geometry/cylinder/length").text.strip())
    cyl_z_min = cyl_pose[2] - cyl_len / 2.0
    cyl_z_max = cyl_pose[2] + cyl_len / 2.0
    assert cyl_z_min <= 0.785 <= cyl_z_max, (
        f"cylinder_obstacle [{cyl_z_min}, {cyl_z_max}] does not intersect LiDAR plane"
    )


def test_generate_sdf_box_and_cylinder():
    """Verify SDF generator outputs valid XML with matching geometry."""
    sdf_box = generate_sdf("test_box", "box", 1.0, 1.0, 1.0, COLOR_MAP["orange"])
    root_box = ET.fromstring(sdf_box)
    assert root_box.find(".//model").attrib["name"] == "test_box"
    assert root_box.find(".//geometry/box/size").text.strip() == "1.000 1.000 1.000"

    sdf_cyl = generate_sdf("test_cyl", "cylinder", 0.8, 0.8, 1.5, COLOR_MAP["cyan"])
    root_cyl = ET.fromstring(sdf_cyl)
    assert root_cyl.find(".//model").attrib["name"] == "test_cyl"
    assert root_cyl.find(".//geometry/cylinder/radius").text.strip() == "0.400"
    assert root_cyl.find(".//geometry/cylinder/length").text.strip() == "1.500"


def test_obstacle_spawner_markers(rclpy_init):
    """Verify ObstacleSpawnerNode produces valid Marker messages."""
    args = parse_args(["--x", "3.0", "--y", "0.0", "--z", "0.3", "--sx", "1.0", "--marker-only"])
    node = ObstacleSpawnerNode(args)
    try:
        assert len(node.obstacles) >= 1
        primary = node.obstacles[0]
        marker = node.create_marker_msg(primary)

        assert marker.ns == "obstacles", "Marker namespace must be 'obstacles' for RViz config"
        assert marker.id == 0
        assert marker.type == Marker.CUBE
        assert marker.action == Marker.ADD
        assert marker.header.frame_id == "base_link"
        assert marker.pose.position.x == 3.0
        assert marker.pose.position.y == 0.0
        assert marker.scale.x == 1.0
        assert marker.color.a > 0.0
    finally:
        node.destroy_node()


def test_scan_to_points_conversion(rclpy_init):
    """Verify scan_to_points converts LaserScan ranges to PointCloud2."""
    node = ScanToPoints()
    published_clouds = []

    # Mock the publisher to capture published cloud
    original_publish = node.publisher.publish
    node.publisher.publish = lambda msg: published_clouds.append(msg)

    try:
        scan = LaserScan()
        scan.header.frame_id = "lidar_link"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 2.0  # 4 rays: -pi, -pi/2, 0, pi/2
        scan.range_min = 0.1
        scan.range_max = 10.0
        # 4 ranges: [out_of_bounds, valid (dist=2.0 at -pi/2), valid (dist=3.0 at 0), inf]
        scan.ranges = [0.05, 2.0, 3.0, float("inf")]
        scan.intensities = [0.0, 50.0, 100.0, 0.0]

        node.callback(scan)

        assert len(published_clouds) == 1
        cloud: PointCloud2 = published_clouds[0]
        assert cloud.header.frame_id == "lidar_link"
        assert cloud.width == 2, f"Expected 2 valid points, got {cloud.width}"

        # Read points
        points = list(point_cloud2.read_points(cloud, field_names=("x", "y", "z", "intensity")))
        assert len(points) == 2

        # Point 1: angle = -pi/2 -> x ~ 0, y ~ -2.0, intensity = 50.0
        p1 = points[0]
        assert abs(p1[0] - 0.0) < 0.01
        assert abs(p1[1] - (-2.0)) < 0.01
        assert abs(p1[2] - 0.0) < 0.01
        assert abs(p1[3] - 50.0) < 0.01

        # Point 2: angle = 0 -> x ~ 3.0, y ~ 0.0, intensity = 100.0
        p2 = points[1]
        assert abs(p2[0] - 3.0) < 0.01
        assert abs(p2[1] - 0.0) < 0.01
        assert abs(p2[2] - 0.0) < 0.01
        assert abs(p2[3] - 100.0) < 0.01
    finally:
        node.publisher.publish = original_publish
        node.destroy_node()


def test_scan_to_points_empty_scan(rclpy_init):
    """Verify scan_to_points handles empty or all-inf scan gracefully."""
    node = ScanToPoints()
    published_clouds = []
    node.publisher.publish = lambda msg: published_clouds.append(msg)

    try:
        scan = LaserScan()
        scan.header.frame_id = "lidar_link"
        scan.angle_min = 0.0
        scan.angle_max = 1.0
        scan.angle_increment = 0.5
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.ranges = [float("inf"), float("nan")]
        scan.intensities = []

        node.callback(scan)

        assert len(published_clouds) == 1
        assert published_clouds[0].width == 0
    finally:
        node.destroy_node()


def test_scan_to_points_zero_intensities_fallback(rclpy_init):
    """Verify scan_to_points falls back to distance when intensities are all 0.0."""
    node = ScanToPoints()
    published_clouds = []
    node.publisher.publish = lambda msg: published_clouds.append(msg)

    try:
        scan = LaserScan()
        scan.header.frame_id = "lidar_link"
        scan.angle_min = 0.0
        scan.angle_max = 1.0
        scan.angle_increment = 0.5
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.ranges = [2.5, 4.0]
        # Gazebo default: intensities array matches ranges length but contains 0.0
        scan.intensities = [0.0, 0.0]

        node.callback(scan)

        assert len(published_clouds) == 1
        cloud: PointCloud2 = published_clouds[0]
        points = list(point_cloud2.read_points(cloud, field_names=("x", "y", "z", "intensity")))
        assert len(points) == 2
        # Intensity must fall back to distance, NOT stay 0.0
        assert abs(points[0][3] - 2.5) < 1e-3, f"Expected intensity 2.5, got {points[0][3]}"
        assert abs(points[1][3] - 4.0) < 1e-3, f"Expected intensity 4.0, got {points[1][3]}"
    finally:
        node.destroy_node()


def test_test_cloud_generation(rclpy_init):
    """Verify ObstacleSpawnerNode --test-cloud mode publishes PointCloud2 on obstacle."""
    args = parse_args(["--test-cloud", "--rate", "2.0"])
    node = ObstacleSpawnerNode(args)
    published_clouds = []
    node.cloud_pub.publish = lambda msg: published_clouds.append(msg)

    try:
        node.publish_test_cloud()
        assert len(published_clouds) == 1
        cloud: PointCloud2 = published_clouds[0]
        assert cloud.header.frame_id == "lidar_link"
        points = list(point_cloud2.read_points(cloud, field_names=("x", "y", "z", "intensity")))
        assert len(points) > 0

        # Lidar is offset by -0.0342 in base_link.
        # Box at x=3.0 with sx=1.0 has front face at x_base=2.5.
        # In lidar_link, front face is at 2.5 - (-0.0342) = 2.5342m.
        box_points = [p for p in points if abs(p[0] - 2.5342) < 0.05 and abs(p[1]) <= 0.55]
        assert len(box_points) > 30, f"Expected >30 box face points, found {len(box_points)}"

        # Cylinder at (3.5, 1.8) has points around y ~ 1.8
        cyl_points = [p for p in points if p[1] > 1.0]
        assert len(cyl_points) > 20, f"Expected >20 cylinder points, found {len(cyl_points)}"
    finally:
        node.destroy_node()


def test_obstacle_spawner_marker_array(rclpy_init):
    """Verify ObstacleSpawnerNode publishes Marker and MarkerArray with all obstacles."""
    args = parse_args(["--rate", "2.0"])
    node = ObstacleSpawnerNode(args)
    published_markers = []
    published_arrays = []

    node.marker_pub.publish = lambda msg: published_markers.append(msg)
    node.marker_array_pub.publish = lambda msg: published_arrays.append(msg)

    try:
        node.publish_markers()
        assert len(published_markers) >= 2, "Expected at least 2 individual markers"
        assert len(published_arrays) == 1, "Expected 1 MarkerArray message"
        assert isinstance(published_arrays[0], MarkerArray)
        assert len(published_arrays[0].markers) >= 2, "MarkerArray must contain all obstacles"
        ids = [m.id for m in published_arrays[0].markers]
        assert 0 in ids and 1 in ids, "Both obstacle 0 and 1 must be in MarkerArray"
    finally:
        node.destroy_node()


def test_dynamic_spawn_update_and_deduplication(rclpy_init):
    """Verify dynamic_spawn_callback updates existing obstacle by ID instead of duplicating."""
    args = parse_args(["--rate", "2.0"])
    node = ObstacleSpawnerNode(args)

    try:
        initial_count = len(node.obstacles)
        # Update existing obstacle 0 (change position to x=4.0)
        msg_update = Marker()
        msg_update.id = 0
        msg_update.type = Marker.CUBE
        msg_update.pose.position.x = 4.0
        msg_update.pose.position.y = 0.5
        msg_update.pose.position.z = 0.3
        msg_update.scale.x = 1.2
        msg_update.scale.y = 1.2
        msg_update.scale.z = 1.2

        node.dynamic_spawn_callback(msg_update)
        assert len(node.obstacles) == initial_count, "Obstacle count should not increase on update"
        updated_obs = next(o for o in node.obstacles if o["id"] == 0)
        assert updated_obs["x"] == 4.0
        assert updated_obs["sx"] == 1.2

        # Add new obstacle with id=99
        msg_new = Marker()
        msg_new.id = 99
        msg_new.type = Marker.CYLINDER
        msg_new.pose.position.x = 5.0
        msg_new.pose.position.y = -1.0
        msg_new.pose.position.z = 0.4
        msg_new.scale.x = 0.6
        msg_new.scale.y = 0.6
        msg_new.scale.z = 1.0

        node.dynamic_spawn_callback(msg_new)
        assert len(node.obstacles) == initial_count + 1, "Obstacle count must increase by 1"
        added_obs = next(o for o in node.obstacles if o["id"] == 99)
        assert added_obs["x"] == 5.0
    finally:
        node.destroy_node()
