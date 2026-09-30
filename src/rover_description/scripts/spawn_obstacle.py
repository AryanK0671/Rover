#!/usr/bin/env python3
"""
Obstacle Spawner & Visualizer Node for ROS 2 and RViz.

Features:
- Publishes visualization_msgs/msg/Marker to /obstacle_marker for RViz rendering.
- Spawns physical obstacle collision/visual models in Gazebo using ros_gz_sim create.
- Supports CLI customization (--x, --y, --z, --sx, --sy, --sz, --shape, --color).
- Supports --test-cloud mode to publish synthetic PointCloud2 data on /scan/points.
- Listens on /spawn_obstacle for runtime obstacle additions.
"""

import argparse
import math
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


COLOR_MAP = {
    "red": (1.0, 0.1, 0.1, 0.85),
    "orange": (1.0, 0.35, 0.1, 0.85),
    "yellow": (1.0, 0.9, 0.1, 0.85),
    "green": (0.1, 0.9, 0.2, 0.85),
    "cyan": (0.1, 0.75, 0.95, 0.85),
    "blue": (0.1, 0.3, 0.95, 0.85),
    "purple": (0.7, 0.1, 0.9, 0.85),
    "white": (0.95, 0.95, 0.95, 0.85),
}

POINT_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
]


def generate_sdf(name: str, shape: str, sx: float, sy: float, sz: float, color: tuple) -> str:
    """Generate SDF XML string for Gazebo Fortress entity."""
    r, g, b, a = color
    if shape in ("cylinder", "cyl"):
        geom = f"<cylinder><radius>{sx / 2.0:.3f}</radius><length>{sz:.3f}</length></cylinder>"
    elif shape in ("sphere", "sph"):
        geom = f"<sphere><radius>{sx / 2.0:.3f}</radius></sphere>"
    else:
        geom = f"<box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>"

    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>{geom}</geometry>
      </collision>
      <visual name="visual">
        <geometry>{geom}</geometry>
        <material>
          <ambient>{r:.2f} {g:.2f} {b:.2f} {a:.2f}</ambient>
          <diffuse>{r:.2f} {g:.2f} {b:.2f} {a:.2f}</diffuse>
          <specular>0.2 0.2 0.2 1.0</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


class ObstacleSpawnerNode(Node):
    def __init__(self, args):
        super().__init__("obstacle_spawner")
        self.args = args

        # Marker publisher for RViz (/obstacle_marker matches robot_config.rviz)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.marker_pub = self.create_publisher(Marker, "/obstacle_marker", qos_profile)
        self.marker_array_pub = self.create_publisher(
            MarkerArray, "/obstacle_marker_array", qos_profile
        )

        # Optional synthetic test pointcloud publisher (/scan/points)
        if self.args.test_cloud:
            self.cloud_pub = self.create_publisher(PointCloud2, "/scan/points", qos_profile)
            self.cloud_timer = self.create_timer(0.1, self.publish_test_cloud)

        # Active obstacles list: list of dicts
        self.obstacles = []
        self._init_obstacles()

        # Timer to periodically publish markers (ensures RViz picks them up even when launched)
        timer_period = 1.0 / max(0.1, self.args.rate)
        self.marker_timer = self.create_timer(timer_period, self.publish_markers)

        # Subscriber for dynamic runtime obstacle injection
        self.create_subscription(Marker, "/spawn_obstacle", self.dynamic_spawn_callback, 10)

        self.get_logger().info(
            f"ObstacleSpawner: publishing markers on /obstacle_marker at {self.args.rate} Hz"
        )

        if not self.args.marker_only and self.args.spawn_gazebo:
            self.spawn_gazebo_obstacles()

    def _init_obstacles(self):
        color = COLOR_MAP.get(self.args.color.lower(), COLOR_MAP["orange"])
        primary = {
            "name": self.args.name,
            "id": 0,
            "shape": self.args.shape,
            "x": self.args.x,
            "y": self.args.y,
            "z": self.args.z,
            "sx": self.args.sx,
            "sy": self.args.sy,
            "sz": self.args.sz,
            "color": color,
            "frame_id": self.args.frame_id,
        }
        self.obstacles.append(primary)

        # Multi-obstacle mode: add cylinder obstacle if using default box_obstacle
        is_default_box = (
            self.args.name == "box_obstacle"
            and abs(self.args.x - 3.0) < 1e-4
            and abs(self.args.y) < 1e-4
        )
        if is_default_box:
            secondary = {
                "name": "cylinder_obstacle",
                "id": 1,
                "shape": "cylinder",
                "x": 3.5,
                "y": 1.8,
                "z": 0.4,
                "sx": 0.7,
                "sy": 0.7,
                "sz": 1.2,
                "color": COLOR_MAP["cyan"],
                "frame_id": self.args.frame_id,
            }
            self.obstacles.append(secondary)

    def dynamic_spawn_callback(self, msg: Marker):
        obs = {
            "name": f"obstacle_{msg.id}",
            "id": msg.id,
            "shape": "cube" if msg.type == Marker.CUBE else "cylinder",
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "z": msg.pose.position.z,
            "sx": msg.scale.x if msg.scale.x > 0 else 1.0,
            "sy": msg.scale.y if msg.scale.y > 0 else 1.0,
            "sz": msg.scale.z if msg.scale.z > 0 else 1.0,
            "color": (msg.color.r, msg.color.g, msg.color.b, msg.color.a or 0.8),
            "frame_id": msg.header.frame_id or self.args.frame_id,
        }
        existing_idx = next((i for i, o in enumerate(self.obstacles) if o["id"] == msg.id), None)
        if existing_idx is not None:
            self.obstacles[existing_idx] = obs
            self.get_logger().info(
                f"Updated obstacle id={msg.id} at ({obs['x']:.2f}, {obs['y']:.2f}, {obs['z']:.2f})"
            )
        else:
            self.obstacles.append(obs)
            self.get_logger().info(
                f"Added obstacle id={msg.id} at ({obs['x']:.2f}, {obs['y']:.2f}, {obs['z']:.2f})"
            )

        if self.args.spawn_gazebo:
            threading.Thread(target=self._spawn_single_gazebo, args=(obs,), daemon=True).start()

    def create_marker_msg(self, obs: dict) -> Marker:
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = obs["frame_id"]
        m.ns = "obstacles"
        m.id = obs["id"]
        m.action = Marker.ADD

        shape = obs["shape"].lower()
        if shape in ("cylinder", "cyl"):
            m.type = Marker.CYLINDER
        elif shape in ("sphere", "sph"):
            m.type = Marker.SPHERE
        else:
            m.type = Marker.CUBE

        m.pose.position.x = float(obs["x"])
        m.pose.position.y = float(obs["y"])
        m.pose.position.z = float(obs["z"])
        m.pose.orientation.w = 1.0

        m.scale.x = float(obs["sx"])
        m.scale.y = float(obs["sy"])
        m.scale.z = float(obs["sz"])

        r, g, b, a = obs["color"]
        m.color.r = float(r)
        m.color.g = float(g)
        m.color.b = float(b)
        m.color.a = float(a)
        return m

    def publish_markers(self):
        if self.args.clear:
            clear_marker = Marker()
            clear_marker.header.stamp = self.get_clock().now().to_msg()
            clear_marker.header.frame_id = self.args.frame_id
            clear_marker.ns = "obstacles"
            clear_marker.action = Marker.DELETEALL
            self.marker_pub.publish(clear_marker)
            clear_array = MarkerArray(markers=[clear_marker])
            self.marker_array_pub.publish(clear_array)
            return

        marker_array = MarkerArray()
        for obs in self.obstacles:
            marker = self.create_marker_msg(obs)
            self.marker_pub.publish(marker)
            marker_array.markers.append(marker)
        self.marker_array_pub.publish(marker_array)

    def publish_test_cloud(self):
        """Publish synthetic point cloud along obstacle surface for RViz validation."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "lidar_link"

        # Rover LiDAR origin in base_link (from URDF kinematics)
        lidar_x = -0.0342
        lidar_y = 0.0

        points = []
        for obs in self.obstacles:
            ox = float(obs["x"]) - lidar_x
            oy = float(obs["y"]) - lidar_y
            shape = obs["shape"].lower()

            if shape in ("cylinder", "cyl"):
                radius = float(obs["sx"]) / 2.0
                dist_center = math.hypot(ox, oy)
                if dist_center <= radius:
                    continue
                angle_center = math.atan2(oy, ox)
                half_angle = math.asin(min(0.999, radius / dist_center))
                steps = 40
                for i in range(steps + 1):
                    ray_angle = (angle_center - half_angle * 0.95) + (
                        2 * half_angle * 0.95 * (i / steps)
                    )
                    dx = math.cos(ray_angle)
                    dy = math.sin(ray_angle)
                    b = -2.0 * (ox * dx + oy * dy)
                    c = ox * ox + oy * oy - radius * radius
                    disc = b * b - 4.0 * c
                    if disc >= 0:
                        t = (-b - math.sqrt(disc)) / 2.0
                        if t > 0:
                            px = t * dx
                            py = t * dy
                            intensity = float(t)
                            points.append((px, py, 0.0, intensity))
            else:
                front_x = ox - float(obs["sx"]) / 2.0
                y_min = oy - float(obs["sy"]) / 2.0
                y_max = oy + float(obs["sy"]) / 2.0
                steps = 50
                for i in range(steps + 1):
                    py = y_min + (y_max - y_min) * (i / steps)
                    dist = math.hypot(front_x, py)
                    intensity = float(dist)
                    points.append((front_x, py, 0.0, intensity))

        cloud = point_cloud2.create_cloud(header, POINT_FIELDS, points)
        self.cloud_pub.publish(cloud)

    def spawn_gazebo_obstacles(self):
        """Spawn obstacle models in running Gazebo Fortress instance."""
        for obs in self.obstacles:
            self._spawn_single_gazebo(obs)

    def _spawn_single_gazebo(self, obs: dict):
        sdf_content = generate_sdf(
            obs["name"], obs["shape"], obs["sx"], obs["sy"], obs["sz"], obs["color"]
        )
        gz_z = obs["z"] + 0.2 if obs["frame_id"] == "base_link" else obs["z"]
        cmd = [
            "ros2", "run", "ros_gz_sim", "create",
            "-world", "rough_terrain_world",
            "-name", obs["name"],
            "-allow_renaming", "true",
            "-x", str(obs["x"]),
            "-y", str(obs["y"]),
            "-z", str(gz_z),
            "-string", sdf_content,
        ]
        try:
            self.get_logger().info(
                f"Spawning '{obs['name']}' at ({obs['x']:.2f}, {obs['y']:.2f}, {gz_z:.2f})..."
            )
            res = subprocess.run(cmd, capture_output=True, text=True, timeout=8)
            if res.returncode == 0:
                self.get_logger().info(f"Successfully spawned '{obs['name']}' in Gazebo.")
            else:
                msg = res.stderr.strip() or res.stdout.strip()
                self.get_logger().warn(f"Gazebo spawn exit {res.returncode}: {msg}")
        except Exception as e:
            self.get_logger().warn(f"Gazebo entity spawner: {e}")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Spawn obstacle for RViz and Gazebo pointcloud testing."
    )
    parser.add_argument("--name", type=str, default="box_obstacle", help="Name of obstacle")
    parser.add_argument("-x", "--x", type=float, default=3.0, help="X position (m)")
    parser.add_argument("-y", "--y", type=float, default=0.0, help="Y position (m)")
    parser.add_argument(
        "-z", "--z", type=float, default=0.3, help="Z position (m) in base_link (default: 0.3m)"
    )
    parser.add_argument("--sx", type=float, default=1.0, help="Size X or diameter (m)")
    parser.add_argument("--sy", type=float, default=1.0, help="Size Y (m)")
    parser.add_argument("--sz", type=float, default=1.0, help="Size Z / height (m)")
    parser.add_argument(
        "--shape", type=str, default="box", choices=["box", "cube", "cylinder", "sphere"]
    )
    parser.add_argument("--color", type=str, default="orange", help="Color (orange, red, cyan)")
    parser.add_argument("--frame-id", type=str, default="base_link", help="TF frame for RViz")
    parser.add_argument("--rate", type=float, default=2.0, help="Marker publishing rate (Hz)")
    parser.add_argument("--marker-only", action="store_true", help="Only publish RViz marker")
    parser.add_argument("--gazebo-only", action="store_true", help="Only spawn in Gazebo")
    parser.add_argument(
        "--spawn-gazebo", action="store_true", default=False, help="Spawn in Gazebo via ros_gz_sim"
    )
    parser.add_argument(
        "--test-cloud",
        "--with-points",
        dest="test_cloud",
        action="store_true",
        help="Publish synthetic PointCloud2 on /scan/points",
    )
    parser.add_argument("--clear", action="store_true", help="Delete all obstacle markers in RViz")
    parser.add_argument("--one-shot", action="store_true", help="Publish once and exit")

    parsed, _ = parser.parse_known_args(argv)
    return parsed


def main(args=None):
    parsed_cli = parse_args(sys.argv[1:])

    rclpy.init(args=args)
    node = ObstacleSpawnerNode(parsed_cli)

    if parsed_cli.one_shot:
        node.publish_markers()
        time.sleep(0.2)
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
