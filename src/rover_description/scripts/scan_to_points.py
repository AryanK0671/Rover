#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from sensor_msgs_py import point_cloud2


class ScanToPoints(Node):
    def __init__(self):
        super().__init__("scan_to_points")
        self.publisher = self.create_publisher(
            point_cloud2.PointCloud2, "/scan/points", 10
        )
        self.create_subscription(
            LaserScan, "/scan", self.callback, qos_profile_sensor_data
        )

    def callback(self, scan):
        points = []
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue

            angle = scan.angle_min + index * scan.angle_increment
            points.append((
                distance * math.cos(angle),
                distance * math.sin(angle),
                0.0,
            ))

        cloud = point_cloud2.create_cloud_xyz32(scan.header, points)
        self.publisher.publish(cloud)


def main():
    rclpy.init()
    node = ScanToPoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
