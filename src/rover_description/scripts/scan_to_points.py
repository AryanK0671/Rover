#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2

POINT_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
]


class ScanToPoints(Node):
    def __init__(self):
        super().__init__("scan_to_points")
        self.publisher = self.create_publisher(
            PointCloud2, "/scan/points", 10
        )
        self.create_subscription(
            LaserScan, "/scan", self.callback, qos_profile_sensor_data
        )
        self._logged_first_points = False
        self.get_logger().info("ScanToPoints node started: /scan -> /scan/points")

    def callback(self, scan: LaserScan):
        points = []
        has_real_intensities = (
            len(scan.intensities) == len(scan.ranges)
            and any(float(v) > 0.0 for v in scan.intensities)
        )
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue

            angle = scan.angle_min + index * scan.angle_increment
            intensity = (
                float(scan.intensities[index])
                if has_real_intensities
                else float(distance)
            )
            points.append((
                distance * math.cos(angle),
                distance * math.sin(angle),
                0.0,
                intensity,
            ))

        cloud = point_cloud2.create_cloud(scan.header, POINT_FIELDS, points)
        self.publisher.publish(cloud)

        if points and not self._logged_first_points:
            self._logged_first_points = True
            self.get_logger().info(
                f"Published PointCloud2 with {len(points)} valid 3D points on /scan/points"
            )


def main():
    rclpy.init()
    node = ScanToPoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
