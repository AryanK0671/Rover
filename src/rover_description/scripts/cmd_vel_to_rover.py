#!/usr/bin/env python3
"""
Convert geometry_msgs/Twist on /cmd_vel to wheel velocity commands.

Translates velocity commands to /rover_base_controller/commands for 6-wheel rover.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class CmdVelToRover(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_rover")

        # Declare rover physical parameters
        self.declare_parameter("wheel_radius", 0.10)       # meters
        # Track width (lateral distance between left and right wheels in meters)
        self.declare_parameter("track_width", 0.53)
        self.declare_parameter("max_wheel_speed", 10.0)    # rad/s

        self.radius = self.get_parameter("wheel_radius").value
        self.width = self.get_parameter("track_width").value
        self.max_speed = self.get_parameter("max_wheel_speed").value

        # Joint signs matching URDF joint orientation
        self.left_sign = -1.0
        self.right_sign = 1.0

        self.sub_cmd_vel = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )
        self.pub_commands = self.create_publisher(
            Float64MultiArray, "/rover_base_controller/commands", 10
        )

        self.get_logger().info(
            f"CmdVelToRover initialized: track_width={self.width}m, wheel_radius={self.radius}m"
        )

    def cmd_vel_callback(self, msg: Twist):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Skid-steer kinematics
        v_left = linear_x - (angular_z * self.width / 2.0)
        v_right = linear_x + (angular_z * self.width / 2.0)

        omega_left = v_left / self.radius
        omega_right = v_right / self.radius

        # Clamp to max speed
        omega_left = max(-self.max_speed, min(self.max_speed, omega_left))
        omega_right = max(-self.max_speed, min(self.max_speed, omega_right))

        cmd_left = self.left_sign * omega_left
        cmd_right = self.right_sign * omega_right

        cmd_msg = Float64MultiArray()
        # Order matches JOINT_ORDER / config/controllers.yaml:
        # [front_left, middle_left, back_left, front_right, middle_right, back_right]
        cmd_msg.data = [cmd_left, cmd_left, cmd_left, cmd_right, cmd_right, cmd_right]
        self.pub_commands.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToRover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop rover on shutdown
        stop_msg = Float64MultiArray()
        stop_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        node.pub_commands.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
