#!/usr/bin/env python3


import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

JOINT_ORDER = [
    "front_wheel_to_rocker_left_joint",
    "middle_wheel_left_joint",
    "back_wheel_left_joint",
    "front_wheel_right_joint",
    "middle_wheel_right_joint",
    "back_wheel_right_joint",
]


LEFT_SIGN = -1
RIGHT_SIGN = 1

# key -> (forward, turn) unit multipliers
MOVE_BINDINGS = {
    'i': (1, 0),
    ',': (-1, 0),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    'o': (1, -1),
    'm': (-1, 1),
    '.': (-1, -1),
}

# key -> (speed_scale, turn_scale)
SPEED_BINDINGS = {
    'q': (1.1, 1.0),
    'z': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}

STOP_KEYS = (' ', 'k')

BANNER = """
Rover skid-steer teleop
------------------------
Reading from the keyboard and publishing wheel velocity commands to
/rover_base_controller/commands.

        u    i    o
        j    k    l
        m    ,    .

i / , : forward / backward
j / l : turn left / right (in place)
u / o : forward + curve left / right
m / . : backward + curve left / right
k or SPACE : STOP

q/z : speed up / down     e/c : turn speed up / down
CTRL-C to quit (stops the rover first)
------------------------
"""


def get_key(settings):
    """Non-blocking single keypress read (0.1s poll), raw terminal mode."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class RoverTeleop(Node):
    def __init__(self):
        super().__init__('rover_teleop_key')
        self.pub = self.create_publisher(
            Float64MultiArray, '/rover_base_controller/commands', 10
        )
        self.speed = 2.0  # rad/s commanded to each wheel when driving straight
        self.turn = 1.5   # rad/s added/subtracted between sides when turning

    def send(self, forward: float, turn: float):
        left = LEFT_SIGN * (self.speed * forward - self.turn * turn)
        right = RIGHT_SIGN * (self.speed * forward + self.turn * turn)

        msg = Float64MultiArray()
        # Order must match JOINT_ORDER / config/controllers.yaml
        msg.data = [left, left, left, right, right, right]
        self.pub.publish(msg)

    def stop(self):
        self.send(0.0, 0.0)


def main():
    if not sys.stdin.isatty():
        print("rover_teleop_key requires an interactive terminal (TTY). Exiting.")
        return
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = RoverTeleop()

    print(BANNER)
    print(f"speed: {node.speed:.2f}  turn: {node.turn:.2f}")

    try:
        while rclpy.ok():
            key = get_key(settings)

            if key in MOVE_BINDINGS:
                forward, turn = MOVE_BINDINGS[key]
                node.send(forward, turn)

            elif key in STOP_KEYS:
                node.stop()

            elif key in SPEED_BINDINGS:
                s, t = SPEED_BINDINGS[key]
                node.speed *= s
                node.turn *= t
                print(f"speed: {node.speed:.2f}  turn: {node.turn:.2f}")

            elif key == '\x03':  # CTRL-C
                break

    except Exception as e:
        print(e)

    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
