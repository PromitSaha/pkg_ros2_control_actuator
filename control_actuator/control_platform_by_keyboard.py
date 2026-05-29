#!/usr/bin/env python3

import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose


class KeyboardPoseController(Node):
    def __init__(self):
        super().__init__('keyboard_pose_controller')
        self.pose_pub = self.create_publisher(Pose, '/desired_pose', 10)

        self.get_logger().info(
            "Keyboard pose control started.\n"
            "Keys:\n"
            "  6 -> +roll    | 4 -> -roll\n"
            "  8 -> +pitch   | 2 -> -pitch\n"
            "  9 -> +yaw     | 7 -> -yaw\n"
            "  w -> +x       | s -> -x\n"
            "  d -> +y       | a -> -y\n"
            "  + -> +z       | - -> -z\n"
            "  5 -> reset to neutral pose\n"
            "  Ctrl+C -> exit"
        )

    def publish_pose(self, x=0.0, y=0.0, z=0.1, ox=0.0, oy=0.0, oz=0.0, ow=1.0):
        msg = Pose()
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = float(z)
        msg.orientation.x = float(ox)
        msg.orientation.y = float(oy)
        msg.orientation.z = float(oz)
        msg.orientation.w = float(ow)

        self.pose_pub.publish(msg)
        self.get_logger().info(
            f"Published Pose -> position: [x={msg.position.x}, y={msg.position.y}, z={msg.position.z}], "
            f"orientation: [x={msg.orientation.x}, y={msg.orientation.y}, z={msg.orientation.z}, w={msg.orientation.w}]"
        )


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def main():
    rclpy.init()
    node = KeyboardPoseController()

    try:
        while True:
            key = get_key()

            # Roll
            if key == '6':
                node.publish_pose(z=0.1, ox=0.3)
            elif key == '4':
                node.publish_pose(z=0.1, ox=-0.3)

            # Pitch
            elif key == '8':
                node.publish_pose(z=0.1, oy=0.3)
            elif key == '2':
                node.publish_pose(z=0.1, oy=-0.3)

            # Yaw
            elif key == '9':
                node.publish_pose(z=0.1, oz=0.5)
            elif key == '7':
                node.publish_pose(z=0.1, oz=-0.5)

            # X movement
            elif key == 'w':
                node.publish_pose(x=0.1, y=0.0, z=0.1)
            elif key == 's':
                node.publish_pose(x=-0.1, y=0.0, z=0.1)

            # Y movement
            elif key == 'd':
                node.publish_pose(x=0.0, y=0.1, z=0.1)
            elif key == 'a':
                node.publish_pose(x=0.0, y=-0.1, z=0.1)

            # Z movement
            elif key == '+':
                node.publish_pose(x=0.0, y=0.0, z=0.2)
            elif key == '-':
                node.publish_pose(x=0.0, y=0.0, z=0.0)

            # Reset / neutral
            elif key == '5':
                node.publish_pose(x=0.0, y=0.0, z=0.1, ox=0.0, oy=0.0, oz=0.0, ow=1.0)

            elif key == '\x03':
                break

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()