#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
import sys
import termios
import tty

def apply_deadband(val, threshold=0.1):
    return val if abs(val) > threshold else 0.0

class KeyboardActuatorController(Node):
    def __init__(self):
        super().__init__('keyboard_actuator_controller')
        self.pub1 = self.create_publisher(Int32, '/actuator_command_1', 10)
        self.pub2 = self.create_publisher(Int32, '/actuator_command_2', 10)
        self.pub3 = self.create_publisher(Int32, '/actuator_command_3', 10)
        self.pub4 = self.create_publisher(Int32, '/actuator_command_4', 10)
        self.pub5 = self.create_publisher(Int32, '/actuator_command_5', 10)
        self.pub6 = self.create_publisher(Int32, '/actuator_command_6', 10)

        self.prev_cmd_1 = None
        self.prev_cmd_2 = None

        self.get_logger().info("Real-time actuator control started. Press 'Ctrl+C' to exit.")

        self.subscription = self.create_subscription(
            Imu,
            '/imu_data',
            self.imu_callback,
            10
        )
        self.subscription  # prevent linter warning

    def imu_callback(self, msg: Imu):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        new_cmd_1 = None
        new_cmd_2 = None

        if ay < -3:
            new_cmd_1 = 1
            new_cmd_2 = -1
        elif ay > 3:
            new_cmd_1 = -1
            new_cmd_2 = 1
        else:
            new_cmd_1 = 0
            new_cmd_2 = 0

        if new_cmd_1 != self.prev_cmd_1:
            self.prev_cmd_1 = new_cmd_1
            self.send_command(1, new_cmd_1)

        if new_cmd_2 != self.prev_cmd_2:
            self.prev_cmd_2 = new_cmd_2
            self.send_command(2, new_cmd_2)

    def send_command(self, actuator, value):
        msg = Int32()
        msg.data = value
        if actuator == 1:
            self.pub1.publish(msg)
        elif actuator == 2:
            self.pub2.publish(msg)
        elif actuator == 3:
            self.pub3.publish(msg)
        elif actuator == 4:
            self.pub4.publish(msg)
        elif actuator == 5:
            self.pub5.publish(msg)
        elif actuator == 6:
            self.pub6.publish(msg)

        self.get_logger().info(f"Actuator {actuator}: Command {value}")
    
def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def main():
    rclpy.init()
    node = KeyboardActuatorController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop actuators on shutdown
        node.send_command(1, 0)
        node.send_command(2, 0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
