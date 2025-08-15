#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
import sys
import termios
import tty

def apply_deadband(val, threshold=0.1):
    return val if abs(val) > threshold else 0.0

class KeyboardActuatorController(Node):
    def __init__(self):
        super().__init__('keyboard_actuator_controller')
        self.desiredPositionPublisher = self.create_publisher(Pose, '/desired_pose', 10)

        self.prev_ax = 0.0
        self.prev_ay = 0.0

        self.ax = 0.0
        self.ay = 0.0

        self.get_logger().info("Real-time actuator control started. Press 'Ctrl+C' to exit.")

        self.subscription = self.create_subscription(
            Imu,
            '/imu_data',
            self.imu_callback,
            10
        )
        self.subscription  # prevent linter warning

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_pose)

    def imu_callback(self, msg: Imu):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y

        self.ax = ax
        self.ay = ay

    def publish_pose(self):
        if abs(self.ax - self.prev_ax) > 1.0 or abs(self.ay - self.prev_ay) > 1.0: 
            msg = Pose()

            msg.position.x = 0.0
            msg.position.y = 0.0
            msg.position.z = 0.0
            msg.orientation.x = self.ax/10.0
            msg.orientation.y = self.ay/10.0
            msg.orientation.z = 0.0
            msg.orientation.w = 1.0

            self.desiredPositionPublisher.publish(msg)
            self.get_logger().info(f'Published desired_pose: {msg}')

            self.prev_ax = self.ax
            self.prev_ay = self.ay
    
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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
