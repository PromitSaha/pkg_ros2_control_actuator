#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import Imu
import sys
import termios
import tty
import time

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
        self.pub7 = self.create_publisher(Int32, '/actuator_command', 10)
        self.global_current_position = 15.0
        self.actuator_pubs = [
            self.create_publisher(Float32, f'/target_position_{i+1}', 10)
            for i in range(6)
        ]

        self.get_logger().info("Real-time actuator control started. Press 'Ctrl+C' to exit.")

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
        elif actuator == 7:
            self.pub7.publish(msg)
            
        self.get_logger().info(f"Actuator {actuator}: Command {value}")

    def upOrDown(self, direction):
        self.global_current_position = self.global_current_position + (direction*5)

        float_msg = Float32()
        float_msg.data = self.global_current_position

        for i in [0, 1]:
            self.actuator_pubs[i].publish(float_msg)
            #time.sleep(0.05)

        for i in [2, 3]:
            self.actuator_pubs[i].publish(float_msg)
            #time.sleep(0.05)

        for i in [4, 5]:
            self.actuator_pubs[i].publish(float_msg)
            #time.sleep(0.05)
    
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
        while True:
            key = get_key()
            if key == "8":
                node.upOrDown(1)
            elif key == '2':
                node.upOrDown(-1)
            elif key == 'q':
                node.send_command(1, 1)  # actuator 1 extend
            elif key == 'z':
                node.send_command(1, -1)  # actuator 1 retract
            elif key == 'w':
                node.send_command(2, 1)  # actuator 2 extend
            elif key == 'x':
                node.send_command(2, -1)  # actuator 2 retract
            elif key == 'a':
                node.send_command(1, 0)
            elif key == 's':
                node.send_command(2, 0)
            elif key == 'e':
                node.send_command(3, 1)  # actuator 3 extend
            elif key == 'c':
                node.send_command(3, -1)  # actuator 3 retract
            elif key == 'r':
                node.send_command(4, 1)  # actuator 4 extend
            elif key == 'v':
                node.send_command(4, -1)  # actuator 4 retract
            elif key == 'd':
                node.send_command(3, 0)
            elif key == 'f':
                node.send_command(4, 0)
            elif key == 't':
                node.send_command(5, 1)  # actuator 5 extend
            elif key == 'b':
                node.send_command(5, -1)  # actuator 5 retract
            elif key == 'y':
                node.send_command(6, 1)  # actuator 6 extend
            elif key == 'n':
                node.send_command(6, -1)  # actuator 6 retract
            elif key == 'g':
                node.send_command(5, 0)
            elif key == 'h':
                node.send_command(6, 0)

            elif key == 'p':
                node.send_command(2, 1)
                node.send_command(1, 1)
                node.send_command(3, 1)
                node.send_command(4, 1)
                node.send_command(5, 1)
                node.send_command(6, 1)
            elif key == 'l':
                node.send_command(1, 0)
                node.send_command(2, 0)
                node.send_command(3, 0)
                node.send_command(4, 0)
                node.send_command(5, 0)
                node.send_command(6, 0)
            elif key == ',':
                node.send_command(1, -1)
                node.send_command(2, -1)
                node.send_command(3, -1)
                node.send_command(4, -1)
                node.send_command(5, -1)
                node.send_command(6, -1)

            elif key == '1':
                node.send_command(7, 1)
            elif key == '2':
                node.send_command(7, 0)
            elif key == '3':
                node.send_command(7, -1)
            
            elif key == '\x03':  # Ctrl+C
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
