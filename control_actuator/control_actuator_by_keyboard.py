#!/usr/bin/env python3

# from pynput import keyboard
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int32

# class KeyboardActuatorController(Node):
#     def __init__(self):
#         super().__init__('keyboard_actuator_controller')
#         self.pub1 = self.create_publisher(Int32, '/actuator_command', 10)
#         self.pub2 = self.create_publisher(Int32, '/actuator_command_2', 10)

#         self.current_cmd_1 = 0
#         self.current_cmd_2 = 0

#         # Timer to publish regularly
#         self.create_timer(0.1, self.timer_callback)

#         self.get_logger().info("Hold keys to move actuators. Press Ctrl+C to exit.")

#     def timer_callback(self):
#         msg1 = Int32()
#         msg1.data = self.current_cmd_1
#         self.pub1.publish(msg1)

#         msg2 = Int32()
#         msg2.data = self.current_cmd_2
#         self.pub2.publish(msg2)

#     def set_command(self, actuator, value):
#         if actuator == 1:
#             self.current_cmd_1 = value
#         elif actuator == 2:
#             self.current_cmd_2 = value
#         self.get_logger().info(f"[KEY] Actuator {actuator} -> {value}")

# def main():
#     rclpy.init()
#     node = KeyboardActuatorController()

#     key_map = {
#         'q': (1, 1),   # actuator 1 extend
#         'z': (1, -1),  # actuator 1 retract
#         'w': (2, 1),   # actuator 2 extend
#         'x': (2, -1),  # actuator 2 retract
#     }

#     def on_press(key):
#         char = getattr(key, 'char', None)
#         if char in key_map:
#             actuator, cmd = key_map[char]
#             node.set_command(actuator, cmd)

#     def on_release(key):
#         char = getattr(key, 'char', None)
#         if char in ['q', 'z']:
#             node.set_command(1, 0)
#         elif char in ['w', 'x']:
#             node.set_command(2, 0)

#     with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
#         try:
#             rclpy.spin(node)
#         except KeyboardInterrupt:
#             pass
#         finally:
#             node.set_command(1, 0)
#             node.set_command(2, 0)
#             node.destroy_node()
#             rclpy.shutdown()

# if __name__ == '__main__':
#     main()



import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import sys
import termios
import tty

class KeyboardActuatorController(Node):
    def __init__(self):
        super().__init__('keyboard_actuator_controller')
        self.pub1 = self.create_publisher(Int32, '/actuator_command', 10)
        self.pub2 = self.create_publisher(Int32, '/actuator_command_2', 10)
        self.get_logger().info("Real-time actuator control started. Press 'Ctrl+C' to exit.")

    def send_command(self, actuator, value):
        msg = Int32()
        msg.data = value
        if actuator == 1:
            self.pub1.publish(msg)
        elif actuator == 2:
            self.pub2.publish(msg)
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
        while True:
            key = get_key()

            if key == 'q':
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
            elif key == '\x03':  # Ctrl+C
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
