import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class KeyboardActuatorController(Node):
    def __init__(self):
        super().__init__('keyboard_actuator_controller')
        self.publisher_ = self.create_publisher(Int32, '/actuator_command', 10)
        self.get_logger().info("Keyboard actuator control started.")

    def send_command(self, value):
        msg = Int32()
        msg.data = value
        self.publisher_.publish(msg)
        self.get_logger().info(f"Sent command: {value}")

def main():
    rclpy.init()
    node = KeyboardActuatorController()

    try:
        while True:
            key = input("Press [i] to extend, [,] to retract, [k] to stop, [q] to quit: ").strip()
            if key == 'i':
                node.send_command(1)
            elif key == ',':
                node.send_command(-1)
            elif key == 'k':
                node.send_command(0)
            elif key == 'q':
                break
            else:
                print("Invalid key")
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
