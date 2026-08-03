import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
import numpy as np
from .inverseKinematics import StewartPlatformIK
from .teensy_serial import TeensySerial

class StewartKinematicsNode(Node):
    def __init__(self):
        super().__init__('stewart_kinematics_node')

        self.kinematics = StewartPlatformIK()
        self.min_extension = 0.0
        self.max_extension = self.kinematics.stroke_length

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.teensy = TeensySerial(serial_port, baud_rate)
        self.get_logger().info(
            f'Connected to Teensy on {serial_port} at {baud_rate} baud'
        )

        self.pose_sub = self.create_subscription(
            Pose,
            '/desired_pose',
            self.pose_callback,
            10
        )

        self.length_pub = self.create_publisher(
            Float64MultiArray,
            '/actuator_lengths',
            10
        )

    def pose_callback(self, msg):
        trans = np.array([msg.position.x, msg.position.y, msg.position.z])
        quaternion = np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ])

        try:
            lengths = self.kinematics.solve(trans, quaternion)
        except ValueError as exc:
            self.get_logger().error(f'Invalid desired pose: {exc}')
            return

        if np.all((lengths >= self.min_extension) & (lengths <= self.max_extension)):
            # Publish full array
            #print(lengths)
            msg_out = Float64MultiArray()
            msg_out.data = lengths.tolist()
            self.length_pub.publish(msg_out)

            try:
                command = self.teensy.send_moveall(lengths)
                self.get_logger().info(f'Sent: {command}')
            except (OSError, RuntimeError) as exc:
                self.get_logger().error(f'Failed to write to Teensy: {exc}')
        else:
            self.get_logger().warning('Desired pose is out of workspace')

    def destroy_node(self):
        self.teensy.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = StewartKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
