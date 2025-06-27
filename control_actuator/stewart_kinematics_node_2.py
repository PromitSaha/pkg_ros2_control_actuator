import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray, Float32
import numpy as np
import transforms3d.euler
import time
from .inverseKinematics import inv_kinematics

class StewartKinematicsNode(Node):
    def __init__(self):
        super().__init__('stewart_kinematics_node_2')

        # Platform and base geometry (hardcoded)
        self.r_B = 0.225      # Base radius in meters
        self.r_P = 0.160      # Platform radius in meters
        self.gamma_B = 12   # Base anchor pair angle
        self.gamma_P = 12   # Platform anchor pair angle
        self.home_pos = np.array([0, 0, 0.457])  # Neutral height in meters

        self.pose_sub = self.create_subscription(
            Pose,
            '/desired_pose_2',
            self.pose_callback,
            10
        )

        self.length_pub = self.create_publisher(
            Float64MultiArray,
            '/actuator_lengths',
            10
        )

        self.actuator_pubs = [
            self.create_publisher(Float32, f'/target_position_{i+1}', 10)
            for i in range(6)
        ]

    def pose_callback(self, msg):
        # Extract translation
        trans = np.array([msg.position.x, msg.position.y, msg.position.z])
        # Extract rotation (quaternion -> RPY)
        rotation = [msg.orientation.x, msg.orientation.y, msg.orientation.z]

        platform = inv_kinematics(self.r_B, self.r_P, self.gamma_B, self.gamma_P)
        lengths = platform.solve(trans, rotation)

        
        # Publish full array
        print(lengths)
        # msg_out = Float64MultiArray()
        # msg_out.data = lengths.tolist()
        # self.length_pub.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = StewartKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
