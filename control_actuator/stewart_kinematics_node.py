import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray, Float32
import numpy as np
import transforms3d.euler
import time

class StewartKinematicsNode(Node):
    def __init__(self):
        super().__init__('stewart_kinematics_node')

        # Platform and base geometry (hardcoded)
        self.r_B = 0.225      # Base radius in meters
        self.r_P = 0.160      # Platform radius in meters
        self.gamma_B = np.deg2rad(25)   # Base anchor pair angle
        self.gamma_P = np.deg2rad(90)   # Platform anchor pair angle
        self.home_pos = np.array([0, 0, 0.457])  # Neutral height in meters
        
        self.pi = np.pi

        self.psi_B = np.array([
            -self.gamma_B,
            self.gamma_B,
            2*self.pi/3 - self.gamma_B,
            2*self.pi/3 + self.gamma_B,
            4*self.pi/3 - self.gamma_B,
            4*self.pi/3 + self.gamma_B
        ])

        self.psi_P = np.array([
            5*self.pi/3 + self.gamma_P,
            self.pi/3 - self.gamma_P,
            self.pi/3 + self.gamma_P,
            self.pi + self.gamma_P,
            self.pi + 2*self.gamma_P,
            5*self.pi/3 - self.gamma_P
        ])

        self.B = self.r_B * np.array([[np.cos(a), np.sin(a), 0] for a in self.psi_B]).T
        self.P = self.r_P * np.array([[np.cos(a), np.sin(a), 0] for a in self.psi_P]).T

        self.home_pos = np.array([0, 0, 2*self.r_B])

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

        self.actuator_pubs = [
            self.create_publisher(Float32, f'/target_position_{i+1}', 10)
            for i in range(6)
        ]

    def pose_callback(self, msg):
        # Extract translation
        trans = np.array([msg.position.x, msg.position.y, msg.position.z])

        # Extract rotation (quaternion -> RPY)
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        rpy = transforms3d.euler.quat2euler(quat, axes='sxyz')

        R = self.rotX(rpy[0]) @ self.rotY(rpy[1]) @ self.rotZ(rpy[2])

        # Compute actuator vectors and lengths
        l = (trans[:, np.newaxis] + self.home_pos[:, np.newaxis] + R @ self.P) - self.B
        lengths = np.linalg.norm(l, axis=0)
        print(lengths)
        # Publish full array
        msg_out = Float64MultiArray()
        msg_out.data = lengths.tolist()
        self.length_pub.publish(msg_out)

        # Publish to each /target_position_i topic once
        for i in [0, 1]:
            float_msg = Float32()
            float_msg.data = lengths[i]
            self.actuator_pubs[i].publish(float_msg)
            time.sleep(0.05)
        
        for i in [2, 3]:
            float_msg = Float32()
            float_msg.data = lengths[i]
            self.actuator_pubs[i].publish(float_msg)
            time.sleep(0.05)
        
        for i in [4, 5]:
            float_msg = Float32()
            float_msg.data = lengths[i]
            self.actuator_pubs[i].publish(float_msg)
            time.sleep(0.05)

    def rotX(self, theta):
        return np.array([
            [1, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta), np.cos(theta)]
        ])

    def rotY(self, theta):
        return np.array([
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)]
        ])

    def rotZ(self, theta):
        return np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ])

def main(args=None):
    rclpy.init(args=args)
    node = StewartKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
