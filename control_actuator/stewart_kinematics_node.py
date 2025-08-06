import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray, Float32, Int64
import numpy as np
import transforms3d.euler
import time
from .inverseKinematics import inv_kinematics

class StewartKinematicsNode(Node):
    def __init__(self):
        super().__init__('stewart_kinematics_node')

        # Platform and base geometry (hardcoded)
        self.r_B = 0.162      # Base radius in meters
        self.r_P = 0.130      # Platform radius in meters
        self.gamma_B = .2269   # Base anchor pair angle
        self.gamma_P = .82   # Platform anchor pair angle
        self.home_pos = np.array([0, 0, 0.457])  # Neutral height in meters

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
            self.create_publisher(Int64, f'/target_positions_{i+1}', 10)
            for i in range(3)
        ]

    def mergeTwoActuatorCommand(self, target1, target2):
        rounded_target1 = round(target1 * 1000, 1)
        rounded_target2 = round(target2 * 1000, 1)

        mergedValue = math.floor(rounded_target1*10*10000 + (rounded_target2*10))
        msg = Int64()
        msg.data = mergedValue

        return msg

    def pose_callback(self, msg):
        # Extract translation
        trans = np.array([msg.position.x, msg.position.y, msg.position.z])
        # Extract rotation (quaternion -> RPY)
        rotation = [msg.orientation.x, msg.orientation.y, msg.orientation.z]

        platform = inv_kinematics()
        lengths = platform.solve(trans, rotation)

        
        # Publish full array
        #print(lengths)
        msg_out = Float64MultiArray()
        msg_out.data = lengths.tolist()
        self.length_pub.publish(msg_out)

        # Publish to each /target_position_i topic once
        target1 = self.mergeTwoActuatorCommand(lengths[0], lengths[1])
        self.actuator_pubs[0].publish(target1)
        time.sleep(0.1)

        target2 = self.mergeTwoActuatorCommand(lengths[2], lengths[3])
        self.actuator_pubs[1].publish(target2)
        time.sleep(0.1)

        target3 = self.mergeTwoActuatorCommand(lengths[4], lengths[5])
        self.actuator_pubs[2].publish(target3)
        time.sleep(0.1)

        #print(target1, " ", target2, " ", target3)

def main(args=None):
    rclpy.init(args=args)
    node = StewartKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
