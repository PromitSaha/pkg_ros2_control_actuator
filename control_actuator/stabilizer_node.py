import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, Point, Quaternion
import math
import time

import numpy as np
import transforms3d.quaternions as quat
import transforms3d.euler as euler

class StabilizerNode(Node):
    def __init__(self):
        super().__init__('stabilizer_node')

        self.kp_roll = 0.5
        self.kp_pitch = 0.5

        self.base_height = 0.1  # meters

        # Cooldown settings
        self.cooldown = 0.1  # seconds (publish rate limit)
        self.last_correction_time = 0

        self.stability_threshold = 0.08  # 10 deg ~ 0.17 rad, 5 deg ~ 0.08 rad)
        self.last_sent_x = 0.0
        self.last_sent_y = 0.0
        self.correction_threshold = 0.04

        self.subscription = self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)
        self.publisher = self.create_publisher(Pose, '/desired_pose', 10)

        self.get_logger().info("✅ Stabilizer node with smarter loop initialized.")

    def imu_callback(self, msg):
        g = 9.81
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # Estimate roll and pitch
        roll = math.atan2(ay, az)
        pitch = math.asin(max(-1.0, min(1.0, ax / g)))

        # Check if within stable range
        stable = abs(roll) <= self.stability_threshold and abs(pitch) <= self.stability_threshold

        # Compute control signal
        correction_x = -self.kp_roll * roll
        correction_y = -self.kp_pitch * pitch

        now = time.time()
        time_since_last = now - self.last_correction_time
        self.get_logger().info(f"🔍 Roll={math.degrees(roll):.2f}°, Pitch={math.degrees(pitch):.2f}°")
        if stable:
            self.get_logger().info("✅ Stable. No correction needed.")
            return

        if time_since_last < self.cooldown:
            self.get_logger().info("🕒 Cooldown active. Skipping correction.")
            return

        # Only publish if correction is different enough
        delta_x = abs(correction_x - self.last_sent_x)
        delta_y = abs(correction_y - self.last_sent_y)

        if delta_x < self.correction_threshold and delta_y < self.correction_threshold:
            self.get_logger().info("⚠️ Correction too small. Skipping.")
            return

        pose = Pose()
        pose.position = Point(x=0.0, y=0.0, z=self.base_height)
        pose.orientation = Quaternion(x=correction_x, y=correction_y, z=0.0, w=0.0)

        self.publisher.publish(pose)
        self.get_logger().info(
            f"📤 Correction published: roll={roll:.2f}, pitch={pitch:.2f} "
            f"→ x={correction_x:.2f}, y={correction_y:.2f}"
        )

        self.last_sent_x = correction_x
        self.last_sent_y = correction_y
        self.last_correction_time = now


def main(args=None):
    rclpy.init(args=args)
    node = StabilizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
