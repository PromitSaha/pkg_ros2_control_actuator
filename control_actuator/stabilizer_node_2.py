import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
import transforms3d.quaternions as quat
import transforms3d.euler as euler
import math
import time


class StabilizerNode(Node):
    def __init__(self):
        super().__init__('stabilizer_node')

        self.kp_roll = 0.2
        self.kp_pitch = 0.2

        self.base_height = 0.15  # meters

        # Cooldown settings
        self.cooldown = 1.0  # seconds (publish rate limit)
        self.last_correction_time = 0

        self.stability_threshold = 0.08  # rad (~11.45 deg)
        self.last_sent_x = 0.0
        self.last_sent_y = 0.0

        self.subscription = self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)
        self.publisher = self.create_publisher(Pose, '/desired_pose', 10)

        self.get_logger().info("✅ Stabilizer node with smarter loop initialized.")

    def tilt_from_accel(ax, ay, az):
        g = np.array([ax, ay, az], dtype=float)
        if np.linalg.norm(g) < 1e-6:
            return None  # invalid
        z_b = -g / np.linalg.norm(g)              # body Z pointing opposite gravity
        x_guess = np.array([1.0, 0.0, 0.0])
        # make x_b orthogonal to z_b
        x_b = x_guess - np.dot(x_guess, z_b) * z_b
        if np.linalg.norm(x_b) < 1e-6:
            x_b = np.array([0.0, 1.0, 0.0]) - np.dot([0.0,1.0,0.0], z_b)*z_b
        x_b /= np.linalg.norm(x_b)
        y_b = np.cross(z_b, x_b)

        # Rotation from body→world with columns as body axes in world
        R = np.column_stack((x_b, y_b, z_b))

        # Convert to quat then to RPY with transforms3d
        qw, qx, qy, qz = quat.mat2quat(R)
        roll, pitch, yaw = euler.quat2euler([qw, qx, qy, qz], axes='sxyz')  # yaw arbitrary without magnetometer
        return roll, pitch, yaw

    def imu_callback(self, msg):
        g = 9.81
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        g = np.array([ax, ay, az], dtype=float)
        if np.linalg.norm(g) < 1e-6:
            return None  # invalid
        z_b = -g / np.linalg.norm(g)              # body Z pointing opposite gravity
        x_guess = np.array([1.0, 0.0, 0.0])
        # make x_b orthogonal to z_b
        x_b = x_guess - np.dot(x_guess, z_b) * z_b
        if np.linalg.norm(x_b) < 1e-6:
            x_b = np.array([0.0, 1.0, 0.0]) - np.dot([0.0,1.0,0.0], z_b)*z_b
        x_b /= np.linalg.norm(x_b)
        y_b = np.cross(z_b, x_b)

        # Rotation from body→world with columns as body axes in world
        R = np.column_stack((x_b, y_b, z_b))

        # Convert to quat then to RPY with transforms3d
        qw, qx, qy, qz = quat.mat2quat(R)
        roll, pitch, yaw = euler.quat2euler([qw, qx, qy, qz], axes='sxyz')  # yaw arbitrary without magnetometer

        # Estimate roll and pitch
        roll = math.atan2(ay, az)
        pitch = math.asin(max(-1.0, min(1.0, ax / g)))

        # Check if within stable range
        stable = abs(roll) < self.stability_threshold and abs(pitch) < self.stability_threshold

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

        if delta_x < 0.005 and delta_y < 0.005:
            self.get_logger().info("⚠️ Correction too small. Skipping.")
            return

        pose = Pose()
        pose.position = Point(x=0.0, y=0.0, z=self.base_height)
        pose.orientation = Quaternion(x=correction_x, y=correction_y, z=0.0, w=0.0)

        self.publisher.publish(pose)
        self.get_logger().info(
            f"📤 Correction published: roll={math.degrees(roll):.2f}°, pitch={math.degrees(pitch):.2f}° "
            f"→ x={correction_x:.3f}, y={correction_y:.3f}"
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
