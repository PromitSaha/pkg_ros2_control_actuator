import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R

class PlatformStabilizer(Node):
    def __init__(self):
        super().__init__('platform_stabilizer')

        self.subscription = self.create_subscription(
            Imu,
            '/imu_data',
            self.imu_callback,
            10
        )
        self.publisher = self.create_publisher(Pose, '/desired_pose', 10)

        self.get_logger().info("Platform Stabilizer Node Started")

    def imu_callback(self, msg):
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        r = R.from_quat([qx, qy, qz, qw])
        roll, pitch, yaw = r.as_euler('xyz')

        self.get_logger().info(f"IMU Roll: {roll:.4f}, Pitch: {pitch:.4f}, Yaw: {yaw:.4f}")

        # Invert roll and pitch to compensate
        corrective_roll = -roll
        corrective_pitch = -pitch

        # Construct Pose message
        pose = Pose()
        pose.position.z = 0.02  # Optional constant height
        pose.orientation.x = corrective_roll
        pose.orientation.y = corrective_pitch
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0  # Simplification — proper quaternion needed if tilt is large

        # Publish to desired_pose
        self.publisher.publish(pose)

    def quaternion_to_euler(qx, qy, qz, qw):
        r = R.from_quat([qx, qy, qz, qw])
        return r.as_euler('xyz', degrees=False)

def main(args=None):
    rclpy.init(args=args)
    node = PlatformStabilizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
