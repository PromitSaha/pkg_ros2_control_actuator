import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from .madgwickahrs import MadgwickAHRS  # Local file you copied
from transforms3d.euler import quat2euler

class MadgwickFilterNode(Node):
    def __init__(self):
        super().__init__('madgwick_filter_node')

        # Madgwick filter: sample rate = 100 Hz
        self.ahrs = MadgwickAHRS(sampleperiod=1.0/100.0, beta=0.1)

        # Subscribers and publishers
        self.subscription = self.create_subscription(
            Imu,
            '/imu_data',
            self.imu_callback,
            10
        )
        self.publisher = self.create_publisher(Imu, '/imu_filtered', 10)

    def imu_callback(self, msg: Imu):
        gyroscope = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ]
        accelerometer = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]

        # Update Madgwick filter
        self.ahrs.update_imu(gyroscope, accelerometer)
        q = self.ahrs.quaternion  # [w, x, y, z]

        # Create new IMU message with filtered orientation
        filtered_msg = Imu()
        filtered_msg.header = msg.header
        filtered_msg.angular_velocity = msg.angular_velocity
        filtered_msg.linear_acceleration = msg.linear_acceleration
        filtered_msg.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])  # ROS uses x,y,z,w

        self.publisher.publish(filtered_msg)
        self.print_euler_from_quaternion(filtered_msg.orientation)

    def print_euler_from_quaternion(self, q: Quaternion):
        euler = quat2euler([q.w, q.x, q.y, q.z])  # [w, x, y, z]
        roll = euler[0] * 180.0 / 3.14159
        pitch = euler[1] * 180.0 / 3.14159
        yaw = euler[2] * 180.0 / 3.14159
        self.get_logger().info(
            f"Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°"
        )

def main(args=None):
    rclpy.init(args=args)
    node = MadgwickFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
