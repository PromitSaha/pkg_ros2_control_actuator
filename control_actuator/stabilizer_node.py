import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, Point, Quaternion
import math
import time

g = 9.81

class StabilizerNode(Node):
    def __init__(self):
        super().__init__('stabilizer_node')

        # Control sensitivity (scaling gain)
        self.kp_roll = 0.01  # adjust based on platform response
        self.kp_pitch = 0.01

        self.prev_x = 0.0
        self.prev_y = 0.0

        self.cur_x = 0.0
        self.cur_y = 0.0

        self.last_publish_time = time.time()

        # Desired base position (optional — keep z constant)
        self.base_height = 0.15  # meters

        self.subscription = self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)
        self.publisher = self.create_publisher(Pose, '/desired_pose', 10)

    def imu_callback(self, msg):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        roll = math.atan2(ay, az)
        pitch = math.asin(ax / g)

        self.get_logger().info(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}")

        if roll > 0.09 or roll < -0.09:
            self.cur_x = y=(-1)*roll
        if pitch > 0.09 or pitch < -0.09:
            self.cur_y = (-1)*pitch

        if self.prev_x is not self.cur_x or self.prev_y is not self.cur_y:
            self.get_logger().info(f"Found difference")
            pose = Pose()
            pose.position = Point(x=0.0, y=0.0, z=self.base_height)
            pose.orientation = Quaternion(x=self.cur_x, y=self.cur_y, z=0.0, w=0.0)

            self.get_logger().info(f"x: {self.cur_x:.2f}, y: {self.cur_y:.2f}")

            self.prev_x = self.cur_x
            self.prev_y = self.cur_y

            self.publisher.publish(pose)
            time.sleep(5)
        
def main(args=None):
    rclpy.init(args=args)
    node = StabilizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
