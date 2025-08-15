import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from transforms3d.euler import quat2euler
import math

g = 9.81

class RollPitchPrinter(Node):
    def __init__(self):
        super().__init__('roll_pitch_printer')
        self.subscription = self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)

    def imu_callback(self, msg: Imu):
        #q = msg.orientation
        # # Convert quaternion to roll, pitch, yaw
        # roll, pitch, _ = quat2euler([q.w, q.x, q.y, q.z]) 
        # roll_deg = roll * 180.0 / 3.14159
        # pitch_deg = pitch * 180.0 / 3.14159

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        roll = math.atan2(ay, az)
        pitch = math.asin(ax / g)

        # Convert to degrees
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)

        #return roll_deg, pitch_deg

        self.get_logger().info(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = RollPitchPrinter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
