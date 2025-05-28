import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
from std_msgs.msg import Int32


def apply_deadband(val, threshold=0.1):
    return val if abs(val) > threshold else 0.0

def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm < 1e-6:  # Very close to zero → invalid quaternion
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]  # Identity matrix (no rotation)

    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm

    return [
        [1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw,     1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw,     1 - 2*qx*qx - 2*qy*qy]
    ]

class KalmanFilter:
    def __init__(self, dt=0.1, process_noise=1e-4, measurement_noise=0.1):
        self.dt = dt
        self.x = [[0.0], [0.0]]  # [position, velocity]
        self.P = [[1, 0], [0, 1]]
        self.A = [[1, dt], [0, 1]]
        self.B = [[0.5 * dt ** 2], [dt]]
        self.H = [[1, 0]]
        self.Q = [[process_noise, 0], [0, process_noise]]
        self.R = [[measurement_noise]]

    def predict(self, accel):
        self.x = [
            [self.A[0][0]*self.x[0][0] + self.A[0][1]*self.x[1][0] + self.B[0][0]*accel],
            [self.A[1][0]*self.x[0][0] + self.A[1][1]*self.x[1][0] + self.B[1][0]*accel]
        ]

        A, P, Q = self.A, self.P, self.Q
        AP = [
            [A[0][0]*P[0][0] + A[0][1]*P[1][0], A[0][0]*P[0][1] + A[0][1]*P[1][1]],
            [A[1][0]*P[0][0] + A[1][1]*P[1][0], A[1][0]*P[0][1] + A[1][1]*P[1][1]]
        ]
        AT = list(zip(*A))
        self.P = [
            [AP[0][0]*AT[0][0] + AP[0][1]*AT[1][0] + Q[0][0], AP[0][0]*AT[0][1] + AP[0][1]*AT[1][1] + Q[0][1]],
            [AP[1][0]*AT[0][0] + AP[1][1]*AT[1][0] + Q[1][0], AP[1][0]*AT[0][1] + AP[1][1]*AT[1][1] + Q[1][1]]
        ]

    def update(self, z):
        H, P, R = self.H, self.P, self.R
        S = H[0][0]*P[0][0]*H[0][0] + R[0][0]
        K = [
            [P[0][0]*H[0][0]/S],
            [P[1][0]*H[0][0]/S]
        ]

        y = z - (H[0][0]*self.x[0][0])
        self.x[0][0] += K[0][0] * y
        self.x[1][0] += K[1][0] * y

        KH = [[K[0][0]*H[0][0], K[0][0]*H[0][1]],
              [K[1][0]*H[0][0], K[1][0]*H[0][1]]]
        I = [[1, 0], [0, 1]]
        I_KH = [
            [I[0][0] - KH[0][0], I[0][1] - KH[0][1]],
            [I[1][0] - KH[1][0], I[1][1] - KH[1][1]]
        ]
        self.P = [
            [I_KH[0][0]*P[0][0] + I_KH[0][1]*P[1][0], I_KH[0][0]*P[0][1] + I_KH[0][1]*P[1][1]],
            [I_KH[1][0]*P[0][0] + I_KH[1][1]*P[1][0], I_KH[1][0]*P[0][1] + I_KH[1][1]*P[1][1]]
        ]

    def get_position(self):
        return self.x[0][0]

class IMUKalmanNode(Node):
    def __init__(self):
        super().__init__('imu_kalman_node')
        self.kfx = KalmanFilter()
        self.kfy = KalmanFilter()
        self.kfz = KalmanFilter()
        self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)

        self.pub1 = self.create_publisher(Int32, '/actuator_command', 10)
        self.pub2 = self.create_publisher(Int32, '/actuator_command_2', 10)

    def send_command(self, actuator, value):
        msg = Int32()
        msg.data = value
        if actuator == 1:
            self.pub1.publish(msg)
        elif actuator == 2:
            self.pub2.publish(msg)
        self.get_logger().info(f"Actuator {actuator}: Command {value}")

    def imu_callback(self, msg):
        ax = apply_deadband(msg.linear_acceleration.x)
        ay = apply_deadband(msg.linear_acceleration.y)
        az = apply_deadband(msg.linear_acceleration.z)

        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        R = quaternion_to_rotation_matrix(qx, qy, qz, qw)

        # Rotate acceleration to global frame
        ax_g = R[0][0]*ax + R[0][1]*ay + R[0][2]*az
        ay_g = R[1][0]*ax + R[1][1]*ay + R[1][2]*az
        az_g = R[2][0]*ax + R[2][1]*ay + R[2][2]*az - 9.81  # gravity removed

        self.kfx.predict(ax_g)
        self.kfy.predict(ay_g)
        self.kfz.predict(az_g)

        self.kfx.update(ax_g)
        self.kfy.update(ay_g)
        self.kfz.update(az_g)

        if ay > 2:
            self.send_command(1, 1)
            self.send_command(2, -1)
        elif ay < -2:
            self.send_command(1, -1)
            self.send_command(2, 1)
        else:
            self.send_command(1, 0)
            self.send_command(2, 0)

        print(f"Position Estimate [m]: x={self.kfx.get_position():.3f}, y={self.kfy.get_position():.3f}, z={self.kfz.get_position():.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = IMUKalmanNode()
    rclpy.spin(node)
    rclpy.shutdown()
