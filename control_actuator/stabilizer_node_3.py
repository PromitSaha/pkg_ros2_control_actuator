import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, Point, Quaternion
import transforms3d.euler
import math
import time

class StabilizerNode(Node):
    def __init__(self):
        super().__init__('stabilizer_node')

        # PID gains for roll, pitch, and yaw (tune these for your specific hardware)
        self.kp_roll, self.ki_roll, self.kd_roll = 0.5, 0.0, 0.01
        self.kp_pitch, self.ki_pitch, self.kd_pitch = 0.5, 0.0, 0.01
        self.kp_yaw, self.ki_yaw, self.kd_yaw = 0.0, 0.0, 0.01

        self.base_height = 0.15  # meters

        # PID state variables
        self.last_roll_error, self.last_pitch_error, self.last_yaw_error = 0.0, 0.0, 0.0
        self.integral_roll, self.integral_pitch, self.integral_yaw = 0.0, 0.0, 0.0
        self.last_time = time.time()
        
        # --- Oscillation Reduction Parameters ---
        self.dead_zone_threshold = 0.12  # rad (~0.57 deg), for PID calculation
        self.max_integral = 0.5  # For preventing integral windup
        
        # Low-pass filter for sensor noise
        self.alpha = 0.8  # Filter coefficient (0 < alpha < 1). Higher alpha = less filtering.
        self.last_filtered_roll, self.last_filtered_pitch = 0.0, 0.0

        # Cooldown and stability settings
        self.cooldown = 3.0  # seconds (publish rate limit, 50Hz)
        self.last_correction_time = 0
        self.stability_threshold = 0.05  # rad, for skipping corrections when stable
        
        self.subscription = self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)
        self.publisher = self.create_publisher(Pose, '/desired_pose', 10)

        self.get_logger().info("✅ Stabilizer node with PID and filters initialized.")

    def imu_callback(self, msg):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0:
            return
        
        # --- Sensor Data Processing ---
        g = 9.81
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        
        roll_raw = math.atan2(ay, az)
        pitch_raw = math.asin(ax / g) if abs(ax / g) <= 1.0 else (math.copysign(math.pi/2, ax))
        
        # Apply low-pass filter to sensor data
        filtered_roll = self.alpha * roll_raw + (1 - self.alpha) * self.last_filtered_roll
        filtered_pitch = self.alpha * pitch_raw + (1 - self.alpha) * self.last_filtered_pitch

        self.last_filtered_roll = filtered_roll
        self.last_filtered_pitch = filtered_pitch

        # Yaw estimate (simplified integration of gyroscope data)
        yaw_rate = msg.angular_velocity.z
        self.integral_yaw += yaw_rate * dt
        yaw = self.integral_yaw

        # --- PID Controller Logic ---
        
        # Calculate PID errors (error is the current measured angle, as target is 0)
        roll_error = filtered_roll
        pitch_error = filtered_pitch
        yaw_error = yaw

        # Apply dead zone to errors to ignore tiny movements and reduce jitter
        if abs(roll_error) < self.dead_zone_threshold:
            roll_error = 0.0
        if abs(pitch_error) < self.dead_zone_threshold:
            pitch_error = 0.0
        if abs(yaw_error) < self.dead_zone_threshold:
            yaw_error = 0.0
            
        # Proportional terms
        p_roll = self.kp_roll * roll_error
        p_pitch = self.kp_pitch * pitch_error
        p_yaw = self.kp_yaw * yaw_error
        
        # Integral terms
        self.integral_roll += roll_error * dt
        self.integral_pitch += pitch_error * dt
        self.integral_yaw += yaw_error * dt
        
        # Clamp integral terms to prevent windup
        self.integral_roll = max(-self.max_integral, min(self.integral_roll, self.max_integral))
        self.integral_pitch = max(-self.max_integral, min(self.integral_pitch, self.max_integral))
        self.integral_yaw = max(-self.max_integral, min(self.integral_yaw, self.max_integral))

        i_roll = self.ki_roll * self.integral_roll
        i_pitch = self.ki_pitch * self.integral_pitch
        i_yaw = self.ki_yaw * self.integral_yaw
        
        # Derivative terms
        d_roll = self.kd_roll * ((roll_error - self.last_roll_error) / dt)
        d_pitch = self.kd_pitch * ((pitch_error - self.last_pitch_error) / dt)
        d_yaw = self.kd_yaw * ((yaw_error - self.last_yaw_error) / dt)

        # Final control signal
        correction_roll = p_roll + i_roll + d_roll
        correction_pitch = p_pitch + i_pitch + d_pitch
        correction_yaw = p_yaw + i_yaw + d_yaw
        
        # Skip publishing if platform is stable or cooldown is active
        time_since_last_correction = now - self.last_correction_time
        if (abs(roll_error) < self.stability_threshold and 
            abs(pitch_error) < self.stability_threshold and
            abs(yaw_error) < self.stability_threshold):
            self.integral_roll, self.integral_pitch, self.integral_yaw = 0.0, 0.0, 0.0
            return
        
        if time_since_last_correction < self.cooldown:
            return

        # Convert Euler angles to a quaternion using transforms3d
        # transforms3d returns the quaternion as [w, x, y, z]
        quat = transforms3d.euler.euler2quat(correction_roll, correction_pitch, correction_yaw, axes='sxyz')
        
        pose = Pose()
        pose.position = Point(x=0.0, y=0.0, z=self.base_height)
        
        # ROS geometry_msgs/Quaternion is ordered as [x, y, z, w]
        pose.orientation = Quaternion(x=(-1)*correction_roll, y=(-1)*correction_pitch, z=(-1)*correction_yaw, w=quat[0])

        self.publisher.publish(pose)
        
        self.get_logger().info(
            f"📤 Published Correction - Roll: {correction_roll:.2f}°, "
            f"Pitch: {correction_pitch:.2f}°, Yaw: {correction_yaw:.2f}°"
        )
        
        # Update state for next iteration
        self.last_roll_error = roll_error
        self.last_pitch_error = pitch_error
        self.last_yaw_error = yaw_error
        self.last_time = now
        self.last_correction_time = now

        #rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = StabilizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()