import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import pygame
import sys
import time

class ImuMover(Node):
    def __init__(self):
        super().__init__('imu_mover')
        self.subscription = self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)
        self.last_time = time.time()

        # Initial position and velocity
        self.pos = [400.0, 300.0]  # center of screen
        self.vel = [0.0, 0.0]
        self.accel = [0.0, 0.0]  # ax, ay

        # Pygame setup
        pygame.init()
        self.screen = pygame.display.set_mode((800, 600))
        pygame.display.set_caption("IMU Object Mover")

    def imu_callback(self, msg):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # Get linear acceleration (in m/s²)
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y

        # Optional: subtract gravity if needed
        # ay -= 9.81  # depending on orientation

        # Integrate accel -> velocity
        self.vel[0] += ax * dt
        self.vel[1] += ay * dt

        # Integrate velocity -> position
        self.pos[0] += self.vel[0] * dt * 100  # scale for visibility
        self.pos[1] += self.vel[1] * dt * 100

        # Boundaries
        self.pos[0] = max(0, min(800, self.pos[0]))
        self.pos[1] = max(0, min(600, self.pos[1]))

        self.draw()

    def draw(self):
        self.screen.fill((30, 30, 30))
        pygame.draw.circle(self.screen, (0, 255, 0), (int(self.pos[0]), int(self.pos[1])), 10)
        pygame.display.flip()

    def spin(self):
        try:
            while rclpy.ok():
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        sys.exit()
                rclpy.spin_once(self, timeout_sec=0.01)
        except KeyboardInterrupt:
            pygame.quit()
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ImuMover()
    node.spin()

if __name__ == '__main__':
    main()
