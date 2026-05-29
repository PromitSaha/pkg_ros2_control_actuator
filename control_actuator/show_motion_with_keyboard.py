# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Pose, Point, Quaternion
# import time


# class KeyboardPoseController(Node):
#     def __init__(self):
#         super().__init__('keyboard_pose_controller')
#         self.publisher_ = self.create_publisher(Pose, '/desired_pose', 10)

#     def send_pose(self, orientation_x, orientation_y, orientation_z, orientation_w=1.0):
#         pose = Pose()
#         pose.position.x = 0.0
#         pose.position.y = 0.0
#         pose.position.z = 0.1
#         pose.orientation.x = orientation_x
#         pose.orientation.y = orientation_y
#         pose.orientation.z = orientation_z
#         pose.orientation.w = orientation_w
#         self.publisher_.publish(pose)
#         self.get_logger().info(
#             f'Published Pose -> roll(x)={orientation_x}, pitch(y)={orientation_y}, yaw(z)={orientation_z}'
#         )

# def main(args=None):
#     rclpy.init(args=args)
#     node = KeyboardPoseController()

#     print("\n--- Keyboard Pose Control ---")
#     print("r = Roll +, f = Roll -")
#     print("p = Pitch +, l = Pitch -")
#     print("y = Yaw +, u = Yaw -")
#     print("q = Quit")
#     print("-----------------------------")

#     while rclpy.ok():
#         cmd = input("Enter command: ").strip().lower()
#         if cmd == 'r':
#             node.send_pose(0.3, 0.0, 0.0)
#         elif cmd == 'f':
#             node.send_pose(-0.3, 0.0, 0.0)
#         elif cmd == 'p':
#             node.send_pose(0.0, 0.3, 0.0)
#         elif cmd == 'l':
#             node.send_pose(0.0, -0.3, 0.0)
#         elif cmd == 'y':
#             node.send_pose(0.0, 0.0, 0.5)
#         elif cmd == 'u':
#             node.send_pose(0.0, 0.0, -0.5)
#         elif cmd == 'q':
#             print("Exiting...")
#             break
#         else:
#             print("Invalid key!")

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
