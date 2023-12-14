import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import math
import time

class CircularMotionPublisher(Node):
    def __init__(self):
        super().__init__('circular_motion_publisher')
        self.publisher_ = self.create_publisher(TransformStamped, '/target_pose', 10)
        self.timer_period = 0.1  # seconds
        self.i = 0
        self.t_0 = self.get_clock().now().to_msg().nanosec * 1e-9
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        t = time.time() - self.t_0
        msg.transform.translation.x = math.cos(t)
        msg.transform.translation.y = math.sin(t)
        msg.transform.translation.z = 0.0
        msg.transform.rotation.x = 0.0
        msg.transform.rotation.y = 0.0
        msg.transform.rotation.z = 0.0
        msg.transform.rotation.w = 1.0
        self.publisher_.publish(msg)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    circular_motion_publisher = CircularMotionPublisher()
    rclpy.spin(circular_motion_publisher)
    circular_motion_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
