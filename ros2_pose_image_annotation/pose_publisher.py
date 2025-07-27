import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import random

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')

        # Publisher: publish PoseStamped messages on 'random_pose'
        self.publisher_ = self.create_publisher(PoseStamped, 'random_pose', 10)

        # Publish every 5 seconds
        timer_period = 5.0
        self.timer = self.create_timer(timer_period, self.publish_pose)

    def publish_pose(self):
        # Generate random (x,y) within the map (-10,10)
        x = random.uniform(-10.0, 10.0)
        y = random.uniform(-10.0, 10.0)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        # No rotation
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing random pose: x={x:.2f}, y={y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
