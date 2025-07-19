import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')

        self.publisher_ = self.create_publisher(PoseStamped, 'pose', 10)

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_pose)

        # Initial pose values (you can change these as needed)
        self.x = 100.0
        self.y = 150.0

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = 0.0

        # Orientation: no rotation
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing pose: x={self.x}, y={self.y}')


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
