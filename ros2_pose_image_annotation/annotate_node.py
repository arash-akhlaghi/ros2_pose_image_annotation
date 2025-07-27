import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2

class AnnotateNode(Node):
    def __init__(self):
        super().__init__('annotate_node')
        self.bridge = CvBridge()

        # Store all received poses
        self.poses = []

        # Subscriptions
        self.create_subscription(Image, 'map_image', self.image_callback, 10)
        self.create_subscription(PoseStamped, 'random_pose', self.pose_callback, 10)

        # Publisher
        self.publisher = self.create_publisher(Image, 'annotated_image', 10)

    def pose_callback(self, msg: PoseStamped):
        # Append every pose (x, y)
        self.poses.append((msg.pose.position.x, msg.pose.position.y))

    def image_callback(self, msg: Image):
        # Convert image (grayscale or color) to BGR for drawing
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if len(cv_image.shape) == 2:
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)

        h, w = cv_image.shape[:2]

        if len(self.poses) > 0:
            # Find min/max from poses to normalize
            xs = [p[0] for p in self.poses]
            ys = [p[1] for p in self.poses]
            min_x, max_x = min(xs), max(xs)
            min_y, max_y = min(ys), max(ys)

            # Prevent divide by zero
            range_x = max(max_x - min_x, 1e-6)
            range_y = max(max_y - min_y, 1e-6)

            # Draw all points
            for i, (x_m, y_m) in enumerate(self.poses):
                # Normalize pose to fit image
                px = int((x_m - min_x) / range_x * (w - 1))
                py = int((y_m - min_y) / range_y * (h - 1))

                # Flip y-axis to match image coordinates
                py_img = h - py

                # Draw red circle
                cv2.circle(cv_image, (px, py_img), 0.1, (0, 0, 255), -1)

                # If last pose, draw its coordinates as text
                if i == len(self.poses) - 1:
                    cv2.putText(cv_image, f"({x_m:.1f},{y_m:.1f})",
                                (px + 8, py_img - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1, cv2.LINE_AA)

        # Publish annotated image
        out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        out_msg.header = msg.header
        self.publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AnnotateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
