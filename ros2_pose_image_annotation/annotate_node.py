import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid
from cv_bridge import CvBridge
import cv2

class AnnotateNode(Node):
    def __init__(self):
        super().__init__('annotate_node')

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_pose = None
        self.map_info = None  # To store map metadata (resolution, origin, width, height)

        self.image_sub = self.create_subscription(
            Image, 'map_image', self.image_callback, 10)

        self.pose_sub = self.create_subscription(
            PoseStamped, 'pose', self.pose_callback, 10)

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        self.image_pub = self.create_publisher(Image, 'annotated_image', 10)

    def map_callback(self, msg: OccupancyGrid):
        self.map_info = msg.info

    def image_callback(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            self.latest_image = None
        self.annotate_and_publish()

    def pose_callback(self, msg: PoseStamped):
        self.latest_pose = msg
        self.annotate_and_publish()

    def annotate_and_publish(self):
        if self.latest_image is None or self.latest_pose is None or self.map_info is None:
            return

        annotated_img = cv2.cvtColor(self.latest_image, cv2.COLOR_GRAY2BGR)  # convert mono8 to BGR for color drawing

        # Map info for conversion
        res = self.map_info.resolution
        origin = self.map_info.origin.position
        width = self.map_info.width
        height = self.map_info.height

        # Convert world coordinates to pixel coordinates
        x_m = self.latest_pose.pose.position.x
        y_m = self.latest_pose.pose.position.y

        pixel_x = int((x_m - origin.x) / res)
        pixel_y = height - int((y_m - origin.y) / res)  # Flip y for image coords

        # Make sure pixel coords are inside image boundaries
        if not (0 <= pixel_x < width and 0 <= pixel_y < height):
            self.get_logger().warn(f"Pose pixel coords ({pixel_x},{pixel_y}) out of image bounds")
            return

        # Draw a red circle at pose position
        cv2.circle(annotated_img, (pixel_x, pixel_y), 10, (0, 0, 255), -1)  # red circle

        # Draw label above circle
        label = f"Pose ({x_m:.2f}, {y_m:.2f})"
        text_offset_y = 15
        text_pos = (pixel_x, max(pixel_y - text_offset_y, 0))

        (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(
            annotated_img,
            (text_pos[0], text_pos[1] - text_height - baseline),
            (text_pos[0] + text_width, text_pos[1] + baseline),
            (0, 0, 0),  # black rectangle for readability
            thickness=cv2.FILLED
        )
        cv2.putText(
            annotated_img,
            label,
            text_pos,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),  # white text
            1,
            cv2.LINE_AA
        )

        # Convert back to ROS Image and publish
        try:
            msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.latest_pose.header.frame_id
            self.image_pub.publish(msg)
            self.get_logger().info(f'Published annotated image with pose at pixel ({pixel_x}, {pixel_y})')
        except Exception as e:
            self.get_logger().error(f"Failed to publish annotated image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = AnnotateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
