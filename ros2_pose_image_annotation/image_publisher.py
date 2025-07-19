import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
from datetime import datetime

class MapToImagePublisher(Node):
    def __init__(self):
        super().__init__('map_to_image_publisher')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)

        self.publisher_ = self.create_publisher(Image, 'map_image', 10)

        self.save_dir = os.path.expanduser('~/map_images')
        os.makedirs(self.save_dir, exist_ok=True)

        self.get_logger().info(f'Listening to /map, publishing to /map_image, saving to {self.save_dir}')

    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        img = np.zeros((height, width), dtype=np.uint8)
        img[data == -1] = 127  # Unknown area gray
        img[data == 0] = 255   # Free space white
        img[data == 100] = 0   # Occupied space black

        # Flip vertically for correct orientation
        img = cv2.flip(img, 0)

        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='mono8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = msg.header.frame_id

        self.publisher_.publish(img_msg)
        self.get_logger().info('Published map as image')

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(self.save_dir, f'map_{timestamp}.png')
        cv2.imwrite(filename, img)
        self.get_logger().info(f'Saved map image to {filename}')


def main(args=None):
    rclpy.init(args=args)
    node = MapToImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
