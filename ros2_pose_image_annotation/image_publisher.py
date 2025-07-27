import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class MapImagePublisher(Node):
    def __init__(self):
        super().__init__('map_image_publisher')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10)

        self.publisher = self.create_publisher(Image, 'map_image', 10)

    def map_callback(self, msg: OccupancyGrid):
        # Convert OccupancyGrid to numpy image
        data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        
        # Convert [-1,100] to [0,255]
        img = np.zeros_like(data, dtype=np.uint8)
        img[data == -1] = 128  # Unknown = gray
        img[data == 0] = 255   # Free = white
        img[data > 0] = 0      # Occupied = black

        ros_image = self.bridge.cv2_to_imgmsg(img, encoding='mono8')
        ros_image.header = msg.header
        self.publisher.publish(ros_image)
        self.get_logger().info('Published map image')

def main(args=None):
    rclpy.init(args=args)
    node = MapImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
