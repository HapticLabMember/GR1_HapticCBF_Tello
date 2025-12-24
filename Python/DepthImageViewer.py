import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DepthViewerNode(Node):
    def __init__(self):
        super().__init__('depth_viewer_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth_image',
            self.image_callback,
            10
        )
        cv2.namedWindow("Depth Map", cv2.WINDOW_NORMAL)
        print("Depth viewer ready. Press Ctrl+C to exit.")

    def image_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            cv2.imshow("Depth Map", depth_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to convert or display image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

